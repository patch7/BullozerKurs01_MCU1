#ifndef __KPP
#define __KPP

#include <list>
#include <queue>
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_flash.h"
#include <utility>
#include <array>
#include "sliding_median.h"

#define Left  (sm[0])
#define Right (sm[1])
#define Throt (sm[2])
#define Brake (sm[3])
#define Decel (sm[4])
#define OtL   (sm[5])
#define OtR   (sm[6])
#define BfL   (sm[7])
#define BfR   (sm[8])
#define For   (sm[9])
#define Rev   (sm[10])
#define Gear1 (sm[11])
#define Gear2 (sm[12])
#define Gear3 (sm[13])
#define Pres  (sm[14])

extern std::queue<CanTxMsg, std::list<CanTxMsg>> QueueCanTxMsg;
extern const uint16_t maxpwm;
extern const uint8_t  minpwm;

static const uint32_t address = 0x08060000;
static const uint8_t  koef    = 4;
static const uint8_t  N       = 0;
static const uint8_t  F       = 1;
static const uint8_t  R       = 2;
static const uint8_t  PLUS    = 1;
static const uint8_t  MINUS   = 2;
static const uint8_t  OFF     = 0;
static const uint8_t  ON      = 1;

union Pressure{ uint32_t i; float f; };
static SlidingMedian<uint16_t> sm[15];

class Calibrate
{
public:
  friend class KPP;

  enum State{Not, OtLeftV, OtRightV, BfLeftV, BfRightV, ForwardV, ReverseV, OneV, TwoV, ThreeV};

  Calibrate() { FlashRead(); }
  Calibrate(const Calibrate&)            = delete;
  Calibrate(Calibrate&&)                 = delete;
  Calibrate& operator=(const Calibrate&) = delete;
  Calibrate& operator=(Calibrate&&)      = delete;
  ~Calibrate()                           = default;

  void OtLeftTime(CanRxMsg&);
  void OtLeftPres(CanRxMsg&);
  void OtRightTime(CanRxMsg&);
  void OtRightPres(CanRxMsg&);
  void BfLeftTime(CanRxMsg&);
  void BfLeftPres(CanRxMsg&);
  void BfRightTime(CanRxMsg&);
  void BfRightPres(CanRxMsg&);
  void ForwardTime(CanRxMsg&);
  void ForwardPres(CanRxMsg&);
  void ReverseTime(CanRxMsg&);
  void ReversePres(CanRxMsg&);
  void OneTime(CanRxMsg&);
  void OnePres(CanRxMsg&);
  void TwoTime(CanRxMsg&);
  void TwoPres(CanRxMsg&);
  void ThreeTime(CanRxMsg&);
  void ThreePres(CanRxMsg&);

  void Save() const;
  void Valve(State&, Pressure);
  void CtrlAndRPM(uint8_t, uint16_t);
  void SendData();
  void SendDataValve();
private:
  void FlashWrite() const;
  void FlashRead();
  void Send(CanTxMsg&, std::pair<uint16_t, uint16_t>*);
  
  struct Data
  {
    std::pair<uint16_t, uint16_t> OtLeftTimePres[8];
    std::pair<uint16_t, uint16_t> OtRightTimePres[8];
    std::pair<uint16_t, uint16_t> BfLeftTimePres[8];
    std::pair<uint16_t, uint16_t> BfRightTimePres[8];
    std::pair<uint16_t, uint16_t> ForwardTimePres[8];
    std::pair<uint16_t, uint16_t> ReverseTimePres[8];
    std::pair<uint16_t, uint16_t> OneTimePres[8];
    std::pair<uint16_t, uint16_t> TwoTimePres[8];
    std::pair<uint16_t, uint16_t> ThreeTimePres[8];
    std::pair<uint16_t, uint16_t> AnalogCtrlAndRPM[6];// Rud; Left; Right; Brake; Decl; RPM;
    //номер каждого элемента массива соответствует значению регистра таймера умноженного на 4, в каждом массиве храним значение давления умноженное на 10.
    std::array<std::array<uint16_t, 125>, 9> Valve;//OTl, OTr, BFl, BFr, F, R, 1, 2, 3
  }d;
};
inline void Calibrate::Save() const { FlashWrite(); }
inline void Calibrate::OtLeftTime(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OtLeftTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::OtLeftPres(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OtLeftTimePres[i].second = RxMsg.Data[i];
}
inline void Calibrate::OtRightTime(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OtRightTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::OtRightPres(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OtRightTimePres[i].second = RxMsg.Data[i];
}
inline void Calibrate::BfLeftTime(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.BfLeftTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::BfLeftPres(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.BfLeftTimePres[i].second = RxMsg.Data[i];
}
inline void Calibrate::BfRightTime(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.BfRightTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::BfRightPres(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.BfRightTimePres[i].second = RxMsg.Data[i];
}
inline void Calibrate::ForwardTime(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ForwardTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::ForwardPres(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ForwardTimePres[i].second = RxMsg.Data[i];
}
inline void Calibrate::ReverseTime(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ReverseTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::ReversePres(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ReverseTimePres[i].second = RxMsg.Data[i];
}
inline void Calibrate::OneTime(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OneTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::OnePres(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OneTimePres[i].second = RxMsg.Data[i];
}
inline void Calibrate::TwoTime(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.TwoTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::TwoPres(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.TwoTimePres[i].second = RxMsg.Data[i];
}
inline void Calibrate::ThreeTime(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ThreeTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::ThreePres(CanRxMsg& RxMsg){
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ThreeTimePres[i].second = RxMsg.Data[i];
}

class KPP
{
public:
  explicit KPP(const uint16_t filter_size):
    //d_generator(0),
    old_direct(0),
    oil_filter(0),
    parking_ch(0),
    direction(0),
    clutch_st(0),
    reverse(0),
    start_eng(0),
    clutch_ch(0),
    parking(1),
    direct_ch(0),
    clutch(0),
    old_clutch_st(0) {}
  KPP(const KPP&)             = delete;
  KPP(KPP&&)                  = delete;
  KPP& operator= (const KPP&) = delete;
  KPP& operator= (KPP&&)      = delete;
  ~KPP()                      = default;

  //logic control
  void BrakeRotate(Calibrate&);
  void SwitchDirection(Calibrate&);
  void Parking();
  void SetClutch();
  void GraphSetFR();

  //Work with the data
  void DigitalSet(const uint32_t);
  void AnalogSet(const uint16_t*);

  void Send();

  void SetRpm(const uint16_t);
  void RequestRpm(Calibrate&, const uint16_t x = 0);

  bool Filter_Is_On()      const;
  bool Parking_Is_On()     const;
  bool Parking_Is_Change() const;
private:
  void SetOtL(const uint16_t d = maxpwm) const;
  void SetOtR(const uint16_t d = maxpwm) const;
  void ResetOtL()                        const;
  void ResetOtR()                        const;

  void SetBfL(const uint16_t d = minpwm) const;
  void SetBfR(const uint16_t d = minpwm) const;
  void ResetBfL()                        const;
  void ResetBfR()                        const;

  void SetFirst()                        const;
  void SetSecond()                       const;
  void SetThird()                        const;

  void ResetFirst()                      const;
  void ResetSecond()                     const;
  void ResetThird()                      const;

  void SetForward()                      const;
  void SetReverse()                      const;
  void ResetForward()                    const;
  void ResetReverse()                    const;

  void OnClutch()                        const;
  void OffClutch()                       const;

  static const uint8_t num_point = 125;
  const uint8_t resol            = 8;//Коэффициент для оборотов ДВС

  std::pair<uint16_t, uint16_t>* pFR       = nullptr;
  std::pair<uint16_t, uint16_t>* pFR_begin = nullptr;
  std::pair<uint16_t, uint16_t>* pFR_end   = nullptr;
  std::array<std::array<uint16_t, num_point>, 9>::const_iterator pValve;

  uint16_t countFR = 1;
  uint16_t rpm     = 0;//Текущие обороты ДВС.
  uint8_t  mul_tim = maxpwm / num_point;//Коэффициент для заполнения ШИМ

  uint8_t direction    : 2;//00 - N,    01 - F,   10 - R,    11 - Not available
  uint8_t clutch_st    : 2;//00 - none, 01 - '+', 10 - '-',  11 - Not available
  uint8_t parking      : 2;//00 - off,  01 - on,  10 - res,  11 - Don't care
  uint8_t reverse      : 2;//00 - off,  01 - on,  10 - res,  11 - Don't care
  uint8_t clutch       : 2;//00 - 0,    01 - 1,   10 - 2,    11 - 3
  uint8_t old_direct   : 2;//00 - N,    01 - F,   10 - R,    11 - Not available
  uint8_t old_clutch_st: 2;//00 - none, 01 - '+', 10 - '-',  11 - Not available
  uint8_t oil_filter   : 1;//true/false
  //uint8_t d_generator  : 1;//true/false
  uint8_t start_eng    : 1;//true/false
  uint8_t parking_ch   : 1;//true/false
  uint8_t clutch_ch    : 1;//true/false
  uint8_t direct_ch    : 1;//true/false

  bool UseRud  = true;
  bool PropF   = false;
  bool PropR   = false;
  bool Prop1   = false;
  bool Prop2   = false;
  bool Prop3   = false;
};

inline bool KPP::Filter_Is_On()      const { return oil_filter; }
inline bool KPP::Parking_Is_On()     const { return parking;    }
inline bool KPP::Parking_Is_Change() const { return parking_ch; }

inline void KPP::SetOtL(const uint16_t d) const { TIM_SetCompare1(TIM4, maxpwm - d); }
inline void KPP::SetOtR(const uint16_t d) const { TIM_SetCompare2(TIM4, maxpwm - d); }
inline void KPP::ResetOtL()               const { TIM_SetCompare1(TIM4, maxpwm); }//ОТ выключен
inline void KPP::ResetOtR()               const { TIM_SetCompare2(TIM4, maxpwm); }//ОТ выключен

inline void KPP::SetBfL(const uint16_t d) const { TIM_SetCompare3(TIM4, maxpwm - d); }
inline void KPP::SetBfR(const uint16_t d) const { TIM_SetCompare4(TIM4, maxpwm - d); }
inline void KPP::ResetBfL()               const { TIM_SetCompare3(TIM4, minpwm); }
inline void KPP::ResetBfR()               const { TIM_SetCompare4(TIM4, minpwm); }

inline void KPP::SetFirst()               const { TIM_SetCompare1(TIM3, maxpwm); }
inline void KPP::SetSecond()              const { TIM_SetCompare2(TIM3, maxpwm); }
inline void KPP::SetThird()               const { TIM_SetCompare3(TIM3, maxpwm); }

inline void KPP::ResetFirst()             const { TIM_SetCompare1(TIM3, minpwm); }
inline void KPP::ResetSecond()            const { TIM_SetCompare2(TIM3, minpwm); }
inline void KPP::ResetThird()             const { TIM_SetCompare3(TIM3, minpwm); }

inline void KPP::SetForward()             const { TIM_SetCompare4(TIM3, maxpwm); }
inline void KPP::SetReverse()             const { TIM_SetCompare1(TIM5, maxpwm); }
inline void KPP::ResetForward()           const { TIM_SetCompare4(TIM3, minpwm); }
inline void KPP::ResetReverse()           const { TIM_SetCompare1(TIM5, minpwm); }

inline void KPP::SetRpm(const uint16_t x)       { rpm = x / resol; };
#endif /* __KPP */