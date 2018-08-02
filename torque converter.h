#ifndef __TC
#define __TC

extern const uint16_t maxpwm;
extern const uint8_t  minpwm;

class TC
{
public:
  enum status { RESET, UNLOCK, LOCK };
  TC()                     = default;
  TC(const TC&)            = delete;
  TC(TC&&)                 = delete;
  TC& operator=(const TC&) = delete;
  TC& operator=(TC&&)      = delete;
  ~TC()                    = default;
  void lock() const;
  void unlock() const;
  void reset() const;
  status state();
private:
  status st = RESET;
};

inline void TC::lock() const
{
  TIM_SetCompare2(TIM5, minpwm);//ТР
  TIM_SetCompare3(TIM5, maxpwm);//ФМ
}
inline void TC::unlock() const
{
  TIM_SetCompare3(TIM5, minpwm);//ФМ
  TIM_SetCompare2(TIM5, maxpwm);//ТР
}
inline void TC::reset() const
{
  TIM_SetCompare2(TIM5, minpwm);//ТР
  TIM_SetCompare3(TIM5, minpwm);//ФМ
}
inline TC::status TC::state() { return st; }

#endif //__TC