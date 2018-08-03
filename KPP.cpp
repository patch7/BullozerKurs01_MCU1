#include "KPP.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
///   ||      ||  ////////  ////////  //    //       ///////       /\     ////////     /\       ///
///   ||  /\  ||     //        //     //    //       //    //     //\\       //       //\\      ///
///   || //\\ ||     //        //     ////////       //     //   //  \\      //      //  \\     ///
///   ||//  \\||     //        //     //    //       //    //   //////\\     //     //////\\    ///
///   | /    \ |  ////////     //     //    //       //////    //      \\    //    //      \\   ///
///////////////////////////////////////////////////////////////////////////////////////////////////
void KPP::DigitalSet(const uint16_t data)//Good
{
  if(parking != (0x0003 & (data >> 4)))
    parking_ch = true;
  else
    parking_ch = false;

  if(direction != (0x0003 & data))
  {
    direct_ch  = true;
    old_direct = direction;
  }
  else
    direct_ch  = false;
  
  direction   = 0x0003 & data;
  clutch_st   = 0x0003 & (data >> 2);
  parking     = 0x0003 & (data >> 4);
  reverse     = 0x0003 & (data >> 6);
  oil_filter  = 0x0001 & (data >> 8);
  //d_generator = 0x0001 & (data >> 9);
}
void KPP::AnalogSet(const uint16_t* data)//Good
{
  Rev.push(data[0]);
  Gear2.push(data[1]);
  BfL.push(data[4]);
  BfR.push(data[6]);
  OtR.push(data[8]);
  For.push(data[11]);
  Gear1.push(data[13]);
  OtL.push(data[14]);
  Gear3.push(data[20]);

  //cal.Left.push(data[0]);
  //cal.Right.push(data[1]);
  //cal.Throt.push(data[2]);
  //cal.Brake.push(data[3]);
  //cal.Decel.push(data[4]);
  //cal.Temp.push(data[5]);
}
void KPP::RequestRpm(Calibrate& cal, const uint16_t x)
{
  CanTxMsg TxMessage;
  TxMessage.ExtId = 0x0C000000;
  TxMessage.RTR   = CAN_RTR_DATA;
  TxMessage.IDE   = CAN_ID_EXT;
  TxMessage.DLC   = 8;

  TxMessage.Data[0] = 0x01;
  TxMessage.Data[3] = 0xFF;
  TxMessage.Data[4] = 0xFF;
  TxMessage.Data[5] = 0xFF;
  TxMessage.Data[6] = 0xFF;
  TxMessage.Data[7] = 0xFF;

  if(UseRud)
  {
    //RPMmin + ((RPMmax-RPMmin) / ((RUDmax - RUDmin) / (Throt - RUDmin)))
    uint16_t temp = cal.d.AnalogCtrlAndRPM[5].first  + ((cal.d.AnalogCtrlAndRPM[5].second -
                    cal.d.AnalogCtrlAndRPM[5].first) / ((cal.d.AnalogCtrlAndRPM[0].second -
                    cal.d.AnalogCtrlAndRPM[0].first) /
                    (Throt.get() - cal.d.AnalogCtrlAndRPM[0].first)));
    TxMessage.Data[1] = (temp * resol) % 256;
    TxMessage.Data[2] = (temp * resol) / 256;
  }
  else if(x)
  {
    TxMessage.Data[1] = (x * resol) % 256;
    TxMessage.Data[2] = (x * resol) / 256;
  }
  else
    return;

  QueueCanTxMsg.push(TxMessage);
}
void KPP::Send()//Good, надо исправить в соответствии с коментариями.
{
  CanTxMsg TxMessage;
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.DLC     = 8;

  for(TxMessage.StdId = 0x001; TxMessage.StdId < 0x004; ++TxMessage.StdId)
  {
    switch(TxMessage.StdId)
    {
      case 0x001:
        for(uint8_t i = 0; i < 8; ++i)
        {
          TxMessage.Data[i]   = static_cast<uint8_t>(sm[i].get());
          TxMessage.Data[++i] = static_cast<uint8_t>(sm[i].get() >> 8);
        }
        break;
      case 0x002:
        TxMessage.Data[0] = static_cast<uint8_t>(Decel.get());
        TxMessage.Data[1] = static_cast<uint8_t>(Decel.get() >> 8);
        for(uint8_t i = 2; i < TxMessage.DLC; ++i)
          TxMessage.Data[i] = static_cast<uint8_t>(sm[5+i-2].get() / 16.38);//На DI3 * на 4.375
        break;
      case 0x003:
        for(uint8_t i = 0; i < 3; ++i)
          TxMessage.Data[i] = static_cast<uint8_t>(sm[11+i].get() / 16.38);//На DI3 * на 4.375

        TxMessage.Data[3] = static_cast<uint8_t>(clutch<<6 | reverse<<4 | parking<<2 | direction);
        TxMessage.Data[4] = static_cast<uint8_t>(oil_filter << 1 | start_eng);
        TxMessage.Data[5] = 0;//Температура!
        TxMessage.Data[6] = 0;//Скорость трактора!
        TxMessage.Data[7] = 0;//Reserved!
        break;
    }
    QueueCanTxMsg.push(TxMessage);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////       ///////   //////   | \   ||  ////////  //////    //////   //            //////////
//////////       //       //    //  ||\\  ||     //     //   //  //    //  //            //////////
//////////       //       //    //  || \\ ||     //     //////   //    //  //            //////////
//////////       //       //    //  ||  \\||     //     // //    //    //  //            //////////
//////////       ///////   //////   ||   \ |     //     //   //   //////   ///////       //////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void KPP::Parking()//Исправить в соответствии с коментариями
{
  if(parking == ON && parking_ch)
  {
    ResetAllValve();
    clutch = 0;
  }
  else if(parking == OFF && parking_ch)//Проверить, возможно не соответствует ТТ
  {
    SetBfL();
    SetBfR();
    ResetOtL();//ОТ выключен
    ResetOtR();//ОТ выключен
    if(rpm > 350)//Убрать магическое число
    {
      clutch = 1;
      OnClutch();
    }
  }
}
void KPP::ResetAllValve() const//Good
{
  SetOtL();
  SetOtR();
  ResetBfL();
  ResetBfR();
  ResetFirst();
  ResetSecond();
  ResetThird();
  ResetForward();
  ResetReverse();
}
void KPP::SetClutch()//Проверить на пропорциональное управление
{
  if(parking == OFF && clutch_st == PLUS && clutch < 3 && rpm > 350)//magic num
  {//за счет времени спада давления в бустере передачи, алгоритм будет соответствовать ТТ, т.е. включается необходимая передача и через 100 мс выключается предыдущая.
    OffClutch();
    ++clutch;
    OnClutch();
    clutch_st = false;
  }
  else if(parking == OFF && clutch_st == MINUS && clutch > 1 && rpm > 350)//magic num
  {
    OffClutch();
    --clutch;
    OnClutch();
    clutch_st = false;
  }
}
void KPP::OnClutch() const//Good
{
  switch(clutch)
  {
  case 1: SetFirst();
    break;
  case 2: SetSecond();
    break;
  case 3: SetThird();
    break;
  }
}
void KPP::OffClutch() const//Good
{
  switch(clutch)
  {
  case 1: ResetFirst();
    break;
  case 2: ResetSecond();
    break;
  case 3: ResetThird();
    break;
  }
}
void KPP::SwitchDirection(Calibrate& cal)//Good привести в соответствии с коментариями
{
  if(direct_ch && parking == OFF)
  {
    if(direction)
    {
      if(rpm > 1000)//магическое число
      {
        UseRud = false;
        RequestRpm(cal, cal.d.AnalogCtrlAndRPM[5].first);//RPMmin, Надо узнать постоянно ли отправлять запрос ДВС или хватит одного!!!
      }
      countFR = 1;

      if(direction == F)
      {
        PropR  = false;
        ResetReverse();
        if(reverse && old_direct == R && clutch > 1)
        {
          OffClutch();
          --clutch;
          OnClutch();
        }
        pFR    = cal.d.ForwardTimePres;
        pValve = cal.d.Valve.begin() + 4;
        PropF  = true;
      }
      else if(direction == R)
      {
        PropF  = false;
        ResetForward();
        if(reverse && old_direct == F && clutch < 3)
        {
          OffClutch();
          ++clutch;
          OnClutch();
        }
        pFR    = cal.d.ReverseTimePres;
        pValve = cal.d.Valve.begin() + 5;
        PropR  = true;
      }
      pFR_begin = pFR;
      pFR_end   = pFR + 7;
    }
    else
    {
      ResetForward();
      ResetReverse();
    }
  }
          
  if(direction == N && parking == ON && rpm < 350)//магическое число
    start_eng = true;
  else
    start_eng = false;
}
void KPP::BrakeRotate(Calibrate& cal)//Good Надо доработать чтобы клапана ОТ отключались полностью
{
  uint16_t left  = maxpwm - (Left.get() - cal.d.AnalogCtrlAndRPM[1].first) *
                   maxpwm / (cal.d.AnalogCtrlAndRPM[1].second -
                             cal.d.AnalogCtrlAndRPM[1].first);
  uint16_t right = maxpwm - (Right.get() - cal.d.AnalogCtrlAndRPM[2].first) *
                   maxpwm / (cal.d.AnalogCtrlAndRPM[2].second -
                             cal.d.AnalogCtrlAndRPM[2].first);
  uint16_t brake =          (Brake.get() - cal.d.AnalogCtrlAndRPM[3].first) *
                   maxpwm / (cal.d.AnalogCtrlAndRPM[3].second -
                             cal.d.AnalogCtrlAndRPM[3].first);

  if(parking == OFF)
  {
    const uint16_t begin = maxpwm / 100 * 5;//5% снизу
    const uint16_t end   = maxpwm / 100 * 95;//5% сверху

    if(left <= begin)
      SetBfL();
    else if(left * 2 < maxpwm)
      SetBfL(left * 2);
    else if(left * 2 >= maxpwm)
      ResetBfL();

    //приоритет имеет тот орган управления, что сильнее тормозит.
    if(left <= begin && brake <= begin)
      ResetOtL();
    else if(brake < end && brake >= left)//педаль тормоза сильнее!
      SetOtL(brake);
    else if(left  < end && brake <  left)//левый джойстик сильнее!
      SetOtL(left);
    else if(brake >= end || left >= end)//если кто-то на максимуме!
      SetOtL();


    if(right <= begin)
      SetBfR();
    else if(right * 2 < maxpwm)
      SetBfR(right * 2);
    else if(right * 2 >= maxpwm)
      ResetBfR();

    //приоритет имеет тот орган управления, что сильнее тормозит.
    if(right <= begin && brake <= begin)
      ResetOtR();
    else if(brake < end && brake >= right)//педаль тормоза сильнее!
      SetOtR(brake);
    else if(right < end && brake <  right)//правый джойстик сильнее!
      SetOtR(right);
    else if(brake >= end || right >= end)//если кто-то на максимуме!
      SetOtR();
  }
}
void KPP::GraphSetFR()//Good
{
  if(PropF || PropR)//надо продумать как поступить с изменением оборотов при вкл/переключении.
  {
    if(countFR == 1 && PropF)//заброс давления
      TIM_SetCompare4(TIM3, maxpwm);
    else if(countFR == 1 && PropR)
      TIM_SetCompare2(TIM1, maxpwm);
    else if(countFR >= pFR->first)//каждая точка (1,2,3,4,5,6,7,8) и если несколько точек на одно t
    {
      auto   cur = std::lower_bound(pValve->begin(), pValve->end(), pFR->second);
      auto   prv = cur - 1;
      double res = 0;

      if(*cur != pFR->second)
        res = prv - pValve->begin() + (pFR->second - *prv) / static_cast<double>(*cur - *prv);
      else
        res = cur - pValve->begin();

      if(PropF)
        TIM_SetCompare4(TIM3, static_cast<uint32_t>(mul_tim + res * mul_tim));
      else
        TIM_SetCompare2(TIM1, static_cast<uint32_t>(mul_tim + res * mul_tim));

      if(pFR == pFR_end)
      {
        if(PropF)
          PropF = false;
        else
          PropR = false;
        countFR = 1;
        UseRud  = true;
        //Затруднительно запросить предыдущие обороты, и все равно через 100 мс они встанут от РУД
        return;
      }
      ++pFR;
    }
    else if(pFR > pFR_begin && countFR > (pFR - 1)->first && countFR < pFR->first)//между точек
    {
      uint16_t time   = pFR->first - (pFR - 1)->first;
      uint16_t res    = 0;
      double   resFR1 = 0;
      double   resFR2 = 0;

      auto cur = std::lower_bound(pValve->begin(), pValve->end(), pFR->second);
      auto prv = cur - 1;

      if(*cur != pFR->second)//запрошена точка не из графика
        resFR1 = prv - pValve->begin() + (pFR->second - *prv) / static_cast<double>(*cur - *prv);
      else
        resFR1 = cur - pValve->begin();

      cur = std::lower_bound(pValve->begin(), pValve->end(), (pFR - 1)->second);
      prv = cur - 1;

      if(*cur != (pFR - 1)->second)//запрошена точка не из графика
        resFR2 = prv - pValve->begin() + ((pFR-1)->second - *prv) / static_cast<double>(*cur-*prv);
      else
        resFR2 = cur - pValve->begin();

      if(pFR->second >= (pFR - 1)->second)//график на возрастание
        res = static_cast<uint16_t>(resFR1 - resFR2);
      else
        res = static_cast<uint16_t>(resFR2 - resFR1);

      double pwm  = static_cast<double>(res * mul_tim) / time * (countFR - (pFR - 1)->first);
      
      if(pFR->second >= (pFR - 1)->second)
        res = static_cast<uint16_t>(mul_tim + resFR2 * mul_tim + pwm);
      else
        res = static_cast<uint16_t>(mul_tim + resFR2 * mul_tim - pwm);

      if(PropF)
        TIM_SetCompare4(TIM3, static_cast<uint32_t>(res));
      else
        TIM_SetCompare2(TIM1, static_cast<uint32_t>(res));
    }
    ++countFR;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
////    ///////      /\      //       //////  //////   //////       /\     ////////  //////    ////
////    //          //\\     //         //    //   //  //   //     //\\       //     //        ////
////    //         //  \\    //         //    //////   //////     //  \\      //     //////    ////
////    //        //////\\   //         //    //   //  // //     //////\\     //     //        ////
////    ///////  //      \\  ///////  //////  //////   //   //  //      \\    //     //////    ////
///////////////////////////////////////////////////////////////////////////////////////////////////
void Calibrate::FlashWrite() const//Good привести в соответствие с коментариями
{
  const uint32_t* flash_address  = (uint32_t*)address;
  const uint32_t* source_address = (uint32_t*)&d;
  FLASH_Unlock();
  FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3);
  for (uint16_t i = 0; i < sizeof(d); i += 4)//магическое число поменять на sizeof(uint32_t)
    FLASH_ProgramWord((uint32_t)flash_address++, *source_address++);
  FLASH_Lock();
}
void Calibrate::FlashRead()//Good
{
  const uint32_t* flash_address = (uint32_t*)address;
  uint32_t* source_address      = (uint32_t*)&d;
  for (uint16_t i = 0; i < sizeof(d); i += 4)//магическое число поменять на sizeof(uint32_t)
    *source_address++ = *(__IO uint32_t*)flash_address++;
}
void Calibrate::CtrlAndRPM(uint8_t state, uint16_t data)//Good
{
  switch(state)
  {
    case 0x21:
      if(Throt.get() - 7 <= 0)
        d.AnalogCtrlAndRPM[0].first = 0;
      else
        d.AnalogCtrlAndRPM[0].first = Throt.get() - 7;         break;//min
    case 0x22:
      if(Left.get() - 7 <= 0)
        d.AnalogCtrlAndRPM[1].first = 0;
      else
        d.AnalogCtrlAndRPM[1].first = Left.get()  - 7;         break;//min
    case 0x23:
      if(Right.get() - 7 <= 0)
        d.AnalogCtrlAndRPM[2].first = 0;
      else
        d.AnalogCtrlAndRPM[2].first = Right.get() - 7;         break;//min
    case 0x24:
      if(Brake.get() - 7 <= 0)
        d.AnalogCtrlAndRPM[3].first = 0;
      else
        d.AnalogCtrlAndRPM[3].first = Brake.get() - 7;         break;//min
    case 0x25:
      if(Decel.get() - 7 <= 0)
        d.AnalogCtrlAndRPM[4].first = 0;
      else
        d.AnalogCtrlAndRPM[4].first = Decel.get() - 7;         break;//min
    case 0x26: d.AnalogCtrlAndRPM[5].first  = data;            break;//min

    case 0x41: d.AnalogCtrlAndRPM[0].second = Throt.get() + 7; break;//max
    case 0x42: d.AnalogCtrlAndRPM[1].second = Left.get()  + 7; break;//max
    case 0x43: d.AnalogCtrlAndRPM[2].second = Right.get() + 7; break;//max
    case 0x44: d.AnalogCtrlAndRPM[3].second = Brake.get() + 7; break;//max
    case 0x45: d.AnalogCtrlAndRPM[4].second = Decel.get() + 7; break;//max
    case 0x46: d.AnalogCtrlAndRPM[5].second = data;            break;//max
  }
}
void Calibrate::Send(CanTxMsg& TxMessage, std::pair<uint16_t, uint16_t>* data)//Good
{
  for(uint8_t i = 0; i < TxMessage.DLC; ++i)
    TxMessage.Data[i] = static_cast<uint8_t>(data[i].first / koef);
  QueueCanTxMsg.push(TxMessage);

  ++TxMessage.StdId;

  for(uint8_t i = 0; i < TxMessage.DLC; ++i)
    TxMessage.Data[i] = static_cast<uint8_t>(data[i].second);
  QueueCanTxMsg.push(TxMessage);
}
void Calibrate::SendData()//Good График включения клапана и органы управления
{
  CanTxMsg TxMessage;
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.DLC     = 8;

  TxMessage.StdId = 0x200;
  Send(TxMessage, d.OtLeftTimePres);
  TxMessage.StdId = 0x202;
  Send(TxMessage, d.OtRightTimePres);
  TxMessage.StdId = 0x204;
  Send(TxMessage, d.BfLeftTimePres);
  TxMessage.StdId = 0x206;
  Send(TxMessage, d.BfRightTimePres);
  TxMessage.StdId = 0x208;
  Send(TxMessage, d.ForwardTimePres);
  TxMessage.StdId = 0x20A;
  Send(TxMessage, d.ReverseTimePres);
  TxMessage.StdId = 0x20C;
  Send(TxMessage, d.OneTimePres);
  TxMessage.StdId = 0x20E;
  Send(TxMessage, d.TwoTimePres);
  TxMessage.StdId = 0x210;
  Send(TxMessage, d.ThreeTimePres);

  TxMessage.StdId = 0x212;
  for(uint8_t i = 0; TxMessage.StdId < 0x215; ++TxMessage.StdId, ++i)
  {
    TxMessage.Data[0] = static_cast<uint8_t>(d.AnalogCtrlAndRPM[i + i].first);
    TxMessage.Data[1] = static_cast<uint8_t>(d.AnalogCtrlAndRPM[i + i].first >> 8);
    TxMessage.Data[2] = static_cast<uint8_t>(d.AnalogCtrlAndRPM[i + i].second);
    TxMessage.Data[3] = static_cast<uint8_t>(d.AnalogCtrlAndRPM[i + i].second >> 8);
    TxMessage.Data[4] = static_cast<uint8_t>(d.AnalogCtrlAndRPM[i + i + 1].first);
    TxMessage.Data[5] = static_cast<uint8_t>(d.AnalogCtrlAndRPM[i + i + 1].first >> 8);
    TxMessage.Data[6] = static_cast<uint8_t>(d.AnalogCtrlAndRPM[i + i + 1].second);
    TxMessage.Data[7] = static_cast<uint8_t>(d.AnalogCtrlAndRPM[i + i + 1].second >> 8);
    QueueCanTxMsg.push(TxMessage);
  }
}
void Calibrate::SendDataValve()//Good Таблица калибровки клапана
{
  CanTxMsg TxMessage;
  TxMessage.RTR   = CAN_RTR_DATA;
  TxMessage.IDE   = CAN_ID_STD;
  TxMessage.DLC   = 8;
  TxMessage.StdId = 0x250;

  for(auto a : d.Valve)
  {
    uint8_t rest = 0;//остаток
    if(a.size() % TxMessage.DLC)
      rest = TxMessage.DLC - (a.size() % TxMessage.DLC);

    for(uint8_t i = 1, j = 1; i <= a.size() + rest; ++i, ++j)
      if(i > a.size())
        TxMessage.Data[(j % 9) - 1] = 0;
      else if(j % 9)
        TxMessage.Data[(j % 9) - 1] = static_cast<uint8_t>(a.at(i - 1));
      else
      {
        QueueCanTxMsg.push(TxMessage);
        ++TxMessage.StdId;
        --i;
      }

    QueueCanTxMsg.push(TxMessage);
    ++TxMessage.StdId;
  }
}
//Функция вызывается раз в 10 мс, каждый 22-й (220 мс) вызов сохраняет давление и увеличивает ток клапана. После каждого увеличения тока на клапане, происходит ожидание задержки реакции электромагнита (100 мс, запас в 25 мс) и реакции клапана (стабилизации давления 50 мс, запас 25 мс). Далее в течении 70 мс (т.е. 7 раз) записывается текущее значение давления в фильтр. После заполнения фильтра записываем давление в таблицу соответствия тока давлению и увеличиваем ток.
void Calibrate::Valve(State& state, Pressure pres)
{
  const uint8_t cycle_time   = 22;// 22 * 10 = 220 мс
  const uint8_t current_step = 4; // 500 / 4 = 125 точек
  static uint16_t count      = 0;

  auto pValve = d.Valve.begin();
  for(uint8_t i = 1; i < state; ++i, ++pValve);

  if(!count)
  {
    CanTxMsg TxMessage;
    TxMessage.RTR     = CAN_RTR_DATA;
    TxMessage.IDE     = CAN_ID_STD;
    TxMessage.DLC     = 1;
    TxMessage.StdId   = 0x114;
    TxMessage.Data[0] = 0x01;
    QueueCanTxMsg.push(TxMessage);//статус калибровки, идет калибровка клапана.
  }

  if(!(count % cycle_time) && count / cycle_time <= pValve->size())
  {
    if(count > 0)
      (*pValve)[count / cycle_time - 1] = Pres.get();//записываем давление
    if(count / cycle_time == pValve->size())//если последняя точка
    {
      switch(state)
      {
        case OtLeftV : TIM_SetCompare1(TIM4, count = 0); break;
        case OtRightV: TIM_SetCompare2(TIM4, count = 0); break;
        case BfLeftV : TIM_SetCompare3(TIM4, count = 0); break;
        case BfRightV: TIM_SetCompare4(TIM4, count = 0); break;
        case ForwardV: TIM_SetCompare4(TIM3, count = 0); break;
        case ReverseV: TIM_SetCompare2(TIM1, count = 0); break;
        case OneV    : TIM_SetCompare1(TIM3, count = 0); break;
        case TwoV    : TIM_SetCompare2(TIM3, count = 0); break;
        case ThreeV  : TIM_SetCompare3(TIM3, count = 0); break;
      }
      state = Not;

      CanTxMsg TxMessage;
      TxMessage.RTR     = CAN_RTR_DATA;
      TxMessage.IDE     = CAN_ID_STD;
      TxMessage.DLC     = 1;
      TxMessage.StdId   = 0x114;
      TxMessage.Data[0] = 0x00;
      QueueCanTxMsg.push(TxMessage);//статус калибровки, калибровка клапана окончена.
      return;
    }
    switch(state)
    {
      case OtLeftV : TIM_SetCompare1(TIM4, current_step + current_step *(count/cycle_time)); break;
      case OtRightV: TIM_SetCompare2(TIM4, current_step + current_step *(count/cycle_time)); break;
      case BfLeftV : TIM_SetCompare3(TIM4, current_step + current_step *(count/cycle_time)); break;
      case BfRightV: TIM_SetCompare4(TIM4, current_step + current_step *(count/cycle_time)); break;
      case ForwardV: TIM_SetCompare4(TIM3, current_step + current_step *(count/cycle_time)); break;
      case ReverseV: TIM_SetCompare2(TIM1, current_step + current_step *(count/cycle_time)); break;
      case OneV    : TIM_SetCompare1(TIM3, current_step + current_step *(count/cycle_time)); break;
      case TwoV    : TIM_SetCompare2(TIM3, current_step + current_step *(count/cycle_time)); break;
      case ThreeV  : TIM_SetCompare3(TIM3, current_step + current_step *(count/cycle_time)); break;
    }
  }
  if(count >= 15 + cycle_time * (count / cycle_time))//магическое число
    Pres.push(static_cast<uint16_t>(pres.f * 10));
  ++count;
}