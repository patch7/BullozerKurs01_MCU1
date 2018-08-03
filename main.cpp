#include <queue>
#include "stm32f4xx.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "sliding_median.h"
#include "KPP.h"
#include "torque converter.h"

#define CURRENT_PWM10  (ADCConvertedValue[0])
#define CURRENT_PWM11  (ADCConvertedValue[1])
#define CURRENT_PWM12  (ADCConvertedValue[2])
#define CURRENT_PWM5   (ADCConvertedValue[3])
#define CURRENT_PWM6   (ADCConvertedValue[4])
#define CURRENT_PWM7   (ADCConvertedValue[5])
#define CURRENT_PWM8   (ADCConvertedValue[6])
#define CURRENT_PWM1   (ADCConvertedValue[7])
#define CURRENT_PWM4   (ADCConvertedValue[8])
#define RESISTANCE_1   (ADCConvertedValue[9])
#define CURRENT_PWM14  (ADCConvertedValue[10])
#define CURRENT_PWM15  (ADCConvertedValue[11])
#define CURRENT_PWM16  (ADCConvertedValue[12])
#define CURRENT_PWM9   (ADCConvertedValue[13])
#define CURRENT_PWM2   (ADCConvertedValue[14])
#define CURRENT_PWM3   (ADCConvertedValue[15])
#define FREQUENCY_3    (ADCConvertedValue[16])
#define RESISTANCE_2   (ADCConvertedValue[17])
#define FREQUENCY_1    (ADCConvertedValue[18])
#define FREQUENCY_2    (ADCConvertedValue[19])
#define CURRENT_PWM13  (ADCConvertedValue[20])
#define FREQUENCY_4    (ADCConvertedValue[21])
#define RESISTANCE_REF (ADCConvertedValue[22])
#define FREQUENCY_REF  (ADCConvertedValue[23])

#define ONE_MS         (time_flag[0])
#define TEN_MS         (time_flag[1])
#define TWENTY_FIVE_MS (time_flag[2])
#define HUNDRED_MS     (time_flag[3])

std::queue<CanTxMsg> QueueCanTxMsg;
static Pressure         pres;
static Calibrate::State state = Calibrate::Not;
static Calibrate        cal;
static KPP              kpp(9);
//static TC               tc;
static uint32_t         time_ms       = 0;
static uint16_t         ADCConvertedValue[24] = {0};

const uint16_t maxpwm  = 500;
const uint8_t  minpwm  = 0;

//static const uint16_t DEFAULT        = 0x010;
static bool             time_flag[4] = {false};

void MaxAllRccBusConfig(void);
void DMAforADCInit(void);
void ADCInputInit(void);
void CANInit(void);
void TIM_PWMInit(void);
void TimerInit(void);
void FlashInit(void);
bool CanTxMailBoxEmpty(CAN_TypeDef*);

void main()
{
  MaxAllRccBusConfig();
  FlashInit();
  
  GPIO_DeInit(GPIOA);//CAN1, ADC2(ch1, ch3 - ch7), TIM1(ch1 - ch3)
  GPIO_DeInit(GPIOB);//ADC2ch8, TIM2ch4, TIM4(ch1 - ch4)
  GPIO_DeInit(GPIOC);//ADC3(ch10, ch11), ADC2(ch12 - ch15), TIM3(ch1 - ch4)
  GPIO_DeInit(GPIOD);
  GPIO_DeInit(GPIOF);//ADC3(ch4, ch6 - ch8)
  GPIO_DeInit(GPIOH);
  GPIO_DeInit(GPIOI);
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
  
  CANInit();
  DMAforADCInit();
  ADCInputInit();
  TIM_PWMInit();
  TimerInit();
  
  while(true)
  {
    //if(time_ms - end_transmit > 300)//при аварийной ситуации(обрыв CAN ДУ) все сигналы default
    //  kpp.DigitalSet(DEFAULT, cal);

    if(ONE_MS)
    {
      kpp.AnalogSet(ADCConvertedValue);
      kpp.GraphSetFR();//управление клапанами в пропорциональном режиме
      ONE_MS = false;
    }
    if(TEN_MS)
    {
      if(state != Calibrate::Not)
        cal.Valve(state, pres);//Калибровка клапана!
      TEN_MS = false;
    }
    if(TWENTY_FIVE_MS)
    {
      kpp.Send();
      TWENTY_FIVE_MS = false;
    }
    if(HUNDRED_MS)
    {
      kpp.Parking();
      kpp.SetClutch();
      kpp.SwitchDirection(cal);
      kpp.BrakeRotate(cal);
      kpp.RequestRpm(cal);

      //if(cal.Parking_Is_On())
      //  tc.reset();//очень часто вызывается
      //else if(cal.Filter_Is_On())
      //  tc.lock();//очень часто вызывается
      //else
      //  tc.unlock();//очень часто вызывается
      HUNDRED_MS = false;
    }

    while(!QueueCanTxMsg.empty() && CanTxMailBoxEmpty(CAN1))
    {
      CAN_Transmit(CAN1, &QueueCanTxMsg.front());
      QueueCanTxMsg.pop();
    }
  }
}

//Настраиваем тактирование ядра и переферии от HSE(25 МГц) на максимальные частоты.
void MaxAllRccBusConfig()
{
  SystemInit();
  SystemCoreClockUpdate();

  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  RCC_ClockSecuritySystemCmd(ENABLE);

  ErrorStatus State = RCC_WaitForHSEStartUp();
  
  if(State != SUCCESS)
    while(true) {}
  else
  {
    RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB clock = SYSCLK
    RCC_PCLK1Config(RCC_HCLK_Div4);//APB1 clock = HCLK/4
    RCC_PCLK2Config(RCC_HCLK_Div2);//APB2 clock = HCLK/2

    RCC_PLLConfig(RCC_PLLSource_HSE, 25, 336, 2, 7);//25 MHz
    RCC_PLLCmd(ENABLE);

    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//SYSCLK clock = PLLCLK
    while(RCC_GetSYSCLKSource() != 8) {}
  }
}
void FlashInit()
{
  FLASH_PrefetchBufferCmd(ENABLE);
  FLASH_SetLatency(FLASH_Latency_5);
}
//Настраиваем модуль DMA2 для автоматической обработки каналов ADC1 и ADC3
void DMAforADCInit()
{
  DMA_DeInit(DMA2_Stream0);//ADC1
  DMA_DeInit(DMA2_Stream1);//ADC3
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  DMA_InitTypeDef DMA_InitStruct;
  DMA_StructInit(&DMA_InitStruct);
  DMA_InitStruct.DMA_Channel            = DMA_Channel_0;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
  DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)ADCConvertedValue;
  DMA_InitStruct.DMA_BufferSize         = 16;
  DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStruct.DMA_Mode               = DMA_Mode_Circular;
  DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_Init(DMA2_Stream0, &DMA_InitStruct);
  
  DMA_InitStruct.DMA_Channel            = DMA_Channel_2;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC3->DR);
  DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)(ADCConvertedValue + 16);
  DMA_InitStruct.DMA_BufferSize         = 8;
  DMA_Init(DMA2_Stream1, &DMA_InitStruct);
  
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);

  DMA_Cmd(DMA2_Stream0, ENABLE);
  DMA_Cmd(DMA2_Stream1, ENABLE);

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x0;
  NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream1_IRQn;
  NVIC_Init(&NVIC_InitStruct);
}
/**************************************************************************************************
Настраиваем ADC1 (0-15), ADC3 (0-7) на преобразование.
**************************************************************************************************/
void ADCInputInit()
{
  ADC_DeInit();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                               GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                               GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 |
                               GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_Init(GPIOF, &GPIO_InitStruct);

  ADC_CommonInitTypeDef ADC_CommonInitStruct;
  ADC_CommonStructInit(&ADC_CommonInitStruct);
  ADC_CommonInit(&ADC_CommonInitStruct);

  ADC_InitTypeDef ADC_InitStruct;
  ADC_StructInit(&ADC_InitStruct);
  ADC_InitStruct.ADC_ScanConvMode         = ENABLE;
  ADC_InitStruct.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T2_TRGO;
  ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStruct.ADC_NbrOfConversion      = 16;
  ADC_Init(ADC1, &ADC_InitStruct);

  ADC_InitStruct.ADC_NbrOfConversion      = 8;
  ADC_Init(ADC3, &ADC_InitStruct);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0,  1,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1,  2,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2,  3,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3,  4,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  5,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5,  6,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6,  7,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7,  8,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8,  9,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9,  10, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 11, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 12, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 13, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 14, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 15, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 16, ADC_SampleTime_3Cycles);
  
  ADC_RegularChannelConfig(ADC3, ADC_Channel_4,  1,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_5,  2,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_6,  3,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_7,  4,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_8,  5,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_9,  6,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_14, 7,  ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_15, 8,  ADC_SampleTime_3Cycles);
  
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC3, ENABLE);
  ADC_SoftwareStartConv(ADC1);
  ADC_SoftwareStartConv(ADC3);
}
/**************************************************************************************************
Настраиваем таймер 7 и 8 для подсчета времени и прерывания ADC соответственно.
Необходимо обеспечить полное заполнение класса скользящей медианы до применения ее результатов,
путем изменения времени прерывания для DMA.
**************************************************************************************************/
void TimerInit()
{
  TIM_DeInit(TIM7);
  TIM_DeInit(TIM2);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 839;//всегда +1, Mgz*10
  TIM_TimeBaseInitStruct.TIM_Period    = 100;//1 мс
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);

  TIM_TimeBaseInitStruct.TIM_Period    = 100;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

  TIM_SetCounter(TIM2, 0);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM7_IRQn);

  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
  
  TIM_Cmd(TIM7, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}
void CANInit()
{
  CAN_DeInit(CAN1);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
  GPIO_Init(GPIOI, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource9,  GPIO_AF_CAN1);

  CAN_InitTypeDef CAN_InitStruct;//частота шины 42МГц / 24 = 1.75МГц / 7 = 250 кбит/с
  CAN_StructInit(&CAN_InitStruct);

  CAN_InitStruct.CAN_ABOM      = ENABLE;
  CAN_InitStruct.CAN_TXFP      = ENABLE;
  CAN_InitStruct.CAN_BS1       = CAN_BS1_3tq;
  CAN_InitStruct.CAN_Prescaler = 24;
  CAN_Init(CAN1, &CAN_InitStruct);

  CAN_FilterInitTypeDef CAN_FilterInitStruct;//без настройки фильтра не работает прием
  CAN_SlaveStartBank(0);//необходимо так же задать номер фильтра, без него не принимает
  CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000 << 5;
  CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000 << 5;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000 << 5;
  CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000 << 5;
  CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_16bit;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  CAN_FilterInitStruct.CAN_FilterNumber         = 0;
  CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStruct);
  
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel                   = CAN1_RX0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  NVIC_InitStruct.NVIC_IRQChannel                   = CAN1_RX1_IRQn;
  NVIC_Init(&NVIC_InitStruct);
  NVIC_InitStruct.NVIC_IRQChannel                   = CAN1_TX_IRQn;
  NVIC_Init(&NVIC_InitStruct);
  NVIC_InitStruct.NVIC_IRQChannel                   = CAN1_SCE_IRQn;
  NVIC_Init(&NVIC_InitStruct);

  CAN_ITConfig(CAN1, CAN_IT_FMP0|CAN_IT_FMP1|CAN_IT_TME|CAN_IT_EWG|CAN_IT_EPV|CAN_IT_LEC|
                     CAN_IT_ERR, ENABLE);
}
void TIM_PWMInit()
{
  //TIM_DeInit(TIM2);
  TIM_DeInit(TIM3);
  TIM_DeInit(TIM4);
  TIM_DeInit(TIM5);
  
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);//Ch. 1
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,  GPIO_AF_TIM2);//Ch. 2
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);//Ch. 3
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);//Ch. 4
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,  GPIO_AF_TIM3);//Ch. 1
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,  GPIO_AF_TIM3);//Ch. 2
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,  GPIO_AF_TIM3);//Ch. 3
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9,  GPIO_AF_TIM3);//Ch. 4
  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);//Ch. 1
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);//Ch. 2
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);//Ch. 3
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);//Ch. 4

  GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5);//Ch. 1
  GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_TIM5);//Ch. 2
  GPIO_PinAFConfig(GPIOH, GPIO_PinSource12, GPIO_AF_TIM5);//Ch. 3
  //GPIO_PinAFConfig(GPIOI, GPIO_PinSource0,  GPIO_AF_TIM5);//Ch. 4
  
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//TIM2 Ch. 1
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//TIM2 Ch. 2, 3, 4
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//TIM3 Ch. 1, 2, 3, 4
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOD, &GPIO_InitStructure);//TIM4 Ch. 1, 2, 3, 4
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_Init(GPIOH, &GPIO_InitStructure);//TIM5 Ch. 1, 2, 3
  //GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  //GPIO_Init(GPIOI, &GPIO_InitStructure);//TIM5 Ch. 4

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 839;//âñåãäà +1
  TIM_TimeBaseInitStruct.TIM_Period    = 500;//200 Hz
  //TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);

  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCStructInit(&TIM_OCInitStruct);
  TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse       = 0;
  //TIM_OC1Init(TIM2, &TIM_OCInitStruct);
  //TIM_OC2Init(TIM2, &TIM_OCInitStruct);
  //TIM_OC3Init(TIM2, &TIM_OCInitStruct);
  //TIM_OC4Init(TIM2, &TIM_OCInitStruct);
  
  TIM_OC1Init(TIM3, &TIM_OCInitStruct);
  TIM_OC2Init(TIM3, &TIM_OCInitStruct);
  TIM_OC3Init(TIM3, &TIM_OCInitStruct);
  TIM_OC4Init(TIM3, &TIM_OCInitStruct);
  
  TIM_OC1Init(TIM4, &TIM_OCInitStruct);
  TIM_OC2Init(TIM4, &TIM_OCInitStruct);
  TIM_OC3Init(TIM4, &TIM_OCInitStruct);
  TIM_OC4Init(TIM4, &TIM_OCInitStruct);

  TIM_OC1Init(TIM5, &TIM_OCInitStruct);
  TIM_OC2Init(TIM5, &TIM_OCInitStruct);
  TIM_OC3Init(TIM5, &TIM_OCInitStruct);
  //TIM_OC4Init(TIM5, &TIM_OCInitStruct);
  
  //TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  TIM_Cmd(TIM5, ENABLE);
}

extern "C"
{
  /************************************************************************************************
  Обработчики прерывания DMA привязаны к T8_TRGO.
  ЗАПОЛНЕНИЕ ФИЛЬТРА СКОЛЬЗЯЩЕЙ МЕДИАНЫ БУДЕТ В ГЛАВНОМ ЦИКЛЕ ПРОГРАММЫ (main, с частотой 1 мс)
  ************************************************************************************************/
  void DMA2_Stream0_IRQHandler()
  {
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
      DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
  }
  void DMA2_Stream1_IRQHandler()
  {
    if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1))
      DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
  }
  /************************************************************************************************
  Прерывание по TIM7 - 1 мс. Ведем отсчет времени (системные часы).
  ************************************************************************************************/
  void TIM7_IRQHandler()
  {
    if(TIM_GetITStatus(TIM7, TIM_IT_Update))
    {
      TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
      ++time_ms;

      time_flag[0] = true;
      if(!(time_ms % 10))
        time_flag[1] = true;
      if(!(time_ms % 25))
        time_flag[2] = true;
      if(!(time_ms % 100))
        time_flag[3] = true;
    }
  }
  void TIM2_IRQHandler()
  {
    if(TIM_GetITStatus(TIM2, TIM_IT_Update))
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
  /************************************************************************************************
  Принимает по CAN дискретные сигналы с ОУ - ЗАДЕЛ НА ДИСТАНЦИОННОЕ УПРАВЛЕНИЕ.
  Обеспечение взаимодействия по протоколу (калибровка и настройка контроллера с помощью ПО на ПК).
  Обработка цифровых датчиков и EEC1.
  ПО ВОЗМОЖНОСТИ ПЕРЕНЕСТИ ВСЮ ЛОГИКУ В ОБЩИЙ ЦИКЛ ПРОГРАММЫ (main).
  ************************************************************************************************/
  void CAN1_RX0_IRQHandler()
  {
    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0))
    {
      CanRxMsg RxMsg;
      CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
      CAN_Receive(CAN1, CAN_FIFO0, &RxMsg);
      CAN_FIFORelease(CAN1, CAN_FIFO0);
      
      if(RxMsg.IDE == CAN_ID_STD)
        switch(RxMsg.StdId)
        {
          //case 0x005:
          //  kpp.DigitalSet(RxMsg.Data[4] << 8 | RxMsg.Data[0], cal);
          //  end_transmit = time_ms;                                                        break;
          case 0x010: cal.CtrlAndRPM(RxMsg.Data[0],RxMsg.Data[2]<<8|RxMsg.Data[1]);          break;
          case 0x100: cal.OtLeftTime(RxMsg);                                                 break;
          case 0x101: cal.OtLeftPres(RxMsg);                                                 break;
          case 0x102: cal.OtRightTime(RxMsg);                                                break;
          case 0x103: cal.OtRightPres(RxMsg);                                                break;
          case 0x104: cal.BfLeftTime(RxMsg);                                                 break;
          case 0x105: cal.BfLeftPres(RxMsg);                                                 break;
          case 0x106: cal.BfRightTime(RxMsg);                                                break;
          case 0x107: cal.BfRightPres(RxMsg);                                                break;
          case 0x108: cal.ForwardTime(RxMsg);                                                break;
          case 0x109: cal.ForwardPres(RxMsg);                                                break;
          case 0x10A: cal.ReverseTime(RxMsg);                                                break;
          case 0x10B: cal.ReversePres(RxMsg);                                                break;
          case 0x10C: cal.OneTime(RxMsg);                                                    break;
          case 0x10D: cal.OnePres(RxMsg);                                                    break;
          case 0x10E: cal.TwoTime(RxMsg);                                                    break;
          case 0x10F: cal.TwoPres(RxMsg);                                                    break;
          case 0x110: cal.ThreeTime(RxMsg);                                                  break;
          case 0x111: cal.ThreePres(RxMsg);                                                  break;
          case 0x112: cal.Save();                                                            break;
          case 0x113: cal.SendData();                                                        break;
          case 0x114: cal.SendDataValve();                                                   break;
          case 0x120: state = static_cast<Calibrate::State>(RxMsg.Data[0]);                  break;
          case 0x181:
            pres.i = RxMsg.Data[3] << 24 | RxMsg.Data[2] << 16 | RxMsg.Data[1] << 8 |RxMsg.Data[0];
            if(pres.f < 0)
              pres.f = 0;
            break;
        }
      else
        switch(RxMsg.ExtId)
        {
          case 0x0CF00400: kpp.SetRpm(RxMsg.Data[4] << 8 | RxMsg.Data[3]); break;
        }
    }
  }
  void CAN1_RX1_IRQHandler()
  {
    if(CAN_GetITStatus(CAN1, CAN_IT_FMP1))
    {
      CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);
      CAN_FIFORelease(CAN1, CAN_FIFO1);
    }
  }
  void CAN1_TX_IRQHandler()
  {
    if(CAN_GetITStatus(CAN1, CAN_IT_TME))
      CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
  }
  void CAN1_SCE_IRQHandler()
  {
    if(CAN_GetITStatus(CAN1, CAN_IT_ERR))
      CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
    if(CAN_GetITStatus(CAN1, CAN_IT_LEC))
      CAN_ClearITPendingBit(CAN1, CAN_IT_LEC);
    if(CAN_GetITStatus(CAN1, CAN_IT_EPV))
      CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);
    if(CAN_GetITStatus(CAN1, CAN_IT_EWG))
      CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);
    if(CAN_GetITStatus(CAN1, CAN_IT_BOF))
      CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
  }
}

bool CanTxMailBoxEmpty(CAN_TypeDef* CANx)
{
  if((CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0 || (CANx->TSR & CAN_TSR_TME1) == CAN_TSR_TME1 ||
     (CANx->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
    return true;
  else
    return false;
}