/*;*******************************************************************************
; FILENAME		: LightPwmDrv.c
; PURPOSE		: PWM-based Lighting Driver
; REVISION		: 1.0
; SYSTEM CLOCK	        : 16MHz
; MCU type              : STM8S105K4
;*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "LightPwmDrv.h"
#include "_global.h"
#include "stm8s.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



// Channel 1
#define R_LIGHT_PWM_PIN_PORT         GPIOD
#define R_LIGHT_PWM_PIN_ID           GPIO_PIN_3

// Channel 2
#define G_LIGHT_PWM_PIN_PORT         GPIOD
#define G_LIGHT_PWM_PIN_ID           GPIO_PIN_4

// Channel 1
#define B_LIGHT_PWM_PIN_PORT         GPIOD
#define B_LIGHT_PWM_PIN_ID           GPIO_PIN_2

// Channel 2
#define W_LIGHT_PWM_PIN_PORT         GPIOD
#define W_LIGHT_PWM_PIN_ID           GPIO_PIN_0




#define TIM2_PWM_PERIOD         254
#define TIM2_PWM_PULSE          200
#define TIM3_PWM_PERIOD         254
#define TIM3_PWM_PULSE          200



/**
  * @brief  Configure peripherals GPIO for WARM/COLD PWM   
  * @param  None
  * @retval None
  */
static void TIM2WarmColdLightPwm_ClkGpioConfig(void)
{
  /* Enable TIM2 clock */
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);

  /* TIM2 Channel1 configuration: PB2 */
  GPIO_Init(G_LIGHT_PWM_PIN_PORT, G_LIGHT_PWM_PIN_ID, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(R_LIGHT_PWM_PIN_PORT, R_LIGHT_PWM_PIN_ID, GPIO_MODE_OUT_PP_LOW_FAST);
}

/**
  * @brief  Configure TIM2 peripheral   
  * @param  None
  * @retval None
  */
static void TIM2PWMFunction_Config(void)
{
   TIM2_DeInit();
  /* TIM2 configuration:
     - TIM2 counter is clocked by HSI div 8 2M
      so the TIM2 counter clock used is HSI / 1 = 16M / 1 = 16 MHz
    TIM2 Channel1 output frequency = TIM2CLK / (TIM2 Prescaler * (TIM2_PERIOD + 1))
    = 16M / (8 * 200) = 10K Hz //TIM2_Prescaler_8; TIM2_PWM_PERIOD = 199
  */
  /* Time Base configuration */
  TIM2_TimeBaseInit(TIM2_PRESCALER_8, TIM2_PWM_PERIOD);
  //TIM2_ETRClockMode2Config(TIM2_ExtTRGPSC_DIV4, TIM2_ExtTRGPolarity_NonInverted, 0);

  /* Channel 1 configuration in PWM1 mode */
  /* TIM2 channel Duty cycle is 100 * (TIM2_PERIOD + 1 - TIM2_PULSE) / (TIM2_PERIOD + 1) = 100 * 4/8 = 50 % */
  TIM2_OC1Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, TIM2_PWM_PULSE, TIM2_OCPOLARITY_HIGH);
  TIM2_OC1PreloadConfig(ENABLE);

  /* Channel 2 configuration in PWM1 mode */
  /* TIM2 channel Duty cycle is 100 * (TIM2_PERIOD + 1 - TIM2_PULSE) / (TIM2_PWM_PERIOD + 1) = 100 * 4/8 = 50 % */
  TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, TIM2_PWM_PULSE, TIM2_OCPOLARITY_HIGH);
  TIM2_OC2PreloadConfig(ENABLE);

  /* TIM2 Main Output Enable */
  //TIM2_ARRPreloadConfig(ENABLE);

  /* TIM2 counter enable */
  TIM2_Cmd(ENABLE);
}


static void TIM3WarmColdLightPwm_ClkGpioConfig(void)
{
  /* Enable TIM2 clock */
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER3, ENABLE);

  /* TIM2 Channel1 configuration: PB2 */
  GPIO_Init(B_LIGHT_PWM_PIN_PORT, B_LIGHT_PWM_PIN_ID, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(W_LIGHT_PWM_PIN_PORT, W_LIGHT_PWM_PIN_ID, GPIO_MODE_OUT_PP_LOW_FAST);
}

/**
  * @brief  Configure TIM2 peripheral   
  * @param  None
  * @retval None
  */
static void TIM3PWMFunction_Config(void)
{
   TIM3_DeInit();
  /* TIM2 configuration:
     - TIM2 counter is clocked by HSI div 8 2M
      so the TIM2 counter clock used is HSI / 1 = 16M / 1 = 16 MHz
    TIM2 Channel1 output frequency = TIM2CLK / (TIM2 Prescaler * (TIM2_PERIOD + 1))
    = 16M / (8 * 200) = 10K Hz //TIM2_Prescaler_8; TIM2_PWM_PERIOD = 199
  */
  /* Time Base configuration */
  TIM3_TimeBaseInit(TIM3_PRESCALER_8, TIM3_PWM_PERIOD);
  //TIM2_ETRClockMode2Config(TIM2_ExtTRGPSC_DIV4, TIM2_ExtTRGPolarity_NonInverted, 0);

  /* Channel 1 configuration in PWM1 mode */
  /* TIM2 channel Duty cycle is 100 * (TIM2_PERIOD + 1 - TIM2_PULSE) / (TIM2_PERIOD + 1) = 100 * 4/8 = 50 % */
  TIM3_OC1Init(TIM3_OCMODE_PWM1, TIM3_OUTPUTSTATE_ENABLE, TIM3_PWM_PULSE, TIM3_OCPOLARITY_HIGH);
  TIM3_OC1PreloadConfig(ENABLE);

  /* Channel 2 configuration in PWM1 mode */
  /* TIM2 channel Duty cycle is 100 * (TIM2_PERIOD + 1 - TIM2_PULSE) / (TIM2_PWM_PERIOD + 1) = 100 * 4/8 = 50 % */
  TIM3_OC2Init(TIM3_OCMODE_PWM1, TIM3_OUTPUTSTATE_ENABLE, TIM3_PWM_PULSE, TIM3_OCPOLARITY_HIGH);
  TIM3_OC2PreloadConfig(ENABLE);

  /* TIM2 Main Output Enable */
  //TIM3_ARRPreloadConfig(ENABLE);

  /* TIM2 counter enable */
  TIM3_Cmd(ENABLE);
}

void initTim3PWMFunction (void)
{
  TIM3WarmColdLightPwm_ClkGpioConfig ();
  TIM3PWMFunction_Config ();
}

void initTim2PWMFunction (void)
{
  TIM2WarmColdLightPwm_ClkGpioConfig ();
  TIM2PWMFunction_Config ();
}

void regulateColdLightPulseWidth (unsigned char ucPercent)
{
#if 0
  // uint16_t pulseWidth;
  if (ucPercent >= 100)
    ucPercent = 100;
  //pulseWidth = 100 * (TIM2_PWM_PERIOD + 1) / ucPercent;
    
  //TIM2_CtrlPWMOutputs(DISABLE);
  //TIM2_Cmd(DISABLE);
  TIM2_OC1Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, ucPercent * 255/100, TIM2_OCPOLARITY_HIGH);
  //TIM2_OC1PreloadConfig(ENABLE);
  //TIM2_CtrlPWMOutputs(ENABLE);
  //TIM2_Cmd(ENABLE);
#endif
}

void regulateWarmLightPulseWidth (unsigned char ucPercent)
{
#if 0
  //uint16_t pulseWidth;
  if (ucPercent >= 100)
    ucPercent = 100;
  
  //pulseWidth = ucPercent * 2;
  //pulseWidth = 100 * (TIM2_PWM_PERIOD + 1) / pulseWidth;
  
  
  //TIM2_CtrlPWMOutputs(DISABLE);
  //TIM2_Cmd(DISABLE);
  TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, ucPercent * 255/100, TIM2_OCPOLARITY_HIGH);  
  //TIM2_OC2PreloadConfig(ENABLE);
  //TIM2_CtrlPWMOutputs(ENABLE);
  //TIM2_Cmd(ENABLE);
#endif
}

void driveColdWarmLightPwm (unsigned char ucCold, unsigned char ucWarm)
{
#if 0
  //Usart1_SendByte ((u8)ucCold);
  //Usart1_SendByte ((u8)ucWarm);
  regulateColdLightPulseWidth (ucCold );
  regulateWarmLightPulseWidth (ucWarm );
  //Usart1_SendByte (ucCold * 100 / 255);
  //Usart1_SendByte (ucWarm * 100 / 255);
#endif
}

void driveRGBWLightPwm(uint8_t  Rvalue, uint8_t Gvalue, uint8_t Bvalue, uint8_t Wvalue)
{
  TIM2_SetCompare1(Rvalue);
  TIM2_SetCompare2(Gvalue);
  TIM3_SetCompare1(Bvalue);
  TIM3_SetCompare2(Wvalue);

}

void driveRGBLightPwm(uint8_t  Rvalue, uint8_t Gvalue, uint8_t Bvalue)
{
  TIM2_SetCompare1(Rvalue);
  TIM2_SetCompare2(Gvalue);
  TIM3_SetCompare1(Bvalue);
  
}

void driveRGBWBRLightPwm(uint8_t  Rvalue, uint8_t Gvalue, uint8_t Bvalue, uint8_t Wvalue ,uint8_t BRPercent)
{

  if(BRPercent>100)
    BRPercent=100;
  Rvalue = Rvalue * BRPercent/100;
  Gvalue = Gvalue * BRPercent/100;
  Bvalue = Bvalue * BRPercent/100;
  Wvalue = Wvalue * BRPercent/100;
  
  TIM2_SetCompare1(Rvalue);
  TIM2_SetCompare2(Gvalue);
  TIM3_SetCompare1(Bvalue);
  TIM3_SetCompare2(Wvalue);

}

void driveRGBBRLightPwm(uint8_t  Rvalue, uint8_t Gvalue, uint8_t Bvalue,uint8_t BRPercent)
{
  if(BRPercent>100)
    BRPercent=100;
  Rvalue = Rvalue * BRPercent/100;
  Gvalue = Gvalue * BRPercent/100;
  Bvalue = Bvalue * BRPercent/100;
  
  
  TIM2_SetCompare1(Rvalue);
  TIM2_SetCompare2(Gvalue);
  TIM3_SetCompare1(Bvalue);
  
  
}


/******************* (C) COPYRIGHT 2016 Eastfield Lighting *****END OF FILE****/