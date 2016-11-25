/*;*******************************************************************************
; FILENAME		: LightPwmDrv.h
; AUTHOR		: Feng Liu
; PURPOSE		: PWM-based Lighting Driver
; REVISION		: 1.0
; SYSTEM CLOCK	        : 16MHz
; MCU type              : STM8L051F3            
; Date Time             : 2016-04-18
; Copy right :          : Eastfield Lighting Co., Ltd.
;*******************************************************************************/

#ifndef __LIGHT_PWM_DRIVER_
#define __LIGHT_PWM_DRIVER_

/* Includes ------------------------------------------------------------------*/
#include "_global.h"

/* Exported variables ------------------------------------------------------- */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#define MODE_CC1_EXTERNAL_PWM_DUTY      
void callbackTim1EventCC (void);
void callbackTim3EventCC (void);

void initExternalPwmCaptureFunction (void);
void initTim2PWMFunction (void);

void regulateWarmLightPulseWidth (unsigned char ucPercent);
void regulateColdLightPulseWidth (unsigned char ucPercent);

void configureTIM1PwmCapture (TIM1_Channel_TypeDef TIM1_Channel);
void driveColdWarmLightPwm (unsigned char ucCold, unsigned char ucWarm);

void captureExtPwmParamRecognition (void);


void initTim3PWMFunction(void);
void driveRGBWLightPwm(uint8_t  Rvalue, uint8_t Gvalue, uint8_t Bvalue, uint8_t Wvalue);
void driveRGBLightPwm(uint8_t  Rvalue, uint8_t Gvalue, uint8_t Bvalue);
void driveRGBWBRLightPwm(uint8_t  Rvalue, uint8_t Gvalue, uint8_t Bvalue, uint8_t Wvalue ,uint8_t BRPercent);
void driveRGBBRLightPwm(uint8_t  Rvalue, uint8_t Gvalue, uint8_t Bvalue,uint8_t BRPercent);


#endif
/******************* (C) COPYRIGHT 2016 Eastfield Lighting *****END OF FILE****/