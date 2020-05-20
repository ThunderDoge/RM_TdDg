/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attentionaBuild started: Project: SentiCloud0
*** Using Compiler 'V5.06 update 6 (build 750)', folder: 'D:\Program Files\Keil_MDK5\ARM\ARMCC\Bin'
Build Project 'SentiCloud0' - Target 'SentiCloud0'
compiling app_vision.cpp...
compiling stm32f4xx_it.c...
compiling SentryCloudCommu.cpp...
compiling task_SentiCloud.cpp...
linking...
Program Size: Code=50448 RO-data=1592 RW-data=684 ZI-data=50372  
"SentiCloud0\SentiCloud0.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:09
Load "SentiCloud0\\SentiCloud0.axf" 
Set JLink Project File to "D:\RM_TdDg_PrivLib\Sentry_\SentiCloud0\MDK-ARM\JLinkSettings.ini"
* JLink Info: Device "STM32F405RG" selected.
 
JLink info:
------------
DLL: V6.46 , compiled May 23 2019 17:49:56
Firmware: J-Link V9 compiled May 17 2019 09:50:41
Hardware: V9.60
S/N : 69611915 
Feature(s) : RDI, GDB, FlashDL, FlashBP, JFlash 
 
* JLink Info: Found SW-DP with ID 0x2BA01477
* JLink Info: Found SW-DP with ID 0x2BA01477
* JLink Info: Scanning AP map to find all available APs
* JLink Info: AP[1]: Stopped AP scan as end of AP map has been reached
* JLink Info: AP[0]: AHB-AP (IDR: 0x24770011)
* JLink Info: Iterating through AP map to find AHB-AP to use
* JLink Info: AP[0]: Core found
* JLink Info: AP[0]: AHB-AP ROM base: 0xE00FF000
* JLink Info: CPUID register: 0x410FC241. Implementer code: 0x41 (ARM)
* JLink Info: Found Cortex-M4 r0p1, Little endian.
* JLink Info: FPUnit: 6 code (BP) slots and 2 literal slots
* JLink Info: CoreSight components:
* JLink Info: ROMTbl[0] @ E00FF000
* JLink Info: ROMTbl[0][0]: E000E000, CID: B105E00D, PID: 000BB00C SCS-M7
* JLink Info: ROMTbl[0][1]: E0001000, CID: B105E00D, PID: 003BB002 DWT
* JLink Info: ROMTbl[0][2]: E0002000, CID: B105E00D, PID: 002BB003 FPB
* JLink Info: ROMTbl[0][3]: E0000000, CID: B105E00D, PID: 003BB001 ITM
* JLink Info: ROMTbl[0][4]: E0040000, CID: B105900D, PID: 000BB9A1 TPIU
* JLink Info: ROMTbl[0][5]: E0041000, CID: B105900D, PID: 000BB925 ETM
ROMTableAddr = 0xE00FF000
* JLink Info: Reset: Halt core after reset via DEMCR.VC_CORERESET.
* JLink Info: Reset: Reset device via AIRCR.SYSRESETREQ.
 
Target info:
------------
Device: STM32F405RGTx
VTarget = 3.254V
State of Pins: 
TCK: 0, TDI: 0, TDO: 0, TMS: 1, TRES: 1, TRST: 0
Hardware-Breakpoints: 6
Software-Breakpoints: 8192
Watchpoints:          4
JTAG speed: 1000 kHz
 
Erase Done.Build started: Project: SentiCloud0
*** Using Compiler 'V5.06 update 6 (build 750)', folder: 'D:\Program Files\Keil_MDK5\ARM\ARMCC\Bin'
Build Project 'SentiCloud0' - Target 'SentiCloud0'
compiling app_vision.cpp...
compiling stm32f4xx_it.c...
compiling SentryCloudCommu.cpp...
compiling task_SentiCloud.cpp...
linking...
Program Size: Code=50448 RO-data=1592 RW-data=684 ZI-data=50372  
"SentiCloud0\SentiCloud0.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed:  00:00:09
Load "SentiCloud0\\SentiCloud0.axf" 
Set JLink Project File to "D:\RM_TdDg_PrivLib\Sentry_\SentiCloud0\MDK-ARM\JLinkSettings.ini"
* JLink Info: Device "STM32F405RG" selected.
 
JLink info:
------------
DLL: V6.46 , compiled May 23 2019 17:49:56
Firmware: J-Link V9 compiled May 17 2019 09:50:41
Hardware: V9.60
S/N : 69611915 
Feature(s) : RDI, GDB, FlashDL, FlashBP, JFlash 
 
* JLink Info: Found SW-DP with ID 0x2BA01477
* JLink Info: Found SW-DP with ID 0x2BA01477
* JLink Info: Scanning AP map to find all available APs
* JLink Info: AP[1]: Stopped AP scan as end of AP map has been reached
* JLink Info: AP[0]: AHB-AP (IDR: 0x24770011)
* JLink Info: Iterating through AP map to find AHB-AP to use
* JLink Info: AP[0]: Core found
* JLink Info: AP[0]: AHB-AP ROM base: 0xE00FF000
* JLink Info: CPUID register: 0x410FC241. Implementer code: 0x41 (ARM)
* JLink Info: Found Cortex-M4 r0p1, Little endian.
* JLink Info: FPUnit: 6 code (BP) slots and 2 literal slots
* JLink Info: CoreSight components:
* JLink Info: ROMTbl[0] @ E00FF000
* JLink Info: ROMTbl[0][0]: E000E000, CID: B105E00D, PID: 000BB00C SCS-M7
* JLink Info: ROMTbl[0][1]: E0001000, CID: B105E00D, PID: 003BB002 DWT
* JLink Info: ROMTbl[0][2]: E0002000, CID: B105E00D, PID: 002BB003 FPB
* JLink Info: ROMTbl[0][3]: E0000000, CID: B105E00D, PID: 003BB001 ITM
* JLink Info: ROMTbl[0][4]: E0040000, CID: B105900D, PID: 000BB9A1 TPIU
* JLink Info: ROMTbl[0][5]: E0041000, CID: B105900D, PID: 000BB925 ETM
ROMTableAddr = 0xE00FF000
* JLink Info: Reset: Halt core after reset via DEMCR.VC_CORERESET.
* JLink Info: Reset: Reset device via AIRCR.SYSRESETREQ.
 
Target info:
------------
Device: STM32F405RGTx
VTarget = 3.254V
State of Pins: 
TCK: 0, TDI: 0, TDO: 0, TMS: 1, TRES: 1, TRST: 0
Hardware-Breakpoints: 6
Software-Breakpoints: 8192
Watchpoints:          4
JTAG speed: 1000 kHz
 
Erase Done.
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	 
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void SPI1_IRQHandler(void);
void USART1_IRQHandler(void);
void USART3_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);
void UART5_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
