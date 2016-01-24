/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ledDriveA_Pin GPIO_PIN_13
#define ledDriveA_GPIO_Port GPIOC
#define ledDriveB_Pin GPIO_PIN_14
#define ledDriveB_GPIO_Port GPIOC
#define ledStep_Pin GPIO_PIN_15
#define ledStep_GPIO_Port GPIOC
#define btnUp_Pin GPIO_PIN_0
#define btnUp_GPIO_Port GPIOC
#define btnDown_Pin GPIO_PIN_1
#define btnDown_GPIO_Port GPIOC
#define btnEnter_Pin GPIO_PIN_2
#define btnEnter_GPIO_Port GPIOC
#define btnUSBMode_Pin GPIO_PIN_3
#define btnUSBMode_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_2
#define A0_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_10
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_11
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_15
#define D7_GPIO_Port GPIOB
#define nWR_Pin GPIO_PIN_6
#define nWR_GPIO_Port GPIOC
#define nRD_Pin GPIO_PIN_7
#define nRD_GPIO_Port GPIOC
#define sdioD0_Pin GPIO_PIN_8
#define sdioD0_GPIO_Port GPIOC
#define nIORQnDOS_Pin GPIO_PIN_8
#define nIORQnDOS_GPIO_Port GPIOA
#define nRunZ80_Pin GPIO_PIN_15
#define nRunZ80_GPIO_Port GPIOA
#define sdioCK_Pin GPIO_PIN_12
#define sdioCK_GPIO_Port GPIOC
#define sdioCMD_Pin GPIO_PIN_2
#define sdioCMD_GPIO_Port GPIOD
#define A5_Pin GPIO_PIN_3
#define A5_GPIO_Port GPIOB
#define A6_Pin GPIO_PIN_4
#define A6_GPIO_Port GPIOB
#define A7_Pin GPIO_PIN_5
#define A7_GPIO_Port GPIOB
#define i2cSCL_Pin GPIO_PIN_6
#define i2cSCL_GPIO_Port GPIOB
#define i2cSDA_Pin GPIO_PIN_7
#define i2cSDA_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_8
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_9
#define D1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define nIORQnDOS (nIORQnDOS_GPIO_Port->IDR & nIORQnDOS_Pin)
#define nRD  (nRD_GPIO_Port->IDR & nRD_Pin)
#define nWR  (nWR_GPIO_Port->IDR & nWR_Pin)

#define ledDriveA_On  (ledDriveA_GPIO_Port->BRR  = ledDriveA_Pin)
#define ledDriveA_Off (ledDriveA_GPIO_Port->BSRR = ledDriveA_Pin)
#define ledDriveB_On  (ledDriveB_GPIO_Port->BRR  = ledDriveB_Pin)
#define ledDriveB_Off (ledDriveB_GPIO_Port->BSRR = ledDriveB_Pin)

#define ledStep_On  (ledStep_GPIO_Port->BRR  = ledStep_Pin)
#define ledStep_Off (ledStep_GPIO_Port->BSRR = ledStep_Pin)

#define A0 (A0_GPIO_Port->IDR & A0_Pin)
#define A5 (A5_GPIO_Port->IDR & A5_Pin)
#define A6 (A6_GPIO_Port->IDR & A6_Pin)
#define A7 (A7_GPIO_Port->IDR & A7_Pin)
#define A765Mask (A7_Pin | A6_Pin | A5_Pin)

#define fdcAddressPortData
#define fdc_GPIO_Port A0_GPIO_Port

#define CPUDataBusPort D0_GPIO_Port


//#define TRACE
#define	MAX_TRACK_LEN	12800	// bytes

#define CYL_NUM_MAX	79

typedef enum { btnUp, btnEnter, btnDown, btnUSBMode } btns;
typedef enum { init, idle,  selectFile, selectDir, usbMode, noneState } machineState;

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
