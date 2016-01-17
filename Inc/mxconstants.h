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
#define usart2TX_Pin GPIO_PIN_2
#define usart2TX_GPIO_Port GPIOA
#define usart2RX_Pin GPIO_PIN_3
#define usart2RX_GPIO_Port GPIOA
#define usbPU_Pin GPIO_PIN_0
#define usbPU_GPIO_Port GPIOB
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
#define nIORQ_Pin GPIO_PIN_8
#define nIORQ_GPIO_Port GPIOA
#define nDOS_Pin GPIO_PIN_9
#define nDOS_GPIO_Port GPIOA
#define nOIORQ_Pin GPIO_PIN_15
#define nOIORQ_GPIO_Port GPIOA
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
#define nDOS (nDOS_GPIO_Port->IDR & nDOS_Pin)
#define nRD  (nRD_GPIO_Port->IDR & nRD_Pin)
#define nWR  (nWR_GPIO_Port->IDR & nWR_Pin)
#define ledDriveA_On  (ledDriveA_GPIO_Port->BRR  = ledDriveA_Pin)
#define ledDriveA_Off (ledDriveA_GPIO_Port->BSRR = ledDriveA_Pin)
#define ledDriveB_On  (ledDriveB_GPIO_Port->BRR  = ledDriveB_Pin)
#define ledDriveB_Off (ledDriveB_GPIO_Port->BSRR = ledDriveB_Pin)
#define A0 (A0_GPIO_Port->IDR & A0_Pin)
#define A5 (A5_GPIO_Port->IDR & A5_Pin)
#define A6 (A6_GPIO_Port->IDR & A6_Pin)
#define A7 (A7_GPIO_Port->IDR & A7_Pin)
#define fdcPort (A5_GPIO_Port->IDR & 0x38)

typedef enum
{
  S_IDLE = 0,
  S_WAIT,

  S_DELAY_BEFORE_CMD,
  S_CMD_RW,
  S_FOUND_NEXT_ID,
  S_READ,
  S_WRSEC,
  S_WRITE,
  S_WRTRACK,
  S_WR_TRACK_DATA,

  S_TYPE1_CMD,
  S_STEP,
  S_SEEKSTART,
  S_SEEK,
  S_VERIFY,

  S_RESET
} enWDState;

typedef enum
{
  NONE = 0x00,
  DRQ = 0x40,
  INTRQ = 0x80,
} enWDDataStatus;

typedef enum
{
  WDS_BUSY = 0x01,
  WDS_INDEX = 0x02,
  WDS_DRQ = 0x02,
  WDS_TRK00 = 0x04,
  WDS_LOST = 0x04,
  WDS_CRCERR = 0x08,
  WDS_NOTFOUND = 0x10,
  WDS_SEEKERR = 0x10,
  WDS_RECORDT = 0x20,
  WDS_HEADL = 0x20,
  WDS_WRFAULT = 0x20,
  WDS_WRITEP = 0x40,
  WDS_NOTRDY = 0x80
} enWDStatus;

#define dataGPIOPort D0_GPIO_Port->IDR
#define dataBus (dataGPIOPort & 0xFF00) >> 8

//#define TRACE
#define	MAX_TRACK_LEN	12800	// bytes

#define CYL_NUM_MAX	79

#define HXCMFM_NOT	80
#define HXCMFM_SD	2
#define HXCMFM_RPM	0x0000
#define HXCMFM_BR	0x00FA
#define HXCMFM_FT	7

#define INDEX_START	0
#define INDEX_STOP	INDEX_START + 64 * 2

#define MAX_PATH_LEN	64

#define FILE_NAME_LEN	14
// 1 символ{d,f} + длина{8.3} + 1 завершающий 0 = 14

#pragma pack(1)

typedef struct
{
	unsigned char headername[7];
	unsigned short trackNumber;
	unsigned char sideNumber;
	unsigned short floppyRPM;
	unsigned short floppyBitRate;
	unsigned char floppyiftype;
	unsigned long mfmTrackListOffset;
} MFMIMG;	// 19 байт

typedef struct
{
	unsigned short track_number;
	unsigned char side_number;
	unsigned long mfmTrackSize;
	unsigned long mfmtrackoffset;
} MFMTRACKIMG;	// 11 байт * number_of_track * number_of_side

#pragma pack()

typedef enum { btnUp, btnEnter, btnDown, btnUSBMode } btns;
typedef enum { init, loadCylinder, idle, readTrack, writeTrack, selectFile, selectDir, usbMode, copier, noneState } machineState;

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
