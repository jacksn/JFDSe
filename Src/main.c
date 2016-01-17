/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32f1xx_hal.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ff.h"
#include "lcd1602.h"
#include <stdint.h>
#include <stdio.h>
//#include "stm32f1xx_hal_rcc.h"
//#include "stm32f1xx_hal_gpio.h"
//#include "stm32f103xe.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define __IO    volatile
#define VERSION "v0.01"

FATFS filesystem;		/* volume lable */
FRESULT ret;			/* Result code */
FIL file;				/* File object */
DIR dir;				/* Directory object */
FILINFO fno;			/* File information object */
char *fn;
UINT bw, br;

uint8_t buff[ MAX_TRACK_LEN * 2 ];
uint32_t track_lengs[ HXCMFM_NOT * HXCMFM_SD ];
uint32_t track_offsets[ HXCMFM_NOT * HXCMFM_SD ];
char Image_File_Name[ MAX_PATH_LEN ];
//                 0    1    2    3    4    5    6    7    8    9    10   11
char LCDbuff[13] = { 0x43,0x3A,0x30,0x20,0x20,0x53,0x3A,0x30,0x20,0x49,0x20,0x6D,0x00};
//                 C    :    0              S    :    0         IRW         mM
char CurrDir[ MAX_PATH_LEN ];
unsigned char TmpBuffer[32];
volatile long int CurrBit;
volatile uint32_t CylNumber;
volatile uint32_t LoadedCylNumber;
volatile long int CurrBitW, CurrBitWStart;
volatile long int SideW;
volatile machineState currentState = init;
volatile machineState lastState = noneState;

//
MFMIMG *mfmimg;
MFMTRACKIMG *mfmTrackImage;
uint16_t mfmNumberOfTracks;				// общее кол-во треков в файле ( с учетом сторон )
volatile uint16_t mfmNumberOfCylinders;		// общее кол-во цилиндров в файле
//
long int trackListBase;
uint32_t temp_l1, temp_l2;
long int i;
char * x;
btns pressedButton;
long int allFiles;
long int FileIndex;
//__IO bool DevWasConfigured = false;
uint32_t trackNumber;


uint8_t fdcCommandReg; 	// 0x1F
uint8_t fdcTrackReg;	// 0x3F
uint8_t fdcSectorReg;	// 0x5F
uint8_t fdcDataReg;		// 0x7F
uint8_t fdcSystemReg;	// 0xFF

volatile enWDState WDState;
volatile enWDDataStatus WDDataStatus;
volatile enWDStatus WDStatus;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void GPIO_Config(void);
void USART2_Config(void);
void SendChar2USART(char c);
btns readKeyboard(void);
long int readHXCMFMFile(void);
long int readCylinder(long int);
long int saveTrack(long int);
void saveCopiedTrack( long int );
long int readDirectory(char * DirPath);
static void fault_err (FRESULT rc);
void Error2LCD(char * ErrorStr);
void Status2LCD (void);
static void _delay(__IO uint32_t nCount);
void Prep4USBFS(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	SystemInit();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	GPIO_Config();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if ( lastState != currentState )
		{
			lastState = currentState;
#ifdef TRACE
	printf( "St%d\n\r", currentState );
#endif
		}
		//WriteStats();
		switch ( currentState )
		{
			case init:
			{
				CurrBit = 0;
				lcdInit();
				lcdClear();
				lcdPrintStr("JFDSe ");
				lcdPrintStr(VERSION);
				HAL_Delay(100);
				// mount the filesystem
				if ( ( ret = f_mount( &filesystem, "/", 1 )) != FR_OK )
					fault_err(ret);
				LoadedCylNumber = 99;

				currentState = selectDir;

				break;
			}
			case selectDir:
			{
				currentState = selectFile;
				strcpy( CurrDir, "/" );
				break;
			}
			case loadCylinder:
			{
				while ( ( LoadedCylNumber = readCylinder( CylNumber ) ) != CylNumber )	;
#ifdef TRACE
	printf("LC=%d\n\r",LoadedCylNumber);
#endif
				if ( LoadedCylNumber == CylNumber )	currentState = idle;
				//
				break;
			}
			case idle:
			{
				if ( LoadedCylNumber != CylNumber )	currentState = loadCylinder;
				else
					if ( ( btnEnter_GPIO_Port->IDR & btnEnter_Pin ) == 0 )
					{
						currentState = selectFile;
					}
					else
						if ( ( btnUSBMode_GPIO_Port->IDR & btnUSBMode_Pin ) == 0 )
					{
						lcdSetCursor(0,0);
						lcdPrintStr("Real disk COPIER");
						lcdSetCursor(0,1);
						lcdPrintStr("Takes real FDD  ");
						_delay(75);

						currentState = copier;
					}
				break;
			}
			case readTrack:
			{
				if ( LoadedCylNumber != CylNumber )
				{
					currentState = loadCylinder;
				}
				break;
			}
			case selectFile:
			{
				allFiles = readDirectory( CurrDir );
				FileIndex = 0;	// index массива
				while ( currentState == selectFile )
				{
					// предполагается, что файлов как минимум, 1 (..) - директория вверх, выход
					lcdSetCursor(0,0);
					if ( allFiles == 1 )
						lcdPrintStr("Dir is empty   \x7E");
					else
						if ( FileIndex == 0 )
							lcdPrintStr("Select file  \xDA \x7E");
						else
							if ( FileIndex + 1 == allFiles )
								lcdPrintStr("Select file   \xD9\x7E");
							else
								lcdPrintStr("Select file  \xDA\xD9\x7E");
					lcdSetCursor(0,1);
					lcdPrintStr( (char *) &buff[ FILE_NAME_LEN * FileIndex ] );
					lcdPrintStr("       ");

					pressedButton = readKeyboard();

					switch ( pressedButton )
					{
						case btnUp:
						{
							if ( FileIndex )
							{
								FileIndex--;
								lcdSetCursor(0,1);
								lcdPrintStr( (char *) &buff[ FILE_NAME_LEN * FileIndex ] );
								lcdPrintStr("       ");
							}
							break;
						}
						case btnEnter:
						{
							if ( buff[ FILE_NAME_LEN * FileIndex ] == 32 )	// файл
								if ( strlen( CurrDir ) <= MAX_PATH_LEN - 1 - 12 )
								{
									strcpy( Image_File_Name, CurrDir );

									if ( *( Image_File_Name + strlen( Image_File_Name ) - 1 ) != '/' )
										strcat( Image_File_Name, "/" );

									strcat( Image_File_Name, (char const *)&buff[ FILE_NAME_LEN * FileIndex + 1 ] );

									if ( readHXCMFMFile() == 0 )
									{
										lcdSetCursor(0,0);
										x = Image_File_Name + strlen( Image_File_Name );
										for ( i = 1 ; i <= FILE_NAME_LEN && x > Image_File_Name; i++ )
											x--;
										lcdPrintStr( (char *)x );
										lcdPrintStr("       ");
										//lcdSetCursor(14,0);
										lcdSetCursor(0,1);
										lcdPrintStr( "C:   S:         " );
										Status2LCD();
										CylNumber = 0;
										currentState = loadCylinder;
									}
									else
										currentState = selectDir;
								}
								else
								{
									Error2LCD( "Path too long  \x7E" );
									break;
								}
							else	// директория
							{
								if ( !strcmp( (char const *)&buff[ FILE_NAME_LEN * FileIndex ], "*.." ) )	// вверх
								{
									x = CurrDir + strlen( CurrDir );
									while ( *x != '/' )	x--;
									*++x = '\0';
								}
								else	// вход
								{
									if ( *( CurrDir + strlen( CurrDir ) - 1 ) != '/' )	strcat( CurrDir, "/" );
									strcat( CurrDir, (char const *)&buff[ FILE_NAME_LEN * FileIndex + 1 ] );
								}
								allFiles = readDirectory( CurrDir );
								FileIndex = 0;	// index массива
							}
							break;
						}
						case btnDown:
						{
							if ( FileIndex + 1 < allFiles )
							{
								FileIndex++;
								lcdSetCursor(0,1);
								lcdPrintStr( (char *)&buff[ FILE_NAME_LEN * FileIndex ] );
								lcdPrintStr("       ");
							}
							break;
						}
						case btnUSBMode:
						{
							lcdSetCursor(0,0);
							lcdPrintStr("USB PC Link mode");
							lcdSetCursor(0,1);
							lcdPrintStr("Connect USB plug");
							_delay(75);
							currentState = usbMode;
							break;
						}
						default:
							break;
					}
				}
				break;
			}
			case writeTrack:
			{
#ifdef TRACE
				printf("\r\nStart=%d Length=%d, SideW=%d\n\r",temp_l2,temp_l1,SideW);
#endif
					saveTrack( LoadedCylNumber );
					//
					currentState = readTrack;
					//
					LCDbuff[9] = (uint8_t)('R');
					Status2LCD();
				break;
			}
			case copier:
			{
				// Вывод начального сообщения
				lcdSetCursor(0,0);
				lcdPrintStr("Insert disk B:  ");
				lcdSetCursor(0,1);
				lcdPrintStr("ENTER to start  ");
				while ( readKeyboard() != btnEnter )	;
				//
				lcdSetCursor(0,0);
				lcdPrintStr("Copying started");
				lcdSetCursor(0,1);
				lcdPrintStr("Wait for reset  ");
				// Проверка, свободна ли шина SHUGART
				i = 0;
#ifdef TRACE
	printf("Drv_Sel_1 active\n\r");
/*	printf("TR00=%d\n\r", pTR00->IDR & FE_TR00 );
	if ( ( pDIR->IDR & FE_DIR ) == 0 )
	{
		printf("\r\nDIR is low\n\r");
		while (1)	;
	}
*/
#endif
				_delay(20);
				//
				for ( trackNumber = 0; trackNumber < mfmNumberOfTracks; trackNumber++ )
				{
#ifdef TRACE
	printf("TrackNumber=%d\n\r", trackNumber);
#endif
					_delay(1);
					// Сохранение в таблице длины трека
					track_lengs[ trackNumber ] = ( CurrBitW + 1 ) >> 3;
					// Запись в файл трека
#ifdef TRACE
					printf("Length[%d] = %d\n\r", trackNumber, track_lengs[ trackNumber ] );
					for ( i = 0; i < 64; i++ )
						printf( "%X ", buff[i] );
					printf("\n\r");
#endif
				saveCopiedTrack( trackNumber );
					// Если трек нечетный и не последний, даем импульс STEP
				}
				NVIC_SystemReset();
				break;
			}
			case usbMode:
			{
				lcdSetCursor(0,0);
				lcdPrintStr("Auto reset after");
				lcdSetCursor(0,1);
				lcdPrintStr("PC disconnected ");
				/*Prep4USBFS();
				Set_System();
				Set_USBClock();
				USB_Interrupts_Config();
				USB_Init();
				while ( bDeviceState != CONFIGURED )	;
				DevWasConfigured = true;
				while ( 1 )	;*/
				break;
			}
			case noneState:
				break;
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
long int readCylinder( long int Cyl_Number )
{
	ret = f_open( &file, Image_File_Name, FA_READ );
	if (ret)	fault_err(ret);
	//
	ret = f_lseek( &file, track_offsets[ Cyl_Number << 1 ] );
	if (ret)	fault_err(ret);
	//
	ret = f_read( &file, buff, MAX_TRACK_LEN, &br );
	if (ret)	fault_err(ret);
	//
	ret = f_lseek( &file, track_offsets[ 1 + ( Cyl_Number << 1 ) ] );
	if (ret)	fault_err(ret);
	//
	ret = f_read( &file, buff + MAX_TRACK_LEN, MAX_TRACK_LEN, &br );
	if (ret)	fault_err(ret);
	//
	ret = f_close( &file );
	if (ret)	fault_err(ret);
	// Cylinder
	LCDbuff[2] = 0x30 + (uint8_t)( Cyl_Number / 10 );
	LCDbuff[3] = 0x30 + (uint8_t)( Cyl_Number % 10 );
	Status2LCD();
	//
	return Cyl_Number;
}

long int saveTrack( long int cylinderNumber )
{
	ret = f_open(&file, Image_File_Name, FA_WRITE);
	if (ret)	fault_err(ret);
	//
	ret = f_lseek(&file, track_offsets[ ( cylinderNumber << 1 ) + ( SideW > 0 ) ]);
	if (ret)	fault_err(ret);
	//
	ret = f_write(&file, buff + SideW, MAX_TRACK_LEN, &bw);

#ifdef TRACE
	printf("written=%d, ret=%d\n\r", bw, ret );
#endif

	if (ret)	fault_err(ret);
	//
/*	ret = f_lseek(&file, TrackListBase + sizeof(MFMTRACKIMG) * ( ( Cyl_Number << 1 ) + ( SideW > 0 ) ) + \
					(offsetof(MFMTRACKIMG, mfmtracksize) - offsetof(MFMTRACKIMG, track_number)) );
	if (ret)	fault_err(ret);

	ret = f_write(&file, (void *)&track_lengs[ ( LoadedCylNumber << 1 ) + ( SideW > 0 ) ] , 4, &bw);
*/	if (ret)	fault_err(ret);
	//
	ret = f_close(&file);
	if (ret)	fault_err(ret);
	//
	return cylinderNumber;
}

void saveCopiedTrack( long int Copied_Track_Number )
{
	ret = f_open(&file, Image_File_Name, FA_WRITE);
	if (ret)	fault_err(ret);
	//
	ret = f_lseek( &file, track_offsets[ Copied_Track_Number ] );
	if (ret)	fault_err(ret);
	//
	ret = f_write( &file, buff, MAX_TRACK_LEN, &bw );

#ifdef TRACE
	printf("writtenCT=%d, ret=%d\n\r", bw, ret );
#endif

	if (ret)	fault_err(ret);
	//
	ret = f_lseek(&file, trackListBase + sizeof(MFMTRACKIMG) * Copied_Track_Number + \
					(offsetof(MFMTRACKIMG, mfmTrackSize) - offsetof(MFMTRACKIMG, track_number)) );
	if (ret)	fault_err(ret);
	//
	ret = f_write( &file, (void *)&track_lengs[ Copied_Track_Number ], 4, &bw );
	if (ret)	fault_err(ret);
	//
#ifdef TRACE
	printf("writtenCTL=%d, ret=%d\n\r", bw, ret );
#endif
	//
	ret = f_close(&file);
	if (ret)	fault_err(ret);
}

long int readHXCMFMFile(void)
{
	ret = f_stat( Image_File_Name, &fno );
	if (ret)	return -1L;
	// Если файл Read-Only, то ставим WP=0
/*	if ( fWP == (fno.fattrib & AM_RDO) )	// fWP=1 -> WP=0
		pWP->BRR = FE_WP;	// WP=0
	else
		pWP->BSRR = FE_WP;	// WP=1
*/
	ret = f_open(&file, Image_File_Name, FA_READ);
	if (ret)	fault_err(ret);
	//
	ret = f_read(&file, buff, 0x800, &br);	// 0x800 - заголовки + таблица смещений треков
	if (ret)	fault_err(ret);
	//
	ret = f_close(&file);
	if (ret)	fault_err(ret);
	//
	mfmimg = (MFMIMG *)buff;
	//
	if ( strcmp( "HXCMFM", (char const *)mfmimg->headername ) )
	{
		Error2LCD("Wrong header");
		return -1;
	}
	if ( mfmimg->trackNumber > HXCMFM_NOT )
	{
		Error2LCD("Tracks > 80");
		return -1;
	}
	if ( mfmimg->sideNumber > HXCMFM_SD || mfmimg->sideNumber < 1 )
	{
		Error2LCD("Sides 2 or 1");
		return -1;
	}
	if ( mfmimg->floppyRPM != HXCMFM_RPM )
	{
		Error2LCD("Not 300 RPM");
		return -1;
	}
	if ( mfmimg->floppyBitRate != HXCMFM_BR )
	{
		Error2LCD("Not 250 kbs");
		return -1;
	}
	if ( mfmimg->floppyiftype != HXCMFM_FT )
	{
		Error2LCD("Not TR-DOS");
		return -1;
	}
	//
	mfmNumberOfTracks = mfmimg->trackNumber * mfmimg->sideNumber;
	mfmNumberOfCylinders = mfmimg->trackNumber;
	//
	mfmTrackImage = (MFMTRACKIMG *)(buff + mfmimg->mfmTrackListOffset);
	trackListBase = mfmimg->mfmTrackListOffset;
	//
	for ( i = 0; i < mfmNumberOfTracks; i++ )
	{
		track_lengs[i] = mfmTrackImage->mfmTrackSize;
		track_offsets[i] = mfmTrackImage->mfmtrackoffset;
		mfmTrackImage++;
	}
	//
	return 0;
}

// Считывает имена файлов и поддиректорий в массив,
// при этом добавляется первой буквой " ",
// если это файл, или "*", если это поддиректория
// Возвращает кол-во файлов и поддиректорий
// Важно! ffconf.h : #define _FS_RPATH 0
long int readDirectory( char * DirPath )
{
	ret = f_opendir( &dir, DirPath );	// Open the directory
	if (ret)	fault_err(ret);
	//
	for ( i = 0; i < ( MAX_TRACK_LEN * 2 / FILE_NAME_LEN ); i++ )
	{
		if ( i == 0 && strlen( DirPath ) > 1 )	// не корневая директория
		{
			memcpy( buff, "*..", 4 );
			continue;
		}
		ret = f_readdir( &dir, &fno );	// Read a directory item
		if ( ret != FR_OK || fno.fname[0] == 0 ) break;  // Break on error or end of dir
		fn = fno.fname;
		memcpy( &buff[ FILE_NAME_LEN * i + 1 ], fn, strlen( fn ) + 1 );
		if ( fno.fattrib & AM_DIR )		// It is a directory
			buff[ FILE_NAME_LEN * i ] = 42;	// "*"
		else
			buff[ FILE_NAME_LEN * i ] = 32;	// " ";
	}
	f_closedir( &dir );
	return i;
}

btns readKeyboard(void)
{
	while (1)
	{
		_delay( 3 );

		if ( ( btnUp_GPIO_Port->IDR & btnUp_Pin ) == 0 )
			return btnUp;
		else
		if ( ( btnDown_GPIO_Port->IDR & btnDown_Pin ) == 0 )
			return btnDown;
		else
		if ( ( btnEnter_GPIO_Port->IDR & btnEnter_Pin ) == 0 )
			return btnEnter;
		else
		if ( ( btnUSBMode_GPIO_Port->IDR & btnUSBMode_Pin ) == 0 )
			return btnUSBMode;
	}
}

void Error2LCD(char * ErrorStr)
{
	lcdClear();
	lcdSetCursor(0,0);
	lcdPrintStr( "E:" );
	lcdSetCursor(2,0);
	lcdPrintStr( ErrorStr );
	lcdSetCursor(0,1);
	lcdPrintStr("- Push any key -" );
	readKeyboard();
}
void GPIO_Config()
{
	ledDriveA_GPIO_Port->BSRR = ledDriveA_Pin;
	ledDriveB_GPIO_Port->BSRR = ledDriveB_Pin;
	ledStep_GPIO_Port->BSRR = ledStep_Pin;
}

void Status2LCD(void)
{
	LCDbuff[7] = 0x30+SideW;
	lcdSetCursor(0, 1);
	lcdPrintStr(LCDbuff);
}

static void fault_err (FRESULT rc)
{
	const char *str =
					"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
					"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
					"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
					"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
	FRESULT i;

	for (i = (FRESULT)0; i != rc && *str; i++)
	{
		while (*str++) ;
	}
	lcdClear();
	lcdPrintStr( (char *)"Error:");
	lcdSetCursor(0,1);
	lcdPrintStr( (char *)str );
	while(1)	;
}

void USART2_Config(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	USART1->BRR = 0x271;	// 115200
	USART1->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

void SendChar2USART(char c)
{
	while ( ( USART2->SR & USART_SR_TXE ) == 0 )	;
	USART2->DR = (uint8_t) c;
}

static void _delay(__IO uint32_t nCount)
{
	__IO uint32_t index;


	for (index = (100000 * nCount); index != 0; index--);
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\n\r", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
