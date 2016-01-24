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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_conf.h"
#include "ff.h"
#include "lcd1602.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "types.h"
#include "specBetadisk.h"
#include "system.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define __IO    volatile
#define VERSION "v0.01"

FATFS fatfs;
FRESULT ret;			/* Result code */
FIL file;				/* File object */
DIR dir;				/* Directory object */
FILINFO fno;			/* File information object */

uint8_t buff[ MAX_TRACK_LEN * 2 ];
//                 0    1    2    3    4    5    6    7    8    9    10   11
char LCDbuff[13] = { 0x43,0x3A,0x30,0x20,0x20,0x53,0x3A,0x30,0x20,0x49,0x20,0x6D,0x00};
//                 C    :    0              S    :    0         IRW         mM
unsigned char TmpBuffer[32];
volatile long int CurrBit;
volatile uint32_t CylNumber;
volatile uint32_t LoadedCylNumber;
volatile long int CurrBitW, CurrBitWStart;
volatile long int SideW;
volatile machineState currentState = init;
volatile machineState lastState = noneState;

volatile uint32_t tempData = 0;
volatile uint32_t tempIORQDOS = 0;
volatile uint32_t tempRDWR = 0;

CDiskImage specImages[4];
//
//
long int trackListBase;
uint32_t temp_l1, temp_l2;
long int i;
char * x;
btns pressedButton;
long int allFiles;
long int FileIndex;
uint32_t trackNumber;

bool LOG_BDI_PORTS = true;

unsigned char timer_flag_1Hz = 0;
unsigned char timer_flag_100Hz = 0;
volatile unsigned int delayTimer = 0;
volatile unsigned int tapeTimer = 0;
volatile bool bdiTimerFlag = true;
volatile unsigned int bdiTimer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
btns readKeyboard(void);
void Error2LCD(char * ErrorStr);
void GPIO_Config(void);
void TIM4_Config(void);
void Status2LCD (void);
static void fault_err (FRESULT rc);
static void _delay(__IO uint32_t nCount);
void BDI_ResetWrite(void);
void BDI_Write(uint8_t);
void BDI_ResetRead(word);
bool BDI_Read(uint8_t*);
void BDI_Routine(void);
dword get_ticks(void);
void bdiStopTimer(void);
void BDI_StartTimer(void);
void __TRACE( const char *str, ... );
uint8_t readCPUDataBus(void);
void writeCPUDataBus(uint8_t data);
void bdiUpdateDisks(void);
void bdiInitDisks(void);
void SD_Init(void);

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
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	GPIO_Config();
	TIM4_Config();

	nRunZ80_GPIO_Port->BRR = nRunZ80_Pin;
	nRunZ80_GPIO_Port->BSRR = nRunZ80_Pin;
	nRunZ80_GPIO_Port->BRR = nRunZ80_Pin;
	nRunZ80_GPIO_Port->BSRR = nRunZ80_Pin;
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
		switch ( currentState )
		{
			case init:
			{
				CurrBit = 0;
				lcdInit();
				lcdClear();
				lcdPrintStr("JFDSe ");
				lcdPrintStr(VERSION);
				SD_Init();
				fdc_init();
				//bdiInitDisks();
				//floppy_open();
				HAL_Delay(100);

				lcdClear();
				lcdSetCursor(0,0);
				lcdPrintStr("Load: DIZZYD1.TRD");
				lcdSetCursor(0,1);

				strcpy(specImages[0].name, "DIZZYD1.TRD");
				//if (fdc_open_image( 0, specImages[0].name ))
				if (open_dsk_image( i, specImages[i].name ))
					lcdPrintStr("Success!");
				else
					lcdPrintStr("Error!");
				currentState = selectDir;

				break;
			}
			case selectDir:
			{
				//currentState = selectFile;
				//strcpy( CurrDir, "/" );
				break;
			}
			case idle:
			{
				break;
			}
			case selectFile:
			{
				allFiles = 0;
				FileIndex = 0;	// index массива
				while ( currentState == selectFile )
				{
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
					lcdPrintStr( (char *) &buff[ 11 * FileIndex ] );
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
								lcdPrintStr( (char *) &buff[ 11 * FileIndex ] );
								lcdPrintStr("       ");
							}
							break;
						}
						case btnEnter:
						{
							/*if ( buff[ FILE_NAME_LEN * FileIndex ] == 32 )	// файл
								if ( strlen( CurrDir ) <= MAX_PATH_LEN - 1 - 12 )
								{

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
								allFiles = 0;
								FileIndex = 0;	// index массива
							}
										*/
							break;
						}
						case btnDown:
						{
							/*
							if ( FileIndex + 1 < allFiles )
							{
								FileIndex++;
								lcdSetCursor(0,1);
								lcdPrintStr( (char *)&buff[ FILE_NAME_LEN * FileIndex ] );
								lcdPrintStr("       ");
							}
							*/
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
        //Serial_Routine();
        //Tape_Routine();
        BDI_Routine();

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
void GPIO_Config(void)
{
	ledDriveA_GPIO_Port->BSRR = ledDriveA_Pin;
	ledDriveB_GPIO_Port->BSRR = ledDriveB_Pin;
	ledStep_GPIO_Port->BSRR = ledStep_Pin;
}

void TIM4_Config(void)
{
	TIM4->DIER = TIM_DIER_UIE;
	TIM4->CR1 |= TIM_CR1_CEN;
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
}

static void _delay(__IO uint32_t nCount)
{
	__IO uint32_t index;

	for (index = (100000 * nCount); index != 0; index--);
}
void BDI_ResetWrite()
{
/*
	trdosFifoReadRst <= '1';
 */
//	SystemBus_WriteAtAddress( 0xc00060, 0x8000 );
}

void BDI_Write( uint8_t data )
{
/*
	trdosFifoReadWrTmp <= data;
	trdosFifoReadWr <= '1';
*/
//    SystemBus_WriteAtAddress( 0xc00060, data );
	writeCPUDataBus(data);
}

void BDI_ResetRead( word counter )
{
/*
	trdosFifoWriteRst <= ARM_AD( 15 );
	trdosFifoWriteCounter <= unsigned( ARM_AD( 10 downto 0 ));
 */
//    SystemBus_WriteAtAddress( 0xc00061, 0x8000 | counter );
}

bool BDI_Read( uint8_t *data )
{
/*
	if addressReg( 7 downto 0 ) = x"61" then
		ARM_AD <= trdosFifoWriteReady & "0000000" & trdosFifoWriteRdTmp;
		if trdosFifoWriteReady = '1' then
			trdosFifoWriteRd <= '1';
		end if;

	word result = SystemBus_ReadAtAddress( 0xc00061 );

    *data = (byte) result;

    return ( result & 0x8000 ) != 0;
*/
	*data = (uint8_t) readCPUDataBus();
	return 1;
}

void BDI_Routine()
{
    //int ioCounter = 0x20;

    /*
    		if addressReg( 7 downto 0 ) = x"19" then
    			ARM_AD <= x"00" & b"000000" & specTrdosWr & specTrdosWait;
    */
    //word trdosStatus = SystemBus_ReadAtAddress( 0xc00019 );

    if (nIORQnDOS == 0)
    {
    	ledStep_On;
/*
        elsif addressReg( 7 downto 0 ) = x"1a" then
			ARM_AD <= cpuA;

*/
        //byte trdosAddr = SystemBus_ReadAtAddress( 0xc0001a );

    	uint32_t tempData = fdc_GPIO_Port->IDR;
    	tempRDWR = nRD_GPIO_Port->IDR & (nRD_Pin | nWR_Pin);

    	//byte trdosAddr = ((temp & (A7_Pin | A6_Pin | A5_Pin)) << 2) | (0x1E) | ((temp & A0_Pin) >> 2);
    	//uint8_t trdosAddr = ((( tempData & 0x38) << 2) | ((tempData & 0x04) >> 2)) | (0x1E);

    	uint8_t trdosAddr = (( tempData & (A765Mask)) << 2) | ((tempData & A0_Pin) >> 2);
		trdosAddr |= (0x1E);

    	uint8_t trdosData = ((tempData & 0xFF00)  >> 8);

//        static int counter = 0;

        if (nWR == 0)
        {
/*
        	if addressReg( 7 downto 0 ) = x"1b" then
				ARM_AD <= x"00" & cpuDout;
*/
        	//byte trdosData = SystemBus_ReadAtAddress( 0xc0001b );
        	trdosData = readCPUDataBus();
            fdc_write( trdosAddr, trdosData );
            //__TRACE( "W: 0x%.2x,  0x%.2x\n", trdosAddr, trdosData );

/*            if( LOG_BDI_PORTS )
            {
                if( trdosAddr == 0x7f )
                {
                    if( counter == 0 ) __TRACE( "Data write : " );

                    __TRACE( "%.2x.", trdosData );

                    counter++;
                    if( counter == 16 )
                    {
                        counter = 0;
                        __TRACE( "\n" );
                    }
                }
                else //if( trdosAddr != 0xff )
                {
                    if( counter != 0 )
                    {
                        counter = 0;
                        __TRACE( "\n" );
                    }

                    __TRACE( "0x%.2x, 0x%.2x\n", trdosAddr, trdosData );
                }
            }
            */
        }
        else if(nRD == 0)
        {
            uint8_t trdosData = fdc_read( trdosAddr );


            //cpuDin <= trdosData;
            //SystemBus_WriteAtAddress( 0xc0001b, trdosData );
            writeCPUDataBus(trdosData);
            //__TRACE( "R: 0x%.2x,  0x%.2x\n", trdosAddr, trdosData );

/*
            if( LOG_BDI_PORTS )
            {
                if( trdosAddr == 0x7f )
                {
                    if( counter == 0 ) __TRACE( "Data read : " );

                    __TRACE( "%.2x.", trdosData );

                    counter++;
                    if( counter == 16 )
                    {
                        counter = 0;
                        __TRACE( "\n" );
                    }
                }
                else if( trdosAddr != 0xff )
                {
                    __TRACE( "0x%.2x, 0x%.2x\n", trdosAddr, trdosData );
                }
            }
*/

		//specTrdosPortFF <= fdc_read( 0xff );
        //SystemBus_WriteAtAddress( 0xc0001d, fdc_read( 0xff ) );

		//specTrdosWait <= '0';
        //SystemBus_WriteAtAddress( 0xc00019, 0 );

        fdc_dispatch();

		//specTrdosPortFF <= fdc_read( 0xff );
        //SystemBus_WriteAtAddress( 0xc0001d, fdc_read( 0xff ) );
        //SystemBus_Write( 0xc00019, 0 );

        //if( --ioCounter == 0 ) break;

/*
		if addressReg( 7 downto 0 ) = x"19" then
			ARM_AD <= x"00" & b"000000" & specTrdosWr & specTrdosWait;
*/
        //trdosStatus = SystemBus_ReadAtAddress( 0xc00019 );
        //if( nDOS > 0 ) break;
        //fdc_dispatch();
        }
    //while ((nIORQ == 0)&(nDOS ==0))
    //{};

	//specTrdosPortFF <= fdc_read( 0xff );
    //SystemBus_WriteAtAddress( 0xc0001d, fdc_read( 0xff ) );
	ledStep_Off;
    }
	nRunZ80_GPIO_Port->BRR = nRunZ80_Pin;
	nRunZ80_GPIO_Port->BSRR = nRunZ80_Pin;
	// Set CPU DataBus pins as Floating Input
	CPUDataBusPort->CRH = 0x44444444;
}

dword get_ticks()
{
    return bdiTimer;
}

void bdiStopTimer()
{
    bdiTimerFlag = false;
}

void BDI_StartTimer()
{
    bdiTimerFlag = true;
}

void __TRACE( const char *str, ... )
{
    static char fullStr[ 0x80 ];
    va_list ap;
    va_start( ap, str );
    vsniprintf( fullStr, sizeof(fullStr), str, ap );
    va_end(ap);

    uint8_t i = 0;
    while (fullStr[i] > 0)
    {
    	i++;
    }
    //printf(fullStr);
    HAL_UART_Transmit(&huart2, (uint8_t *)fullStr, i, 10000);
    /*

    if( traceNewLine )
    {
        Serial_Routine();
        //const char delChar = 0x08;
        //for( int i = 0; i <= cmdSize; i++ ) uart0.WriteFile( (byte*) &delChar, 1 );
    }

    char lastChar = 0;
    char *strPos = fullStr;

    while( *strPos != 0 )
    {
        lastChar = *strPos++;

        if( lastChar == '\n' ) uart0.WriteFile( (byte*) "\r\n", 2 );
        else uart0.WriteFile( (byte*) &lastChar, 1 );

        WDT_Kick();
    }

    traceNewLine = ( lastChar == '\n' );

    if( traceNewLine )
    {
        UART0_WriteText( ">" );

        Serial_Routine();
        if( cmdSize > 0 ) uart0.WriteFile( (byte*) cmd, cmdSize );
    }
    */
}

uint8_t readCPUDataBus(void)
{
	// Set CPU DataBus pins as Floating Input
	CPUDataBusPort->CRH = 0x44444444;


	// Read and return CPU data
	return (CPUDataBusPort->IDR & 0xFF00) >> 8;
}

void writeCPUDataBus(uint8_t data)
{
	// Reset CPU DataBus
	CPUDataBusPort->BRR = 0xFF00;
	// Set CPU DataBus pins as Open-Drain output
	CPUDataBusPort->CRH = 0x11111111;
	// Send data on CPU DataBus
	CPUDataBusPort->ODR |= ((data << 8) & 0xFF00);
}

void bdiUpdateDisks()
{
    uint8_t i;
	for( i = 0; i < BETADSK_NUM_DRIVES; i++ )
    {
        //fdc_open_image( i, specImages[i].name );
		open_dsk_image( i, specImages[i].name );
		beta_set_disk_wp( i, specImages[i].readOnly );
    }
}

void bdiInitDisks()
{
	uint8_t i;
    for( i = 0; i < BETADSK_NUM_DRIVES; i++ )
    {
        strcpy( specImages[i].name, "" );
        specImages[i].readOnly = false;
    }
}

void SD_Init()
{
	if ( ( ret = f_mount( &fatfs, "/", 1 )) != FR_OK )
	{
		fault_err(ret);
		__TRACE( "SD card init error :(\n" );
	}
	else
	{
		__TRACE( "SD card init OK\n" );
		//MassStorage_UpdateCharacteristics();

		//f_mount( 0, &fatfs );
	}

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
