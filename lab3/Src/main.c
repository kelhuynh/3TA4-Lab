/****** 



1.The Fist Extern button (named extBtn1)  connected to PC1 (canot use PB1, for 429i-DISCO ,pb1 is used by LCD), ExtBtn1_Config()  //
		2014: canot use pin PB1, for 429i-DISCO ,pb1 is used by LCD. if use this pin, always interrupt by itself
					can not use pin PA1, used by gyro. if use this pin, never interrupt
					pd1----WILL ACT AS PC13, To trigger the RTC timestamp event
					....ONLY PC1 CAN BE USED TO FIRE EXTI1 !!!!
2. the Second external button (extBtn2) may connect to pin PD2.  ExtBtn2_Config() --The pin PB2 on the board have trouble.
    when connect extern button to the pin PB2, the voltage at this pin is not 3V as it is supposed to be, it is 0.3V, why?
		so changed to use pin PD2.
	
	 
		PA2: NOT OK. (USED BY LCD??)
		PB2: ??????
		PC2: ok, BUT sometimes (every 5 times around), press pc2 will trigger exti1, which is configured to use PC1. (is it because of using internal pull up pin config?)
		      however, press PC1 does not affect exti 2. sometimes press PC2 will also affect time stamp (PC13)
		PD2: OK,     
		PE2:  OK  (PE3, PE4 PE5 , seems has no other AF function, according to the table in manual for discovery board)
		PF2: NOT OK. (although PF2 is used by SDRAM, it affects LCD. press it, LCD will flick and displayed chars change to garbage)
		PG2: OK
		
	3. RTC and RTC_Alarm have been configured.
 
*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"


/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)



/* Private macro -------------------------------------------------------------*/

 

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  I2c3_Handle;

RTC_HandleTypeDef RTCHandle;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;



HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 


//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range


char lcd_buffer[14];
char lcd_buffer1[14];
char timestr[10] = {0}; //initialize time string
char datestr[6] = {0}; //initialize date string

uint8_t wd, dd, mo, yy, ss, mm, hh; //day, numerical day, month, year, seconds, minutes, hours

__IO uint32_t ubpressStartTick; // sysTick when user button pressed
__IO uint8_t bpresshold; // button press held down

void RTC_Config(void);
void RTC_AlarmAConfig(void);

void currentDate(void); // Function to display current date
void RTC_SET_TIME(void); // Function to set time using external buttons
void RTC_SET_DATE(void); // Function to set date using external buttons
void setTime(void); // EEPROM Write
void getTime(void); // EEPROM Read
void showRecentTime(void);

void ExtBtn1_Config(void); // for the first External button
void ExtBtn2_Config(void); // for the second External button

int ubpush = 0;
int ext1push = 0;
int ext2push = 0;
bool ext1press = false;
bool ext2press = false;
bool screen = false;
int dateCount = 0;
int ubpress = 0; // if user button pressed
bool write = false; // determines if writing to EEPROM

uint8_t Time1[7]; // Stores most recent time of the two
uint8_t Time2[7]; // Stores the older time of the two

uint8_t data1 =0x67,  data2=0x68;
uint8_t readData=0x00;
uint32_t EE_status;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  //the following variables are for testging I2C_EEPROM
	
	char AA[34]= "efghijklmnopqstuvefghijklmnopqstuv";
	uint8_t * bufferdata=(uint8_t *)AA;	
	int i;
	uint8_t readMatch=1;

	
	/* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
	
	 /* Configure the system clock to 180 MHz */
  SystemClock_Config();
	
	
	//Init Systic interrupt so can use HAL_Delay() function 
	HAL_InitTick(0x0000); // set systick's priority to the highest.
                        
  
	
	//Configure LED3 and LED4 ======================================

	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);

	//configure the USER button as exti mode. 
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);   // BSP_functions in stm32f429i_discovery.c
	


	//Use this line, so do not need extiline0_config()
	//configure external push-buttons and interrupts
	
	
//-----------Init LCD 
	BSP_LCD_Init();
	//BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address);
	BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
														// the LayerIndex may be 0 and 1. if is 2, then the LCD is dark.
	//BSP_LCD_SelectLayer(uint32_t LayerIndex);
	BSP_LCD_SelectLayer(0);
	//BSP_LCD_SetLayerVisible(0, ENABLE); //do not need this line.
	BSP_LCD_Clear(LCD_COLOR_WHITE);  //need this line, otherwise, the screen is dark	
	BSP_LCD_DisplayOn();
 
	BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	
	
		LCD_DisplayString(5, 2, (uint8_t *)"MT3TA4 LAB 3");
		
		HAL_Delay(1000);   //display for 1 second
		
		//BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_ClearStringLine(5);
		BSP_LCD_ClearStringLine(6);
		BSP_LCD_ClearStringLine(7);
		
		
		
	//	LCD_DisplayInt(7, 6, 123456789);
	//	LCD_DisplayFloat(8, 6, 12.3456789, 4);

//Init I2C for EEPROM		
	I2C_Init(&I2c3_Handle);
	getTime();
	showRecentTime(); // Used to show that EEPROM is working as intended, not actual implementation, solely for demo
	HAL_Delay(5000); // Mainly due to external buttons not functioning
	
	//configure real-time clock
	RTC_Config();
		
	RTC_AlarmAConfig();
	
	
	


//*********************Testing I2C EEPROM------------------
/*
	EE_status=I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation , data1);
	if (EE_status==HAL_OK)
			LCD_DisplayString(0, 0, (uint8_t *)"w data1 OK");
	else
			LCD_DisplayString(0, 0, (uint8_t *)"w data1 failed");

	EE_status=I2C_ByteWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation+1 , data2);
	if (EE_status==HAL_OK)
			LCD_DisplayString(1, 0, (uint8_t *)"w data2 OK");
	else
			LCD_DisplayString(1, 0, (uint8_t *)"w data2 failed");
	
	
	
	
	readData=I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation);
	if (data1 == readData) {
			LCD_DisplayString(3, 0, (uint8_t *)"r data1 success");
	}else{
			LCD_DisplayString(3, 0, (uint8_t *)"r data1 mismatch");
	}	
	LCD_DisplayInt(3, 14, readData);
	
	readData=I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation+1);
	if (data2 == readData) {
			LCD_DisplayString(4, 0, (uint8_t *)"r data2 success");
	}else{
			LCD_DisplayString(4, 0, (uint8_t *)"r data2 mismatch");
	}	
	LCD_DisplayInt(4, 14, readData);
	

	EE_status=I2C_BufferWrite(&I2c3_Handle,EEPROM_ADDRESS, memLocation, bufferdata, 34);
	if (EE_status==HAL_OK)
		LCD_DisplayString(6, 0, (uint8_t *)"w buffer OK");
	else
		LCD_DisplayString(6, 0, (uint8_t *)"W buffer failed");

	for (i=0;i<=33;i++) { 
			readData=I2C_ByteRead(&I2c3_Handle,EEPROM_ADDRESS, memLocation+i);
			HAL_Delay(5);   // just for display effect. for EEPROM read, do not need dalay		
		//BUT :  if here delay longer time, the floowing display will have trouble,???
	
			BSP_LCD_DisplayChar(COLUMN(i%16),LINE(8+ 2*(int)(i/16)), (char) readData);	
			BSP_LCD_DisplayChar(COLUMN(i%16),LINE(9+ 2*(int)(i/16)),  bufferdata[i]);
			if (bufferdata[i]!=readData)
					readMatch=0;
	}

	if (readMatch==0)
		LCD_DisplayString(15, 0, (uint8_t *)"r buffer mismatch");
	else 
		LCD_DisplayString(15, 0, (uint8_t *)"r buffer success");
	*/
//******************************testing I2C EEPROM*****************************/



  /* Infinite loop */

while (1)
  {	
		/*if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
			ubpressStartTick = HAL_GetTick();
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){ // while button is pressed
				if((HAL_GetTick()-ubpressStartTick) > 500){ // if held more than 0.5s
					currentDate(); // display current date
				}
			//}
		}*/
  } //end of while
}  

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE; // Initialize LSI Oscillator for use as clock source
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */




/**
 * Use this function to configure the GPIO to handle input from
 * external pushbuttons and configure them so that you will handle
 * them through external interrupts.
 */
void ExtBtn1_Config(void)     // for GPIO C pin 1
// can only use PA0, PB0... to PA4, PB4 .... because only  only  EXTI0, ...EXTI4,on which the 
	//mentioned pins are mapped to, are connected INDIVIDUALLY to NVIC. the others are grouped! 
		//see stm32f4xx.h, there is EXTI0_IRQn...EXTI4_IRQn, EXTI15_10_IRQn defined
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1; // Pin 1
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure); // PC -> PC1 port

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void ExtBtn2_Config(void){  //**********PD2.***********
 
	 
	  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2; // Pin 2
	GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure); // PD -> PD2 port

	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}


void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure; // time
	RTC_DateTypeDef RTC_DateStructure; // date
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
	/****************
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RTC_AlarmTypeDef RTC_AlarmStructure;
	****************/
	
	//1: Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE(); 
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();
	/*
			//1.3: Select the RTC clock source
				RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
				RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
				RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
				RCC_OscInitStruct.LSIState = RCC_LSI_ON;
				if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
				{
					Error_Handler();
				}
	
				__HAL_RCC_RTC_CLKPRESCALER(RCC_RTCCLKSOURCE_LSI);
				*/
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);  //RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
					//can not use LSE!!!---LSE is not available, at leaset not available for stm32f407 board.
				//****"Without parts at X3, C16, C27, and removing SB15 and SB16, the LSE is not going to tick or come ready"*****.
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC
	
			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {}    //defind in rcc.c
	
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
				
				
				
	//2.  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        
				RTCHandle.Instance = RTC;
				RTCHandle.Init.HourFormat = RTC_HOURFORMAT_12;
				//RTC time base frequency =LSE/((AsynchPreDiv+1)*(SynchPreDiv+1))=1Hz
				//see the AN3371 Application Note: if LSE=32.768K, PreDiv_A=127, Prediv_S=255
				//    if LSI=32K, PreDiv_A=127, Prediv_S=249
				//also in the note: LSI accuracy is not suitable for calendar application!!!!!! 
				RTCHandle.Init.AsynchPrediv = 127; //if using LSE: Asyn=127, Asyn=255: 
				RTCHandle.Init.SynchPrediv = 249;  //if using LSI(32Khz): Asyn=127, Asyn=249: 
				// but the UM1725 says: to set the Asyn Prescaler a higher value can mimimize power comsumption
				
				RTCHandle.Init.OutPut = RTC_OUTPUT_ALARMA; // Configure RTC output to AlarmA
				RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
				RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
				
				//HAL_RTC_Init(); 
				if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
				{
						BSP_LCD_Clear(LCD_COLOR_CYAN);
						LCD_DisplayString(1, 0, (uint8_t *)"RTC Init Error!");
				}
	
	//3. init the time and date
				RTC_DateStructure.Year = 21; // 2021
				RTC_DateStructure.Month = 11; // nov
				RTC_DateStructure.Date = 1; // 1st of nov | if use RTC_FORMAT_BCD, NEED TO SET IT AS 0x18 for the 18th.
				RTC_DateStructure.WeekDay = RTC_WEEKDAY_MONDAY; //???  if the real weekday is not correct for the given date, still set as 
																												//what is specified here.
				
				if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better 
															//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_Clear(LCD_COLOR_CYAN);
					LCD_DisplayString(2, 0, (uint8_t *)"Date Init Error!");
				} 
  
  
				RTC_TimeStructure.Hours = 4;  
				RTC_TimeStructure.Minutes = 17; //if use RTC_FORMAT_BCD, NEED TO SET IT AS 0x19
				RTC_TimeStructure.Seconds = 29;
				RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
				RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;//?????/
				
				if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better
																																					//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_Clear(LCD_COLOR_CYAN);
					LCD_DisplayString(3, 0, (uint8_t *)"TIME Init Error!");
				}
  
			//Writes a data in a RTC Backup data Register0   --why need this line?
			//HAL_RTCEx_BKUPWrite(&RTCHandle,RTC_BKP_DR0,0x32F2);   

	/*
			//The RTC Resynchronization mode is write protected, use the
			//__HAL_RTC_WRITEPROTECTION_DISABLE() befor calling this function.
			__HAL_RTC_WRITEPROTECTION_DISABLE(&RTCHandle);
			//wait for RTC APB registers synchronisation
			HAL_RTC_WaitForSynchro(&RTCHandle);
			__HAL_RTC_WRITEPROTECTION_ENABLE(&RTCHandle);				
	 */
				
				
			__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
			__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
				//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
				//Timestamp on tamper event
				//With TAMPTS set to �1 , any tamper event causes a timestamp to occur. In this case, either
				//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
				//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
				//same time that TSF or TSOVF is set. ---P802, about Tamper detection
				//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
				//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
				//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
				//---function.
				
			HAL_RTC_WaitForSynchro(&RTCHandle);	
			//To read the calendar through the shadow registers after Calendar initialization,
			//		calendar update or after wake-up from low power modes the software must first clear
			//the RSF flag. The software must then wait until it is set again before reading the
			//calendar, which means that the calendar registers have been correctly copied into the
			//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
			//implements the above software sequence (RSF clear and RSF check).	
}


void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;

	RTC_Alarm_Structure.Alarm = RTC_ALARM_A;
  	RTC_Alarm_Structure.AlarmMask = RTC_ALARMMASK_ALL;
				// See reference manual. especially 
				//p11-12 of AN3371 Application Note.
				// this mask mean alarm occurs every second.
				//if MaskAll, the other 3 fieds of the AlarmStructure do not need to be initiated
				//the other three fieds are: RTC_AlarmTime(for when to occur), 
				//RTC_AlarmDateWeekDaySel (to use RTC_AlarmDateWeekDaySel_Date or RTC_AlarmDateWeekDaySel_WeekDay
				//RTC_AlarmDateWeekDay (0-31, or RTC_Weekday_Monday, RTC_Weekday_Tuesday...., depends on the value of AlarmDateWeekDaySel)	
	//RTC_Alarm_Structure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  //RTC_Alarm_Structure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  //RTC_Alarm_Structure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
		   //RTC_ALARMSUBSECONDMASK_ALL --> All Alarm SS fields are masked. 
        //There is no comparison on sub seconds for Alarm 
			
  //RTC_Alarm_Structure.AlarmTime.Hours = 0x02;
  //RTC_Alarm_Structure.AlarmTime.Minutes = 0x20;
  //RTC_Alarm_Structure.AlarmTime.Seconds = 0x30;
  //RTC_Alarm_Structure.AlarmTime.SubSeconds = 0x56;
  
  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			LCD_DisplayString(4, 0, (uint8_t *)"Alarm setup Error!");
  }
  
	//Enable the RTC Alarm interrupt
//	__HAL_RTC_ALARM_ENABLE_IT(&RTCHandle,RTC_IT_ALRA);   //already in function HAL_RTC_SetAlarm_IT()
	
	//Enable the RTC ALARMA peripheral.
//	__HAL_RTC_ALARMA_ENABLE(&RTCHandle);  //already in function HAL_RTC_SetAlarm_IT()
	
	__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //need it? !!!!, without it, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		
	

	
		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 4, 0);  //not important
	
				
	
}

/*
Used disable AlarmA Interrupt
*/
HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}

/*
Used to re-enable AlarmA Interrupt
*/
HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

}







void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				BSP_LCD_DisplayChar(COLUMN(ColumnNumber),LINE(LineNumber), *ptr); //new version of this function need Xpos first. so COLUMN() is the first para.
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if ((ColumnNumber+1)*(((sFONT *)BSP_LCD_GetFont())->Width)>=BSP_LCD_GetXSize() ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}



/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
  if(GPIO_Pin == KEY_BUTTON_PIN)  // User Button, press to write to EEPROM, hold for 0.5s to display current date and time
  {
			
			ubpressStartTick = HAL_GetTick();
		
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){ // while button is pressed
				RTC_AlarmA_IT_Disable(&RTCHandle);
				BSP_LCD_Clear(LCD_COLOR_BLUE);
				if((HAL_GetTick()- ubpressStartTick) > 500){ // if held more than 0.5s
					currentDate(); // display current date
					HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN); //stores current time in rtc_structure
					sprintf(lcd_buffer, "Time:%02d:%02d:%02d", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes,  RTC_TimeStructure.Seconds);// store to lcd_buffer
					LCD_DisplayString(2,2,(uint8_t*)lcd_buffer);
					HAL_Delay(1000);

				}
			}
			if(ext2press == false && (HAL_GetTick()- ubpressStartTick) < 500){
				HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
				ss = RTC_TimeStructure.Seconds;
				mm = RTC_TimeStructure.Minutes;
				hh = RTC_TimeStructure.Hours;
				wd = RTC_DateStructure.WeekDay;
				dd = RTC_DateStructure.Date;
				mo = RTC_DateStructure.Month;
				yy = RTC_DateStructure.Year;
				setTime();	
				ubpush++;	
		}
			BSP_LCD_Clear(LCD_COLOR_BLUE);
			RTC_AlarmA_IT_Enable(&RTCHandle);
  }
	
	
	if(GPIO_Pin == GPIO_PIN_1) // External 1, used to select element to modify after edit mode or show recent times from EEPROM
  {
		if(ext2press == false){
			ext1press = !ext1press;
		}
		if(ext1press == true && ext2press == false){
			getTime();
			showRecentTime();
		}
		switch(ext1push){
			case 1:
				RTC_TimeStructure.Minutes = (RTC_TimeStructure.Minutes + ubpush)%60;
				break;
			case 2:
				RTC_TimeStructure.Hours = (RTC_TimeStructure.Hours + ubpush)%23;
				break;
			case 3:
				RTC_DateStructure.Date = (RTC_DateStructure.Date + ubpush)%31;
				break;
			case 4:
				RTC_DateStructure.Month = (RTC_DateStructure.Month + ubpush)%12;
				break;
			case 5:
				RTC_DateStructure.Year = (RTC_DateStructure.Year + ubpush)%100;
				break;
			case 6:
				RTC_DateStructure.WeekDay = (RTC_DateStructure.WeekDay + ubpush)%7;
				break;
			case 0:
				RTC_TimeStructure.Seconds = (RTC_TimeStructure.Seconds + ubpush)%60;
				break;
		}
		ext1push++;
		ubpush = 0;
		ext1push %= 7;
	}  //end of PIN_1 / PC1

	if(GPIO_Pin == GPIO_PIN_2) // External 2, used to enter edit mode for setting time
  {
		ext2press = !ext2press;
		ext2push++;
		ext2push %= 3;
		ext2push = 0;
		ext1push = 0;
		ubpush = 0;
		if (ext2press == true){
			RTC_SET_TIME();
		}
	} //end of PIN_2 / PD2
}

/*
Function used to enter edit mode and allow modification of time
*/
void RTC_SET_TIME(){
	
	sprintf(lcd_buffer, "%02d:%02d:%02d", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes,  RTC_TimeStructure.Seconds); //stores time in lcd_buffer
	sprintf(lcd_buffer1, "Set the time:"); 
	BSP_LCD_Clear(LCD_COLOR_CYAN); // display to lcd
	LCD_DisplayString(5, 0, (uint8_t*)lcd_buffer1);
	LCD_DisplayString(8, 0, (uint8_t*)lcd_buffer);

	//select_push = 0;
	if(ext1push == 0){ //Flashes seconds to signify we are setting seconds
	
		if(screen == false){ 
			sprintf(lcd_buffer, "%02d:%02d:  ", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes);
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			LCD_DisplayString(8,0,(uint8_t*)lcd_buffer);
		}
		else{
			sprintf(lcd_buffer, "%02d:%02d:%02d", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes,  (RTC_TimeStructure.Seconds + ubpush)%60); //shows the incremented value of seconds by adding number of times user has pushed select push
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			LCD_DisplayString(8,0,(uint8_t*)lcd_buffer);
		}
	}
	else if(ext1push == 1){ //Flashes minutes to signify we are setting minutes
		if(screen == false){
			sprintf(lcd_buffer, "%02d:  :%02d", RTC_TimeStructure.Hours, RTC_TimeStructure.Seconds);
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			LCD_DisplayString(8,0,(uint8_t*)lcd_buffer);
		}
		else{
			sprintf(lcd_buffer, "%02d:%02d:%02d", RTC_TimeStructure.Hours, (RTC_TimeStructure.Minutes + ubpush)%60,  RTC_TimeStructure.Seconds);
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			LCD_DisplayString(8,0,(uint8_t*)lcd_buffer);
		}
	}
	else if(ext1push == 2){ //Flashes hours to signify we are setting hours
		if(screen == false){
			sprintf(lcd_buffer, "  :%02d:%02d", RTC_TimeStructure.Minutes, RTC_TimeStructure.Seconds);
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			LCD_DisplayString(8,0,(uint8_t*)lcd_buffer);
		}
		else{
			sprintf(lcd_buffer, "%02d:%02d:%02d", (RTC_TimeStructure.Hours + ubpush)%23, RTC_TimeStructure.Minutes,  RTC_TimeStructure.Seconds);
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			LCD_DisplayString(8,0,(uint8_t*)lcd_buffer);
		}
	}
	else{
		RTC_SET_DATE(); // Enter function to modify date
	}
}

/*
Function used to retain edit mode and modify date
*/
void RTC_SET_DATE(void){

	if(ext1push == 3){ //flashes date to change
		if(screen == false){
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			sprintf(lcd_buffer, "DATE:%02d", (RTC_DateStructure.Date + ubpush)%31 + 1); //modulus 31 to make sure that we dont go over number of days in a month
			LCD_DisplayString(5,0,(uint8_t*)lcd_buffer);
		}
		else{
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			sprintf(lcd_buffer, "DATE"); //flashes the date value on and off
			LCD_DisplayString(5,0,(uint8_t*)lcd_buffer);
		}
	}

	else if(ext1push == 4){ //toggling the month for user edit
		if(screen == false){
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			sprintf(lcd_buffer, "MNTH:%02d", (RTC_DateStructure.Month + ubpush)%12 + 1); //add one since you cant have month 00
			LCD_DisplayString(7,0,(uint8_t*)lcd_buffer);
		}
		else{
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			sprintf(lcd_buffer, "MNTH"); //flashes the month value on and off
			LCD_DisplayString(7,0,(uint8_t*)lcd_buffer);
		}
	}

	else if(ext1push == 5){ //toggling the year for user edit
		if(screen == false){
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			sprintf(lcd_buffer, "YEAR%02d", (RTC_DateStructure.Year + ubpush)%100);
			LCD_DisplayString(9,0,(uint8_t*)lcd_buffer);
		}
		else{
			BSP_LCD_Clear(LCD_COLOR_CYAN);
			sprintf(lcd_buffer, "YEAR"); //flashes the year value on and off
			LCD_DisplayString(9,0,(uint8_t*)lcd_buffer);
		}
	}
}

/*
Function to store date and time into EEPROM
*/
void setTime(void){
	write = true; //signify we are writing to eeprom
	for(int x = 0; x < 7; x++){ //copy time1 to time2
		Time2[x] = Time1[x]; 
	}
	Time1[0] = yy; //setting each list indicie to a time variable
	Time1[1] = mo;
	Time1[2] = dd;
	Time1[3] = wd;
	Time1[4] = hh;
	Time1[5] = mm;
	Time1[6] = ss;
	for(int x = 0; x < 7; x++){ //write time 1 to the eeprom
		EE_status = I2C_ByteWrite(&I2c3_Handle, EEPROM_ADDRESS, memLocation + x, Time1[x]);
		if(EE_status != HAL_OK){
			I2C_Error(&I2c3_Handle);
		}
		
		if (EE_status == HAL_OK){ //let user know that the daa is being written to eeprom
			BSP_LCD_Clear(LCD_COLOR_WHITE);
			LCD_DisplayString(5,0,(uint8_t*)"Writing to EEPROM");
			HAL_Delay(100);
		}
	}
	for(int x = 7; x< 14; x++){ //write time2 to a different memory location than time1
		EE_status = I2C_ByteWrite(&I2c3_Handle, EEPROM_ADDRESS, memLocation + x, Time2[x-7]);
		if(EE_status != HAL_OK){
			I2C_Error(&I2c3_Handle);
		}
		
		if (EE_status == HAL_OK){
			BSP_LCD_Clear(LCD_COLOR_WHITE);
			LCD_DisplayString(5,0,(uint8_t*)"Writing to EEPROM");
			HAL_Delay(100);
		}
	}
	write = false; //ends the writing loop
	
}

/*
Function to retrieve 2 recent times from EEPROM
*/
void getTime(void){
	BSP_LCD_Clear(LCD_COLOR_CYAN);
	LCD_DisplayString(5,0,(uint8_t*)"Reading");
	for(int x = 0; x < 14; x++){
		readData = I2C_ByteRead(&I2c3_Handle, EEPROM_ADDRESS, memLocation+x);
		if(x < 7){ //stores data into time 1
			Time1[x] = readData;
		}
		else{
			Time2[x-7] = readData; //stores data into time 2
		}
	}
}

/*
Displays the 2 recent times stored in the EEPROM on the LCD display
*/
void showRecentTime(void){
	BSP_LCD_Clear(LCD_COLOR_CYAN);
	LCD_DisplayString(6,0,(uint8_t*)"Time 1:");
	sprintf(lcd_buffer, "%02d:%02d:%02d", Time1[4], Time1[5], Time1[6]); //displays the hh, mm, ss values from the time array
	LCD_DisplayString(6,8,(uint8_t*)lcd_buffer);
	LCD_DisplayString(8,0,(uint8_t*)"Time 2:");
	sprintf(lcd_buffer1, "%02d:%02d:%02d", Time2[4], Time2[5], Time2[6]); //displays second most recent time 
	LCD_DisplayString(8,8,(uint8_t*)lcd_buffer1);

	ext1press = false;
}

/*
Alarm callback, periodic update of time every 1s
*/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
 
	BSP_LED_Toggle(LED4); //toggle led

	if(ext2press == false && write == false && ext1press == false){ // shows current time on screen
	HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN); //stores current time in rtc_structure
	sprintf(lcd_buffer, "Time:%02d:%02d:%02d", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes,  RTC_TimeStructure.Seconds);// store to lcd_buffer
	BSP_LCD_Clear(LCD_COLOR_BLUE); //get time and store in string, then display to lcd
	LCD_DisplayString(2,2,(uint8_t*)lcd_buffer);

  HAL_Delay(1000);
	HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
	}

	if(ext2press == true){ //toggles value of screen_on for setting time
		 screen = !screen;
		 RTC_SET_TIME(); //function to set time
	}
	dateCount++;
}

/*
Function that displays the current date when user button is held
*/
void currentDate(void){ //displays current date
	HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);

  //BSP_LCD_Clear(LCD_COLOR_BLUE);
		sprintf(lcd_buffer, "DATE:%02d", RTC_DateStructure.Date); //displays the current date value
	LCD_DisplayString(5,4,(uint8_t*)lcd_buffer);

	sprintf(lcd_buffer, "MNTH:%02d", RTC_DateStructure.Month); //displays the current month value
	LCD_DisplayString(6,4,(uint8_t*)lcd_buffer);

	sprintf(lcd_buffer, "YEAR:%02d", RTC_DateStructure.Year); //displays the current year value
	LCD_DisplayString(7,4,(uint8_t*)lcd_buffer);
	
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}





#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
