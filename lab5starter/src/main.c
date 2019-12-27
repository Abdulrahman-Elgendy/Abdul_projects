/**
******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

__IO uint16_t TIM3_CCR;   //Timer 3 CCR variable .

TIM_HandleTypeDef    Tim3_Handle, TIM4_Handle; //Timer handles 
TIM_OC_InitTypeDef Tim3_OCInitStructure;

uint16_t Tim3_PrescalerValue; //Timers 3 Prescaler value variable 

char lcd_buffer[8];    // LCD display buffer
__O uint8_t factor = 0;
__IO uint8_t Up_pressed, Down_pressed, Sel_pressed, Right_pressed, Left_pressed; //variables for GPIO joystick .
uint16_t Full_Step_Number = 0; //variable that increments between steps in full-step mode
uint16_t Half_Step_Number = 0; //Variable that increments between steps in half-step mode


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void); //configure system clock
static void Error_Handler(void);
void TIM3_Config(void); // onfigure timer 3
void DisplayInfo(void);
void Set_Pin_Values(GPIO_PinState, GPIO_PinState, GPIO_PinState, GPIO_PinState); //set pin values of the 4 pins we are controlling
void Step_Number_Update(void); //updates the step number to alternate between steps in full-step and half-step modes
void Full_Step_CW(void); //Implements Full-step clockwise direction
void Full_Step_CCW(void); //Implements Full-step counter clockwise direction
void Half_Step_CW(void); //Implements half-step clockwise direction
void Half_Step_CCW(void); //Implements half-step counter clockwise direction




//static void EXTILine14_Config(void); // configure the exti line4, for exterrnal button, WHICH BUTTON?


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

	Up_pressed=0;
	Down_pressed=0;
	Sel_pressed=0;
	Right_pressed = 0;
	Left_pressed = 0;
	TIM3_CCR = 3000;
	
	//initialize system clock and LCD
	HAL_Init();
	SystemClock_Config();   
	HAL_InitTick(0x0000); // set systick's priority to the highest.
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI); 

 //initialize timers
  TIM3_Config();     
	
  while (1)
  {

    HAL_Delay(100);
	
	} //end of while 1
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */

void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;            
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  
	//the PLL will be MSI (4Mhz)*N /M/R = 

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
	
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz??


void  TIM3_Config(void)
{
	Tim3_PrescalerValue=(uint16_t)(SystemCoreClock/ 10000) - 1;  // timer3 runs at 10Khz, 
	TIM3_CCR= TIM3_CCR;   //it will take 1s for interrupt to happen
	/* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3;   
 	Tim3_Handle.Init.Period = 65000;  //does no matter, so set it to max. 
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	//config the OC
	Tim3_OCInitStructure.OCMode=  TIM_OCMODE_TIMING; //what is TIM_OCMODE_ACTIVE??
	Tim3_OCInitStructure.Pulse=TIM3_CCR;
	Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
	
	if (HAL_TIM_OC_Init(&Tim3_Handle)!=HAL_OK) { // if TIM3 has not been set, this line will call the callback function _MspInit() 
													//in stm32XXX_hal_msp.c to set up peripheral clock and NVIC.
													//this line has the same function as HAL_TIM_Base_Init
				Error_Handler();
	}
		
	if (HAL_TIM_OC_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_1) !=HAL_OK) {
				Error_Handler();
	}
	if (HAL_TIM_OC_Start_IT(&Tim3_Handle, TIM_CHANNEL_1)!=HAL_OK) { //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
						Error_Handler();
	}			
}

void Set_Pin_Values(GPIO_PinState A_State, GPIO_PinState B_State, GPIO_PinState C_State, GPIO_PinState D_State){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, A_State); //Set pin PE12 to value in A_State
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, B_State); //Set pin PE13 to value in B_State
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, C_State); //Set pin PE14 to value in C_State
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, D_State);	//Set pin PE15 to value in D_State		 
}


void DisplayInfo(){
	
		if(Sel_pressed == 0){ //If in full-step mode
				sprintf((char*) lcd_buffer, "FStep%d", Full_Step_Number);
				BSP_LCD_GLASS_DisplayString((uint8_t *)&lcd_buffer); //display full-step number on lcd		
		}else{ //if in half-step mode
				sprintf((char*) lcd_buffer, "HStep%d", Half_Step_Number);
				BSP_LCD_GLASS_DisplayString((uint8_t *)&lcd_buffer); //display half-step number on lcd
		}
}

void Step_Number_Update(){
	
		if(Sel_pressed == 0){ //if in full-step mode
					if (Full_Step_Number < 3){ //Max step number in full-step mode is 3. [0,3]
							Full_Step_Number++;
					}else{ 
							Full_Step_Number = 0;
					}
		}else{ //if in half-step mode
					if (Half_Step_Number < 7){ //Max step number in half-step mode is 7. [0,7]
							Half_Step_Number++;
					}else{
							Half_Step_Number = 0;
					}
		}
}

void Full_Step_CW(){
	
						switch (Full_Step_Number) { 
							case 0: 
										Set_Pin_Values(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET); //Set pin PE12 to high (A)
										DisplayInfo();
										Step_Number_Update();
										break;	
							case 1:    
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); //Set pin PE14 to high (C)
										DisplayInfo();
										Step_Number_Update();
										break;
							case 2:
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET); //Set pin PE13 to high (B)
										DisplayInfo();
										Step_Number_Update();					
										break;
							case 3:   
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); //Set pin PE15 to high (D)
										DisplayInfo();
										Step_Number_Update();						
										break;
							default:
										break;
						}	
}


void Full_Step_CCW(){
	
						switch (Full_Step_Number) {
							case 0: 
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET); //Set pin PE13 to high (B)
										DisplayInfo();
										Step_Number_Update();
										break;	
							case 1:    
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); //Set pin PE14 to high (C)
										DisplayInfo();
										Step_Number_Update();
										break;
							case 2:
										Set_Pin_Values(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET); //Set pin PE12 to high (A)
										DisplayInfo(); 
										Step_Number_Update();					
										break;
							case 3:   
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); //Set pin PE15 to high (D)
										DisplayInfo();
										Step_Number_Update();						
										break;
							default:
										break;
						}
}


void Half_Step_CW(){
	
						switch (Half_Step_Number) {
							case 0: 
										Set_Pin_Values(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET); //Set pin PE12 to high (A)
										DisplayInfo();
										Step_Number_Update();
										break;	
							case 1:  
										Set_Pin_Values(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); //Set pin PE12 to high (A)
										DisplayInfo();																														  //Set pin PE14 to high (C)
										Step_Number_Update();
										break;
							case 2: 
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); //Set pin PE14 to high (C)
										DisplayInfo();																																
										Step_Number_Update();						
										break;
							case 3: 
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET); //Set pin PE13 to high (B)
										DisplayInfo();																															//Set pin PE14 to high (C)
										Step_Number_Update();						
										break;
							case 4:
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET); //Set pin PE13 to high (B)
										DisplayInfo();
										Step_Number_Update();		
										break;
							case 5:
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET); //Set pin PE13 to high (B)
										DisplayInfo();																															//Set pin PE15 to high (D)
										Step_Number_Update();									
										break;
							case 6:
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); //Set pin PE15 to high (D)
										DisplayInfo();																													
										Step_Number_Update();										
										break;
							case 7:
										Set_Pin_Values(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); //Set pin PE12 to high (A)
										DisplayInfo();																															//Set pin PE15 to high (D)
										Step_Number_Update();										
										break;
							default:
										break;
						}	
}

void Half_Step_CCW(){
	
						switch (Half_Step_Number) {
							case 0: 
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET); //Set pin PE13 to high (B)
										DisplayInfo();
										Step_Number_Update();
										break;	
							case 1:  
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET); //Set pin PE13 to high (B)
										DisplayInfo();																															//Set pin PE14 to high (C)
										Step_Number_Update();
										break;
							case 2:
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); //Set pin PE14 to high (C)
										DisplayInfo();
										Step_Number_Update();						
										break;
							case 3: 
										Set_Pin_Values(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); //Set pin PE14 to high (C)
										DisplayInfo();																															//Set pin PE12 to high (A)
										Step_Number_Update();						
										break;
							case 4:
										Set_Pin_Values(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET); //Set pin PE12 to high (A)
										DisplayInfo();
										Step_Number_Update();		
										break;
							case 5:
										Set_Pin_Values(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); //Set pin PE12 to high (A)
										DisplayInfo();																															//Set pin PE15 to high (D)
										Step_Number_Update();									
										break;
							case 6:
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); //Set pin PE15 to high (D)
										DisplayInfo();
										Step_Number_Update();										
										break;
							case 7:
										Set_Pin_Values(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET); //Set pin PE15 to high (D)
										DisplayInfo();																															//Set pin PE13 to high (B)
										Step_Number_Update();										
										break;
							default:
										break;
						}	
}



/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
						Sel_pressed=(Sel_pressed+1)%2; //Sel_Pressed can only be 0 or 1 which allows us to alternate between full-step and half-step
						if (Sel_pressed == 0){ //If in full-step mode
								Full_Step_Number = Half_Step_Number / 2; //Full_Step requires half the number of step of half-step
						}else{ //If in half-step mode
								Half_Step_Number = 2 * Full_Step_Number; //half-step requires twice the number of steps of full-step
						}
						break;	
			case GPIO_PIN_1:     //left button
							break;
			case GPIO_PIN_2:    //right button						 
						Right_pressed =(Right_pressed+1)%2; //right button decided whether motor turns in CW or CCW directions 
							break;
			case GPIO_PIN_3:    //up button to increase speed
							if (TIM3_CCR >= 10000){
								TIM3_CCR = TIM3_CCR - 5000; //decreasing CCR to interrupt more frequently -> Stepping will happen faster 
							}else if (TIM3_CCR > 2000){
								TIM3_CCR = TIM3_CCR - 1000;
							}else{
								TIM3_CCR = 2000;
							}
							__HAL_TIM_SET_COMPARE(&Tim3_Handle, TIM_CHANNEL_1, TIM3_CCR); //set the new CCR
							break;
			
			case GPIO_PIN_5:    //down button to decrease the speed						
							if(TIM3_CCR < 50000){
									TIM3_CCR = TIM3_CCR + 5000; //increasing CCR to interrupt less frequently -> stepping will happen slower
							}else{
									TIM3_CCR = 50000;
							}
							__HAL_TIM_SET_COMPARE(&Tim3_Handle, TIM_CHANNEL_1, TIM3_CCR); //set the new CCR
							break;
			default://
						//default
						break;
	  } 
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer4 
		if ((*htim).Instance==TIM3){
				BSP_LED_Toggle(LED5);
		
				if (Sel_pressed == 0){// full step mode 
						if( Right_pressed ==0){//turn clockwise (default)
							Full_Step_CW();
							BSP_LED_On(LED5); //turn yellow LED on if turning clockwise 
							BSP_LED_Off(LED4);
						}else{ //right pressed is one, turn CCW
							Full_Step_CCW();
							BSP_LED_Off(LED5);
							BSP_LED_On(LED4); //turn red LED on if turning counter clockwise
						}
				}else{ //half step mode , sel_pressed = 1
						if( Right_pressed ==0){//turn clockwise (default)
							Half_Step_CW();
							BSP_LED_On(LED5);
							BSP_LED_Off(LED4);
						}else{ //right pressed is one, turn CCW
							Half_Step_CCW();
							BSP_LED_Off(LED5);
							BSP_LED_On(LED4);
						}						
				}		
				__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h
		}
	__HAL_TIM_SET_COUNTER(htim, 0x0000); //Reset counter 			
}
 


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
void assert_failed(char *file, uint32_t line)
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
