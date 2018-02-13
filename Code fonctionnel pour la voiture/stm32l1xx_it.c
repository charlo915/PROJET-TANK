/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/stm32l1xx_it.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    16-May-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  
    
    extern uint32_t etat_led; 
    extern uint32_t etat_led2;
    extern uint8_t cnt_max;
    extern char chaine; 
    extern float distance; 

    extern int largeurEcho; 
    extern int compteur; 
    extern int m;
/* Private function prototypes -----------------------------------------------*/
 
 extern void RX_BYTE(void); 
 extern void wait (void); 
/* Private functions ---------------------------------------------------------*/




/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
extern uint8_t var;
/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_xx.s).                                            */
/******************************************************************************/
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/                         

////////////////// INTERRUPTION BLUETOOTH MANAGEMENT ////////////////

void USART3_IRQHandler(void)
{
        chaine = USART3->DR; 
   
        switch(chaine)
        {
        case '2' : //active la marche arriere PB6 et led PB8-PB11
                TIM4->CR1 |= TIM_CR1_CEN; //tim4 counter enable  
                TIM4->CCER |= TIM_CCER_CC2E; // tim4 ch2 enable
                GPIOB->ODR |= (1<<9); //turn the LED ON -PB9
                GPIOC->ODR |= (1<<5); // turn the LED ON -PC5
                GPIOB->ODR &= ~((1<<8)|(1<<11));
                //NVIC->ISER[0] |= (1<<23); 
                break;      
                
                
        case '1' :  
                //active la marche avant PB7 et led PB9-PC5
                TIM4->CR1 |= TIM_CR1_CEN; //tim4 counter enable 
                TIM4->CCER |= TIM_CCER_CC1E; // tim4_ch1 enable
                GPIOB->ODR |= (1<<8)|(1<<11); // turn the LED ON - PB8-PB117
                break;
        
        case '3' : //tourne à gauche 
                m=m+1;
                TIM3->CR1 |= TIM_CR1_CEN; // active compteur timer moteur
                TIM3->CCER |= TIM_CCER_CC2E; // tim4 ch2 enable 
                TIM2->CR1 |= TIM_CR1_CEN; // active compteur timer clignotement led
                TIM2->CCER |=TIM_CCER_CC2E; //tim2 ch2 enable
                break;
           
      
        case '4' : //tourne à droite
                m=0;
                TIM3->CR1 |= TIM_CR1_CEN; //tim3 counter enable
                TIM3->CCER |= TIM_CCER_CC1E; // tim3_ch1 enable
                TIM2->CR1 |= TIM_CR1_CEN; //active timer clignotement led
                TIM2->CCER |= TIM_CCER_CC1E; //tim2 ch1 enable     
                break;      
     
                
                
       case '5' : //arret moteurs traction + led         
                //arret marche avant
               TIM4->CR1 &=~ TIM_CR1_CEN; // tim4 counter disable 
               TIM4->CCER &=~ TIM_CCER_CC2E; //tim4 ch2 disable
               //arret marche arriere
               TIM4->CR1 &=~ TIM_CR1_CEN; //tim4 counter disable
               TIM4->CCER &=~TIM_CCER_CC1E; //tim4 ch1 disable
               //arret led               
               GPIOB->ODR &= ~((1<<8)|(1<<11)); // turn the LED OFF PB8-PB11 -marche avant
               GPIOB->ODR &=~(1<<9); //turn the LED OFF PB9 - marche arrière
               GPIOC->ODR &= ~(1<<5);
               break; 
            
        case '6' : // gestion du buzzer - activation 
                   TIM5->CR1 |= TIM_CR1_CEN; 
                   TIM5->CCER |= TIM_CCER_CC1E; 
                    break;   
               
      case '7' :            
               // arret moteur direction + led
                //arret direction droite
               TIM3->CR1 &=~ TIM_CR1_CEN; //tim3 counter disable
               TIM3->CCER &=~ TIM_CCER_CC1E;//tim3 ch1
               //arret direction gauche 
               TIM3->CR1 &=~ TIM_CR1_CEN; // tim3 counter disable 
               TIM3->CCER &=~ TIM_CCER_CC2E;//tim3 ch2 disable
               //arret clignotement led  
               TIM2->CR1 &=~TIM_CR1_CEN; // arret timer clignotement led 
               TIM2->CCER &= ~TIM_CCER_CC2E; // tim2 ch2 disable
               //arret led 
               GPIOB->ODR &=~ ((1<<2)|(1<<1)); 
               GPIOB->ODR &=~ ((1<<15)|(1<<14)); 
               break;
                
        case '8' : //gestion buzzer - arret 
          TIM5->CR1 &=~ TIM_CR1_CEN;
          TIM5->CCER &=~ TIM_CCER_CC1E; 
           
          break; 
               
     }
     
     
   
    USART3->SR &=~ USART_SR_RXNE; 
   
  
}

/////////////////// INTERRUPTION BLINKING LED ////////////////////////////

void TIM2_IRQHandler(void)
{
    TIM2->SR &=~ TIM_SR_UIF; 
    if(m==1)
    {
              if(etat_led==1)
                {      
                  GPIOB->ODR |= (1<<2)|(1<<1); //turn the LED PB1-PB2 ON
                  etat_led =0; 
                }
              else
                {
                  GPIOB->ODR &= ~(1<<2); //turn the LED PB1-PB2 OFF
                  GPIOB->ODR &= ~(1<<1);
                  etat_led = etat_led+1; 
                }
    }
    else
    {
              if(etat_led2==1)
              {
                GPIOB->ODR |= (1<<15); //turn the LED PB15-PB14 ON ((1<<15)|(1<<2));
                GPIOB->ODR |=(1<<14);
                etat_led2=0;
              }
              else
              {
                GPIOB->ODR &= ~(1<<15); //turn the LED PB15-PB14 OFF ((1<<15)|(1<<2));
                GPIOB->ODR &= ~(1<<14);
                etat_led2=etat_led2+1;
              }
    }
     
}

//////////////  INTERRUPTION FRONT ULTRASOUND SENSOR ///////////////////////

void EXTI9_5_IRQHandler(void)
{
  EXTI->PR |= EXTI_PR_PR9;
  largeurEcho = compteur;   
  compteur = 0;  
}

/*
void EXTI2_IRQHandler(void)
{

  EXTI->PR |= EXTI_PR_PR2;
  largeurEcho = compteur;
  compteur= 0;
}
*/
/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
