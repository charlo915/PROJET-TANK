/*
================================================================================
  DATE                  15 Décembre 2018 - 30 Mai 2018

  NAME                  SHERBY
  DESCRIPTION           PROJET VOITURE TELECOMMANDEE BLUETOOTH M1 ISEN-TOULON

  GROUPE                 MORANA - PITARD - PRUVOST - VIELFAURE
================================================================================
*/

#include "stm32l1xx.h"

void GPIO_INIT (void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                                          // Alimentation GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Alimentation GPIOC
}

void INIT_MOTEUR(void)
{
    // PB4 : Direction gauche 
    
    GPIOB->MODER &=~ GPIO_MODER_MODER4_1;                                        // Définition du mode : OUTPUT
    GPIOB->MODER |= GPIO_MODER_MODER4_0;
          
    GPIOB->OTYPER &=~ GPIO_OTYPER_OT_4;                                         // Push Pull
              
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_0;                                 // High Speed
    GPIOB->OSPEEDR &=~ GPIO_OSPEEDER_OSPEEDR4_1;
              
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR4_1;                                       // No pull up, no pull down
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR4_0;
    
    GPIOB->AFR[0] |= (1<<17);
    
    // PC7 : Direction droite 
  
    GPIOC->MODER &=~ GPIO_MODER_MODER7_1;                                        // Définition du mode : OUTPUT
    GPIOC->MODER |= GPIO_MODER_MODER7_0; 
  
    GPIOC->OTYPER &=~ GPIO_OTYPER_OT_7;                                         // Push Pull
  
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_0;                                 // High Speed
    GPIOC->OSPEEDR &=~ GPIO_OSPEEDER_OSPEEDR7_1;
              
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR7_1;                                       // No pull up, no pull down
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR7_0;
          
    GPIOC->AFR[0] |= (1<<29);
    
    // PB3 : ENABLE du driver 
    
    GPIOB->MODER &=~ GPIO_MODER_MODER3_1;                                        // Définition du mode : OUTPUT
    GPIOB->MODER |= GPIO_MODER_MODER3_0;
          
    GPIOB->OTYPER &=~ GPIO_OTYPER_OT_3;                                         // Push Pull
              
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3_0;                                 // High Speed
    GPIOB->OSPEEDR &=~ GPIO_OSPEEDER_OSPEEDR3_1;
              
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR3_1;                                       // No pull up, no pull down
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR3_0;
    
    // GPIOB->AFR[0] |= (1<<17);                                                on est en Output pas de AFR
    
}

void init_timer(){
  
    // Alimentation du périférique TIMER
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    // Initialisation de son prescaler (Timer 2 à 1 kHz -> 1 ms)
    TIM2->PSC = 15999;
    
    // Initialisation de son l'auto-reload (Auto-reload programmé pour 1 ms)
    TIM2->ARR = 1;
    
    // Activation du préchargement de l'auto-reload (Changement d'auto-reload pris en compte seulement lors de l'événement suivant)
    TIM2->CR1 |= TIM_CR1_ARPE;
    
    // On indique que le compteur s'actualise sur front montant ou descendant
    TIM2->CR1 &= ~TIM_CR1_CMS;
    
    // On indique que le compteur s'actualise sur front montant
    TIM2->CR1 &= ~TIM_CR1_DIR;  
}

void wait_t()
{
    // Lancement du compteur
    TIM2->CR1 |= TIM_CR1_CEN;
    
    // On attend que le compteur atteint la valeur d'auto-reload
    while ((TIM2->SR&TIM_SR_UIF)==0);
    
    // Arrêt du compteur
    TIM2->CR1&=~TIM_CR1_CEN;
    
    // On remet à 0 l'indicateur d'auto-reload
    TIM2->SR&=~TIM_SR_UIF;
    
    // Remise à 0 du compteur
    TIM2->CNT=0;
    
}

void tempo(int32_t n)
{
    // On répète 'n' fois la temporisation de 1 ms
    for (int32_t i = 0; i < n; i++)
        wait_t();
}

int main(void)
{
  GPIO_INIT();
  INIT_MOTEUR();
  init_timer();
  GPIOB->ODR |= GPIO_ODR_ODR_3;  // permet d'enable le driver 

   while(1)
  {   
    // Tout a 0
    GPIOB->ODR &=~ GPIO_ODR_ODR_4;
    GPIOC->ODR &=~ GPIO_ODR_ODR_7;
    tempo(500);
    printf("Bonjour\n");
    // Activation de la PB4 pendant 10s alors que PC7 est a 0
    GPIOB->ODR |= GPIO_ODR_ODR_4;
    GPIOC->ODR &=~ GPIO_ODR_ODR_7;
    tempo(10000);
     printf("Tout est ok \n");
    // On éteint tout donc tout a 0
    
    GPIOB->ODR &=~ GPIO_ODR_ODR_4;
    GPIOC->ODR &=~ GPIO_ODR_ODR_7;
    tempo(500);
    printf("Tout est éteint \n");
     // Activation de la PC7 pendant 10s alors que PB4 est a 0
     GPIOB->ODR &=~ GPIO_ODR_ODR_4;
     GPIOC->ODR |= GPIO_ODR_ODR_7;
     tempo(10000);
   printf("Parfait \n");
        }
       
  }
