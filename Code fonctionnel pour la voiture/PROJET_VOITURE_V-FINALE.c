/*
================================================================================
  DATE                  15 Décembre 2018 - 30 Mai 2018

  NAME                  SHERBY
  DESCRIPTION           PROJET VOITURE TELECOMMANDEE BLUETOOTH M1 ISEN-TOULON

  GROUPE                 MORANA - PITARD - PRUVOST - VIELFAURE
================================================================================
*/

#include "stm32l1xx.h"

/*=============================================================================*/
/*==================================VARIABLES =================================*/
/*=============================================================================*/

    char chaine; 
    uint32_t etat_led = 0;
    uint32_t etat_led2 = 0; 
    int m = 0; 
    float distance ;
    uint16_t largeurEcho;
    uint8_t var = 0; 
    uint32_t delay_us = 20000; 
    int compteur = 0;  
  
    
    
/*=============================================================================*/
/*============================ INITIALISATION GPIO ============================*/
/*=============================================================================*/
    
void GPIO_INIT (void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Alimentation GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;                                          // Alimentation GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Alimentation GPIOC
}



/*=============================================================================*/
/*============================ INITIALISATION USART ===========================*/
/*=============================================================================*/

void INIT_USART(void)
{    
    // PC11 en ALTERNATE MODE : Réception de données via l'USART3 (pin droite) 
    GPIOC->MODER |= GPIO_MODER_MODER11_1;                                       // Définition du mode 
    GPIOC->MODER &=~ GPIO_MODER_MODER11_0;                                      
    GPIOC->OSPEEDR |= (1<<23)|(1<<22);                                          // High Speed
    GPIOC->OTYPER |= (1<<11);                                                   // Open drain
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR11_0;                                      // No pull up, No pull down
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR11_1;
    
    
    // PC10 en ALTERNATE MODE : Transmission de données via l'USART3 (pin gauche) 
    GPIOC->MODER |= GPIO_MODER_MODER10_1;                                       // Définition du mode 
    GPIOC->MODER &=~ GPIO_MODER_MODER10_0;      
    GPIOC->OSPEEDR |= (1<<21)|(1<<20);                                          // High Speed
    GPIOC->OTYPER |= (1<<10);                                                   // Open drain
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR10_0;                                       // Pull up 
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR10_1; 
       
    GPIOC->AFR[1] |= 0x00007700;                                                // AF7 = 0111 pour USART3 
}       
  


/*=============================================================================*/
/*=========================== INITIALISATION MOTEURS ==========================*/
/*=============================================================================*/

void INIT_MOTEUR(void)
{
    // PB4 : Direction gauche 
    
    GPIOB->MODER |= GPIO_MODER_MODER4_1;                                        // Définition du mode : Alternate
    GPIOB->MODER &=~ GPIO_MODER_MODER4_0;
          
    GPIOB->OTYPER &=~ GPIO_OTYPER_OT_4;                                         // Push Pull
              
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_0;                                 // High Speed
    GPIOB->OSPEEDR &=~ GPIO_OSPEEDER_OSPEEDR4_1;
              
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR4_1;                                       // No pull up, no pull down
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR4_0;
    
    GPIOB->AFR[0] |= (1<<17);

    
    // PB6 : Marche arrière
          
    GPIOB->MODER |= GPIO_MODER_MODER6_1;                                        // Définition du mode : Alternate
    GPIOB->MODER &=~ GPIO_MODER_MODER6_0;
              
    GPIOB->OTYPER &=~ GPIO_OTYPER_OT_6;                                         // Push Pull
              
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_0;                                 // High Speed
    GPIOB->OSPEEDR &=~ GPIO_OSPEEDER_OSPEEDR6_1;
              
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR6_1;                                       // No pull up, no pull down
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR6_0;
              
    GPIOB->AFR[0] |= (1<<25);


    // PC7 : Direction droite 
  
    GPIOC->MODER |= GPIO_MODER_MODER7_1;                                        // Définition du mode : Alternate
    GPIOC->MODER &=~ GPIO_MODER_MODER7_0; 
  
    GPIOC->OTYPER &=~ GPIO_OTYPER_OT_7;                                         // Push Pull
  
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_0;                                 // High Speed
    GPIOC->OSPEEDR &=~ GPIO_OSPEEDER_OSPEEDR7_1;
              
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR7_1;                                       // No pull up, no pull down
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR7_0;
          
    GPIOC->AFR[0] |= (1<<29);
    
          
    // PB7 : Marche avant
    
    GPIOB->MODER |= GPIO_MODER_MODER7_1;                                        // Définition du mode : Alternate
    GPIOB->MODER &= ~ GPIO_MODER_MODER7_0;
          
    GPIOB->OTYPER &=~ GPIO_OTYPER_OT_7;                                         // Push Pull
  
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_0;                                 // High Speed
    GPIOB->OSPEEDR &=~ GPIO_OSPEEDER_OSPEEDR7_1;
              
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR7_1;                                       // No pull up, no pull down
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR7_0;
          
    GPIOB->AFR[0] |= (1<<29);
}        
    


/*=============================================================================*/
/*============================ INITIALISATION LEDS ============================*/
/*=============================================================================*/

void INIT_LEDS(void)
{
    // PB1, PB2, PB8, PB9, PB11, PB14, PB15
    
    GPIOB->MODER |= GPIO_MODER_MODER15_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER1_0;
    GPIOB->MODER &= (~(GPIO_MODER_MODER15_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER1_0));
    
    GPIOB->OTYPER &= (~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_2 | GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_15 | GPIO_OTYPER_OT_14 | GPIO_OTYPER_OT_1 ));
    
    GPIOB->OSPEEDR &= (~(GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR9_0 | GPIO_OSPEEDER_OSPEEDR2_0 | GPIO_OSPEEDER_OSPEEDR11_0 | GPIO_OSPEEDER_OSPEEDR15_0 | GPIO_OSPEEDER_OSPEEDR14_0 | GPIO_OSPEEDER_OSPEEDR1_0)); 
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR11_1 | GPIO_OSPEEDER_OSPEEDR15_1 | GPIO_OSPEEDER_OSPEEDR14_1 | GPIO_OSPEEDER_OSPEEDR1_1);
    
    GPIOB->PUPDR &= (~(GPIO_PUPDR_PUPDR15_0 | GPIO_PUPDR_PUPDR14_0 | GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR1_0));
    GPIOB->PUPDR &= (~(GPIO_PUPDR_PUPDR15_1 | GPIO_PUPDR_PUPDR14_1 | GPIO_PUPDR_PUPDR11_1 | GPIO_PUPDR_PUPDR9_1 | GPIO_PUPDR_PUPDR8_1 | GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR1_1));
    
    // PC5
       
    GPIOC->MODER |= GPIO_MODER_MODER5_0;
    GPIOC->MODER &=~ GPIO_MODER_MODER5_1;
     
    GPIOC->OTYPER &=~ GPIO_OTYPER_OT_5; 
          
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_1;
    GPIOC->OSPEEDR &=~ GPIO_OSPEEDER_OSPEEDR5_0; 
                
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR5_1|GPIO_PUPDR_PUPDR5_0);      
}



/*=============================================================================*/
/*=========================== INITIALISATION BUZZERS ==========================*/
/*=============================================================================*/
    
void INIT_BUZZER(void)
{
    GPIOB->MODER |= GPIO_MODER_MODER12_1;
    GPIOB->MODER &=~ GPIO_MODER_MODER12_0;
    
    GPIOB->OTYPER &=~ GPIO_OTYPER_OT_12;
    
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR12_1;
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR12_0;
    
    GPIOB->AFR[1]|=0x00010000;
}                     
    


/*=============================================================================*/
/*=========================== INITIALISATION CAPTEUR ==========================*/
/*=============================================================================*/

void INIT_CAPTEUR(void)
{
    // Capteur ultrason avant trigger : PB5
    
    GPIOB->MODER &=~ GPIO_MODER_MODER5_1;                                        // Définition du mode : Output
    GPIOB->MODER |= GPIO_MODER_MODER5_0;
   
    GPIOB->OTYPER &=~ GPIO_OTYPER_OT_5;                                          // Push-pull 
   
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR5_1;                                        // No pull up, no pull down
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR5_0; 
   
    // Capteur ultrason avant echo : PA9

    GPIOA->MODER &=~(GPIO_MODER_MODER9_1|GPIO_MODER_MODER9_0);                   // Définition du mode : Input
   
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9_1|GPIO_PUPDR_PUPDR9_0);                  // No pull up, no pull down
   
    // Capteur ultrason arrière echo : PA12

    /* GPIOA->MODER &= (~(1<<5)|(1<<4));    // PA2 input mode // input2 echo
   GPIOA->PUPDR &= (~(1<<5)|(1<<4));    // no pull up, no pull down */
   
   // Capteur ultrason arrière trigger : PB3

    /* GPIOB->MODER &=~(1<<7);    // PB3 output mode trigger
   GPIOB->MODER |= (1<<6);
   GPIOB->OTYPER &=~(1<<3);    //push-pull 
   GPIOB->PUPDR &= ~(1<<7);    // no pull up, no pull down
   GPIOB->PUPDR &= ~(1<<6); */
}
   


/*=============================================================================*/
/*========================== INITIALISATION BATTERIE ==========================*/
/*=============================================================================*/
   
void INIT_BATTERIE(void)
{
    // LED : PA6, PA7, PA8
    
    GPIOA->MODER &= ~(GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1|GPIO_MODER_MODER8_1);
    GPIOA->MODER |= GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0; 
    
    GPIOA->OTYPER &= (~(GPIO_OTYPER_OT_6| GPIO_OTYPER_OT_7| GPIO_OTYPER_OT_8)); 
    
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR8_1 ;
    GPIOA->OSPEEDR &= (~(GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR8_0));
    
    GPIOA->PUPDR &= (~(GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR7_1 | GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1)); 
    
    // E5V : PA0
    
    GPIOA->MODER |= GPIO_MODER_MODER0_1; 
    GPIOA->MODER |= GPIO_MODER_MODER0_0;
   
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1;
    GPIOA->OSPEEDR &=~ GPIO_OSPEEDER_OSPEEDR0_0;
    
    GPIOA->OTYPER |= GPIO_OTYPER_OT_0;                                          // Open drain 
    
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR0_1);
    
    GPIOA->AFR[0] &= (~(1<<1)|(1<<2)|(1<<3)); 
    GPIOA->AFR[0] |= (1<<0);    
}



/*=============================================================================*/
/*============================ INITIALISATION TIM2 ============================*/
/*============================== Gestion des leds =============================*/
/*=============================================================================*/

void TIM2_CONFIG(void) 
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                                         // Activation du Timer 2

    TIM2->PSC = 15999;                                                          // Clignotement toutes les 500 ms
    TIM2->ARR = 299;
    
    TIM2->CCR1 = 1;                                                             // TIM2_CH1
    TIM2->CCR2 = 1;                                                             // TIM2_CH2
    
    TIM2->CR1 |= TIM_CR1_ARPE;    
    TIM2->CR1 &=~ TIM_CR1_CMS_1;
    TIM2->CR1 &=~ TIM_CR1_CMS_0;    
    TIM2->CR1 &=~ TIM_CR1_DIR; 

    TIM2->CCMR1|= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;       //TIM2_CH1
    TIM2->CCMR1|= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;       //TIM2_CH2
     
    TIM2->DIER |= TIM_DIER_UIE;
    
    NVIC->ISER[0] |= NVIC_ISER_SETENA_28; 
  
}



/*=============================================================================*/
/*============================ INITIALISATION TIM3 ============================*/
/*======================= Gestion des moteurs direction =======================*/
/*=============================================================================*/
 
void TIM3_CONFIG(void) 
{
    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;                                       // Activer le Timer 3
   
    TIM3->PSC = 32000;                                                          // Départ voiture marche avant : 500 us
    TIM3->ARR = 200;
    
    TIM3->CCR1 =1;                                                              // TIM3_CH1
    TIM3->CCR2 = 1;                                                             // TIM3_CH2
       
    TIM3->CR1 |= TIM_CR1_ARPE;    
    TIM3->CR1 &=~ TIM_CR1_CMS_1; 
    TIM3->CR1 &=~ TIM_CR1_CMS_0;     
    TIM3->CR1 &=~ TIM_CR1_DIR; 
       
    TIM3->CCMR1|= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;       // TIM3_CH1
    TIM3->CCMR1|= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;       // TIM3_CH2
 
}



/*=============================================================================*/
/*============================ INITIALISATION TIM4 ============================*/
/*======================= Gestion des moteurs traction ========================*/
/*=============================================================================*/
 
void TIM4_CONFIG(void)  
{
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;                                           // Activer le Timer 4
  
  TIM4->PSC = 32000;                                                            // Départ voiture marche arrière : 500us
  TIM4->ARR = 200;
  
  TIM4->CCR1 = 1;
  TIM4->CCR2= 1;
  
  TIM4->CR1 |= TIM_CR1_ARPE; 
  TIM4->CR1 &=~ TIM_CR1_CMS_1; 
  TIM4->CR1 &=~ TIM_CR1_CMS_0;
  TIM4->CR1 &=~ TIM_CR1_DIR; 
   
  TIM4->CCMR1|= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;         //TIM4_CH1
  TIM4->CCMR1|= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;         //TIM4_CH2
}



/*=============================================================================*/
/*============================ INITIALISATION TIM6 ============================*/
/*========================== Gestion de la batterie ===========================*/
/*=============================================================================*/
 
void TIM6_CONFIG(uint32_t delay_us)
{
  RCC-> APB1ENR |= RCC_APB1ENR_TIM6EN;
  TIM6->PSC = 15;                                                               // f = 1MHz soit T = 1us
  TIM6->ARR = delay_us - 1;
  
  TIM6->CR1 |= TIM_CR1_CEN; 
  TIM6->CR1 |= TIM_CR1_URS; 
  TIM6->CR1 |= TIM_CR1_ARPE; 
  
  TIM6->CR1 &=~ TIM_CR1_DIR;
  TIM6->CR1 &=~ TIM_CR1_CMS_0;
  TIM6->CR1 &=~ TIM_CR1_CMS_1;
}



/*=============================================================================*/
/*============================ INITIALISATION TIM9 ============================*/
/*====================== Gestion capteur ultrason avant =======================*/
/*=============================================================================*/

void TIM9_CONFIG(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;                                           // Activation du Timer 9
   
  TIM9->PSC = 16;                                                               // F = 1 MHz => 1 us
  
  TIM9->CR1 |= TIM_CR1_ARPE; 
  TIM9->CR1 &=~ TIM_CR1_DIR | TIM_CR1_CMS_0 |TIM_CR1_CMS_1;                     // Upcounter
}



/*=============================================================================*/
/*=========================== INITIALISATION TIM10 ============================*/
/*===================== Gestion capteur ultrason arriere ======================*/
/*=============================================================================*/

void TIM10_CONFIG(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; 
  
  TIM10->PSC = 16; 
  
  TIM10->CR1 |= TIM_CR1_ARPE; 
  TIM10->CR1 &=~ TIM_CR1_DIR | TIM_CR1_CMS_0 |TIM_CR1_CMS_1;  
}



/*=============================================================================*/
/*========================= CALCUL DISTANCE OBSTACLE ==========================*/
/*=============================================================================*/

void calcul_Distance(uint16_t largeur)
{
  distance = largeur/58.8235;                                                   // 17/100 = 58.8235 => distance en cm/us 
 
}



/*=============================================================================*/
/*======== FONCTION ATTENTE POUR LE TRIGGER DU CAPTEUR ULTRASON AVANT =========*/
/*=============================================================================*/

void wait_TIM9_10us(void)
{
  TIM9->ARR = 9;
  TIM9->CR1 |= TIM_CR1_CEN;                                                     // On lance le compteur
  while( (TIM9->SR & TIM_SR_UIF)==0);                                           // On attend que mis a un 1 par le hardware donc détection de quelque chose par l'ultrason
  TIM9->SR &= ~TIM_SR_UIF;                                                      // Remise a zéro pour pouvoir refaire une détection après
  TIM9->CR1 &=~TIM_CR1_CEN;                                                     // On éteint le compteur
}



/*=============================================================================*/
/*======= FONCTION ATTENTE POUR LE TRIGGER DU CAPTEUR ULTRASON ARRIERE ========*/
/*=============================================================================*/

void wait_TIM10_10us(void)
{
  TIM10->ARR = 9;                                           
  TIM10->CR1 |= TIM_CR1_CEN;                                                    // On lance le compteur 
  while( (TIM10->SR & TIM_SR_UIF)==0);                                          // On attend que mis a un 1 par le hardware donc détection de quelque chose par l'ultrason
  TIM10->SR &= ~TIM_SR_UIF;                                                     // Remise a zéro pour pouvoir refaire une détection après
  TIM10->CR1 &=~TIM_CR1_CEN;                                                    // On éteint le compteur
}



/*=============================================================================*/
/*============== INTERRUPTION EXTERNE SUR ECHO DU CAPTEUR AVANT ===============*/
/*=============================================================================*/

void ConfigInterrupt_PA9(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;                                         // Clock syscfg enable
 
  SYSCFG->EXTICR[2] |= 0x0000;                                                  // Autorisation de l'interruption sur le port A
  EXTI->FTSR |= EXTI_FTSR_TR9;                                                  // Autorisation de l'interruption sur front descendant 
  EXTI->IMR |= EXTI_IMR_MR9;                                                    // Autorise l'interruption sur PA9

}



/*=============================================================================*/
/*============= INTERRUPTION EXTERNE SUR ECHO DU CAPTEUR ARRIERE ==============*/
/*=============================================================================*/

void ConfigInterrupt_PA2(void)
{
  SYSCFG->EXTICR[0] |= 0x0000;                                                  // Autorisation de l'interruption sur le port A
  EXTI->FTSR |= EXTI_FTSR_TR2;                                                  // Autorite l'interruption sur front descendant 
  EXTI->IMR |= EXTI_IMR_MR2;                                                    // Autorise l'interruption sur PA3
}



/*=============================================================================*/
/*=================== GENRATION TRIGGER POUR CAPTEUR AVANT ====================*/
/*=============================================================================*/

void Generation_Trigger_avant(void)                                             // Generation du trigger pour le capteur avant sur PB5
{

  GPIOB->ODR |= GPIO_ODR_ODR_5;                                                 // Trigger a 1
  wait_TIM9_10us();   
  GPIOB->ODR &=~ GPIO_ODR_ODR_5;                                                // Trigger a 0
}



/*=============================================================================*/
/*================== GENRATION TRIGGER POUR CAPTEUR ARRIERE ===================*/
/*=============================================================================*/

void Generation_Trigger_arriere(void)                                           // Generation du trigger pour le capteur arriere sur PB3
{
  GPIOB->ODR |= GPIO_ODR_ODR_3;                                                 // Trigger a 1
  wait_TIM10_10us();  
  GPIOB->ODR &=~ GPIO_ODR_ODR_3;                                                // Trigger a 0
}



/*=============================================================================*/
/*============================= INITIALISATION ADC ============================*/
/*=========================== Gestion de la batterie ==========================*/
/*=============================================================================*/

void ADC_INIT (void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 
  RCC->APB1ENR |= RCC_APB1ENR_COMPEN; 
  RCC->CR |= RCC_CR_HSION;                                                      // Horloge HSI on
  ADC->CCR &=~ ADC_CCR_ADCPRE_0;                                                // HSI prescaler divise par 4
  ADC->CCR |= ADC_CCR_ADCPRE_1; 
  RCC->CFGR &=~ (RCC_CFGR_MCOSEL_2|RCC_CFGR_MCOSEL_0); 
  RCC->CFGR |= RCC_CFGR_MCOSEL_1; 
  ADC1->CR1 &= ~(ADC_CR1_AWDCH_0 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_4 );
  ADC1->CR1 &=~ ADC_CR1_SCAN;                                                   // Vérifie que la conversion a eu lieu (scan)
  ADC1->CR1 |= ADC_CR1_RES_1;                                                   // Résolution sur 6 bit
  ADC1->CR1 |= ADC_CR1_RES_0; 
  RI->ICR &=~ (RI_ICR_IC1Z_0 | RI_ICR_IC1Z_1 | RI_ICR_IC1Z_2 | RI_ICR_IC1Z_3);  // Roouting interface
  ADC1->SQR1 &=~ ADC_SQR1_L;  // 1 voie de convertion
  ADC1->SQR5 &=~ (ADC_SQR5_SQ1_0 | ADC_SQR5_SQ1_1 | ADC_SQR5_SQ1_2 | ADC_SQR5_SQ1_3 | ADC_SQR5_SQ1_4);  // Première conversion
  ADC1->CR2 &=~ ADC_CR2_CFG;                                                    // Bank A
  ADC1->CR2 |= ADC_CR2_CONT;                                                    // Continous mode
  RI->ASCR1 &=~RI_ASCR1_CH_0;                                                   // Analog switch controlled by ADC
  ADC1->CR2 |= ADC_CR2_ADON;                                                    // ADON ADC power on
  while ((ADC1->SR & ADC_SR_ADONS)==0);                                         // Tant que l'ADC n'est pas prêt

}



/*=============================================================================*/
/*============================ FONCTION ACQUISITION ===========================*/
/*============== Déclenche la conversion du niveau de la batterie =============*/
/*=============================================================================*/

void acquisition(void)
{
  ADC1->CR2 |= ADC_CR2_SWSTART;                                                 // Start conversion
  while ((ADC1->SR & ADC_SR_EOC )==0);                                          // Tant que la conversion n'est pas fini j'attend
  ADC1->SR |= ADC_SR_EOC;                                                       // Fin de la conversion
  var = ADC1->DR;                                                               // On stocke dans var la valeur actuelle du data register
}



/*=============================================================================*/
/*============================ GESTION LED BATTERIE ===========================*/
/*=============================================================================*/

void LED (void)
{
  if (var >=62)                                                                 // LED verte s'allume
  {
    GPIOA->ODR |= GPIO_ODR_ODR_6;                                               // Active LED Verte
    GPIOA->ODR &=~ (GPIO_ODR_ODR_7 | GPIO_ODR_ODR_8);                           // Désactive les LEDS Orange et Rouge
  }
  else
  {
    if(var < 62 && var > 60)                                                    //LED orange s'allume
    {
      GPIOA->ODR &=~ (GPIO_ODR_ODR_6 | GPIO_ODR_ODR_8);                         // Désactive les LEDS verte et rouge
      GPIOA->ODR |= GPIO_ODR_ODR_7;                                             // Active LED Orange
    }
    else
    {
      if(var <= 60)                                                             // LED rouge s'allume
      { 
        GPIOA->ODR &=~ (GPIO_ODR_ODR_6 | GPIO_ODR_ODR_7);                       // Désactive les LEDS verte et orange
        GPIOA->ODR |= GPIO_ODR_ODR_8 ;                                          // Active LED Rouge
      }
    }
  }
  
}



/*=============================================================================*/
/*======= INITIALISATION DE L'USART POUR LA COMMUNICATION PAR BLUETOOTH =======*/
/*=============================================================================*/

void USART_CONFIG(void)
{

  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;                                         // Clock USART3 enable 

  USART3->CR1 &=~ USART_CR1_OVER8;                                              // Oversampling on 16bits 
  USART3->CR1 &=~ USART_CR1_M;                                                  // start bits 8 data bits 
  USART3->CR2 &= ~(USART_CR2_STOP_1|USART_CR2_STOP_0);                          // 1 stop bit 
  USART3->BRR = 0x658;                                                          // Pour un baudrate de 9600 bauds, USART_DIV = 0d208.3125 , DIV_FR = 0x5, DIV_MANTISSA= 0xD0 pour 16Mhz
  USART3->CR1 |= USART_CR1_RXNEIE;                                              // RXNE interruption enable 
  NVIC->ISER[1] |= NVIC_ISER_SETENA_7 ;                                         // Interrupt generation for USART3 
     
  USART3->CR1 |= USART_CR1_UE;                                                  // USART enable  
}



/*=============================================================================*/
/*========================= TRANSMISSION DE LA DONNEE =========================*/
/*=============================================================================*/

void TX_BYTE(char data)
{
  USART3->CR1 |= USART_CR1_TE;                                                  // Transmittion enable 

  USART3->DR = data;                                                            // Writtng the data in the register for transmition 
  
  while((USART3->SR & USART_SR_TC)==0);                                         // J'attend tant que la donnée n'a pas été transferé
}
   

int main(void)
{
  GPIO_INIT();
  INIT_USART();
  INIT_MOTEUR();
  INIT_LEDS();
  INIT_BUZZER();
  INIT_CAPTEUR();
  INIT_BATTERIE();
  
  USART_CONFIG(); 
  
  TIM2_CONFIG();
  TIM3_CONFIG();
  TIM4_CONFIG();
  TIM6_CONFIG(delay_us); 
  TIM9_CONFIG(); 
  
  ConfigInterrupt_PA9();
  //TIM10_CONFIG();
 // ConfigInterrupt_PA2();
  
 
  ADC_INIT();  
   
    USART3->CR1 |= USART_CR1_RE;                                                // Receive enable
    
    GPIOB->ODR &=~ GPIO_ODR_ODR_5 ;                                             // Trigger à 0 sur PB5 capteur distance avant
    //GPIOA->ODR &=~ GPIO_ODR_ODR_3;                                            // Trigger à 0 sur PA3 capteur distance arrière     
    
    
  while(1)
  {   
      // Gestion capteur ultrason avant
      Generation_Trigger_avant(); 
      
      // Gestion capteur ultrason arriere
      // Generation_Trigger_arriere();
      
      // Incrémentation du compteur
      compteur++;
      
      // Autorisation des interruptions
      NVIC->ISER[0] |= NVIC_ISER_SETENA_23;                                     // Interruption EXTI9_5_IRQHandler & EXTI2_IRQHandler enable
   
      calcul_Distance(largeurEcho); 
     
    
      if(distance < 3)
      {
               // Arret marche avant
               TIM4->CR1 &=~ TIM_CR1_CEN ;                                      // TIM4 counter disable 
               TIM4->CCER &= ~(1<<0);                                           // TIM4 CH2
      }
      else  
      {   
        
        chaine = USART3->DR ; 
        
          switch(chaine)
        {                    
        case '1' :  
                // Active la marche avant PB7 et LED PB9-PC5
                TIM4->CR1 |= TIM_CR1_CEN;                                       // TIM4 counter enable 
                TIM4->CCER |= TIM_CCER_CC1E;                                    // TIM4 CH1 enable 
                GPIOB->ODR |= (1<<8)|(1<<11);                                   // Allume les LEDS PB8-PB117
                break;
          
          
        }
      } 
      
      
      // Gestion de la batterie 
      acquisition();                                                            // Acquiert les valeurs de la tension sur PA0
      LED();                                                                    // Active les LEDS en fonction du niveau de tension 
       
  }
}