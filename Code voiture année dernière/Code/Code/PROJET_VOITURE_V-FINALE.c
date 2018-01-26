/* --------------------------------------------------
  *@date                  Janvier 2017
  *@name                  PROJET VOITURE TELECOMMANDEE BLUETOOTH M1 ISEN-TOULON
  *@author                Fabien Le Tellier
  *@group                 QUIGNON-ODE-DAVID-EL_KAMAR-LE_TELLIER
  *@ProjectOfficer        QUIGNON Jérémy       
-----------------------------------------------------*/
#include "stm32l1xx.h"

/*=============== VARIABLES DECLARATION ===============*/ 

    char chaine; 
    uint32_t etat_led = 0;
    uint32_t etat_led2 = 0; 
    int m = 0; 
    float distance ;
    uint16_t largeurEcho;
    uint8_t var = 0; 
    uint32_t delay_us = 20000; 
    int compteur = 0;  
  
/*================================================*/


/*=========================== I/O INITIALISATION =========*/
void GPIO_INIT (void)
{
    //clock GPIOA enable
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
    //activation de la clock pour le port B
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 
    //activation de la clock sur le port C 
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
    
    
    //PC11 en alternate mode pour la reception de données via l'USART3 (pin droite) 
    GPIOC->MODER |= GPIO_MODER_MODER11_1;//(1<<23); 
    GPIOC->MODER &=~ GPIO_MODER_MODER11_0;//(1<<22); 
    GPIOC->OSPEEDR |= (1<<23)|(1<<22); // high speed
    GPIOC->OTYPER |= (1<<11); //open drain
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR11_0;      //(1<<22); //no pull up no pull down
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR11_1;      //(1<<23); 
    
    //PC10 en alernate mode pour la transmission de données via l'USART3 (pin gauche) 
    GPIOC->MODER |= GPIO_MODER_MODER10_1;       //(1<<21);
    GPIOC->MODER &=~ GPIO_MODER_MODER10_0;      //(1<<20); 
    GPIOC->OSPEEDR |= (1<<21)|(1<<20); //high speed
    GPIOC->OTYPER |= (1<<10); //open drain
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR10_0;    //(1<<20); //pull up 
    GPIOC->PUPDR &=~ GPIO_PUPDR_PUPDR10_1; //(1<<21); 
       
    GPIOC->AFR[1] |= 0x00007700; //AF7:0111 pour usart3 
    
    
    ////////////////////// INITIALISATION MOTEURS ////////////////////////// 
            /*PB4*/ //direction gauche 
          GPIOB->MODER |= GPIO_MODER_MODER4_1;      //(1<<9);
          GPIOB->MODER &= ~ GPIO_MODER_MODER4_0;    //(1<<8);
          
          GPIOB->OTYPER &= ~ (1<<4);
          
          GPIOB->OSPEEDR &= ~(1<<9);
          GPIOB->OSPEEDR |= (1<<8);
          
          GPIOB->PUPDR &= ~ GPIO_PUPDR_PUPDR4_1; //(1<<9);
          GPIOB->PUPDR &= ~ GPIO_PUPDR_PUPDR4_0; //(1<<8);
          GPIOB->AFR[0] |= (1<<17);
        /****************************/
          
          /*PB6*/
          GPIOB->MODER |= GPIO_MODER_MODER6_1; //(1<<13);
          GPIOB->MODER &= ~ GPIO_MODER_MODER6_0; //(1<<12);
          
          GPIOB->OTYPER &= ~(1<<6);
          
          GPIOB->OSPEEDR &= ~(1<<13);
          GPIOB->OSPEEDR |= (1<<12);
          
          GPIOB->PUPDR &= ~ GPIO_PUPDR_PUPDR6_1; //(1<<13);
          GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR6_0; //(1<<12);
          
          GPIOB->AFR[0] |= (1<<25);
        /*****************************/
          /* PC7 */ //direction droite 
  
          GPIOC->MODER |= GPIO_MODER_MODER7_1; //(1<<15);
          GPIOC->MODER &= ~GPIO_MODER_MODER7_0; //(1<<14);
  
          GPIOC->OTYPER &= ~(1<<7);

  
          GPIOC->OSPEEDR &= ~(1<<15);
          GPIOC->OSPEEDR |= (1<<14);
          
          GPIOC->PUPDR &= ~ GPIO_PUPDR_PUPDR7_1; //(1<<15);
          GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR7_0; //(1<<14);
          
          GPIOC->AFR[0] |= (1<<29);
          
          /*PB7*/
          GPIOB->MODER |= GPIO_MODER_MODER7_1; //(1<<15);
          GPIOB->MODER &= ~ GPIO_MODER_MODER7_0; //(1<<14);
          
          GPIOB->OTYPER &= ~(1<<7);
          
          GPIOB->OSPEEDR &= ~(1<<15);
          GPIOB->OSPEEDR |= (1<<14);
          
          GPIOB->PUPDR &= ~ GPIO_PUPDR_PUPDR7_1; //(1<<15);
          GPIOB->PUPDR &= ~ GPIO_PUPDR_PUPDR7_0; //(1<<14);
          
          GPIOB->AFR[0] |= (1<<29);
       //////////////////////////////////////////////////////////////////////////////
          
         /*================LED PB8-PB9 - PB2-PB11 - PB15-PC5 PB14-PB1===================*/ 
          GPIOB->MODER |= GPIO_MODER_MODER15_0|GPIO_MODER_MODER14_0|GPIO_MODER_MODER11_0|GPIO_MODER_MODER9_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER2_0|GPIO_MODER_MODER1_0;//(1<<30)|(1<<28)|(1<<22)|(1<<18)|(1<<16)|(1<<4)|(1<<2);  
          GPIOB->MODER &= (~ (GPIO_MODER_MODER15_0|GPIO_MODER_MODER14_0|GPIO_MODER_MODER11_0|GPIO_MODER_MODER9_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER2_0|GPIO_MODER_MODER1_0));//(~(1<<31)|(1<<29)|(1<<23)|(1<<19)|(1<<17)|(1<<5)|(1<<3)); 
          GPIOC->MODER |= GPIO_MODER_MODER5_0;//(1<<10);
          GPIOC->MODER &=~ GPIO_MODER_MODER5_1; //(1<<11);
          
          GPIOB->OTYPER &= (~(1<<8)|(1<<9)|(1<<2)|(1<<11)|(1<<15)|(1<<14)|(1<<1));
          GPIOC->OTYPER &= ~(1<<5); 
          
          GPIOB->OSPEEDR |= ((1<<31)|(1<<29)|(1<<23)|(1<<19)|(1<<17)|(1<<5)|(1<<3)); 
          GPIOB->OSPEEDR &= (~(1<<30)|(1<<28)|(1<<22)|(1<<18)|(1<<16)|(1<<4)|(1<<2));
          GPIOC->OSPEEDR |= (1<<11);
          GPIOC->OSPEEDR &= ~(1<<10); 
                
                    
          
          GPIOB->PUPDR &= ~( GPIO_PUPDR_PUPDR15_0| GPIO_PUPDR_PUPDR14_0|GPIO_PUPDR_PUPDR11_0|GPIO_PUPDR_PUPDR9_0|GPIO_PUPDR_PUPDR8_0|GPIO_PUPDR_PUPDR2_0|GPIO_PUPDR_PUPDR1_0); //~((1<<30)|(1<<28)|(1<<22)|(1<<18)|(1<<16)|(1<<4)|(1<<2));
          GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR15_1| GPIO_PUPDR_PUPDR14_1|GPIO_PUPDR_PUPDR11_1|GPIO_PUPDR_PUPDR9_1|GPIO_PUPDR_PUPDR8_1|GPIO_PUPDR_PUPDR2_1|GPIO_PUPDR_PUPDR1_1);//~((1<<31)|(1<<29)|(1<<23)|(1<<19)|(1<<17)|(1<<5)|(1<<3)); 
          GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR5_1|GPIO_PUPDR_PUPDR5_0);//~((1<<10)|(1<<11));      
          /*****************************************************************************/
          
          /*==================BUZZER PB12=================*/
           GPIOB->MODER |= GPIO_MODER_MODER12_1; //(1<<25);  
           GPIOB->MODER &=~ GPIO_MODER_MODER12_0; //(1<<24); 
           GPIOB->OTYPER &=~ (1<<12);
           GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR12_1;//(1<<25);
           GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR12_0; //(1<<24);
           GPIOB->AFR[1]|=0x00010000;
                     
                        
 //////////////// PIN CAPTEUR ULTRASON AVANT  TRIGGER  PB5////////////////////////////////////
   GPIOB->MODER &=~ GPIO_MODER_MODER5_1;//(1<<11);     // PB5 output mode
   GPIOB->MODER |= GPIO_MODER_MODER5_0; //(1<<10);
   GPIOB->OTYPER &=~(1<<5);    //push-pull 
   GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR5_1; //(1<<11);    // no pull up, no pull down
   GPIOB->PUPDR &=~ GPIO_PUPDR_PUPDR5_0; //(1<<10);
  
   ////////////////// PIN CAPTEUR ULTRASON AVANT ECHO PA9///////////////////////////////////
   GPIOA->MODER &=~(GPIO_MODER_MODER9_1|GPIO_MODER_MODER9_0); //(~(1<<19)|(1<<18));    // PA9 input mode
   GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9_1|GPIO_PUPDR_PUPDR9_0);//(~(1<<19)|(1<<18));    // no pull up, no pull down
   
   //////////////////// PIN CAPTEUR ULTRASON ARRIERE ECHO PA2 ///////////////////////
  /* GPIOA->MODER &= (~(1<<5)|(1<<4));    // PA2 input mode // input2 echo
   GPIOA->PUPDR &= (~(1<<5)|(1<<4));    // no pull up, no pull down
   
   //////////////////// PIN CAPTEUR ULTRASON ARRIERE TRIGGER PB3 /////////////////////
   GPIOB->MODER &=~(1<<7);    // PB3 output mode trigger
   GPIOB->MODER |= (1<<6);
   GPIOB->OTYPER &=~(1<<3);    //push-pull 
   GPIOB->PUPDR &= ~(1<<7);    // no pull up, no pull down
   GPIOB->PUPDR &= ~(1<<6); 
   */
   
   //initialisation port pour gestion batterie - PA6/PA7/PA8 led - E5V sur PA0 
   GPIOA->MODER &= ~(GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1|GPIO_MODER_MODER8_1);//(~(1<<13)|(1<<15)|(1<<17)); 
   GPIOA->MODER |= GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0; //(1<<12)|(1<<14)|(1<<16); 
   GPIOA->OTYPER &= (~(1<<6)|(1<<7)|(1<<8)); 
   GPIOA->OSPEEDR |= (1<<13)|(1<<15)|(1<<17);
   GPIOA->OSPEEDR &= (~(1<<12)|(1<<14)|(1<<16));
   GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR6_0|GPIO_PUPDR_PUPDR6_1|GPIO_PUPDR_PUPDR7_0|GPIO_PUPDR_PUPDR7_1|GPIO_PUPDR_PUPDR8_0|GPIO_PUPDR_PUPDR8_1); //(~(1<<12)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<17)); 
  
   GPIOA->MODER |= GPIO_MODER_MODER0_1; //(1<<1);
   GPIOA->MODER |= GPIO_MODER_MODER0_0; //(1<<0); 
   
   GPIOA->OSPEEDR |= (1<<1);
   GPIOA->OSPEEDR &=~ (1<<0);
   GPIOA->OTYPER |= (1<<0); //open drain 
   GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_0|GPIO_PUPDR_PUPDR0_1); //(~(1<<0)|(1<<1)); 
   GPIOA->AFR[0] &= (~(1<<1)|(1<<2)|(1<<3)); 
   GPIOA->AFR[0] |= (1<<0);  
   
}

/************************* FIN INITIALISATION I/O ****************************/


/****************** TIM2 INIT GESTION LEDS *********************************/
void TIM2_CONFIG(void) 
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // tim2 enable

   //fréquence clignotement led toutes les 500ms
    TIM2->PSC = 15999; 
    TIM2->ARR = 299;
    
  
    TIM2->CCR1 =1; //TIM2_CH1
    TIM2->CCR2 = 1; //TIM2_CH2
    
    
    TIM2->CR1 |= TIM_CR1_ARPE; //(1<<7);
    TIM2->CR1 &=~ TIM_CR1_CMS_1; //(1<<6);
    TIM2->CR1 &=~ TIM_CR1_CMS_0; //(1<<5);
    TIM2->CR1 &=~ TIM_CR1_DIR; //(1<<4);

    TIM2->CCMR1|= TIM_CCMR1_OC1M_0|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2; //(1<<4)|(1<<5)|(1<<6); //TIM2_CH1
    TIM2->CCMR1|= TIM_CCMR1_OC2M_0|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2; //(1<<12)|(1<<13)|(1<<14); //TIM2_CH2

  
      
    TIM2->DIER |= TIM_DIER_UIE; //(1<<0); 
    NVIC->ISER[0] |= (1<<28); 
  
}

/************************* FIN TIM2 INIT *******************************/
   
/************************* TIM3 INIT GESTION MOTEUR DIRECTION **********************/
void TIM3_CONFIG(void) 
{
    //Active la clock du timer 3
    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
   
  //depart voiture marche avant 
  TIM3->PSC = 32000;
  TIM3->ARR = 200;
  
  TIM3->CCR1 =1; //TIM3_CH1
  TIM3->CCR2 = 1; //TIM3_CH2
  
  
  TIM3->CR1 |= TIM_CR1_ARPE; //(1<<7);
  TIM3->CR1 &=~ TIM_CR1_CMS_1; //(1<<6);
  TIM3->CR1 &=~ TIM_CR1_CMS_0; //(1<<5);
  TIM3->CR1 &=~ TIM_CR1_DIR; //(1<<4);
  
  
  TIM3->CCMR1|= TIM_CCMR1_OC1M_0|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2;//(1<<4)|(1<<5)|(1<<6); //TIM3_CH1
  TIM3->CCMR1|= TIM_CCMR1_OC2M_0|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2; //(1<<12)|(1<<13)|(1<<14); //TIM3_CH2
 
}
/********************** FIN TIM3 INIT ***********************************************/

/**************************** TIM4 INIT GESTION MOTEUR TRACTION ***********************/
void TIM4_CONFIG(void)  
{
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  
  //depart voiture marche arriere 
  TIM4->PSC = 32000;
  TIM4->ARR = 200;
  
  TIM4->CCR1 = 1;
  TIM4->CCR2= 1;
  
  TIM4->CR1 |= TIM_CR1_ARPE; //(1<<7); 
  TIM4->CR1 &=~ TIM_CR1_CMS_1; //(1<<6);
  TIM4->CR1 &=~ TIM_CR1_CMS_0; //(1<<5);
  TIM4->CR1 &=~ TIM_CR1_DIR; //(1<<4);
   
  TIM4->CCMR1|= TIM_CCMR1_OC1M_0|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2;//(1<<4)|(1<<5)|(1<<6); //TIM4_CH1
  TIM4->CCMR1|= TIM_CCMR1_OC2M_0|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2;//(1<<12)|(1<<13)|(1<<14); //TIM4_CH2
}
/************************* FIN TIM4 INIT *************************************/


/************************** TIM6 INIT GESTION BATTERIE ************************/
void TIM6_CONFIG(uint32_t delay_us)
{
  RCC-> APB1ENR |= RCC_APB1ENR_TIM6EN;
  TIM6->PSC = 15;// f=1MHz soit T=1us
  TIM6->ARR = delay_us - 1;
  
  TIM6->CR1 |= TIM_CR1_CEN; //(1<<0);
  TIM6->CR1 |= TIM_CR1_URS; //(1<<2);
  TIM6->CR1 |= TIM_CR1_ARPE; //(1<<7);
  
  TIM6->CR1 &=~ TIM_CR1_DIR; //(1<<4);
  TIM6->CR1 &=~ TIM_CR1_CMS_0; //(1<<5);
  TIM6->CR1 &=~ TIM_CR1_CMS_1; //(1<<6);
}
/***************************** FIN TIM6 INIT ************************************/

/********************** TIM9 INIT GESTION CAPTEUR ULTRASON AVANT ****************/
void TIM9_CONFIG(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;  // On lance l'horloge du TIM9
  
  TIM9->PSC = 16; // Préscalaire a 16 pour avoir 16Mhz/16=1Mhz =>1us
  
  TIM9->CR1 |= TIM_CR1_ARPE; // Auto-reload est initialisé
  TIM9->CR1 &=~ TIM_CR1_DIR | TIM_CR1_CMS_0 |TIM_CR1_CMS_1;  // Pour DIR direction est a 0 donc upcounter    CMS_0 CMS_1 On met 10 mode 2 car on est en mode couting up p463
}
/************************ FIN TIM9 INIT **********************************/

/****************** TIM10 INIT GESTION CAPTEUR ULTRASON ARRIERE ******************/
void TIM10_CONFIG(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; 
  
  TIM10->PSC = 16; 
  
  TIM10->CR1 |= TIM_CR1_ARPE; // Auto-reload est initialisé
  TIM10->CR1 &=~ TIM_CR1_DIR | TIM_CR1_CMS_0 |TIM_CR1_CMS_1;  // Pour DIR direction est a 0 donc upcounter    CMS_0 CMS_1 On met 10 mode 2 car on est en mode couting up p463
}
/******************************** FIN TIM10 INIT *****************************/

/*************************** FONCTION CALCUL DISTANCE OBSTACLE *************************/
void calcul_Distance(uint16_t largeur)
{
  distance = largeur/58.8235;  // 17/100=58.8235 ->distance en cm/us 
}
/**************************************** FIN FONCTION CALCUL DISTANCE ***************************************/

/*************************** FONCTION ATTENTE POUR LE TRIGGER DU CAPTEUR ULTRASON AVANT **********************/
void wait_TIM9_10us(void)
{
  TIM9->ARR = 9; 
  TIM9->CR1 |= TIM_CR1_CEN; // On lance le compteur 
  while( (TIM9->SR & TIM_SR_UIF)==0); // On attend que mis a un 1 par le hardware donc détection de quelque chose par l'ultrason
  TIM9->SR &= ~TIM_SR_UIF; // Remise a zéro pour pouvoir refaire une détection après
  TIM9->CR1 &=~TIM_CR1_CEN; // On éteint le compteur
}
/**************************************** FIN FONCTION ATTENTE **********************************************/


/****************** FONCTION ATTENTE POUR LE TRIGGER DU CAPTEUR ULTRASON ARRIERE ****************************/
void wait_TIM10_10us(void)
{
  TIM10->ARR = 9;
  TIM10->CR1 |= TIM_CR1_CEN; // On lance le comteur 
  while( (TIM10->SR & TIM_SR_UIF)==0);// On attend que mis a un 1 par le hardware donc détection de quelque chose par l'ultrason
  TIM10->SR &= ~TIM_SR_UIF; // Remise a zéro pour pouvoir refaire une détection après
  TIM10->CR1 &=~TIM_CR1_CEN;// On éteint le compteur
}
/*************************************** FIN FONCTION ATTENTE **********************************************/

/********************** CONFIGURATION DE L'INTERRUPTION EXTERNE SUR ECHO DU CAPTEUR AVANT ******************/
void ConfigInterrupt_PA9(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // clock syscfg enable
 
  SYSCFG->EXTICR[2] |= 0x0000; //autorisation de l'interruption sur le port A
  EXTI->FTSR |= EXTI_FTSR_TR9; //autorisation de l'interruption sur front descendant 
  EXTI->IMR |= EXTI_IMR_MR9; // autorise l'interruption sur PA9

}
/******************************************* FIN CONFIGURATION ***********************************************/


/*********************** CONFIGURATION DE L'INTERRUPTION EXTERNE SUR ECHO DU CAPTEUR ARRIERE ****************/
void ConfigInterrupt_PA2(void)
{
  SYSCFG->EXTICR[0] |= 0x0000;  //autorisation de l'interruption sur le port A
  EXTI->FTSR |= EXTI_FTSR_TR2; //autorite l'interruption sur front descendant 
  EXTI->IMR |= EXTI_IMR_MR2; // autorise l'interruption sur PA3
}
/***************************************** FIN CONFIGURATION ***********************************************/



/**************************** FONCTION GENERATION TRIGGER POUR CAPTEUR AVANT **************************/
void Generation_Trigger_avant(void) //generation du trigger pour le capteur avant sur PB5
{

  GPIOB->ODR |= GPIO_ODR_ODR_5; //(1<<5)  // trigger a 1
  wait_TIM9_10us();   
  GPIOB->ODR &=~ GPIO_ODR_ODR_5; //(1<<5)   // trigger a 0
}
/******************************************* FIN FONCTION *******************************************/

/****************** FONCTION GENERATION TRIGGER POUR CAPTEUR ARRIRERE ********************************/
void Generation_Trigger_arriere(void) // generation du trigger pour le capteur arriere sur PB3
{
  GPIOB->ODR |= GPIO_ODR_ODR_3; //(1<<3)   // trigger a 1
  wait_TIM10_10us();  
  GPIOB->ODR &=~ GPIO_ODR_ODR_3; //(1<<3)   // trigger a 0
}
/*********************************** FIN FONCTION ********************************************/


/************************** FONCTION INITIALISATION ADC POUR GESTION BATTERIE **********************/
void ADC_INIT (void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //(1<<9);
  RCC->APB1ENR |= RCC_APB1ENR_COMPEN; //(1<<31);
  RCC->CR |= RCC_CR_HSION; //(1<<0);    // Horloge HSI on
  ADC->CCR &=~ ADC_CCR_ADCPRE_0; //(1<<16); // HSI prescaler divise par 4
  ADC->CCR |= ADC_CCR_ADCPRE_1; //(1<<17);
  RCC->CFGR &=~ (RCC_CFGR_MCOSEL_2|RCC_CFGR_MCOSEL_0); //((1<<26)|(1<<24));
  RCC->CFGR |= RCC_CFGR_MCOSEL_1; //(1<<25)
  ADC1->CR1 &= ~(ADC_CR1_AWDCH_0 | ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_2 | ADC_CR1_AWDCH_3 | ADC_CR1_AWDCH_4 ); //((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4));  // activer ADC_IN0
  ADC1->CR1 &=~ ADC_CR1_SCAN; //(1<<8);   // vérifie que lma conversion a eu lieu (scan)
  ADC1->CR1 |= ADC_CR1_RES_1; //(1<<25);     // résolution sur 6 bit
  ADC1->CR1 |= ADC_CR1_RES_0; //(1<<24);
  RI->ICR &=~ (RI_ICR_IC1Z_0 | RI_ICR_IC1Z_1 | RI_ICR_IC1Z_2 | RI_ICR_IC1Z_3); //(1<<0)|(1<<1)|(1<<2)|(1<<3));   // roouting interface
  ADC1->SQR1 &=~(ADC_SQR1_L_0 | ADC_SQR1_L_1 | ADC_SQR1_L_2 | ADC_SQR1_L_3 | ADC_SQR1_L_4); //(1<<20)|(1<<21)|(1<<22)|(1<<23)|(1<<24));  // une seule voie de convertion
  ADC1->SQR5 &=~ (ADC_SQR5_SQ1_0 | ADC_SQR5_SQ1_1 | ADC_SQR5_SQ1_2 | ADC_SQR5_SQ1_3 | ADC_SQR5_SQ1_4); //(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4));     // first conversion
  ADC1->CR2 &=~ ADC_CR2_CFG; //(1<<2);  // Bank A
  ADC1->CR2 |= ADC_CR2_CONT; //(1<<1);  // continous mode
  RI->ASCR1 &=~RI_ASCR1_CH_0; //(1<<0);   // Analog swotch controlle by ADC
  ADC1->CR2 |= ADC_CR2_ADON; //(1<<0); // ADON ADC power on
  while ((ADC1->SR & ADC_SR_ADONS)==0); //tant que l'ADC n'est pas prêt

}
/******************************** FIN FONCTION ****************************************/


/******************************** FONCTION ACQUISITION QUI DECLENCHE LA CONVERSION DU NIVEAU DE LA BATTERIE ************************/
void acquisition(void)
{
  ADC1->CR2 |= ADC_CR2_SWSTART; //(1<<30);//Start conversion
  while ((ADC1->SR & ADC_SR_EOC )==0);//tant que la conversion n'est pas fini j'attend
  ADC1->SR |= ADC_SR_EOC;// end of conversion
  var = ADC1->DR;// on stocke dans var la valeur actuelle du data register
}
/****************************************************** FIN FONCTION ************************************************/




/**************************************** FONCTION GESTION LED POUR AFFICHAGE NIVEAU DE LA BATTERIE ***********************************/
void LED (void)
{
  if (var >=62)//LED verte s'allume
  {
    GPIOA->ODR |= GPIO_ODR_ODR_6; // (1<<6);//active LEDV
    GPIOA->ODR &=~ (GPIO_ODR_ODR_7 | GPIO_ODR_ODR_8);  //((1<<7)|(1<<8));//désactive les led orange et rouge
  }
  else
  {
    if(var < 62 && var > 60)//LED orange s'allume
    {
      GPIOA->ODR &=~ (GPIO_ODR_ODR_6 | GPIO_ODR_ODR_8); //((1<<6)|(1<<8));//désactive les led verte et rouge
      GPIOA->ODR |= GPIO_ODR_ODR_7; // (1<<7);//active LEDO
    }
    else
    {
      if(var <= 60)//LED rouge s'allume
      { 
        GPIOA->ODR &=~ (GPIO_ODR_ODR_6 | GPIO_ODR_ODR_7); //((1<<6)|(1<<7));//désactive les led verte et orange
        GPIOA->ODR |= GPIO_ODR_ODR_8 ; //(1<<8);//active LEDR
      }
    }
  }
  
}
/************************************************** FIN FONCTION *****************************************************/

/****************************** INISIALISATION DE L'USART POUR LA COMMUNOCATION PAR BLUETOOTH *****************************/
void USART_CONFIG(void)
{

  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // clock USART3 enable 

  USART3->CR1 &=~ USART_CR1_OVER8; //oversampling on 16bits 
  USART3->CR1 &=~ USART_CR1_M; // start bits 8 data bits 
  USART3->CR2 &= ~(USART_CR2_STOP_1|USART_CR2_STOP_0);//~((1<<13)|(1<<12)); // 1 stop bit 
  USART3->BRR = 0x658; //0x5D0; // pour un baudrate de 9600 bauds, USART_DIV = 0d208.3125 , DIV_FR = 0x5, DIV_MANTISSA= 0xD0 pour 16Mhz
  USART3->CR1 |= USART_CR1_RXNEIE; //(1<<5); // RXNE interruption enable 
  NVIC->ISER[1] |= (1<<7); // interrupt generation for USART3 
     
  USART3->CR1 |= USART_CR1_UE; // USART enable  
}
/******************************************* FIN FONCTION ******************************************************/

/**************************************** FONCTION TRANSMISSION DE LA DONNEE *****************************/
void TX_BYTE(char data)
{
  USART3->CR1 |= USART_CR1_TE; // transmittion enable 

  //writtng the data in the register for transmition 
  USART3->DR = data; 
  
  while((USART3->SR & USART_SR_TC)==0); // j'attend tant que la donnée n'a pas été transferé
}
/******************************************* FIN FONCTION *********************************/ 
   

int main(void)
{
  //CALLING FUNCTIONS   
  GPIO_INIT();
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
   
    USART3->CR1 |= USART_CR1_RE; //receive enable
    
    GPIOB->ODR &=~ GPIO_ODR_ODR_5 ; //(1<<5);  // trigger à 0 sur PB5 capteur distance avant
    //GPIOA->ODR &= ~(1<<3); //trigger à 0 sur PA3 capteur distance arrière     
    
    
  while(1)
  {   
      //gestion capteur ultrason avant
      Generation_Trigger_avant(); 
      //gestion capteur ultrason arriere
    
      //Generation_Trigger_arriere();
      //intrément du compteur
      compteur++;
      //autorisation des interruptions
      NVIC->ISER[0] |= (1<<23);// |(1<<8); // interrupt EXTI9_5_IRQHandler & EXTI2_IRQHandler enable
   
      calcul_Distance(largeurEcho); 
     
    
      if(distance < 3)
      {
               //arret marche avant
               TIM4->CR1 &=~ TIM_CR1_CEN ; //(1<<0); // TIM4 counter disable 
               TIM4->CCER &= ~(1<<0); //tim4 ch2 
      }
      else  
      {   
        
        chaine = USART3->DR ; 
          switch(chaine)
        {                    
        case '1' :  
                //active la marche avant PB7 et led PB9-PC5
                TIM4->CR1 |= TIM_CR1_CEN; // TIM4 counter enable 
                TIM4->CCER |= TIM_CCER_CC1E; // tim4_ch1 enable 
                GPIOB->ODR |= GPIO_ODR_ODR_8 | GPIO_ODR_ODR_11; //(1<<8)|(1<<11); // turn the LED ON - PB8-PB117
                break;
          
          
        }
      } 
      
      
      //gestion de la batterie 
      acquisition(); // acquiert les valeurs de la tension sur PA0
      LED(); // active les LED en fonction du niveau de tension 
       
  }
}