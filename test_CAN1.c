/*----------------------------------------------------------------------------
 * Name:  test_CAN1.c
 * Notes : mettre un autre noeud CAN sur le reseau en mode silence pour les tests (voir silence.c) 
 * Periphérique CAN 1 : 1 thread R sur objet 0, 1 thread T sur objet 2
 * Réception uniquement des trames ID 0x5F8 et 0x5F9
 * STM32F746G-Discovery
 * Maj 5/03/2024 - XM
 *----------------------------------------------------------------------------
 * Code développé à partir de sources fournies par Keil uVision/ARM development tools.
 *----------------------------------------------------------------------------*/

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "Board_LED.h"
#include "Board_Buttons.h"
#include "RTE_Components.h"
#include "Driver_CAN.h"                 // ::CMSIS Driver:CAN

osThreadId id_CANthreadR;
osThreadId id_CANthreadT;

extern   ARM_DRIVER_CAN         Driver_CAN1;

// CAN1 callback
void myCAN1_callback(uint32_t obj_idx, uint32_t event)
{
    if (event&ARM_CAN_EVENT_RECEIVE)
      {
        osSignalSet(id_CANthreadR, 0x04);
      }
    if (event&ARM_CAN_EVENT_SEND_COMPLETE)
      {
        osSignalSet(id_CANthreadT, 0x02);
      }

/*----------------------------------------------------------------------------
* Version en switch
    switch (event)
    {
    case ARM_CAN_EVENT_RECEIVE:
        //  Message was received successfully by the obj_idx object.
       osSignalSet(id_CANthreadR, 0x04);
        break;
		case ARM_CAN_EVENT_SEND_COMPLETE:
        // 	Message was sent successfully by the obj_idx object.
        osSignalSet(id_CANthreadT, 0x02);
        break;
    }
 *----------------------------------------------------------------------------*/
}


#ifdef RTE_CMSIS_RTOS_RTX
extern uint32_t os_time;

uint32_t HAL_GetTick(void) { 
  return os_time; 
}
#endif

/**
  * System Clock Configuration
  *   System Clock source            = PLL (HSE)
  *   SYSCLK(Hz)                     = 216000000
  *   HCLK(Hz)                       = 216000000
  *   AHB Prescaler                  = 1
  *   APB1 Prescaler                 = 4
  *   APB2 Prescaler                 = 2
  *   HSE Frequency(Hz)              = 25000000
  *   PLL_M                          = 25
  *   PLL_N                          = 432
  *   PLL_P                          = 2
  *   PLL_Q                          = 9
  *   VDD(V)                         = 3.3
  *   Main regulator output voltage  = Scale1 mode
  *   Flash Latency(WS)              = 7
  */
static void SystemClock_Config (void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  HAL_PWREx_EnableOverDrive();
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
}

/**
  * Configure the MPU attributes as Write Through for SRAM1/2
  *   The Base Address is 0x20010000 since this memory interface is the AXI.
  *   The Region Size is 256KB, it is related to SRAM1 and SRAM2 memory size.
  */
static void MPU_Config (void) {
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * CPU L1-Cache enable
  */
static void CPU_CACHE_Enable (void) {

  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

// CAN1 utilise pour reception, Objet 0 pour RX, Objet 2 pour TX
void InitCan1 (void) {
	
	/* Pour test des "Capabilities" du controleur -> recherche des objets R et T
	ARM_CAN_CAPABILITIES     can_cap;
  ARM_CAN_OBJ_CAPABILITIES can_obj_cap;
  int32_t                  status;
  uint32_t                 i, num_objects;
	uint32_t                 rx_obj_idx  = 0xFFFFFFFFU;
	uint32_t                 tx_obj_idx  = 0xFFFFFFFFU;
 
  can_cap = Driver_CAN1.GetCapabilities (); // Get CAN driver capabilities
  num_objects = can_cap.num_objects;    // Number of receive/transmit objects
	
	// Recherche des objets pour emission et recep
	for (i = 0U; i < num_objects; i++) {                                          // Find first available object for receive and transmit
    can_obj_cap = Driver_CAN1.ObjectGetCapabilities (i);                        // Get object capabilities
    if      ((rx_obj_idx == 0xFFFFFFFFU) && (can_obj_cap.rx == 1U)) { rx_obj_idx = i; }
    else if ((tx_obj_idx == 0xFFFFFFFFU) && (can_obj_cap.tx == 1U)) { tx_obj_idx = i; break; }
  }
	// fin test recherche sur objets	*/
	
	
	Driver_CAN1.Initialize(NULL,myCAN1_callback);
	Driver_CAN1.PowerControl(ARM_POWER_FULL);
	
	Driver_CAN1.SetMode(ARM_CAN_MODE_INITIALIZATION);
	Driver_CAN1.SetBitrate( ARM_CAN_BITRATE_NOMINAL,
							125000,
							ARM_CAN_BIT_PROP_SEG(5U)   |         // Set propagation segment to 5 time quanta
							ARM_CAN_BIT_PHASE_SEG1(1U) |         // Set phase segment 1 to 1 time quantum (sample point at 87.5% of bit time)
							ARM_CAN_BIT_PHASE_SEG2(1U) |         // Set phase segment 2 to 1 time quantum (total bit is 8 time quanta long)
							ARM_CAN_BIT_SJW(1U));                // Resynchronization jump width is same as phase segment 2
	// Mettre ici les filtres ID de r�ception sur objet 0
	Driver_CAN1.ObjectSetFilter(0,ARM_CAN_FILTER_ID_EXACT_ADD, ARM_CAN_STANDARD_ID(0x5f8),0);
	Driver_CAN1.ObjectSetFilter(0,ARM_CAN_FILTER_ID_EXACT_ADD, ARM_CAN_STANDARD_ID(0x5f9),0);
	
	Driver_CAN1.ObjectConfigure(0,ARM_CAN_OBJ_RX);				// Objet 0 du CAN1 pour reception
	Driver_CAN1.ObjectConfigure(2,ARM_CAN_OBJ_TX);				// Objet 2 du CAN1 pour emission
	
	Driver_CAN1.SetMode(ARM_CAN_MODE_NORMAL);					// fin init
}

// thread T toutes les secondes
void CANthreadT(void const *argument)
{
	ARM_CAN_MSG_INFO                tx_msg_info;
	uint8_t data_buf[8];
	int i;	
	
	while (1) {
		
		// envoie de 2 trames CAN d'info

		// Parametres Trame 1 CAN a envoyer (Controleur CAN 1)
		tx_msg_info.id = ARM_CAN_STANDARD_ID(0x0f6);   /* pour regime + vitesse   */
		for (i = 0; i < 8; i++) data_buf[i] = 0;	// mise � z�ro zone de data
		data_buf[0] = 0x20;        
		data_buf[1] = 0x02;         	   
		data_buf[2] = 0x03;       
		data_buf[3] = 0x04;           
		Driver_CAN1.MessageSend(2, &tx_msg_info, data_buf, 4);   // Envoie trame avec 1 data par buffer 2	du CAN1	
		osSignalWait(0x02, osWaitForever);		// sommeil en attente fin emission
		
		osDelay(500);		// pour laisser le temps au niveau de la reception
		
		// Parametres Trame 2 CAN a envoyer (Controleur CAN 1)
		tx_msg_info.id = ARM_CAN_STANDARD_ID(0x128);   /* pour carburant   */
		for (i = 0; i < 8; i++) data_buf[i] = 0;	// mise � z�ro zone de data
		data_buf[0] = 0x40;           		
		Driver_CAN1.MessageSend(2, &tx_msg_info, data_buf, 1);   // Envoie trame avec 1 data par buffer 2 du CAN1	
		osSignalWait(0x02, osWaitForever);		// sommeil en attente fin emission
		
    LED_On(0);      // pour observation envoi ok

		osDelay(500);;	// pour laisser le temps au niveau de la reception
	}		
}

// thread R
void CANthreadR(void const *argument)
{
	ARM_CAN_MSG_INFO   rx_msg_info;
	uint8_t data_buf[8];
	char data_reception;
	int identifiant; 
	
		while(1)
	{		
		osSignalWait(0x04, osWaitForever);		// sommeil en attente reception CAN
		
		// Reception trames CAN 
		Driver_CAN1.MessageRead(0, &rx_msg_info, data_buf, 8);	// 8 data max

		identifiant = rx_msg_info.id;	// recup id
		data_reception = data_buf [0] ;			// 1ère donnée de la trame récupérée
		
		// Allumage LED
		switch (identifiant)
			{
			case 0x5f8 :	LED_On(1);
										break;
											
			case 0x5f9 :	LED_On(2);
										break;							
			}
	}
}

osThreadDef(CANthreadR,osPriorityNormal, 1,0);
osThreadDef(CANthreadT,osPriorityNormal, 1,0);

/**
  * Main function
  */
int main (void) {

  MPU_Config();                             /* Configure the MPU              */
  CPU_CACHE_Enable();                       /* Enable the CPU Cache           */

  osKernelInitialize();                     /* initialize CMSIS-RTOS          */

  HAL_Init();                               /* Initialize the HAL Library     */
  SystemClock_Config();                     /* Configure the System Clock     */
  InitCan1();                               /* A mettre APRES init des horloges sinon pb     */
  LED_Initialize();                         /* LED Initialization             */

  id_CANthreadT = osThreadCreate (osThread(CANthreadT), NULL);
  id_CANthreadR = osThreadCreate (osThread(CANthreadR), NULL);

  osKernelStart();                          /* start thread execution         */

  osDelay(osWaitForever);
}
