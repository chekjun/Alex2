#include "RTE_Components.h"
#include "MKL25Z4.h"
#include "cmsis_os2.h"

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK32(x) ((uint32_t)(1 << ((uint32_t)x))) // Changes all bits to 0 except x
enum color_t {NONE, RED, GREEN, BLUE, CYAN, YELLOW, MAGENTA, WHITE};

#define PTD0_Pin 0
#define PTD1_Pin 1
#define PTD2_Pin 2
#define PTD3_Pin 3

#define FREQUENCY_TO_MOD(x) (375000 / x)

#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 0
#define BAUD_RATE 9600
#define Q_SIZE (8)

#define UCHAR unsigned char // byte
#define UINT unsigned int // word

#define ERR_NONE 0
#define ERR_BUFOVF 1
#define ERR_QEMPTY 2
#define ERR_UARTERR 3

volatile UINT errcode;
volatile UCHAR errdata;
volatile uint8_t global_rx = 0x0;


osMessageQueueId_t txq;
osMessageQueueId_t rxq;
osMessageQueueId_t LED;
osMessageQueueId_t music;

osSemaphoreId_t mySem;


void led_control(enum color_t color) {
	PTB->PDOR &= ~MASK32(RED_LED) & ~MASK32(GREEN_LED); // Switch on RED and GREEN LED
	PTD->PDOR &= ~MASK32(BLUE_LED); // Switch on BLUE LED
	switch(color) {
		case NONE:
			PTB->PDOR |= MASK32(RED_LED) | MASK32(GREEN_LED); // Switch off GREEN and RED LED
			PTD->PDOR |= MASK32(BLUE_LED); // Switch off BLUE LED
		case RED:
			PTB->PDOR |= MASK32(GREEN_LED); // Switch off GREEN LED
			PTD->PDOR |= MASK32(BLUE_LED); // Switch off BLUE LED
		break;
		case GREEN:
			PTB->PDOR |= MASK32(RED_LED); // Switch off RED LED
			PTD->PDOR |= MASK32(BLUE_LED); // Switch off BLUE LED
		break;
		case BLUE:
			PTB->PDOR |= MASK32(RED_LED) | MASK32(GREEN_LED); // Switch off GREEN and RED LED
		break;
		case MAGENTA:
			PTB->PDOR |= MASK32(GREEN_LED); // Switch off GREEN LED
		break;
		case CYAN:
			PTB->PDOR |= MASK32(RED_LED); // Switch off RED LED
		break;
		case YELLOW:
			PTD->PDOR |= MASK32(BLUE_LED); // Switch off BLUE LED
		break;
		case WHITE:
		break;
	}
}

/* initPWM() */
void InitPWM(void)
{
	// Enable Clock Gating for PORTB and PORTD
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	// Configure Mode 4 for the PWM pin operation
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD1_Pin] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(4);
	
	// Enable Clock Gating for Timer0 and Timer1
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	// Select clock for TPM module
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // MCGCLLCLK or  MCGLLCLK/2
	
	// Set Modulo Value 20971520 / 128 = 163840 / 3276 = 50 Hz
	TPM0->MOD = 7500;
	
	/* Edge-Aligned PWM */
	// Update SnC register: CMOD = 01, PS = 111 (128)
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Enable PWM on TPM0 Channel 0 -> PTD0
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM0 Channel 1 -> PTD1
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM0 Channel 2 -> PTD2
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM0 Channel 3 -> PTD3
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM0_C0V = 0;
	TPM0_C1V = 0;
	TPM0_C2V = 0;
	TPM0_C3V = 0;
}

void motor() {
	uint8_t data;
	for (;;) {
		osMessageQueueGet(rxq, &data, NULL, 0);
		switch(data) {
			case 1: //forward
				TPM0_C0V = 0;
				TPM0_C1V = FREQUENCY_TO_MOD(100);
				TPM0_C2V = 0;
				TPM0_C3V = FREQUENCY_TO_MOD(100);
				break;
			case 2: //left
				TPM0_C0V = FREQUENCY_TO_MOD(100);
				TPM0_C1V = 0;
				TPM0_C2V = 0;
				TPM0_C3V = FREQUENCY_TO_MOD(100);
				break;
			case 3: //back
				TPM0_C0V = FREQUENCY_TO_MOD(100);
				TPM0_C1V = 0;
				TPM0_C2V = FREQUENCY_TO_MOD(100);
				TPM0_C3V = 0;
				break;
			case 4: //right
				TPM0_C0V = 0;
				TPM0_C1V = FREQUENCY_TO_MOD(100);
				TPM0_C2V = FREQUENCY_TO_MOD(100);
				TPM0_C3V = 0;
				break;
			case 5: //forward left
				TPM0_C0V = 0;
				TPM0_C1V = FREQUENCY_TO_MOD(50);
				TPM0_C2V = 0;
				TPM0_C3V = FREQUENCY_TO_MOD(50);
				break;
			case 6: //forward right
				TPM0_C0V = 0;
				TPM0_C1V = FREQUENCY_TO_MOD(50);
				TPM0_C2V = 0;
				TPM0_C3V = FREQUENCY_TO_MOD(50);
				break;
		}
	}
}

void playSound() {
	
}

void InitGPIO(void)
{
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK32(RED_LED) | MASK32(GREEN_LED));
	PTD->PDDR |= MASK32(BLUE_LED);
}

void InitUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
  SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8); // also set stop bit to 1 and disable extra interrupts
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0; // disable parity, set 8 bits, lsb first 
	UART2->S2 = 0;
	UART2->C3 = 0;
  
	//UART2->C2 &= ~UART_C2_TIE_MASK; // we do not need to transmit
  UART2->C2 |= UART_C2_RE_MASK | UART_C2_TE_MASK;
	
  NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
  NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_RIE_MASK;

}

// flags cleared after reading UART2->S1 and UART2->D
void UART2_IRQHandler(void) {
  NVIC_ClearPendingIRQ(UART2_IRQn);
	
	UCHAR S1 = UART2->S1; // to prevent any part from clearing S1
	uint8_t currdata = UART2->D;
	UCHAR d;
	if (S1 & UART_S1_RDRF_MASK) {
		global_rx = currdata;
		//osMessageQueuePut(rxq, (uint8_t*)&currdata, 0, 0);
		osSemaphoreRelease(mySem);
	}
	/**
	// transmit not used
  if (S1 & UART_S1_TDRE_MASK) {
    // can send another character
		if (osMessageQueueGet(txq, &d, 0, 0) == osOK) {
			UART2->D = d;
		} else {
      // queue is empty so disable tx
      UART2->C2 &= ~UART_C2_TIE_MASK;
    }
  } 
	// receive code todo here
  if (S1 & UART_S1_RDRF_MASK) {
    // received a character
		if (osMessageQueuePut(rxq, (uint8_t*)&currdata, 0, 0) == osOK) {
      // error -queue full.
      errcode = ERR_BUFOVF;
      errdata = currdata;
			osSemaphoreRelease(mySem);

    }
  }

  // handle error
  if (S1 & (UART_S1_OR_MASK |
        UART_S1_NF_MASK |
        UART_S1_FE_MASK |
        UART_S1_PF_MASK)) {
    // handle the error
    errcode = ERR_UARTERR;
    errdata = currdata;
  }**/
}

// move len bytes from buf to queue, return number of bytes moved
/*UINT uart_tx(UCHAR *buf, UINT len) {
	UINT i = 0;
	while (i < len) {
		if (osMessageQueuePut(txq, buf + (i++), 0, 0) != osOK) {
			i -= 2;
			break;
		}
	}
	UART2->C2 |= UART_C2_TIE_MASK;
	return i;
}*/

// move len bytes from queue to buf, return number of bytes moved
UINT uart_rx(UCHAR *buf, UINT len) {
	UINT i = 0;
	while (i < len) {
		if (osMessageQueueGet(rxq, buf + (i++), 0, 0) != osOK) {
			return i - 1;
		}
	}
	return i;
}

void comms_test_thread(void *argument) {				
		UCHAR rx_char;
		_Bool isRunning;
		_Bool playingMusic;
		uint8_t messageData = global_rx;
    for (;;) {
			osSemaphoreAcquire(mySem, osWaitForever);
			osMessageQueuePut(rxq, (uint8_t*)&global_rx, 0, 0);
			messageData = global_rx;
			if (messageData > 0) {
				isRunning = 1;
			}	else {
				isRunning = 0;
			}
			if (messageData == 10) {
				playingMusic = 1;
			}
			osMessageQueuePut(LED, &isRunning, 0, 0);
			osMessageQueuePut(music, &playingMusic, 0, 0);
			/**
			led_control(RED);
			osDelay(1000);
			led_control(GREEN);
			osDelay(1000);
      // if the app sent data is 3, flash blue
      if (uart_rx(&rx_char, 1) == 1) {
				led_control(BLUE);
				switch(rx_char) {
					case 0x01:
						led_control(MAGENTA);
						break;
					case 0x02:
						led_control(YELLOW);
						break;
					case 0x03:
						led_control(CYAN);

						motorForward();
					break;
					case 0x04:
						motorLeft();
					break;
					case 0x05:
						motorRight();
					break;
					case 0x06:
						motorBackward();
					break;
				}
				
				osDelay(1000);
			}**/
		}
}

int main(void) {
		InitGPIO();
		InitUART2(BAUD_RATE);
	InitPWM();
    SystemCoreClockUpdate();
		
		osKernelInitialize();
		mySem = osSemaphoreNew(1, 0, NULL);
		led_control(YELLOW);
		txq = osMessageQueueNew(Q_SIZE, sizeof(UCHAR), NULL);
		rxq = osMessageQueueNew(Q_SIZE, sizeof(UCHAR), NULL);
		LED = osMessageQueueNew(Q_SIZE, sizeof(_Bool), NULL);
		music = osMessageQueueNew(Q_SIZE, sizeof(_Bool), NULL);
		osThreadNew(comms_test_thread, NULL, NULL);
		osThreadNew(motor, NULL, NULL);
		osKernelStart();
		for (;;);
}
