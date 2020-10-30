#include "RTE_Components.h"
#include "MKL25Z4.h"
#include "cmsis_os2.h"

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK32(x) ((uint32_t)(1 << ((uint32_t)x))) // Changes all bits to 0 except x
enum color_t {NONE, RED, GREEN, BLUE, CYAN, YELLOW, MAGENTA, WHITE};

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
osMessageQueueId_t txq;
osMessageQueueId_t rxq;

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
	UART2->C2 |= UART_C2_RIE_MASK;
  UART2->C2 |= UART_C2_RE_MASK | UART_C2_TE_MASK;
	
  NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
  NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
}

// flags cleared after reading UART2->S1 and UART2->D
void UART2_IRQHandler(void) {
  NVIC_ClearPendingIRQ(UART2_IRQn);
	UCHAR S1 = UART2->S1; // to prevent any part from clearing S1
	UCHAR currdata = UART2->D;
	UCHAR d;
  if (S1 & UART_S1_TDRE_MASK) {
    // can send another character
		if (osMessageQueueGet(txq, &d, 0, 0) == osOK) {
			UART2->D = d;
		} else {
      // queue is empty so disable tx
      UART2->C2 &= ~UART_C2_TIE_MASK;
    }
  } 

  if (S1 & UART_S1_RDRF_MASK) {
    // received a character
		if (osMessageQueuePut(rxq, &currdata, 0, 0) == osOK) {
      // error -queue full.
      errcode = ERR_BUFOVF;
      errdata = currdata;
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
  }
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
		/* led_control(CYAN);
		for (UINT i = 0; i < len; ++i) {
			uart_tx(buf + i, 1);
		}
		
		for (;;) {		
			if (uart_tx(tx_data, 2) == 2) {
				led_control(RED);
			}
			
			if (uart_rx(rx_data, 2) == 2) {
				led_control(GREEN);
			}
			
			if (rx_data[0] == 'O') {
				led_control(BLUE);
			}
		} */
		
		UCHAR rx_char;
    for (;;) {
			led_control(RED);
			osDelay(1000);
			led_control(GREEN);
			osDelay(1000);
      // if the app sent data is 3, flash blue
      if (uart_rx(&rx_char, 1) == 1) {
				led_control(BLUE);
				if (rx_char == 0x01) {
					led_control(MAGENTA);
				}
				if (rx_char == 0x02) {
					led_control(YELLOW);
				}
				if (rx_char == 0x03) {
					led_control(CYAN);
				}
				osDelay(1000);
			}
		}
}

int main(void) {
		InitGPIO();
		InitUART2(BAUD_RATE);
    SystemCoreClockUpdate();
		
		osKernelInitialize();
		led_control(YELLOW);
		txq = osMessageQueueNew(Q_SIZE, sizeof(UCHAR), NULL);
		rxq = osMessageQueueNew(Q_SIZE, sizeof(UCHAR), NULL);
		
		osThreadNew(comms_test_thread, NULL, NULL);
		osKernelStart();
		
		for (;;);
}
