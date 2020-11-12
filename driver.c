#include "RTE_Components.h"
#include "MKL25Z4.h"
#include "cmsis_os2.h"
#include "driver.h"

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1

#define REAR_LED 16 // PortC Pin 16

// PWM Pins
#define SPEAKER 0
#define PTD0_Pin 0
#define PTD1_Pin 1
#define PTD2_Pin 2
#define PTD3_Pin 3

#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 0

#define MASK32(x) ((uint32_t)(1 << ((uint32_t)x))) // Changes all bits to 0 except x

// Movement
// smaller oveflow value => higher duty cycle => faster
#define FREQUENCY_TO_MOD(x) (375000 / x)
#define MOTOR_SLOW (375000 / 100)
#define MOTOR_FAST (375000 / 50)

extern osMessageQueueId_t rxq;
extern UINT errcode;
extern UCHAR errdata;

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

void InitBoardLED(void) {
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

void InitExtLED(void) {
    // Enable Clock Gating for PORTC
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
		// Configure MUX
		for (int i = 0; i < FRONT_LEDS_LEN; i++) {				
		    PORTC->PCR[FRONT_LEDS[i]] &= ~PORT_PCR_MUX_MASK;
        PORTC->PCR[FRONT_LEDS[i]] |= PORT_PCR_MUX(1);
    }
    PORTC->PCR[REAR_LED] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[REAR_LED] |= PORT_PCR_MUX(1);
		
    // Set Data Direction Registers for PortC
    for (int i = 0; i < FRONT_LEDS_LEN; i++) {
        PTC->PDDR |= MASK32(FRONT_LEDS[i]);
    }
    PTC->PDDR |= MASK32(REAR_LED);
}

void InitMotor(void) {
  // Enable Clock Gating for PORTB and PORTD
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

  // Configure Mode 3 for speaker PWM
  PORTB->PCR[SPEAKER] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[SPEAKER] |= PORT_PCR_MUX(3);

  // Configure Mode 4 for motor PWM
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
  TPM0->MOD = MOTOR_FAST;

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

  // Enable PWM on TPM1 Channel 0 -> PTB0
  TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
  TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void InitAudio(void) {
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

  PORTB->PCR[SPEAKER] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[SPEAKER] |= PORT_PCR_MUX(3);

	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
  SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
  SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
  TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
  TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
  TPM1->SC &= ~(TPM_SC_CPWMS_MASK); 
}

// flags cleared after reading UART2->S1 and UART2->D
void UART2_IRQHandler(void) {
  NVIC_ClearPendingIRQ(UART2_IRQn);

  UCHAR S1 = UART2->S1; // to prevent any part from clearing S1
  uint8_t currdata = UART2->D;
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

// set LED to color
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

void motor_control(enum move_t move) {
	switch(move) { // TODO reorganise
		case STOP:
			TPM0_C0V = 0;
      TPM0_C1V = 0;
      TPM0_C2V = 0;
      TPM0_C3V = 0;
			break;
		case FORWARDS: // Forward
			TPM0_C0V = 0;
			TPM0_C1V = MOTOR_FAST; 
			TPM0_C2V = 0;
			TPM0_C3V = MOTOR_FAST;
			break;
		case CURVE_LEFT: // Forward + Left
			TPM0_C0V = 0;
			TPM0_C1V = MOTOR_FAST;
			TPM0_C2V = 0;
			TPM0_C3V = MOTOR_SLOW;
			break;
		case CURVE_RIGHT: // Forward + Right
			TPM0_C0V = 0;
			TPM0_C1V = MOTOR_SLOW;
			TPM0_C2V = 0;
			TPM0_C3V = MOTOR_FAST;
			break;
		case BACKWARDS: // Backward
			TPM0_C0V = MOTOR_FAST;
			TPM0_C1V = 0;
			TPM0_C2V = MOTOR_FAST;
			TPM0_C3V = 0;
			break;
		case TURN_LEFT: // Rotate Left
			TPM0_C0V = 0;
			TPM0_C1V = MOTOR_SLOW;
			TPM0_C2V = MOTOR_SLOW;
			TPM0_C3V = 0;
			break;
		case TURN_RIGHT: // Rotate Right
			TPM0_C0V = MOTOR_SLOW;
			TPM0_C1V = 0;
			TPM0_C2V = 0;
			TPM0_C3V = MOTOR_SLOW;
			break;
	}
}

void play_note(uint32_t freq) {
  TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | 
      (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
  uint32_t mod = 48E6 / (128 * freq);
  TPM1->MOD = mod;
  TPM1_C0V = mod / 2;
	TPM1_CNT = 0; 
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void stop_music(void) {
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | 
      (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
}

void led_toggle_red(void) {
	PTC->PTOR |= MASK32(REAR_LED);
}

void led_on_green(uint32_t pin) {
	PTC->PDOR |= MASK32(pin);
}

void led_off_green(uint32_t pin) {
	PTC->PDOR &= ~MASK32(pin);
}

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
