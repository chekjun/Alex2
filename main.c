#include "driver.h"

#define Q_SIZE (16)
#define BAUD_RATE 9600

void tBrain(void *argument);
void tMotor(void *argument);
void tLED(void *argument);
void tAudio(void *argument);

osEventFlagsId_t flags;
osMessageQueueId_t moveq;

int main(void) {
    // System Initialization
	  SystemCoreClockUpdate();

	  InitUART2(BAUD_RATE);
	  InitGPIO();
	  InitPWM();

	  osKernelInitialize();
	  led_control(YELLOW);
    
    flags = osEventFlagsNew(NULL);
	  rxq = osMessageQueueNew(Q_SIZE, sizeof(UCHAR), NULL);
	  moveq = osMessageQueueNew(Q_SIZE, sizeof(UCHAR), NULL);

    osThreadNew(tBrain, NULL, NULL); // TODO give high prio
    osThreadNew(tLED, NULL, NULL);
	  osThreadNew(tAudio, NULL, NULL);
	  osThreadNew(tMotor, NULL, NULL);

	  osKernelStart();
		for (;;);
}

void tBrain(void *argument) {
	
}

void tMotor(void *argument) {
	uint8_t data;
	for (;;) {
		osMessageQueueGet(rxq, &data, NULL, 0);
		switch(data) {
			case 1: // Forward
				TPM0_C0V = 0;
				TPM0_C1V = FREQUENCY_TO_MOD(50); // smaller mod value gives larger overall value means faster
				TPM0_C2V = 0;
				TPM0_C3V = FREQUENCY_TO_MOD(50);
				break;
			case 2: // Forward + Left
				TPM0_C0V = 0;
				TPM0_C1V = FREQUENCY_TO_MOD(50);
				TPM0_C2V = 0;
				TPM0_C3V = FREQUENCY_TO_MOD(100);
				break;
			case 3: // Forward + Right
				TPM0_C0V = 0;
				TPM0_C1V = FREQUENCY_TO_MOD(100);
				TPM0_C2V = 0;
				TPM0_C3V = FREQUENCY_TO_MOD(50);
				break;
			case 4: // Backward
				TPM0_C0V = FREQUENCY_TO_MOD(50);
				TPM0_C1V = 0;
				TPM0_C2V = FREQUENCY_TO_MOD(50);
				TPM0_C3V = 0;
				break;
			case 5: // Backward + Left
				TPM0_C0V = FREQUENCY_TO_MOD(50);
				TPM0_C1V = 0;
				TPM0_C2V = FREQUENCY_TO_MOD(100);
				TPM0_C3V = 0;
				break;
			case 6: // Backward + Right
				TPM0_C0V = FREQUENCY_TO_MOD(100);
				TPM0_C1V = 0;
				TPM0_C2V = FREQUENCY_TO_MOD(50);
				TPM0_C3V = 0;
				break;
		}
	}
}

void tAudio(void *argument) {
	
}


void tLED(void *argument) {
	
}
