#include "driver.h"

// UART settings
#define Q_SIZE (16)
#define BAUD_RATE 9600
#define CMD_CONN 100
#define CMD_END 200
#define CMD_W 'W'
#define CMD_A 'A'
#define CMD_S 'S'
#define CMD_D 'D'

// event flags
#define FLAG_CONN ((uint32_t) 0x00000001)
#define FLAG_END  ((uint32_t) 0x00000010)
#define FLAG_MOVE ((uint32_t) 0x00000100)

// movement settings
#define MOVE_DUR 200

void tBrain(void *argument);
void tMotor(void *argument);
void tLED(void *argument);
void tEventAudio(void *argument);
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
    osThreadNew(tEventAudio, NULL, NULL); // TODO give high prio
    osThreadNew(tLED, NULL, NULL);
	  osThreadNew(tAudio, NULL, NULL);
	  osThreadNew(tMotor, NULL, NULL);

	  osKernelStart();
		for (;;);
}

void tBrain(void *argument) {
  UCHAR cmd;
  for (;;) {
    osMessageQueueGet(rxq, &cmd, NULL, osWaitForever);
    switch(cmd) {
      case CMD_CONN: 
        osEventFlagsSet(flags, FLAG_CONN);
        break;
      case CMD_END:
        osEventFlagsSet(flags, FLAG_END);
        break;
      default:
        osMessageQueuePut(moveq, &cmd, 0, osWaitForever);
        break;
    }
  }
}

void tMotor(void *argument) {
	UCHAR data;
	for (;;) {
		osMessageQueueGet(moveq, &data, NULL, MOVEDUR);
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
