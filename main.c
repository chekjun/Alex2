#include "RTE_Components.h"
#include "MKL25Z4.h"
#include "cmsis_os2.h"
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

// song lengths
#define SONG_LEN 8
#define CONNSONG_LEN 1
#define ENDSONG_LEN 1

uint8_t green_leds[] = {0, 0, 0, 0, 0, 0, 0, 0}; // TODO
uint32_t song_notes[SONG_LEN] = {NOTE_C, NOTE_D, NOTE_E, NOTE_F, NOTE_G, NOTE_A, NOTE_B};
uint32_t song_dur[SONG_LEN] = {DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART};
uint32_t conn_notes[CONNSONG_LEN] = {0};
uint32_t conn_dur[CONNSONG_LEN] = {0};
uint32_t end_notes[ENDSONG_LEN] = {0};
uint32_t end_dur[ENDSONG_LEN] = {0};

void tBrain(void *argument);
void tMotor(void *argument);
void tLED(void *argument);
void tEvent(void *argument);
void tAudio(void *argument);

// vars for driver
osMessageQueueId_t rxq;
volatile UINT errcode;
volatile UCHAR errdata;

osEventFlagsId_t flags;
osMessageQueueId_t moveq;
osMutexId_t green_led_mutex;
osMutexId_t red_led_mutex;
osMutexId_t audio_mutex;

const osThreadAttr_t thread_highprio = {
	.priority = osPriorityHigh
};

const osThreadAttr_t thread_midprio = {
	.priority = osPriorityAboveNormal
};

const osMutexAttr_t mutex_inherit = {
	.attr_bits = osMutexRecursive
};

int main(void) {
    // System Initialization
	  SystemCoreClockUpdate();

	  InitUART2(BAUD_RATE);
	  InitMotor();
	  InitAudio();

	  osKernelInitialize();
	  led_control(YELLOW);
    
    flags = osEventFlagsNew(NULL);
	  rxq = osMessageQueueNew(Q_SIZE, sizeof(UCHAR), NULL);
	  moveq = osMessageQueueNew(Q_SIZE, sizeof(UCHAR), NULL);
		green_led_mutex = osMutexNew(&mutex_inherit);
		red_led_mutex = osMutexNew(&mutex_inherit);
		audio_mutex = osMutexNew(&mutex_inherit);

    osThreadNew(tBrain, NULL, NULL); 
    osThreadNew(tEvent, NULL, &thread_midprio); 
		osThreadNew(tAudio, NULL, NULL);
    osThreadNew(tLED, NULL, NULL);
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
    if (osMessageQueueGet(moveq, &data, NULL, MOVE_DUR) == osOK) {
      osEventFlagsSet(flags, FLAG_MOVE);
      switch(data) { // TODO reorganise
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
    } else { // no command received, stop
      osEventFlagsClear(flags, FLAG_MOVE);
      TPM0_C0V = 0;
      TPM0_C1V = 0;
      TPM0_C2V = 0;
      TPM0_C3V = 0;
    }
	}
}

void tEvent(void *argument) {
	uint32_t events = osEventFlagsWait(flags, FLAG_CONN | FLAG_END, NULL, osWaitForever);
	if (events & FLAG_CONN) {
		osMutexAcquire(green_led_mutex, osWaitForever);		
		osMutexAcquire(audio_mutex, osWaitForever);
		// TODO flash green LEDs twice and play conn tune
		osMutexRelease(audio_mutex);
		osMutexRelease(green_led_mutex);
	} else if (events & FLAG_END) {
		osMutexAcquire(audio_mutex, osWaitForever);
		// TODO play end tune
		osMutexRelease(audio_mutex);
	}
}

void tAudio(void *argument) {
	uint32_t idx = 0;
	for (;;) {
		osMutexAcquire(audio_mutex, osWaitForever);
		stop_music();
		osDelay(100);
		play_note(song_notes[idx]);
		osMutexRelease(audio_mutex);
		osDelay(song_dur[idx]);
		idx = (idx + 1) % SONG_LEN;
	}
}

void tLED(void *argument) {
	uint8_t idx = 0;
  for (;;) {
		osMutexAcquire(green_led_mutex, osWaitForever);
		osMutexAcquire(red_led_mutex, osWaitForever);
    if (osEventFlagsWait(flags, FLAG_MOVE, osFlagsNoClear, 0) 
        == osOK) { // moving
      // turn off green_leds[idx]/turn off all greens
      idx = (idx + 1) % (sizeof(green_leds) / sizeof(green_leds[0]));
      // turn on green_leds[idx]
			// toggle red LEDs
			osMutexRelease(green_led_mutex);
			osMutexRelease(red_led_mutex);
			osDelay(250);
    } else {
			// turn on green LEDs
      // toggle red LEDs
			osMutexRelease(green_led_mutex);
			osMutexRelease(red_led_mutex);
			osDelay(500);
    }
  }
}
