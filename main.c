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
#define SONG_LEN (8 + 8 + 11 + 1)
#define CONNSONG_LEN 1
#define ENDSONG_LEN 1

uint8_t green_leds[] = {0, 0, 0, 0, 0, 0, 0, 0}; // TODO
// uint32_t song_notes[SONG_LEN] = {NOTE_F4, NOTE_E4, NOTE_F4, NOTE_D4, NOTE_E4, NOTE_C4, NOTE_D4, NOTE_D4, 0};
uint32_t song_notes[SONG_LEN] = {NOTE_C4, NOTE_D4, NOTE_F4, NOTE_D4, NOTE_F4, NOTE_F4, NOTE_E4, 
  NOTE_C4, NOTE_D4, NOTE_F4, NOTE_D4, NOTE_F4, NOTE_F4, NOTE_D4, 
  NOTE_C4, NOTE_D4, NOTE_F4, NOTE_D4, NOTE_C4, NOTE_G4, NOTE_E4, NOTE_C4, NOTE_G4, NOTE_F4,
  0};
uint32_t song_dur[SONG_LEN] = {DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_HALF, 
  DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_HALF, 
  DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_QUART, DUR_EIGHT, DUR_EIGHT, DUR_QUART, DUR_QUART, DUR_QUART, 
  DUR_HALF};
uint32_t conn_notes[CONNSONG_LEN] = {0};
uint32_t conn_dur[CONNSONG_LEN] = {0};
uint32_t end_notes[ENDSONG_LEN] = {0};
uint32_t end_dur[ENDSONG_LEN] = {0};

void tBrain(void *argument);
void tMotor(void *argument);
void tLED(void *argument);
void tEventLED(void *argument);
void tAudio(void *argument);
void tEventAudio(void *argument);

osThreadId_t event_audio_thread, event_led_thread;

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
	.attr_bits = osMutexPrioInherit
};

int main(void) {
    // System Initialization
	  SystemCoreClockUpdate();

	  InitUART2(BAUD_RATE);
		InitBoardLED(); // TODO remove when not needed
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
    event_audio_thread = osThreadNew(tEventAudio, NULL, &thread_midprio); 
		event_led_thread = osThreadNew(tEventLED, NULL, &thread_midprio); 
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
				osThreadFlagsSet(event_audio_thread, FLAG_CONN);
				osThreadFlagsSet(event_led_thread, FLAG_CONN);
        break;
      case CMD_END:
        osThreadFlagsSet(event_audio_thread, FLAG_END);
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

void tEventAudio(void *argument) {
	uint32_t events = osThreadFlagsWait(FLAG_CONN | FLAG_END, NULL, osWaitForever);
	if (events & FLAG_CONN) {
		osMutexAcquire(audio_mutex, osWaitForever);
		for (uint32_t idx = 0; idx < CONNSONG_LEN; ++idx) {
			stop_music();
			osDelay(0.1 * conn_dur[idx]);
			play_note(conn_notes[idx]);
			osDelay(end_dur[idx]);
		}
		osMutexRelease(audio_mutex);
	} else if (events & FLAG_END) {
		osMutexAcquire(audio_mutex, osWaitForever);
		for (uint32_t idx = 0; idx < ENDSONG_LEN; ++idx) {
			stop_music();
			osDelay(0.1 * end_dur[idx]);
			play_note(end_notes[idx]);
			osDelay(end_dur[idx]);
		}
		osMutexRelease(audio_mutex);
	}
}

void tEventLED(void *argument) {
	uint32_t events = osThreadFlagsWait(FLAG_CONN, NULL, osWaitForever);
	if (events & FLAG_CONN) {
		osMutexAcquire(green_led_mutex, osWaitForever);		
		// TODO flash green LEDs twice
		osMutexRelease(green_led_mutex);
	}
}

void tAudio(void *argument) {
	uint32_t idx = 0;
	for (;;) {
		osMutexAcquire(audio_mutex, osWaitForever);
		stop_music();
		osDelay(0.1 * song_dur[idx]);
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
