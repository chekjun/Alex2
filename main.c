#include "RTE_Components.h"
#include "MKL25Z4.h"
#include "cmsis_os2.h"
#include "driver.h"
#include "songs.h"

// UART settings
#define Q_SIZE (16)
#define BAUD_RATE 9600
#define CMD_CONN 0x01
#define CMD_END 0xFF
#define CMD_W 'W'
#define CMD_A 'A'
#define CMD_S 'S'
#define CMD_D 'D'
#define CMD_Q 'Q'
#define CMD_E 'E'

// event flags
#define FLAG_CONN ((uint32_t) 0x00000001)
#define FLAG_END  ((uint32_t) 0x00000010)
#define FLAG_MOVE ((uint32_t) 0x00000100)

// movement settings
#define MOVE_DUR 200

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
		//InitBoardLED();
		InitExtLED();
	  InitMotor();
	  InitAudio();

	  osKernelInitialize();
    
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
      switch(data) {
				case CMD_W:
					motor_control(FORWARDS);
					break;
				case CMD_A:
					motor_control(TURN_LEFT);
					break;
				case CMD_S:
					motor_control(BACKWARDS);
					break;
				case CMD_D:
					motor_control(TURN_RIGHT);
					break;
				case CMD_Q:
					motor_control(CURVE_LEFT);
					break;
				case CMD_E:
					motor_control(CURVE_RIGHT);
					break;
			}
    } else { // no command received, stop
      osEventFlagsClear(flags, FLAG_MOVE);
			motor_control(STOP);
    }
	}
}

void tEventAudio(void *argument) {
	uint32_t events = osThreadFlagsWait(FLAG_CONN | FLAG_END, NULL, osWaitForever);
	if (events & FLAG_CONN) {
		osMutexAcquire(audio_mutex, osWaitForever);
		for (uint32_t idx = 0; idx < CONNSONG_LEN; ++idx) {
			stop_music();
			// osDelay(0.1 * conn_dur[idx]);
			osDelay(20);
			play_note(conn_notes[idx]);
			osDelay(conn_dur[idx]);
		}
		osMutexRelease(audio_mutex);
	} else if (events & FLAG_END) {
		osMutexAcquire(audio_mutex, osWaitForever);
		for (uint32_t idx = 0; idx < ENDSONG_LEN; ++idx) {
			stop_music();
			// osDelay(0.1 * end_dur[idx]);
			osDelay(20);
			play_note(end_notes[idx]);
			osDelay(end_dur[idx]);
		}
		osMutexRelease(audio_mutex);
	}
}

void tEventLED(void *argument) {
	uint32_t events = osThreadFlagsWait(FLAG_CONN, NULL, osWaitForever);
	if (events & FLAG_CONN) { // flash green leds
		osMutexAcquire(green_led_mutex, osWaitForever);		
		for (uint32_t i = 0; i < FRONT_LEDS_LEN; ++i) {
				led_on_green(FRONT_LEDS[i]);
		}
		osDelay(250);
		for (uint32_t i = 0; i < FRONT_LEDS_LEN; ++i) {
				led_off_green(FRONT_LEDS[i]);
		}
		osDelay(250);
		for (uint32_t i = 0; i < FRONT_LEDS_LEN; ++i) {
				led_on_green(FRONT_LEDS[i]);
		}
		osDelay(250);
		for (uint32_t i = 0; i < FRONT_LEDS_LEN; ++i) {
				led_off_green(FRONT_LEDS[i]);
		}
		osDelay(250);
		osMutexRelease(green_led_mutex);
	}
}

void tAudio(void *argument) {
	uint32_t idx = 0;
	for (;;) {
		osMutexAcquire(audio_mutex, osWaitForever);
		stop_music();
		// osDelay(0.1 * song_dur[idx]);
		osDelay(20);
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
			led_off_green(FRONT_LEDS[idx]);
      idx = (idx + 1) % FRONT_LEDS_LEN;
      led_on_green(FRONT_LEDS[idx]);
			led_toggle_red();
			osMutexRelease(green_led_mutex);
			osMutexRelease(red_led_mutex);
			osDelay(250);
    } else { // stationary
			for (uint32_t i = 0; i < FRONT_LEDS_LEN; ++i) {
				led_on_green(FRONT_LEDS[i]);
			}
      led_toggle_red();
			osMutexRelease(green_led_mutex);
			osMutexRelease(red_led_mutex);
			osDelay(500);
    }
  }
}
