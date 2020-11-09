#include <stdint.h>

#define UCHAR unsigned char // byte
#define UINT unsigned int // word

#define ERR_NONE 0
#define ERR_BUFOVF 1
#define ERR_QEMPTY 2
#define ERR_UARTERR 3

enum color_t {NONE, RED, GREEN, BLUE, CYAN, YELLOW, MAGENTA, WHITE};

enum move_t {STOP, FORWARDS, BACKWARDS, CURVE_LEFT, CURVE_RIGHT, TURN_LEFT, TURN_RIGHT};

// note frequencies from https://pages.mtu.edu/~suits/notefreq432.html
#define NOTE_C4	256
#define NOTE_D4	288
#define NOTE_E4	323
#define NOTE_F4	342
#define NOTE_G4	384
#define NOTE_A4	432
#define NOTE_B4	484
#define NOTE_C5	513
#define NOTE_D5	576
#define NOTE_E5	647
#define NOTE_F5	685
#define NOTE_G5	769
#define NOTE_A5	864	
#define NOTE_B5	969

// durations
#define DUR_FULL 1000
#define DUR_HALF (DUR_FULL/2)
#define DUR_QUART (DUR_FULL/4)
#define DUR_EIGHT (DUR_FULL/8)
#define DUR_SXTN (DUR_FULL/16)
#define DUR_THTWO (DUR_FULL/32)

// driver functions
void InitUART2(uint32_t baud_rate);
void InitBoardLED(void);
void InitExtLED(void);
void InitMotor(void);
void InitAudio(void);

void play_note(uint32_t freq);
void stop_music(void);
void led_control(enum color_t color);
void motor_control(enum move_t move);
UINT uart_rx(UCHAR *buf, UINT len);

void led_toggle_red(void);
void led_on_green(uint32_t pin);
void led_off_green(uint32_t pin);
