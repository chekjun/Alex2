#include <stdint.h>

#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 0

#define UCHAR unsigned char // byte
#define UINT unsigned int // word

#define ERR_NONE 0
#define ERR_BUFOVF 1
#define ERR_QEMPTY 2
#define ERR_UARTERR 3

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK32(x) ((uint32_t)(1 << ((uint32_t)x))) // Changes all bits to 0 except x
enum color_t {NONE, RED, GREEN, BLUE, CYAN, YELLOW, MAGENTA, WHITE};

// PWM Pins
#define SPEAKER 0
#define PTD0_Pin 0
#define PTD1_Pin 1
#define PTD2_Pin 2
#define PTD3_Pin 3

#define FREQUENCY_TO_MOD(x) (375000 / x)

// notes from https://pages.mtu.edu/~suits/notefreq432.html
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
UINT uart_rx(UCHAR *buf, UINT len);
