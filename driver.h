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

// notes
#define NOTE_C 262
#define NOTE_D 294
#define NOTE_E 330
#define NOTE_F 349
#define NOTE_G 392
#define NOTE_A 440
#define NOTE_B 494

// durations
#define DUR_FULL 1000
#define DUR_HALF (DUR_FULL/2)
#define DUR_QUART (DUR_FULL/4)

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
