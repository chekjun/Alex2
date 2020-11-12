#include <stdint.h>

#ifndef DRIVER_H
#define DRIVER_H

#define UCHAR unsigned char // byte
#define UINT unsigned int // word

#define ERR_NONE 0
#define ERR_BUFOVF 1
#define ERR_QEMPTY 2
#define ERR_UARTERR 3

enum color_t {NONE, RED, GREEN, BLUE, CYAN, YELLOW, MAGENTA, WHITE};

const static int FRONT_LEDS[] = {7, 0, 3, 4, 5, 6, 10, 11, 12, 13}; // PortC Pin 7 upwards to Pin 13

enum move_t {STOP, FORWARDS, BACKWARDS, CURVE_LEFT, CURVE_RIGHT, TURN_LEFT, TURN_RIGHT};

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

#endif // DRIVER_H
