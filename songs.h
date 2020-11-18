#include <stdint.h>

#ifndef SONGS_H
#define SONGS_H

// note frequencies from https://pages.mtu.edu/~suits/notefreq432.html
#define NOTE_A3 216
#define NOTE_C4	256
#define NOTE_C4S	272
#define NOTE_D4	288
#define NOTE_D4S	305
#define NOTE_E4	323
#define NOTE_F4	342
#define NOTE_F4S	363
#define NOTE_G4	384
#define NOTE_G4S	407
#define NOTE_A4	432
#define NOTE_A4S	457
#define NOTE_B4	484
#define NOTE_C5	513
#define NOTE_C5S	544
#define NOTE_D5	576
#define NOTE_D5S	610
#define NOTE_E5	647
#define NOTE_F5	685
#define NOTE_F5S 726
#define NOTE_G5	769
#define NOTE_G5S	815
#define NOTE_A5	864
#define NOTE_A5S	915
#define NOTE_B5	969
#define NOTE_C6	1027
#define NOTE_C6S	1088
#define NOTE_D6	1153
#define NOTE_D6S	1221
#define NOTE_E6	1294
#define NOTE_F6	1371
#define NOTE_F6S	1453
#define NOTE_G6	1539
#define NOTE_G6S	1631
#define NOTE_A6	1728
#define NOTE_A6S	1830
#define NOTE_B6	1939

// durations
#define DUR_FULL 1000
#define DUR_HALF (DUR_FULL/2)
#define DUR_QUART (DUR_FULL/4)
#define DUR_EIGHT (DUR_FULL/8)
#define DUR_SXTN (DUR_FULL/16)
#define DUR_THTWO (DUR_FULL/32)

// uint32_t song_notes[SONG_LEN] = {NOTE_F4, NOTE_E4, NOTE_F4, NOTE_D4, NOTE_E4, NOTE_C4, NOTE_D4, NOTE_D4, 0};

#define SONG_LEN 314
const uint32_t song_notes[SONG_LEN] = {0, NOTE_G5, NOTE_C5, NOTE_G5, NOTE_A5, NOTE_C6, NOTE_A5S, NOTE_A5,
NOTE_F5, NOTE_G5, NOTE_C5, NOTE_C5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_F5,
NOTE_F5, NOTE_G5, NOTE_C5, NOTE_G5, NOTE_A5, NOTE_C6, NOTE_A5S, NOTE_A5,
NOTE_F5, NOTE_G5, NOTE_C5, NOTE_C5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_F5,
NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_C5,
NOTE_D5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_D5, NOTE_C5, NOTE_C6, NOTE_C6,
NOTE_G5, NOTE_D5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_D5, NOTE_F5, NOTE_G5,
NOTE_E5, NOTE_D5, NOTE_C5, NOTE_D5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_D5,
NOTE_C5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_G5,
NOTE_A5, NOTE_F5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_C5,
NOTE_D5, NOTE_E5, NOTE_F5, NOTE_D5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_C5,
NOTE_D5, NOTE_F5, NOTE_D5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_C5, NOTE_D5,
NOTE_F5, NOTE_D5, NOTE_G5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5,
NOTE_D5, NOTE_F5, NOTE_D5, NOTE_F5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_C5,
NOTE_C5, NOTE_C5, NOTE_G5, NOTE_F5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_D5,
NOTE_A5, NOTE_A5, NOTE_G5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_D5, NOTE_C6,
NOTE_E5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_D5,
NOTE_F5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_C5, NOTE_G5, NOTE_F5,
NOTE_D5, NOTE_F5, NOTE_D5, NOTE_F5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_C5,
NOTE_D5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_D5, NOTE_C5, NOTE_C6, NOTE_C6,
NOTE_G5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_D5, NOTE_F5, NOTE_D5, NOTE_F5,
NOTE_D5, NOTE_F5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_D5, NOTE_D5,
NOTE_E5, NOTE_F5, NOTE_D5, NOTE_C5, NOTE_G5, NOTE_G5, NOTE_A5, NOTE_G5,
NOTE_F5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_A5, NOTE_G5,
NOTE_C5, NOTE_C5, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_D5, NOTE_G5,
NOTE_A5, NOTE_G5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_D5, NOTE_A5, NOTE_A5,
NOTE_G5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_D5, NOTE_G5, NOTE_G5, NOTE_F5,
NOTE_E5, NOTE_D5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_D5, NOTE_F5, NOTE_G5,
NOTE_E5, NOTE_D5, NOTE_C5, NOTE_C5, NOTE_G5, NOTE_F5, NOTE_C5, NOTE_D5,
NOTE_F5, NOTE_D5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_C5, NOTE_D5, NOTE_F5,
NOTE_D5, NOTE_C6, NOTE_E5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_D5,
NOTE_F5, NOTE_D5, NOTE_F5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_C5,
NOTE_G5, NOTE_F5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_D5, NOTE_A5, NOTE_A5,
NOTE_G5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_D5, NOTE_G5, NOTE_G5, NOTE_F5,
NOTE_E5, NOTE_D5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_D5, NOTE_F5, NOTE_G5,
NOTE_E5, NOTE_D5, NOTE_C5, NOTE_C5, NOTE_C5, NOTE_G5, NOTE_F5, NOTE_C5,
NOTE_D5, NOTE_F5, NOTE_D5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_C5, NOTE_D5,
NOTE_F5, NOTE_D5, NOTE_G5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5,
NOTE_D5, NOTE_F5, NOTE_D5, NOTE_F5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_C5,
NOTE_C5, NOTE_G5};
const uint32_t song_dur[SONG_LEN] = {1000, 1200, 800, 1200, 1200, 200, 200, 400,
1200, 1200, 2800, 200, 200, 200, 400, 200,
1200, 1200, 800, 1200, 1200, 200, 200, 400,
1200, 1200, 2800, 200, 200, 200, 400, 1000,
400, 400, 400, 400, 400, 600, 200, 3200,
400, 400, 400, 400, 800, 400, 800, 400,
2000, 400, 400, 400, 400, 400, 400, 1200,
400, 400, 2400, 400, 400, 400, 400, 400,
800, 400, 400, 400, 400, 1600, 2000, 400,
400, 400, 400, 400, 400, 400, 800, 2400,
400, 400, 400, 800, 400, 400, 1200, 200,
200, 200, 200, 600, 600, 1200, 200, 200,
200, 200, 600, 600, 600, 200, 400, 200,
200, 200, 200, 800, 400, 600, 200, 400,
400, 400, 800, 1600, 200, 200, 200, 200,
600, 600, 1200, 200, 200, 200, 200, 800,
400, 600, 200, 400, 200, 200, 200, 200,
800, 400, 600, 200, 800, 400, 800, 2800,
400, 400, 400, 400, 1600, 400, 400, 2400,
400, 400, 400, 400, 400, 1200, 400, 400,
800, 400, 400, 800, 400, 400, 400, 400,
400, 400, 800, 400, 400, 2000, 400, 400,
400, 400, 400, 1600, 400, 400, 800, 1200,
2000, 400, 400, 800, 400, 400, 400, 400,
400, 2000, 400, 400, 400, 400, 800, 400,
400, 1200, 200, 200, 200, 200, 600, 600,
1200, 200, 200, 200, 200, 600, 600, 600,
200, 400, 200, 200, 200, 200, 800, 400,
600, 200, 800, 400, 800, 1600, 200, 200,
200, 200, 600, 600, 1200, 200, 200, 200,
200, 800, 400, 600, 200, 400, 200, 200,
200, 200, 800, 400, 600, 200, 800, 400,
800, 1600, 200, 200, 200, 200, 600, 600,
1200, 200, 200, 200, 200, 600, 600, 600,
200, 400, 200, 200, 200, 200, 800, 400,
600, 200, 400, 400, 400, 800, 1600, 200,
200, 200, 200, 600, 600, 1200, 200, 200,
200, 200, 600, 600, 600, 200, 400, 200,
200, 200, 200, 800, 400, 600, 200, 800,
400, 800};

#define ENDSONG_LEN 466
const uint32_t end_notes[ENDSONG_LEN] = {0, NOTE_F4S, NOTE_B4, NOTE_C5S, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_C5S,
NOTE_B4, NOTE_A4, NOTE_G4, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_A4, NOTE_D5,
NOTE_A4, NOTE_D5, NOTE_A4, NOTE_D5, NOTE_E5, NOTE_C5S, NOTE_B4, NOTE_F4S,
NOTE_B4, NOTE_C5S, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_C5S, NOTE_B4, NOTE_A4,
NOTE_G4, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_D5, NOTE_A4, NOTE_D5, NOTE_A4,
NOTE_D5, NOTE_A4, NOTE_D5, NOTE_E5, NOTE_C5S, NOTE_B4, NOTE_B4, NOTE_B4,
NOTE_B4, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_D5, NOTE_C5S,
NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_D5,
NOTE_C5S, NOTE_D5, NOTE_E5, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4,
NOTE_D5, NOTE_A4, NOTE_D5, NOTE_A4, NOTE_D5, NOTE_E5, NOTE_E5, NOTE_C5S,
NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_D5,
NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C5S,
NOTE_D5, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_D5, NOTE_E5, NOTE_A4, NOTE_A4,
NOTE_A4, NOTE_A4, NOTE_A4, NOTE_D5, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_D5,
NOTE_E5, NOTE_E5, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_B4, NOTE_F4S, NOTE_F4S,
NOTE_F4S, NOTE_F4S, NOTE_F4S, NOTE_F4S, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4,
NOTE_A4, NOTE_B4, NOTE_B4, NOTE_G4, NOTE_G4, NOTE_G4, NOTE_G4, NOTE_G4,
NOTE_G4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C5S, NOTE_D5, NOTE_A4,
NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_D5, NOTE_D5, NOTE_D5,
NOTE_D5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_B4,
NOTE_F4S, NOTE_F4S, NOTE_F4S, NOTE_F4S, NOTE_F4S, NOTE_F4S, NOTE_B4, NOTE_B4,
NOTE_B4, NOTE_B4, NOTE_A4, NOTE_B4, NOTE_B4, NOTE_G4, NOTE_G4, NOTE_G4,
NOTE_G4, NOTE_G4, NOTE_G4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C5S,
NOTE_D5, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_D5,
NOTE_D5, NOTE_D5, NOTE_D5, NOTE_E5, NOTE_E5, NOTE_C5S, NOTE_F5S, NOTE_E5,
NOTE_F5S, NOTE_E5, NOTE_F5S, NOTE_E5, NOTE_F5S, NOTE_E5, NOTE_F5S, NOTE_E5,
NOTE_F5S, NOTE_G5, NOTE_G5, NOTE_D5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5,
NOTE_G5, NOTE_A5, NOTE_A5, NOTE_F5S, NOTE_F5S, NOTE_F5S, NOTE_F5S, NOTE_F5S,
NOTE_A5, NOTE_G5, NOTE_F5S, NOTE_E5, NOTE_F5S, NOTE_E5, NOTE_F5S, NOTE_E5,
NOTE_F5S, NOTE_E5, NOTE_F5S, NOTE_E5, NOTE_F5S, NOTE_E5, NOTE_F5S, NOTE_G5,
NOTE_G5, NOTE_D5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_A5,
NOTE_A5, NOTE_F5S, NOTE_F5S, NOTE_F5S, NOTE_F5S, NOTE_F5S, NOTE_A5, NOTE_G5,
NOTE_F5S, NOTE_E5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5,
NOTE_B4, NOTE_D5, NOTE_B4, NOTE_B4, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_B4,
NOTE_D5, NOTE_B4, NOTE_B4, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_B4, NOTE_D5,
NOTE_B4, NOTE_B4, NOTE_C5S, NOTE_D5, NOTE_C5S, NOTE_B4, NOTE_D5, NOTE_B4,
NOTE_E5, NOTE_E5, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5,
NOTE_E5, NOTE_F5S, NOTE_E5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_E5,
NOTE_E5, NOTE_D5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_F5S,
NOTE_E5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_B4, NOTE_D5, NOTE_C5S, NOTE_D5,
NOTE_B4, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5,
NOTE_B4, NOTE_E5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5,
NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_B4, NOTE_E5,
NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_G5, NOTE_G5,
NOTE_F5S, NOTE_A5, NOTE_A5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_F5S,
NOTE_F5S, NOTE_F5S, NOTE_F5S, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_D5,
NOTE_E5, NOTE_D5, NOTE_F4S, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_B4, NOTE_B4,
NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_B4, NOTE_B4, NOTE_D5, NOTE_D5,
NOTE_D5, NOTE_D5, NOTE_B4, NOTE_G4, NOTE_G4, NOTE_D5, NOTE_D5, NOTE_D5,
NOTE_B4, NOTE_B4, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_D5,
NOTE_E5, NOTE_E5, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_D5, NOTE_D5,
NOTE_D5, NOTE_D5, NOTE_C5S, NOTE_C5S, NOTE_C5S, NOTE_C5S, NOTE_D5, NOTE_B4,
NOTE_D5, NOTE_B4, NOTE_F4S, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_B4, NOTE_B4,
NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_B4, NOTE_B4, NOTE_D5, NOTE_D5,
NOTE_D5, NOTE_D5, NOTE_B4, NOTE_G4, NOTE_G4, NOTE_D5, NOTE_D5, NOTE_D5,
NOTE_B4, NOTE_B4, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_D5,
NOTE_E5, NOTE_E5, NOTE_E5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_D5, NOTE_D5,
NOTE_D5, NOTE_D5, NOTE_C5S, NOTE_C5S, NOTE_C5S, NOTE_C5S, NOTE_D5, NOTE_B4,
NOTE_D5, 0};
const uint32_t end_dur[ENDSONG_LEN] = {800, 200, 200, 200, 200, 400, 200, 200,
400, 200, 600, 600, 2000, 400, 400, 400,
400, 400, 400, 400, 200, 3400, 1000, 200,
200, 200, 200, 400, 200, 200, 400, 200,
600, 600, 400, 1600, 400, 400, 400, 400,
400, 400, 400, 200, 3400, 800, 200, 200,
200, 200, 200, 200, 400, 200, 400, 200,
800, 200, 200, 200, 200, 200, 200, 400,
200, 400, 200, 800, 200, 200, 200, 200,
200, 400, 200, 200, 200, 200, 400, 3000,
800, 200, 200, 200, 200, 200, 200, 200,
400, 400, 200, 800, 200, 200, 200, 200,
200, 200, 200, 200, 400, 200, 1000, 200,
200, 200, 200, 200, 200, 200, 200, 400,
200, 400, 1400, 800, 800, 400, 400, 200,
200, 200, 200, 200, 200, 200, 200, 400,
200, 400, 200, 400, 200, 200, 200, 200,
200, 200, 200, 200, 400, 200, 400, 600,
200, 200, 200, 200, 200, 200, 200, 200,
400, 200, 200, 200, 1400, 800, 800, 400,
400, 200, 200, 200, 200, 200, 200, 200,
200, 400, 200, 400, 200, 400, 200, 200,
200, 200, 200, 200, 200, 200, 400, 200,
400, 600, 200, 200, 200, 200, 200, 200,
200, 200, 200, 400, 400, 3000, 200, 200,
200, 200, 200, 400, 400, 200, 200, 400,
400, 200, 400, 1200, 200, 200, 200, 400,
200, 200, 400, 1400, 200, 200, 200, 400,
200, 200, 400, 3000, 200, 200, 200, 200,
200, 200, 600, 400, 200, 200, 400, 200,
400, 1200, 200, 200, 200, 200, 400, 200,
400, 1400, 200, 200, 200, 400, 200, 200,
400, 1600, 200, 200, 200, 200, 200, 200,
200, 200, 400, 200, 200, 200, 200, 200,
200, 400, 200, 200, 200, 200, 200, 200,
400, 200, 200, 200, 200, 200, 200, 600,
200, 200, 200, 200, 200, 200, 200, 200,
200, 200, 200, 200, 400, 400, 200, 200,
200, 200, 200, 200, 200, 200, 200, 200,
200, 200, 400, 200, 200, 200, 200, 200,
200, 200, 200, 200, 200, 200, 200, 200,
200, 400, 400, 200, 200, 200, 200, 200,
200, 200, 200, 200, 200, 200, 200, 400,
400, 200, 200, 200, 200, 200, 200, 200,
200, 200, 200, 200, 200, 400, 400, 200,
200, 200, 200, 200, 400, 400, 400, 200,
400, 200, 200, 400, 200, 200, 400, 400,
200, 200, 200, 200, 400, 400, 200, 200,
200, 200, 400, 200, 200, 400, 200, 200,
400, 400, 200, 200, 200, 200, 400, 400,
200, 200, 200, 200, 400, 400, 200, 200,
200, 200, 200, 400, 400, 200, 200, 200,
400, 200, 200, 400, 200, 200, 400, 400,
200, 200, 200, 200, 400, 400, 200, 200,
200, 200, 400, 200, 200, 400, 200, 200,
400, 400, 200, 200, 200, 200, 400, 200,
400, 200, 200, 200, 400, 400, 200, 200,
200, 200, 200, 400, 400, 200, 200, 200,
400, 1000};

#define CONNSONG_LEN 16
const uint32_t conn_notes[CONNSONG_LEN] = {NOTE_D4, NOTE_F4S, NOTE_A4, NOTE_D5, NOTE_C5S, NOTE_C5S, NOTE_A4,
NOTE_B4, NOTE_B4, NOTE_G4, NOTE_A4, NOTE_A3, NOTE_C4S, NOTE_E4, NOTE_G4,
NOTE_F4S};
const uint32_t conn_dur[CONNSONG_LEN] = {200, 200, 200, 200, 400, 200, 200,
400, 200, 200, 800, 200, 200, 200, 200,
1200};

#endif // SONGS_H
