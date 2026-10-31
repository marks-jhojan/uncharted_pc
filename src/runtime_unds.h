/* UNDS event bus: the runtime publishes what the game does with the controller and the audio
 * (rumble, light bar, pad state, audio levels) as UDP datagrams for an external companion
 * (DualSense adaptive triggers and haptics). Off unless UNDS_EVENTS=host:port is set; then each
 * event is one non-blocking sendto of a compact JSON object. Nothing here waits on the network.
 *
 *   UNDS_EVENTS=127.0.0.1:47111   destination (numeric IPv4, "localhost", or a port alone)
 *   UNDS_RUMBLE=0                 scePadSetVibration no longer drives SDL rumble (still published)
 *
 * Protocol v1: every datagram is one JSON object with
 *   "v":1  "seq":<uint32, per process>  "t":<CLOCK_MONOTONIC us>  "e":"<event>"  ...fields
 * Only integers are sent (no locale-dependent float formatting). See runtime_unds.c. */
#ifndef BB_RUNTIME_UNDS_H
#define BB_RUNTIME_UNDS_H
#include <stdint.h>

#define UNDS_PROTOCOL 1
#define UNDS_VERSION "0.2.0"

/* Reads UNDS_EVENTS / UNDS_RUMBLE; with UNDS_EVENTS, opens the socket, sends "hello" and starts
 * the 1 s heartbeat. `image` is the guest image base. Silent when neither variable is set. */
void runtime_unds_start(uintptr_t image);
/* 1 unless UNDS_RUMBLE=0. */
int runtime_unds_rumble_enabled(void);
/* "bye" with a reason ("loadexec", "exit", "restart", "window", ...), at most once per process. */
void runtime_unds_bye(const char *reason);
/* scePadSetVibration: large and small motor, 0-255 (sent when they change, refreshed every 0.5 s). */
void runtime_unds_rumble(uint8_t large, uint8_t small);
/* Light bar colour (sent when it changes). */
void runtime_unds_led(uint8_t r, uint8_t g, uint8_t b);
/* The pad state the game read (after injection/replay): sent when it changes, at most every 8 ms;
 * a change held back by the limit is sent by a later read or by the heartbeat. */
void runtime_unds_pad(uint32_t buttons, uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry, uint8_t l2, uint8_t r2);
/* An audio port was opened: resets that port's analysis and announces it. */
void runtime_unds_audio_open(int port, int type, int channels, int frames);
/* One output buffer of a port, after volume and channel mapping (SDL order for 8 channels:
 * FL FR FC LFE BL BR SL SR). Levels are analysed in 512-frame windows (10.7 ms at 48 kHz). Call
 * with the port's lock held (one writer per port). */
void runtime_unds_audio(int port, int type, const void *pcm, int frames, int channels, int is_float);
/* Counters for runtime_report (prints nothing when the bus is off). */
void runtime_unds_report(void);
#endif
