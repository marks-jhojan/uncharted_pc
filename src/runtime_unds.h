/* UNDS event bus: the runtime publishes what the game does with the controller and the audio
 * (rumble, light bar, pad state, audio levels) as UDP datagrams for an external companion
 * (DualSense adaptive triggers and haptics). Off unless UNDS_EVENTS=host:port is set; then each
 * event is one non-blocking sendto of a compact JSON object. Nothing here waits on the network.
 *
 *   UNDS_EVENTS=127.0.0.1:47111   destination (numeric IPv4, "localhost", or a port alone)
 *   UNDS_RUMBLE=0                 scePadSetVibration no longer drives SDL rumble (still published)
 *   UNDS_LED=0                    the light bar no longer goes to SDL (still published): the
 *                                 companion (DSX, later SDL in-process) owns the LED
 *   UNDS_TRIGGER=<l2>,<r2>        trigger travel (1-254 of 255) above which L2 / R2 count as
 *                                 pressed; default 30,30 (the port's built-in threshold)
 *   UNDS_STATE=0                  no game-state callbacks (health, weapon, ammunition, damage;
 *                                 on by default with UNDS_EVENTS, U1 1.00 only so far)
 *   UNDS_STATE_DEBUG=1            the objects the game-state callbacks find, in the log
 *   UNDS_FX=0                     no control channel (on by default with UNDS_EVENTS, see below)
 * UNDS_RUMBLE, UNDS_LED and UNDS_TRIGGER work with or without UNDS_EVENTS.
 *
 * Control channel ("fx", companion -> port, on with UNDS_EVENTS unless UNDS_FX=0): the companion
 * answers to the address the events come from with 37-byte binary datagrams that set the
 * DualSense adaptive triggers and light bar; the port writes them with SDL_SendGamepadEffect, so
 * SDL stays the only writer of the controller's output report (no hidapi in the companion, USB
 * and Bluetooth alike). Only datagrams from the UNDS_EVENTS host are taken; the socket is bound
 * to 127.0.0.1 when that host is loopback.
 *   0 "UNDC"  4 version (1)  5 flags: 1 R2, 2 L2, 4 light bar, 8 release (both triggers off, the
 *   light bar back to the game; the other bits are then ignored)  6-7 zero  8 seq (u32 LE)
 *   12 R2 effect (11 bytes)  23 L2 effect (11 bytes)  34 r g b
 * Trigger effects are the controller's own 11-byte blocks (mode byte first; modes 0x01 0x02 0x05
 * 0x06 0x11 0x12 0x21 0x22 0x23 0x25 0x26 0x27 only). Only the trigger and light bar bits of the
 * report are ever set: never rumble emulation (which turns the audio haptics off). While the
 * companion holds the light bar, the game's scePadSetLightBar is published but not shown.
 * Without a datagram for 5 s (the companion re-sends its state every 2 s), at "bye" and on
 * "release", the triggers go off and the light bar goes back to the game; after a controller
 * reconnects, the held state is written again. Every command gets an "fx" event back.
 *
 * Protocol v1: every datagram is one JSON object with
 *   "v":1  "seq":<uint32, per process>  "t":<CLOCK_MONOTONIC us>  "e":"<event>"  ...fields
 * Only integers are sent (no locale-dependent float formatting). See runtime_unds.c. */
#ifndef BB_RUNTIME_UNDS_H
#define BB_RUNTIME_UNDS_H
#include <stdint.h>

#define UNDS_PROTOCOL 1
#define UNDS_VERSION "0.6.0"

/* Control datagram (see above). */
#define UNDS_FX_SIZE 37
#define UNDS_FX_RIGHT 0x01
#define UNDS_FX_LEFT 0x02
#define UNDS_FX_LED 0x04
#define UNDS_FX_RELEASE 0x08
/* SDL_hidapi_ps5.c DS5EffectsState_t (the USB report after its id): what runtime_pad_unds_effect takes. */
#define UNDS_DS5_EFFECTS_SIZE 47
#define UNDS_DS5_ENABLE1 0      /* 0x04 right trigger, 0x08 left trigger */
#define UNDS_DS5_ENABLE2 1      /* 0x04 light bar colour */
#define UNDS_DS5_RIGHT 10
#define UNDS_DS5_LEFT 21
#define UNDS_DS5_RGB 44

/* runtime_pad.c, for the control channel (any thread; they take the pad lock themselves).
 * runtime_pad_unds_effect writes one DS5EffectsState_t through SDL_SendGamepadEffect: 1 sent,
 * 0 no controller yet (or the game has not opened the pad), -1 not a DualSense, -2 SDL refused
 * (both logged once). runtime_pad_unds_generation counts controller connections (a change: the
 * held effects must be written again). runtime_pad_unds_game_led puts the game's colour back
 * through SDL (UNDS_LED=0 still wins). */
int runtime_pad_unds_effect(const uint8_t *effects, int size);
unsigned runtime_pad_unds_generation(void);
void runtime_pad_unds_game_led(uint8_t r, uint8_t g, uint8_t b);
/* 1 while the companion holds the light bar (set_led then leaves SDL alone). */
int runtime_unds_fx_led_owned(void);

/* Reads UNDS_EVENTS / UNDS_RUMBLE / UNDS_LED / UNDS_TRIGGER; with UNDS_EVENTS, opens the socket, sends "hello" and starts
 * the 1 s heartbeat. `image` is the guest image base. Silent when neither variable is set. */
void runtime_unds_start(uintptr_t image);
/* 1 unless UNDS_RUMBLE=0. */
int runtime_unds_rumble_enabled(void);
/* 1 unless UNDS_LED=0. */
int runtime_unds_led_enabled(void);
/* Press threshold of L2 (right=0) or R2 (right=1) on the 0-255 travel: 30, or UNDS_TRIGGER. */
uint8_t runtime_unds_trigger_threshold(int right);
/* "bye" with a reason ("loadexec", "exit", "restart", "window", ...), at most once per process. */
void runtime_unds_bye(const char *reason);
/* scePadSetVibration: large and small motor, 0-255 (sent when they change, refreshed every 0.5 s). */
void runtime_unds_rumble(uint8_t large, uint8_t small);
/* Light bar colour (sent when it changes). */
void runtime_unds_led(uint8_t r, uint8_t g, uint8_t b);
/* The pad state the game read (after injection/replay), plus the physical L2/R2 travel (l2a,
 * r2a: 0-255 before the press threshold; the game itself gets 0 or 255): sent when it changes,
 * at most every 8 ms; a change held back by the limit is sent by a later read or by the
 * heartbeat. */
void runtime_unds_pad(uint32_t buttons, uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry, uint8_t l2, uint8_t r2,
                      uint8_t l2a, uint8_t r2a);
/* An audio port was opened: resets that port's analysis and announces it. */
void runtime_unds_audio_open(int port, int type, int channels, int frames);
/* One output buffer of a port, after volume and channel mapping (SDL order for 8 channels:
 * FL FR FC LFE BL BR SL SR). Levels are analysed in 512-frame windows (10.7 ms at 48 kHz). Call
 * with the port's lock held (one writer per port). */
void runtime_unds_audio(int port, int type, const void *pcm, int frames, int channels, int is_float);
/* Counters for runtime_report (prints nothing when the bus is off). */
void runtime_unds_report(void);
#endif
