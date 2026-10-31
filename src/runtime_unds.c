/* UNDS event bus (see runtime_unds.h): UDP datagrams with what the game does with the pad and
 * the audio, for an external DualSense companion. Without UNDS_EVENTS every entry point returns
 * after one relaxed atomic load and nothing is printed.
 *
 * Design:
 * - One datagram per event, compact JSON, integers only (the guest may change the C locale; a
 *   float printed as "0,5" would break the parser). ~60-150 bytes; one sendto (non-blocking
 *   socket, numeric address: no DNS) of ~10-30 us on loopback, from the calling thread. A
 *   sender thread would add a wakeup (and its latency) to every event for no gain at these
 *   rates (~100-400 events/s). The time of every sendto is measured: the heartbeat reports the
 *   longest, so a blocking send would show.
 * - Change detection runs under a small lock of its own; the send happens after it is released.
 *   The pad and audio hooks call in after (pad) or under (audio, per port: one writer) the
 *   port's locks, never taking those locks here.
 * - "seq" counts datagrams handed to sendto in this process: a gap at the receiver is a loss
 *   (a datagram that sendto refused counts as a gap too). "t" is CLOCK_MONOTONIC in us.
 * - Heartbeat thread (250 ms tick): "hello" every 1 s (a companion started late, or restarted,
 *   picks up the emitter), rumble refresh every 0.5 s, pad changes held back by the rate limit,
 *   error summary at most every 10 s.
 *
 * Events (fields after v, seq, t, e):
 *   hello  boot(1 first, 0 heartbeat) exe(u1 u2 u3 or file) bin pid base("0x...") ver up(ms)
 *          rumble(1 SDL rumble on) led(1 SDL light bar on) trig("l2,r2" press thresholds)
 *          sent drop err smax(longest sendto since last hello, us)
 *          vib(scePadSetVibration calls) ports("id:type:channels:frames,...")
 *          fx(1 control channel open) fxp(its UDP port) fxc fxr fxe (commands taken, datagrams
 *          rejected, SDL failures) fxo(UNDS_FX_* bits the companion holds now)
 *   bye    reason
 *   fx     the control channel did something: c (command seq, 0 if not a command) f (its flags)
 *          ok (1 written, 0 no controller yet, -1 not a DualSense, -2 SDL refused) q (us from
 *          the datagram to SDL's return) why (cmd, release, reconnect, timeout)
 *   rumble l s (0-255) re(1 refresh)
 *   led    r g b re(1 refresh)
 *   pad    b(button mask, BTN_* of runtime_pad.c) lx ly rx ry l2 r2 (0-255, what the game read;
 *          the port gives 0 or 255 for the triggers) l2a r2a (physical trigger travel 0-255)
 *   aopen  p(port) ty(type) ch fr(frames per buffer)
 *   audio  p ty f(first frame of the window, port timeline) n(frames) rms pk lf
 *          levels x10000 of full scale over the window: rms of L/R (8 channels folded like the
 *          stereo output: centre and surrounds at -3 dB, LFE dropped), peak |L|,|R|, and rms
 *          of (L+R)/2 low-passed at 150 Hz (one pole). Digital silence (pk 0) is sent once and
 *          then refreshed every 1 s instead of every window.
 *   gs     game state (UNDS_STATE, see the "Game state" section; U1 and U2):
 *          hp (player health x1000, 0-1000; -1 unknown) inj (1 injured) ws (weapon controller
 *          slot index, -1 none) w ("0x..." weapon type string id, "0x0" none) clip res cap (rounds
 *          in the clip, in reserve, clip size; -1 without a weapon) gr (grenades, -1 unknown);
 *          refreshes (re:1, every 0.5 s) add ups/wps (player / weapon controller updates in the
 *          last 0.5 s: 0 while the game does not run the player, e.g. cutscenes [HIPÓTESIS]);
 *          U2 adds upc (calls of a helper of the player's controllers) and upf (player frames)
 *          in the last 0.5 s; ups is upc unless UNDS_U2_UPS=frame
 *   shot   w clip (before this round) res cap: the player's weapon fired one round
 *   dmg    ty (damage type) amt hp (x1000 of full health, health before) src(1 with a source) sx sy sz
 *          (source position, mm) px py pz (player position, mm) qx qy qz qw (player rotation
 *          x10000) */
#define _GNU_SOURCE
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#ifndef SIO_UDP_CONNRESET /* mstcpip.h; not every SDK exposes it without that header */
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR,12)
#endif
#endif
#include "runtime.h"
#include "runtime_unds.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#ifndef _WIN32
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/uio.h>
typedef int unds_socket_t;
#define UNDS_INVALID_SOCKET (-1)
#else
typedef SOCKET unds_socket_t;
#define UNDS_INVALID_SOCKET INVALID_SOCKET
#endif

#define UNDS_PORTS 25          /* runtime_audio.c PORTS */
#define AUDIO_WINDOW 512       /* frames per audio event: 10.67 ms at 48 kHz */
#define PAD_MIN_INTERVAL_US 8000
#define STICK_DEADBAND 2
#define TRIGGER_THRESHOLD_DEFAULT 30   /* runtime_pad.c sample_host() before UNDS_TRIGGER */
#define HEARTBEAT_TICK_NS 250000000u
#define SLOW_SEND_NS 1000000u

static _Atomic int active;     /* sending */
static _Atomic int configured; /* UNDS_EVENTS was set (the report prints) */
static _Atomic int rumble_on=1;
static _Atomic int led_on=1;
static _Atomic int trigger_threshold[2]={TRIGGER_THRESHOLD_DEFAULT,TRIGGER_THRESHOLD_DEFAULT};
static _Atomic int bye_done;
static unds_socket_t sock=UNDS_INVALID_SOCKET;
static struct sockaddr_in destination;
static _Atomic uint32_t seq;
static _Atomic uint64_t sent, dropped, errors, slow_sends, vibration_calls;
static _Atomic uint64_t send_max_ns, send_max_ns_total;
static _Atomic int last_error, error_logged;
static uintptr_t image_base;
static uint64_t start_us;
static unsigned long process_id;
static char exe_id[16], exe_file[64];

static pthread_mutex_t state_lock=PTHREAD_MUTEX_INITIALIZER;
typedef struct { uint32_t b; uint8_t a[8]; } PadState; /* lx ly rx ry l2 r2 l2a r2a */
static PadState pad_sent, pad_now;
static int pad_valid, pad_pending;
static uint64_t pad_sent_us;
static int rumble_valid; static uint8_t rumble_l, rumble_s;
static int led_valid; static uint8_t led_rgb[3];

typedef struct {
    int open, type, channels, frames, n, silent;
    uint64_t frame, window_frame, silence_sent_us;
    double sum2, sum_lf2;
    float peak, lp;
} AudioAcc;
static AudioAcc audio_acc[UNDS_PORTS];

static void game_state_refresh(uint64_t now);
static void game_state_counters(char *out, size_t size);

/* Control channel (fx). */
#define FX_POLL_MS 100
#define FX_BYE_FLUSH_NS 30000000u /* SDL writes effects from its rumble thread, 10 ms apart */
static _Atomic uint64_t fx_timeout_us=5000000u; /* a variable for the test */
static _Atomic int fx_on, fx_led, fx_held, fx_port, fx_reject_logged;
static _Atomic uint64_t fx_commands, fx_rejected, fx_failures, fx_reapplied, fx_timeouts;
static pthread_mutex_t fx_lock=PTHREAD_MUTEX_INITIALIZER;
/* under fx_lock: fx_held (atomic for the hello) is only written here */
static int fx_closed;
static uint8_t fx_right[11], fx_left[11], fx_rgb[3];
static uint64_t fx_last_us;
static unsigned fx_generation;
static void fx_shutdown(void);

static uint64_t now_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return (uint64_t)t.tv_sec*1000000000u+(uint64_t)t.tv_nsec; }
static uint64_t now_us(void) { return now_ns()/1000u; }
static void sleep_ns(uint64_t ns) {
#ifdef _WIN32
    compat_sleep_ns(ns);
#else
    struct timespec t={(time_t)(ns/1000000000u),(long)(ns%1000000000u)};
    while (nanosleep(&t,&t)) {}
#endif
}
static int socket_error(void) {
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}
static int would_block(int error) {
#ifdef _WIN32
    return error==WSAEWOULDBLOCK || error==WSAENOBUFS;
#else
    return error==EAGAIN || error==EWOULDBLOCK || error==ENOBUFS;
#endif
}

/* "a.b.c.d:port", "localhost:port" or "port" (127.0.0.1). Numeric only: a name lookup could block. */
static int parse_endpoint(const char *text, uint32_t *ip, uint16_t *port) {
    if (!text || !*text) return -1;
    const char *colon=strrchr(text,':'), *digits=colon ? colon+1 : text;
    uint32_t address=0x7f000001u;
    if (colon) {
        size_t host=(size_t)(colon-text);
        if (host==9 && !strncmp(text,"localhost",9)) address=0x7f000001u;
        else {
            unsigned part[4]; int count=0; const char *s=text;
            while (count<4) {
                if (s>=colon || *s<'0' || *s>'9') return -1;
                unsigned v=0; int len=0;
                while (s<colon && *s>='0' && *s<='9') { v=v*10u+(unsigned)(*s-'0'); ++s; if (++len>3) return -1; }
                if (v>255) return -1;
                part[count++]=v;
                if (count<4) { if (s>=colon || *s!='.') return -1; ++s; }
            }
            if (s!=colon) return -1;
            address=part[0]<<24 | part[1]<<16 | part[2]<<8 | part[3];
        }
    }
    if (!*digits) return -1;
    unsigned long value=0;
    for (const char *s=digits;*s;++s) {
        if (*s<'0' || *s>'9') return -1;
        value=value*10u+(unsigned long)(*s-'0');
        if (value>65535u) return -1;
    }
    if (!value) return -1;
    *ip=address; *port=(uint16_t)value;
    return 0;
}

static void note_send_time(uint64_t ns) {
    uint64_t seen=atomic_load_explicit(&send_max_ns,memory_order_relaxed);
    while (ns>seen && !atomic_compare_exchange_weak_explicit(&send_max_ns,&seen,ns,memory_order_relaxed,memory_order_relaxed)) {}
    seen=atomic_load_explicit(&send_max_ns_total,memory_order_relaxed);
    while (ns>seen && !atomic_compare_exchange_weak_explicit(&send_max_ns_total,&seen,ns,memory_order_relaxed,memory_order_relaxed)) {}
    if (ns>SLOW_SEND_NS) atomic_fetch_add_explicit(&slow_sends,1,memory_order_relaxed);
}
static void send_datagram(const char *data, size_t size) {
    const uint64_t begin=now_ns();
#ifdef _WIN32
    const int result=sendto(sock,data,(int)size,0,(const struct sockaddr *)&destination,(int)sizeof(destination));
    const int failed=result==SOCKET_ERROR;
#else
    const ssize_t result=sendto(sock,data,size,MSG_DONTWAIT,(const struct sockaddr *)&destination,sizeof(destination));
    const int failed=result<0;
#endif
    const int error=failed ? socket_error() : 0;
    note_send_time(now_ns()-begin);
    if (!failed) { atomic_fetch_add_explicit(&sent,1,memory_order_relaxed); return; }
    atomic_store_explicit(&last_error,error,memory_order_relaxed);
    if (would_block(error)) atomic_fetch_add_explicit(&dropped,1,memory_order_relaxed);
    else atomic_fetch_add_explicit(&errors,1,memory_order_relaxed);
    if (!atomic_exchange_explicit(&error_logged,1,memory_order_relaxed))
        printf("Runtime: UNDS: sendto failed (error %d); further failures are only counted\n",error);
}
/* One event: the common header, then `fields` (",\"k\":v..." or empty). */
__attribute__((format(printf,2,3)))
static void emit(const char *event, const char *fields, ...) {
    if (!atomic_load_explicit(&active,memory_order_relaxed)) return;
    if (atomic_load_explicit(&bye_done,memory_order_relaxed) && strcmp(event,"bye")) return;
    char text[1024]; /* hello is the longest: ~450 bytes with the audio ports and the counters */
    const uint32_t n=atomic_fetch_add_explicit(&seq,1,memory_order_relaxed);
    int head=snprintf(text,sizeof(text),"{\"v\":%d,\"seq\":%" PRIu32 ",\"t\":%" PRIu64 ",\"e\":\"%s\"",
                      UNDS_PROTOCOL,n,now_us(),event);
    if (head<0 || (size_t)head>=sizeof(text)) { atomic_fetch_add_explicit(&errors,1,memory_order_relaxed); return; }
    va_list args;
    va_start(args,fields);
    int body=vsnprintf(text+head,sizeof(text)-(size_t)head,fields,args);
    va_end(args);
    if (body<0 || (size_t)(head+body)+2>sizeof(text)) { atomic_fetch_add_explicit(&errors,1,memory_order_relaxed); return; }
    size_t size=(size_t)(head+body);
    text[size++]='}'; text[size]=0;
    send_datagram(text,size);
}

static void ports_text(char *out, size_t size) {
    size_t used=0; out[0]=0;
    for (int i=0;i<UNDS_PORTS;++i) {
        const AudioAcc *a=&audio_acc[i];
        if (!a->open) continue;
        int w=snprintf(out+used,size-used,"%s%d:%d:%d:%d",used ? "," : "",i,a->type,a->channels,a->frames);
        if (w<0 || (size_t)w>=size-used) break;
        used+=(size_t)w;
    }
}
static void send_hello(int boot) {
    char ports[256], state[128];
    pthread_mutex_lock(&state_lock);
    ports_text(ports,sizeof(ports));
    pthread_mutex_unlock(&state_lock);
    game_state_counters(state,sizeof(state));
    const uint64_t smax=atomic_exchange_explicit(&send_max_ns,0,memory_order_relaxed);
    emit("hello",",\"boot\":%d,\"exe\":\"%s\",\"bin\":\"%s\",\"pid\":%lu,\"base\":\"0x%" PRIxPTR "\",\"ver\":\"%s\","
         "\"up\":%" PRIu64 ",\"rumble\":%d,\"led\":%d,\"trig\":\"%d,%d\",\"sent\":%" PRIu64 ",\"drop\":%" PRIu64 ",\"err\":%" PRIu64 ","
         "\"smax\":%" PRIu64 ",\"vib\":%" PRIu64 ",\"ports\":\"%s\","
         "\"fx\":%d,\"fxp\":%d,\"fxc\":%" PRIu64 ",\"fxr\":%" PRIu64 ",\"fxe\":%" PRIu64 ",\"fxo\":%d%s",
         boot,exe_id,exe_file,process_id,image_base,UNDS_VERSION,(now_us()-start_us)/1000u,
         atomic_load_explicit(&rumble_on,memory_order_relaxed),atomic_load_explicit(&led_on,memory_order_relaxed),
         atomic_load_explicit(&trigger_threshold[0],memory_order_relaxed),
         atomic_load_explicit(&trigger_threshold[1],memory_order_relaxed),
         atomic_load_explicit(&sent,memory_order_relaxed),atomic_load_explicit(&dropped,memory_order_relaxed),
         atomic_load_explicit(&errors,memory_order_relaxed),smax/1000u,
         atomic_load_explicit(&vibration_calls,memory_order_relaxed),ports,
         atomic_load_explicit(&fx_on,memory_order_relaxed),atomic_load_explicit(&fx_port,memory_order_relaxed),
         atomic_load_explicit(&fx_commands,memory_order_relaxed),atomic_load_explicit(&fx_rejected,memory_order_relaxed),
         atomic_load_explicit(&fx_failures,memory_order_relaxed),atomic_load_explicit(&fx_held,memory_order_relaxed),state);
}

static void send_pad(const PadState *s) {
    emit("pad",",\"b\":%" PRIu32 ",\"lx\":%u,\"ly\":%u,\"rx\":%u,\"ry\":%u,\"l2\":%u,\"r2\":%u,\"l2a\":%u,\"r2a\":%u",
         s->b,s->a[0],s->a[1],s->a[2],s->a[3],s->a[4],s->a[5],s->a[6],s->a[7]);
}
/* The pending pad state, when the rate limit allows it now (called with state_lock held). */
static int take_pad(uint64_t now, PadState *out) {
    if (!pad_pending || now-pad_sent_us<PAD_MIN_INTERVAL_US) return 0;
    *out=pad_now; pad_sent=pad_now; pad_pending=0; pad_sent_us=now;
    return 1;
}
static int pad_differs(const PadState *a, const PadState *b) {
    if (a->b!=b->b || a->a[4]!=b->a[4] || a->a[5]!=b->a[5]) return 1;
    /* sticks and physical trigger travel: noise below the deadband is not a change */
    for (int i=0;i<8;++i) {
        if (i==4 || i==5) continue;
        int d=(int)a->a[i]-(int)b->a[i];
        if (d>=STICK_DEADBAND || d<=-STICK_DEADBAND) return 1;
    }
    return 0;
}

static void *heartbeat(void *argument) {
    (void)argument;
    uint64_t errors_logged_count=0, errors_logged_us=0;
    for (unsigned tick=1;;++tick) {
        sleep_ns(HEARTBEAT_TICK_NS);
        if (atomic_load_explicit(&bye_done,memory_order_relaxed)) return NULL;
        const uint64_t now=now_us();
        PadState pad; int have_pad;
        int refresh_rumble=0, refresh_led=0; uint8_t l=0, s=0, rgb[3]={0,0,0};
        pthread_mutex_lock(&state_lock);
        have_pad=take_pad(now,&pad);
        if (!(tick%2) && rumble_valid) { refresh_rumble=1; l=rumble_l; s=rumble_s; }
        if (!(tick%4) && led_valid) { refresh_led=1; memcpy(rgb,led_rgb,3); }
        pthread_mutex_unlock(&state_lock);
        if (have_pad) send_pad(&pad);
        if (!(tick%4)) send_hello(0);
        if (!(tick%2)) game_state_refresh(now);
        if (refresh_rumble) emit("rumble",",\"l\":%u,\"s\":%u,\"re\":1",l,s);
        if (refresh_led) emit("led",",\"r\":%u,\"g\":%u,\"b\":%u,\"re\":1",rgb[0],rgb[1],rgb[2]);
        const uint64_t failures=atomic_load_explicit(&errors,memory_order_relaxed)+atomic_load_explicit(&dropped,memory_order_relaxed);
        if (failures!=errors_logged_count && now-errors_logged_us>=10000000u) {
            printf("Runtime: UNDS: %" PRIu64 " failed sends so far (last error %d)\n",failures,
                   atomic_load_explicit(&last_error,memory_order_relaxed));
            errors_logged_count=failures; errors_logged_us=now;
        }
    }
}

static void exe_names(void) {
    const char *value=getenv("BB_EXE");
    const char *name=value && *value ? value : "eboot.bin";
    const char *slash=strrchr(name,'/'), *back=strrchr(name,'\\');
    if (back && (!slash || back>slash)) slash=back;
    if (slash) name=slash+1;
    size_t i=0;
    for (;name[i] && i+1<sizeof(exe_file);++i) {
        char c=name[i];
        exe_file[i]=(c>='a' && c<='z') || (c>='A' && c<='Z') || (c>='0' && c<='9') || c=='.' || c=='-' || c=='_' ? c : '_';
    }
    exe_file[i]=0;
    const char *id=!strcmp(exe_file,"eboot.bin") || !strcmp(exe_file,"u1") ? "u1" :
                   !strncmp(exe_file,"big2",4) || !strcmp(exe_file,"u2") ? "u2" :
                   !strncmp(exe_file,"big3",4) || !strcmp(exe_file,"u3") ? "u3" : "other";
    snprintf(exe_id,sizeof(exe_id),"%s",id);
}

static int open_socket(void) {
#ifdef _WIN32
    WSADATA data;
    int error=WSAStartup(MAKEWORD(2,2),&data);
    if (error) { printf("Runtime: UNDS: WSAStartup failed (%d); events off\n",error); return -1; }
    /* WSA_FLAG_OVERLAPPED (what socket() gives): without it the handle is synchronous and Windows
     * serializes every operation on it, so a sendto from the game's threads waited for the fx
     * receiver's select() to time out, again and again (6 s for three pad events in unds-test). */
#ifdef WSA_FLAG_NO_HANDLE_INHERIT
    sock=WSASocketW(AF_INET,SOCK_DGRAM,IPPROTO_UDP,NULL,0,WSA_FLAG_OVERLAPPED|WSA_FLAG_NO_HANDLE_INHERIT);
#else
    sock=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
#endif
    if (sock==INVALID_SOCKET) { printf("Runtime: UNDS: socket failed (%d); events off\n",WSAGetLastError()); return -1; }
    u_long nonblocking=1;
    if (ioctlsocket(sock,FIONBIO,&nonblocking)) {
        printf("Runtime: UNDS: non-blocking mode failed (%d); events off\n",WSAGetLastError());
        closesocket(sock); sock=INVALID_SOCKET; return -1;
    }
    int buffer=1<<18;
    setsockopt(sock,SOL_SOCKET,SO_SNDBUF,(const char *)&buffer,sizeof(buffer));
#else
    sock=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC|SOCK_NONBLOCK,IPPROTO_UDP);
    if (sock<0) { printf("Runtime: UNDS: socket failed (%d); events off\n",errno); return -1; }
    int buffer=1<<18;
    setsockopt(sock,SOL_SOCKET,SO_SNDBUF,&buffer,sizeof(buffer));
#endif
    return 0;
}

/* "l2,r2", each 1-254. */
static int parse_thresholds(const char *text, int out[2]) {
    for (int i=0;i<2;++i) {
        if (*text<'0' || *text>'9') return -1;
        int v=0, len=0;
        while (*text>='0' && *text<='9') { v=v*10+(*text++-'0'); if (++len>3) return -1; }
        if (v<1 || v>254) return -1;
        out[i]=v;
        if (!i) { if (*text!=',') return -1; ++text; }
    }
    return *text ? -1 : 0;
}

/* ------------------------------------------------------------------------------------------
 * Control channel (fx, see runtime_unds.h). One thread waits on the event socket (select, 100 ms)
 * for the companion's datagrams and hands each one to SDL at once: SDL_SendGamepadEffect only
 * queues the report for SDL's HIDAPI rumble thread, which writes it (and waits 10 ms before the
 * next one; a newer report with the same enable bits replaces a queued one). The game's threads
 * never wait on this: they only read fx_led. Lock order: fx_lock, then state_lock or the pad lock. */
static const uint8_t fx_off[11]={0x05};

/* The controller's trigger modes (Nielk1's TriggerEffectGenerator; dualsense-controller). */
static int fx_mode_ok(uint8_t mode) {
    switch (mode) {
    case 0x01: case 0x02: case 0x05: case 0x06: case 0x11: case 0x12:
    case 0x21: case 0x22: case 0x23: case 0x25: case 0x26: case 0x27: return 1;
    default: return 0;
    }
}
/* One report with the parts in `flags` (fx_lock held). Only the trigger and light bar enable bits
 * are set: rumble emulation (bit 0x01 / 0x02 of the first byte) would turn the audio haptics off. */
static int fx_write(int flags, const uint8_t *right, const uint8_t *left, const uint8_t *rgb) {
    uint8_t e[UNDS_DS5_EFFECTS_SIZE];
    memset(e,0,sizeof(e));
    if (flags&UNDS_FX_RIGHT) { e[UNDS_DS5_ENABLE1]|=0x04; memcpy(e+UNDS_DS5_RIGHT,right,11); }
    if (flags&UNDS_FX_LEFT) { e[UNDS_DS5_ENABLE1]|=0x08; memcpy(e+UNDS_DS5_LEFT,left,11); }
    if (flags&UNDS_FX_LED) { e[UNDS_DS5_ENABLE2]|=0x04; memcpy(e+UNDS_DS5_RGB,rgb,3); }
    if (!e[UNDS_DS5_ENABLE1] && !e[UNDS_DS5_ENABLE2]) return 1;
    const int result=runtime_pad_unds_effect(e,(int)sizeof(e));
    if (result<0) atomic_fetch_add_explicit(&fx_failures,1,memory_order_relaxed);
    return result;
}
/* Both triggers off; the light bar back to the game's colour if the companion held it and SDL
 * may show the game's light bar (UNDS_LED is not 0). fx_lock held. */
static int fx_release_locked(void) {
    uint8_t rgb[3]={0,0,0};
    int give_back=0;
    if ((atomic_load_explicit(&fx_held,memory_order_relaxed)&UNDS_FX_LED) && atomic_load_explicit(&led_on,memory_order_relaxed)) {
        pthread_mutex_lock(&state_lock);
        if (led_valid) { memcpy(rgb,led_rgb,3); give_back=1; }
        pthread_mutex_unlock(&state_lock);
    }
    atomic_store_explicit(&fx_held,0,memory_order_relaxed);
    atomic_store_explicit(&fx_led,0,memory_order_relaxed);
    const int result=fx_write(UNDS_FX_RIGHT|UNDS_FX_LEFT|(give_back ? UNDS_FX_LED : 0),fx_off,fx_off,rgb);
    /* and through SDL's own call, so its cached colour is the game's again */
    if (give_back) runtime_pad_unds_game_led(rgb[0],rgb[1],rgb[2]);
    return result;
}
static void fx_event(uint32_t command, int flags, int result, uint64_t since_ns, const char *why) {
    emit("fx",",\"c\":%" PRIu32 ",\"f\":%d,\"ok\":%d,\"q\":%" PRIu64 ",\"why\":\"%s\"",command,flags,result,
         (now_ns()-since_ns)/1000u,why);
}
static void fx_reject(const char *why, const struct sockaddr_in *from) {
    atomic_fetch_add_explicit(&fx_rejected,1,memory_order_relaxed);
    if (atomic_exchange_explicit(&fx_reject_logged,1,memory_order_relaxed)) return;
    const uint32_t ip=from ? ntohl(from->sin_addr.s_addr) : 0;
    printf("Runtime: UNDS: fx: datagram from %u.%u.%u.%u:%u rejected (%s); further ones are only counted\n",
           ip>>24,(ip>>16)&255u,(ip>>8)&255u,ip&255u,from ? (unsigned)ntohs(from->sin_port) : 0u,why);
}
/* A valid datagram (size, magic, version and source already checked). */
static void fx_command(const uint8_t *d, uint64_t received_ns, const struct sockaddr_in *from) {
    const int flags=d[5];
    const uint32_t command=(uint32_t)d[8] | (uint32_t)d[9]<<8 | (uint32_t)d[10]<<16 | (uint32_t)d[11]<<24;
    const uint8_t *right=d+12, *left=d+23, *rgb=d+34;
    if (!(flags&UNDS_FX_RELEASE) && (((flags&UNDS_FX_RIGHT) && !fx_mode_ok(right[0])) ||
                                     ((flags&UNDS_FX_LEFT) && !fx_mode_ok(left[0])))) {
        fx_reject("unknown trigger mode",from);
        return;
    }
    atomic_fetch_add_explicit(&fx_commands,1,memory_order_relaxed);
    int result;
    const char *why="cmd";
    pthread_mutex_lock(&fx_lock);
    if (fx_closed) { pthread_mutex_unlock(&fx_lock); return; }
    fx_last_us=now_us();
    if (flags&UNDS_FX_RELEASE) {
        result=fx_release_locked();
        why="release";
    } else {
        const int parts=flags&(UNDS_FX_RIGHT|UNDS_FX_LEFT|UNDS_FX_LED);
        if (parts&UNDS_FX_RIGHT) memcpy(fx_right,right,11);
        if (parts&UNDS_FX_LEFT) memcpy(fx_left,left,11);
        if (parts&UNDS_FX_LED) { memcpy(fx_rgb,rgb,3); atomic_store_explicit(&fx_led,1,memory_order_relaxed); }
        atomic_fetch_or_explicit(&fx_held,parts,memory_order_relaxed);
        result=fx_write(parts,right,left,rgb); /* flags 0: a keep-alive, nothing to write */
    }
    pthread_mutex_unlock(&fx_lock);
    fx_event(command,flags,result,received_ns,why);
}
/* Every datagram waiting on the socket. */
static void fx_receive(void) {
    for (int i=0;i<64;++i) {
        uint8_t d[64];
        struct sockaddr_in from;
        socklen_t length=sizeof(from);
        memset(&from,0,sizeof(from));
        const int n=(int)recvfrom(sock,(char *)d,(int)sizeof(d),0,(struct sockaddr *)&from,&length);
        const uint64_t received=now_ns();
        if (n<0) {
            const int error=socket_error();
            if (would_block(error)) return;
#ifdef _WIN32
            if (error==WSAECONNRESET) continue;           /* ICMP of an earlier send (SIO_UDP_CONNRESET is off anyway) */
            if (error==WSAEMSGSIZE) { fx_reject("size",&from); continue; }
#else
            if (error==ECONNREFUSED) continue;
#endif
            atomic_fetch_add_explicit(&fx_rejected,1,memory_order_relaxed);
            return;
        }
        if (from.sin_family!=AF_INET || from.sin_addr.s_addr!=destination.sin_addr.s_addr) { fx_reject("not the UNDS_EVENTS host",&from); continue; }
        if (n!=UNDS_FX_SIZE || memcmp(d,"UNDC",4) || d[4]!=1) { fx_reject("not an UNDS fx v1 datagram",&from); continue; }
        fx_command(d,received,&from);
    }
}
/* 100 ms tick: the watchdog, and the held state again after the controller reconnects. */
static void fx_tick(void) {
    const unsigned generation=runtime_pad_unds_generation();
    const uint64_t begin=now_ns(), now=begin/1000u;
    const char *why=NULL; int result=0, flags=0;
    pthread_mutex_lock(&fx_lock);
    const int held=atomic_load_explicit(&fx_held,memory_order_relaxed);
    if (!fx_closed && held) {
        if (now-fx_last_us>fx_timeout_us) {
            result=fx_release_locked(); why="timeout"; flags=UNDS_FX_RELEASE;
            atomic_fetch_add_explicit(&fx_timeouts,1,memory_order_relaxed);
        } else if (generation!=fx_generation) {
            result=fx_write(held,fx_right,fx_left,fx_rgb); why="reconnect"; flags=held;
            atomic_fetch_add_explicit(&fx_reapplied,1,memory_order_relaxed);
        }
    }
    fx_generation=generation;
    pthread_mutex_unlock(&fx_lock);
    if (!why) return;
    if (!strcmp(why,"timeout"))
        printf("Runtime: UNDS: fx: no command for %" PRIu64 " ms (companion gone?): triggers off, light bar back to the game\n",fx_timeout_us/1000u);
    fx_event(0,flags,result,begin,why);
}
static void *fx_thread(void *argument) {
    (void)argument;
    while (!atomic_load_explicit(&bye_done,memory_order_relaxed)) {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(sock,&set);
        struct timeval timeout={0,FX_POLL_MS*1000};
        const int ready=select((int)sock+1,&set,NULL,NULL,&timeout); /* the first argument is ignored on Windows */
        if (atomic_load_explicit(&bye_done,memory_order_relaxed)) break;
        if (ready>0) fx_receive();
        else if (ready<0) sleep_ns((uint64_t)FX_POLL_MS*1000000u);
        fx_tick();
    }
    return NULL;
}
/* Binds the event socket (to 127.0.0.1 for a loopback companion: nothing else on the network can
 * reach the channel) before its first send, so the companion can answer to it. */
static int fx_open(uint32_t ip) {
    struct sockaddr_in local;
    memset(&local,0,sizeof(local));
    local.sin_family=AF_INET;
    local.sin_addr.s_addr=htonl((ip>>24)==127u ? 0x7f000001u : INADDR_ANY);
    local.sin_port=0;
    if (bind(sock,(const struct sockaddr *)&local,(int)sizeof(local))) {
        printf("Runtime: UNDS: fx: bind failed (%d); control channel off\n",socket_error());
        return -1;
    }
    socklen_t length=sizeof(local);
    if (getsockname(sock,(struct sockaddr *)&local,&length)) {
        printf("Runtime: UNDS: fx: getsockname failed (%d); control channel off\n",socket_error());
        return -1;
    }
#ifdef _WIN32
    /* Without this, every ICMP "port unreachable" (a hello sent while no companion listens) would
     * come back as WSAECONNRESET on the next recvfrom. */
    BOOL report=FALSE; DWORD bytes=0;
    WSAIoctl(sock,SIO_UDP_CONNRESET,&report,sizeof(report),NULL,0,&bytes,NULL,NULL);
#endif
    atomic_store(&fx_port,(int)ntohs(local.sin_port));
    return 0;
}
/* At bye: the triggers off before the process goes (SDL's rumble thread gets 30 ms to write
 * it), and no command is taken after that. */
static void fx_shutdown(void) {
    if (!atomic_load(&fx_on)) return;
    pthread_mutex_lock(&fx_lock);
    fx_closed=1;
    const int held=atomic_load_explicit(&fx_held,memory_order_relaxed);
    if (held) fx_release_locked();
    pthread_mutex_unlock(&fx_lock);
    if (held) sleep_ns(FX_BYE_FLUSH_NS);
}
int runtime_unds_fx_led_owned(void) { return atomic_load_explicit(&fx_led,memory_order_relaxed); }

void runtime_unds_start(uintptr_t image) {
    const char *rumble=getenv("UNDS_RUMBLE");
    if (rumble && !strcmp(rumble,"0")) {
        atomic_store(&rumble_on,0);
        puts("Runtime: UNDS: SDL rumble off (UNDS_RUMBLE=0); scePadSetVibration is still published");
    }
    const char *led=getenv("UNDS_LED");
    if (led && !strcmp(led,"0")) {
        atomic_store(&led_on,0);
        puts("Runtime: UNDS: SDL light bar off (UNDS_LED=0); the light bar is still published");
    }
    const char *thresholds=getenv("UNDS_TRIGGER");
    if (thresholds && *thresholds) {
        int t[2];
        if (parse_thresholds(thresholds,t))
            printf("Runtime: UNDS: UNDS_TRIGGER=\"%s\" is not <l2>,<r2> (1-254 each); thresholds stay %d,%d\n",thresholds,
                   atomic_load(&trigger_threshold[0]),atomic_load(&trigger_threshold[1]));
        else {
            atomic_store(&trigger_threshold[0],t[0]); atomic_store(&trigger_threshold[1],t[1]);
            printf("Runtime: UNDS: trigger press thresholds L2>%d R2>%d of 255 (UNDS_TRIGGER)\n",t[0],t[1]);
        }
    }
    const char *target=getenv("UNDS_EVENTS");
    if (!target || !*target || atomic_load(&active)) return;
    atomic_store(&configured,1);
    uint32_t ip; uint16_t port;
    if (parse_endpoint(target,&ip,&port)) {
        printf("Runtime: UNDS: UNDS_EVENTS=\"%s\" is not IPv4:port, localhost:port or a port; events off\n",target);
        return;
    }
    if (open_socket()) return;
    memset(&destination,0,sizeof(destination));
    destination.sin_family=AF_INET;
    destination.sin_port=htons(port);
    destination.sin_addr.s_addr=htonl(ip);
    image_base=image;
    start_us=now_us();
#ifdef _WIN32
    process_id=(unsigned long)GetCurrentProcessId();
#else
    process_id=(unsigned long)getpid();
#endif
    exe_names();
    const char *fx=getenv("UNDS_FX");
    const int want_fx=!(fx && !strcmp(fx,"0"));
    if (want_fx && !fx_open(ip)) atomic_store(&fx_on,1);
    atomic_store(&active,1);
    printf("Runtime: UNDS: events to %u.%u.%u.%u:%u (protocol %d, %s, %s)\n",ip>>24,(ip>>16)&255u,(ip>>8)&255u,ip&255u,
           port,UNDS_PROTOCOL,UNDS_VERSION,exe_id);
    pthread_t thread;
    if (atomic_load(&fx_on)) {
        if (pthread_create(&thread,NULL,fx_thread,NULL)) {
            atomic_store(&fx_on,0);
            puts("Runtime: UNDS: fx: no receiver thread (pthread_create failed); control channel off");
        } else {
            pthread_detach(thread);
            printf("Runtime: UNDS: fx: control channel on UDP port %d (adaptive triggers and light bar through SDL; "
                   "UNDS_FX=0 turns it off)\n",atomic_load(&fx_port));
        }
    } else if (!want_fx) puts("Runtime: UNDS: fx: control channel off (UNDS_FX=0)");
    send_hello(1);
    if (pthread_create(&thread,NULL,heartbeat,NULL)) puts("Runtime: UNDS: no heartbeat thread (pthread_create failed)");
    else pthread_detach(thread);
}

int runtime_unds_rumble_enabled(void) { return atomic_load_explicit(&rumble_on,memory_order_relaxed); }
int runtime_unds_led_enabled(void) { return atomic_load_explicit(&led_on,memory_order_relaxed); }
uint8_t runtime_unds_trigger_threshold(int right) {
    return (uint8_t)atomic_load_explicit(&trigger_threshold[right ? 1 : 0],memory_order_relaxed);
}

void runtime_unds_bye(const char *reason) {
    if (!atomic_load_explicit(&active,memory_order_relaxed)) return;
    if (atomic_exchange(&bye_done,1)) return;
    fx_shutdown(); /* triggers off while SDL is still there */
    char clean[32]; size_t i=0;
    for (;reason && reason[i] && i+1<sizeof(clean);++i)
        clean[i]=(reason[i]>='a' && reason[i]<='z') || (reason[i]>='0' && reason[i]<='9') || reason[i]=='_' ? reason[i] : '_';
    clean[i]=0;
    /* "sent" counts this datagram too: the receiver compares it with what it got. */
    emit("bye",",\"reason\":\"%s\",\"up\":%" PRIu64 ",\"sent\":%" PRIu64,clean,(now_us()-start_us)/1000u,
         atomic_load_explicit(&sent,memory_order_relaxed)+1u);
}

void runtime_unds_rumble(uint8_t large, uint8_t small) {
    if (!atomic_load_explicit(&active,memory_order_relaxed)) return;
    atomic_fetch_add_explicit(&vibration_calls,1,memory_order_relaxed);
    pthread_mutex_lock(&state_lock);
    const int changed=!rumble_valid || rumble_l!=large || rumble_s!=small;
    rumble_valid=1; rumble_l=large; rumble_s=small;
    pthread_mutex_unlock(&state_lock);
    if (changed) emit("rumble",",\"l\":%u,\"s\":%u",large,small);
}

void runtime_unds_led(uint8_t r, uint8_t g, uint8_t b) {
    if (!atomic_load_explicit(&active,memory_order_relaxed)) return;
    pthread_mutex_lock(&state_lock);
    const int changed=!led_valid || led_rgb[0]!=r || led_rgb[1]!=g || led_rgb[2]!=b;
    led_valid=1; led_rgb[0]=r; led_rgb[1]=g; led_rgb[2]=b;
    pthread_mutex_unlock(&state_lock);
    if (changed) emit("led",",\"r\":%u,\"g\":%u,\"b\":%u",r,g,b);
}

void runtime_unds_pad(uint32_t buttons, uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry, uint8_t l2, uint8_t r2,
                      uint8_t l2a, uint8_t r2a) {
    if (!atomic_load_explicit(&active,memory_order_relaxed)) return;
    const uint64_t now=now_us();
    const PadState state={buttons,{lx,ly,rx,ry,l2,r2,l2a,r2a}};
    PadState out; int send;
    pthread_mutex_lock(&state_lock);
    pad_now=state;
    pad_pending=!pad_valid || pad_differs(&pad_now,&pad_sent);
    /* Buttons are never held back: a press and its release must both reach the companion. */
    if (pad_pending && (!pad_valid || pad_now.b!=pad_sent.b)) pad_sent_us=0;
    pad_valid=1;
    send=take_pad(now,&out);
    pthread_mutex_unlock(&state_lock);
    if (send) send_pad(&out);
}

void runtime_unds_audio_open(int port, int type, int channels, int frames) {
    if (!atomic_load_explicit(&active,memory_order_relaxed) || port<0 || port>=UNDS_PORTS) return;
    pthread_mutex_lock(&state_lock);
    memset(&audio_acc[port],0,sizeof(audio_acc[port]));
    audio_acc[port].open=1; audio_acc[port].type=type; audio_acc[port].channels=channels; audio_acc[port].frames=frames;
    pthread_mutex_unlock(&state_lock);
    emit("aopen",",\"p\":%d,\"ty\":%d,\"ch\":%d,\"fr\":%d",port,type,channels,frames);
}

static unsigned level(double value) {
    const double scaled=value*10000.0+0.5;
    return scaled>=4.0e9 ? 4000000000u : (unsigned)scaled;
}
static void audio_window(int port, int type, AudioAcc *a) {
    const unsigned rms=level(sqrt(a->sum2/(double)a->n)), peak=level(a->peak), low=level(sqrt(a->sum_lf2/(double)a->n));
    const uint64_t first=a->window_frame;
    const int frames=a->n;
    a->window_frame=a->frame; a->n=0; a->sum2=a->sum_lf2=0.0; a->peak=0.0f;
    if (!peak) {
        const uint64_t now=now_us();
        if (a->silent && now-a->silence_sent_us<1000000u) return;
        a->silent=1; a->silence_sent_us=now;
    } else a->silent=0;
    emit("audio",",\"p\":%d,\"ty\":%d,\"f\":%" PRIu64 ",\"n\":%d,\"rms\":%u,\"pk\":%u,\"lf\":%u",
         port,type,first,frames,rms,peak,low);
}
void runtime_unds_audio(int port, int type, const void *pcm, int frames, int channels, int is_float) {
    if (!atomic_load_explicit(&active,memory_order_relaxed) || port<0 || port>=UNDS_PORTS || !pcm ||
        frames<=0 || channels<=0 || channels>8) return;
    AudioAcc *a=&audio_acc[port];
    /* One-pole low-pass at 150 Hz: 1 - exp(-2*pi*150/48000). */
    const float coefficient=0.019443f;
    for (int f=0;f<frames;++f) {
        float s[8]={0,0,0,0,0,0,0,0};
        for (int c=0;c<channels;++c) {
            const size_t index=(size_t)f*(size_t)channels+(size_t)c;
            if (is_float) memcpy(&s[c],(const char *)pcm+index*4,4);
            else { int16_t v; memcpy(&v,(const char *)pcm+index*2,2); s[c]=(float)v/32768.0f; }
        }
        float l, r;
        if (channels==8) { l=s[0]+0.7071f*(s[2]+s[4]+s[6]); r=s[1]+0.7071f*(s[2]+s[5]+s[7]); }
        else if (channels==1) l=r=s[0];
        else { l=s[0]; r=s[1]; }
        if (!(l==l)) l=0.0f; /* NaN */
        if (!(r==r)) r=0.0f;
        const float al=fabsf(l), ar=fabsf(r);
        if (al>a->peak) a->peak=al;
        if (ar>a->peak) a->peak=ar;
        a->sum2+=0.5*((double)l*l+(double)r*r);
        a->lp+=coefficient*(0.5f*(l+r)-a->lp);
        if (fabsf(a->lp)<1e-12f) a->lp=0.0f; /* no denormals */
        a->sum_lf2+=(double)a->lp*a->lp;
        ++a->frame;
        if (++a->n==AUDIO_WINDOW) audio_window(port,type,a);
    }
}

/* ------------------------------------------------------------------------------------------
 * Game state (UNDS_STATE). Callbacks in the game's own code (probe.c install_game_callbacks,
 * through runtime_guest_callbacks) read the player's health, weapon and ammunition where the game
 * itself uses them. They are handed to the loader for U1 and U2 (BB_EXE), with UNDS_EVENTS set and
 * UNDS_STATE not "0". The loader finds each site by its instruction pattern (RIP-relative
 * displacements and call targets are wildcards), which must match exactly once; a build without
 * that code gets 'Callbacks: UNDS U1 ... not found' and nothing is installed.
 *
 * U1 1.00 (CUSA02320 eboot.bin, sha256 74cf17ba...a5c424; image offsets) [VERIFICADO by
 * disassembly of that executable; each pattern below matches once in its executable segment
 * and in the linked image, and none in U2 1.00]. U1 1.02 is the same code with other
 * displacements (port 0.3.7+, scripts/cheats.py: the GoldHEN health jump and the ConsumeRound
 * call at 0x26244b / 0x730f73 in 1.02, 0x261edb / 0x71b003 here) [HIPÓTESIS: the same
 * patterns match 1.02 once; check 'Callbacks: UNDS U1 ...' in the log of a 1.02 run]:
 *   0x2645f0  player update (oldplayer.cpp; rdi = the player; its callers include the death
 *             controller). Health: float at +0x1428, 0..1. The debug option "Enable health
 *             effect test (overrides player health)" writes 0.505 + 0.495 sin(t) there,
 *             regeneration adds to it and clamps it at 1.0, the death path writes 0 and the
 *             damage branch subtracts the damage. +0x1430: byte, health below the "injured"
 *             threshold. +0xf0: the locator (+0x10 position, +0x20 rotation quaternion).
 *   0x71e920  WeaponController::PostJointUpdate (weaponcontroller.cpp; rdi = the controller).
 *             m_currentWeapon: u32 at +0x198. m_weaponHandles[6] at +0x130, 16 bytes each:
 *             slot pointer, u32 id at +8; valid when (*slot)+0x64 == id (WeaponController::Fire
 *             checks exactly this). kGrenade = 4 (PostJointUpdate's assertion). +0x128:
 *             m_pGameObject (Fire: ->m_pNpc->GetWeaponInventory()).
 *   weapon    +0x2e4: u32 weapon type string id (GetWeaponSettings looks it up). +0x440:
 *             rounds in the clip (ConsumeRound: dec dword [w+0x440] for fire mode 0), +0x444:
 *             reserve, +0x448: clip size (Reload moves min(size - clip, reserve)). GoldHEN's
 *             1.02 "No Reload" / "Inf Ammo" cheats patch the same two instructions (0x346469 /
 *             0x345e98 in 1.02; 0x345cf9 / 0x345728 here).
 *   0x71affe  WeaponController::Fire, right before ConsumeRound(weapon, 0): r14 = the weapon.
 *   0x261eea  Player::EventHandler (oldplayer.cpp), damage branch: health -= [r15+0x14];
 *             r14 = player, r15 = the damage (+0 u32 type, +8 source object or 0, +0x14
 *             amount). For its hit direction the handler takes the player's position minus
 *             the source's +0x10 vec4 [HIPÓTESIS: that vec4 is the source's position].
 * The callbacks run on the game's threads: they read a few fields, compare and send, without
 * any lock of the game. Pointers kept between calls (the controller's weapon handles) are read
 * through peek(), which fails instead of faulting on memory that is gone. */
#define GS_HP_STEP 5                 /* x1000: smaller health changes wait for GS_SLOW_US */
#define GS_SLOW_US 100000u
#define GS_WEAPON_STALE_US 500000u   /* no controller update for this long: no weapon */
#define GS_DEBUG_LINES 24

typedef struct {
    uint32_t health, injured, locator, controller_current, controller_handles, controller_owner,
             handle_id, weapon_type, weapon_ammo, damage_type, damage_source, damage_amount, source_position;
    int handles, grenade_slot;
} GameOffsets;
static const GameOffsets u1_100={
    .health=0x1428, .injured=0x1430, .locator=0xf0,
    .controller_current=0x198, .controller_handles=0x130, .controller_owner=0x128,
    .handle_id=0x64, .weapon_type=0x2e4, .weapon_ammo=0x440,
    .damage_type=0x0, .damage_source=0x8, .damage_amount=0x14, .source_position=0x10,
    .handles=6, .grenade_slot=4,
};
static const GameOffsets *offsets=&u1_100;

typedef struct { int32_t hp, inj, ws, clip, res, cap, gr, grr; uint32_t w; } GameState;
typedef struct { int32_t ws, clip, res, cap, gr, grr; uint32_t w; uint64_t us; } WeaponState;
static _Atomic int state_enabled, state_debug, state_debug_lines, owner_confirmed;
static _Atomic uintptr_t gs_player, gs_controller;
static _Atomic uint64_t gs_player_calls, gs_controller_calls, gs_controller_foreign, gs_shots, gs_damages, gs_peek_failures;
static _Atomic uint64_t gs_control_calls, gs_frame_calls; /* U2: both raw counts (upc, upf) */
static _Atomic int ups_from_control;                      /* U2: which of them is ups */
static uint64_t gs_player_refresh, gs_controller_refresh, gs_control_refresh, gs_frame_refresh; /* heartbeat thread only */
/* under state_lock */
static WeaponState gs_weapon;
static GameState gs_sent;
static int gs_valid, gs_have_player;
static int32_t gs_hp=-1, gs_inj;
static uint64_t gs_sent_us;

/* Reads `size` bytes at `address` of this process; 0 (counted) where nothing is mapped. */
static int peek(uintptr_t address, void *out, size_t size) {
    if (address<0x10000 || address+size<address) return 0;
#ifdef _WIN32
    SIZE_T got=0;
    if (ReadProcessMemory(GetCurrentProcess(),(LPCVOID)address,out,size,&got) && got==size) return 1;
#else
    struct iovec local={out,size}, remote={(void *)address,size};
    if (process_vm_readv(getpid(),&local,1,&remote,1,0)==(ssize_t)size) return 1;
#endif
    atomic_fetch_add_explicit(&gs_peek_failures,1,memory_order_relaxed);
    return 0;
}
static float load_float(uintptr_t address) { float v; memcpy(&v,(const void *)address,4); return v; }
static uint32_t load_u32(uintptr_t address) { uint32_t v; memcpy(&v,(const void *)address,4); return v; }
static uint64_t load_u64(uintptr_t address) { uint64_t v; memcpy(&v,(const void *)address,8); return v; }
/* Health 0..1 as 0-1000; -1 when the value cannot be a health (a wrong object or version). */
static int32_t health_milli(float h) {
    if (!(h==h) || h<-0.01f || h>1.5f) return -1;
    if (h<=0.0f) return 0;
    const int32_t v=(int32_t)(h*1000.0f+0.5f);
    return v>1000 ? 1000 : v;
}
/* float * scale as an integer for the JSON (no locale), 0 for NaN, clamped. */
static long scaled(float v, float scale) {
    if (!(v==v)) return 0;
    const double x=(double)v*(double)scale;
    if (x>2.0e9) return 2000000000L;
    if (x<-2.0e9) return -2000000000L;
    return lround(x);
}
__attribute__((format(printf,1,2)))
static void state_log(const char *format, ...) {
    if (!atomic_load_explicit(&state_debug,memory_order_relaxed)) return;
    if (atomic_fetch_add_explicit(&state_debug_lines,1,memory_order_relaxed)>=GS_DEBUG_LINES) return;
    va_list args;
    va_start(args,format);
    fputs("Runtime: UNDS: state: ",stdout);
    vprintf(format,args);
    va_end(args);
    fflush(stdout);
}

/* The process a weapon controller handle points to, as WeaponController::Fire checks it; 0 if
 * the handle is empty or stale. `controller` is valid (the game is updating it). */
static uintptr_t handle_target(uintptr_t controller, int slot) {
    const GameOffsets *o=offsets;
    if (slot<0 || slot>=o->handles) return 0;
    const uintptr_t handle=controller+o->controller_handles+16u*(unsigned)slot;
    const uint64_t pointer=load_u64(handle);
    const uint32_t id=load_u32(handle+8);
    uint64_t process=0; uint32_t check=0;
    if (!pointer || !id || !peek((uintptr_t)pointer,&process,8) || !process ||
        !peek((uintptr_t)process+o->handle_id,&check,4) || check!=id) return 0;
    return (uintptr_t)process;
}
static void read_weapon(uintptr_t controller, WeaponState *w) {
    const GameOffsets *o=offsets;
    w->ws=-1; w->w=0; w->clip=w->res=w->cap=-1; w->gr=w->grr=-1;
    const int32_t current=(int32_t)load_u32(controller+o->controller_current);
    const uintptr_t weapon=handle_target(controller,current);
    int32_t ammo[3]; uint32_t type;
    if (weapon && peek(weapon+o->weapon_type,&type,4) && peek(weapon+o->weapon_ammo,ammo,sizeof(ammo))) {
        w->ws=current; w->w=type; w->clip=ammo[0]; w->res=ammo[1]; w->cap=ammo[2];
    }
    const uintptr_t grenade=handle_target(controller,o->grenade_slot);
    if (grenade && peek(grenade+o->weapon_ammo,ammo,8)) { w->gr=ammo[0]; w->grr=ammo[1]; }
}
static void compose(GameState *s, uint64_t now) { /* state_lock held */
    s->hp=gs_hp; s->inj=gs_inj;
    if (gs_weapon.us && now-gs_weapon.us<GS_WEAPON_STALE_US) {
        s->ws=gs_weapon.ws; s->w=gs_weapon.w; s->clip=gs_weapon.clip; s->res=gs_weapon.res; s->cap=gs_weapon.cap;
        s->gr=gs_weapon.gr; s->grr=gs_weapon.grr;
    } else { s->ws=-1; s->w=0; s->clip=s->res=s->cap=-1; s->gr=s->grr=-1; }
}
static int state_differs(const GameState *a, const GameState *b) {
    return a->inj!=b->inj || a->ws!=b->ws || a->w!=b->w || a->clip!=b->clip || a->res!=b->res || a->cap!=b->cap ||
           a->gr!=b->gr || a->grr!=b->grr;
}
/* Whether `now_state` is worth a datagram after `sent` (state_lock held). Health regenerates a
 * little every frame: steps under GS_HP_STEP wait GS_SLOW_US, reaching 0 or full never waits. */
static int state_due(const GameState *now_state, uint64_t now) {
    if (!gs_valid || state_differs(now_state,&gs_sent)) return 1;
    if (now_state->hp==gs_sent.hp) return 0;
    const int32_t d=now_state->hp>gs_sent.hp ? now_state->hp-gs_sent.hp : gs_sent.hp-now_state->hp;
    if (d>=GS_HP_STEP || now_state->hp<=0 || now_state->hp==1000) return 1;
    return now-gs_sent_us>=GS_SLOW_US;
}
static _Atomic int control_enabled;
static void send_state(const GameState *s, int refresh, uint64_t ups, uint64_t wps, uint64_t upc, uint64_t upf) {
    char extra[128]="";
    if (refresh && atomic_load_explicit(&control_enabled,memory_order_relaxed))
        snprintf(extra,sizeof(extra),",\"re\":1,\"ups\":%" PRIu64 ",\"wps\":%" PRIu64 ",\"upc\":%" PRIu64 ",\"upf\":%" PRIu64,
                 ups,wps,upc,upf);
    else if (refresh) snprintf(extra,sizeof(extra),",\"re\":1,\"ups\":%" PRIu64 ",\"wps\":%" PRIu64,ups,wps);
    emit("gs",",\"hp\":%" PRId32 ",\"inj\":%" PRId32 ",\"ws\":%" PRId32 ",\"w\":\"0x%" PRIx32 "\",\"clip\":%" PRId32
         ",\"res\":%" PRId32 ",\"cap\":%" PRId32 ",\"gr\":%" PRId32 ",\"grr\":%" PRId32 "%s",
         s->hp,s->inj,s->ws,s->w,s->clip,s->res,s->cap,s->gr,s->grr,extra);
}
static void publish_state(uint64_t now) {
    GameState s; int send;
    pthread_mutex_lock(&state_lock);
    compose(&s,now);
    send=gs_have_player && state_due(&s,now);
    if (send) { gs_sent=s; gs_valid=1; gs_sent_us=now; }
    pthread_mutex_unlock(&state_lock);
    if (send) send_state(&s,0,0,0,0,0);
}
static void game_state_refresh(uint64_t now) {
    if (!atomic_load_explicit(&state_enabled,memory_order_relaxed)) return;
    const uint64_t players=atomic_load_explicit(&gs_player_calls,memory_order_relaxed);
    const uint64_t controllers=atomic_load_explicit(&gs_controller_calls,memory_order_relaxed);
    const uint64_t controls=atomic_load_explicit(&gs_control_calls,memory_order_relaxed);
    const uint64_t frames=atomic_load_explicit(&gs_frame_calls,memory_order_relaxed);
    const uint64_t ups=players-gs_player_refresh, wps=controllers-gs_controller_refresh, upc=controls-gs_control_refresh,
                   upf=frames-gs_frame_refresh;
    gs_player_refresh=players; gs_controller_refresh=controllers; gs_control_refresh=controls; gs_frame_refresh=frames;
    if (!players && !controllers && !frames) return; /* nothing seen yet (menus, or another version) */
    GameState s;
    pthread_mutex_lock(&state_lock);
    compose(&s,now);
    gs_sent=s; gs_valid=1; gs_sent_us=now;
    pthread_mutex_unlock(&state_lock);
    send_state(&s,1,ups,wps,upc,upf);
}
static void game_state_counters(char *out, size_t size) {
    out[0]=0;
    if (!atomic_load_explicit(&state_enabled,memory_order_relaxed)) return;
    snprintf(out,size,",\"gsp\":%" PRIu64 ",\"gsw\":%" PRIu64 ",\"gss\":%" PRIu64 ",\"gsd\":%" PRIu64 ",\"gsf\":%" PRIu64,
             atomic_load_explicit(&gs_player_calls,memory_order_relaxed),
             atomic_load_explicit(&gs_controller_calls,memory_order_relaxed),
             atomic_load_explicit(&gs_shots,memory_order_relaxed),
             atomic_load_explicit(&gs_damages,memory_order_relaxed),
             atomic_load_explicit(&gs_peek_failures,memory_order_relaxed));
}

/* rdi = the player (oldplayer.cpp update). */
static void __attribute__((ms_abi)) state_player(uint64_t player, uint64_t rsi, uint64_t image, uint64_t r14,
                                                  const RuntimeGuestRegs *regs) {
    (void)rsi; (void)r14; (void)regs;
    if (!atomic_load_explicit(&active,memory_order_relaxed) || !player) return;
    atomic_fetch_add_explicit(&gs_player_calls,1,memory_order_relaxed);
    const GameOffsets *o=offsets;
    if (atomic_exchange_explicit(&gs_player,(uintptr_t)player,memory_order_relaxed)!=(uintptr_t)player) {
        state_log("player object %#" PRIx64 " (image %#" PRIx64 "), health %.3f\n",player,image,
                  (double)load_float((uintptr_t)player+o->health));
        pthread_mutex_lock(&state_lock);
        gs_valid=0; /* a new player (level load): the next state goes out at once */
        pthread_mutex_unlock(&state_lock);
    }
    const int32_t hp=health_milli(load_float((uintptr_t)player+o->health));
    const int32_t inj=*(const volatile uint8_t *)(uintptr_t)(player+o->injured) ? 1 : 0;
    const uint64_t now=now_us();
    pthread_mutex_lock(&state_lock);
    gs_hp=hp; gs_inj=inj; gs_have_player=1;
    pthread_mutex_unlock(&state_lock);
    publish_state(now);
}
/* rdi = a WeaponController (PostJointUpdate). Once one controller's m_pGameObject has been the
 * player, controllers of other objects are ignored. */
static void __attribute__((ms_abi)) state_controller(uint64_t controller, uint64_t rsi, uint64_t image, uint64_t r14,
                                                      const RuntimeGuestRegs *regs) {
    (void)rsi; (void)image; (void)r14; (void)regs;
    if (!atomic_load_explicit(&active,memory_order_relaxed) || !controller) return;
    const GameOffsets *o=offsets;
    const uintptr_t owner=(uintptr_t)load_u64((uintptr_t)controller+o->controller_owner);
    const uintptr_t player=atomic_load_explicit(&gs_player,memory_order_relaxed);
    if (player && owner==player) {
        if (!atomic_exchange_explicit(&owner_confirmed,1,memory_order_relaxed))
            state_log("weapon controller %#" PRIx64 " belongs to the player\n",controller);
    } else if (atomic_load_explicit(&owner_confirmed,memory_order_relaxed)) {
        atomic_fetch_add_explicit(&gs_controller_foreign,1,memory_order_relaxed);
        return;
    }
    atomic_fetch_add_explicit(&gs_controller_calls,1,memory_order_relaxed);
    if (atomic_exchange_explicit(&gs_controller,(uintptr_t)controller,memory_order_relaxed)!=(uintptr_t)controller)
        state_log("weapon controller %#" PRIx64 " (owner %#" PRIxPTR ", player %#" PRIxPTR ")\n",controller,owner,player);
    WeaponState w;
    read_weapon((uintptr_t)controller,&w);
    w.us=now_us();
    pthread_mutex_lock(&state_lock);
    gs_weapon=w;
    pthread_mutex_unlock(&state_lock);
    publish_state(w.us);
}
/* r14 = the weapon WeaponController::Fire is about to take a round from. */
static void __attribute__((ms_abi)) state_shot(uint64_t rdi, uint64_t rsi, uint64_t image, uint64_t weapon,
                                                const RuntimeGuestRegs *regs) {
    (void)rdi; (void)rsi; (void)image; (void)regs;
    if (!atomic_load_explicit(&active,memory_order_relaxed) || !weapon) return;
    atomic_fetch_add_explicit(&gs_shots,1,memory_order_relaxed);
    const GameOffsets *o=offsets;
    const uint32_t type=load_u32((uintptr_t)weapon+o->weapon_type);
    int32_t ammo[3];
    memcpy(ammo,(const void *)(uintptr_t)(weapon+o->weapon_ammo),sizeof(ammo));
    emit("shot",",\"w\":\"0x%" PRIx32 "\",\"clip\":%" PRId32 ",\"res\":%" PRId32 ",\"cap\":%" PRId32,type,ammo[0],ammo[1],ammo[2]);
}
/* The damage branch: player = r14, damage = r15 (saved by the loader's thunk in regs). */
static void __attribute__((ms_abi)) state_damage(uint64_t rdi, uint64_t rsi, uint64_t image, uint64_t player,
                                                 const RuntimeGuestRegs *regs) {
    (void)rdi; (void)rsi; (void)image;
    const uint64_t damage=regs ? regs->r15 : 0;
    if (!atomic_load_explicit(&active,memory_order_relaxed) || !player || !damage) return;
    atomic_fetch_add_explicit(&gs_damages,1,memory_order_relaxed);
    const GameOffsets *o=offsets;
    const uint32_t type=load_u32((uintptr_t)damage+o->damage_type);
    const uint64_t source=load_u64((uintptr_t)damage+o->damage_source);
    const float amount=load_float((uintptr_t)damage+o->damage_amount);
    const float health=load_float((uintptr_t)player+o->health);
    float loc[8]={0,0,0,0,0,0,0,1}, src[4]={0,0,0,0};
    const uint64_t locator=load_u64((uintptr_t)player+o->locator);
    const int have_loc=locator && peek((uintptr_t)locator+0x10,loc,sizeof(loc));
    const int have_src=source && peek((uintptr_t)source+o->source_position,src,sizeof(src));
    emit("dmg",",\"ty\":%" PRIu32 ",\"amt\":%ld,\"hp\":%" PRId32 ",\"src\":%d,\"sx\":%ld,\"sy\":%ld,\"sz\":%ld,"
         "\"loc\":%d,\"px\":%ld,\"py\":%ld,\"pz\":%ld,\"qx\":%ld,\"qy\":%ld,\"qz\":%ld,\"qw\":%ld",
         type,scaled(amount,1000.0f),health_milli(health),have_src,scaled(src[0],1000.0f),scaled(src[1],1000.0f),
         scaled(src[2],1000.0f),have_loc,scaled(loc[0],1000.0f),scaled(loc[1],1000.0f),scaled(loc[2],1000.0f),
         scaled(loc[4],10000.0f),scaled(loc[5],10000.0f),scaled(loc[6],10000.0f),scaled(loc[7],10000.0f));
}
/* U1 code at the hooks: pattern, then the hook's distance into it and the bytes moved to the thunk
 * (whole position-independent instructions). Wildcards: RIP-relative displacements, call targets,
 * and the two instructions GoldHEN-style cheats rewrite (the damage branch's jne, the
 * ConsumeRound call), so the hooks still attach with the port's Infinite Health / Ammo cheats on.
 * With Infinite Health the game skips the damage branch: no dmg events then. */
static const RuntimeGuestCallback u1_state_callbacks[]={
    /* 0x2645f0: push rbp; mov rbp, rsp; push r15 r14 r13 r12 rbx; sub rsp, 0x1d8; mov r12, rdi; ... */
    {"UNDS U1 player update (state)",
      "55 48 89 e5 41 57 41 56 41 55 41 54 53 48 81 ec d8 01 00 00 49 89 fc 48 8b 05 ?? ?? ?? ?? "
      "48 8b 00 48 89 45 d0 e8 ?? ?? ?? ?? c7 45 b8 08 00 00 00 48 8d 0d ?? ?? ?? ?? 48 89 4d b0 "
      "c7 45 a8 00 00 00 00 48 c7 45 a0 00 00 00 00",
      0, 6, state_player, NULL},
    /* 0x71e920: WeaponController::PostJointUpdate prologue; mov r14, rdi; ... cmp [kGrenade], 4;
     * [r14+0x190], [r14+0x128]->+0xa8, mov esi, 0xcd670e33 */
    {"UNDS U1 weapon controller (state)",
      "55 48 89 e5 41 57 41 56 41 54 53 48 81 ec 80 00 00 00 49 89 fe 4c 8b 3d ?? ?? ?? ?? "
      "49 8b 07 48 89 45 d8 48 8d 05 ?? ?? ?? ?? 83 38 04 74 0d 49 8b be 90 01 00 00 48 8b 07 ff 50 70 "
      "49 8b 86 28 01 00 00 48 8b b8 a8 00 00 00 be 33 0e 67 cd",
      0, 6, state_controller, NULL},
    /* 0x71affe (pattern at 0x71afe9): WeaponController::Fire, xor esi, esi; mov rdi, r14; call ConsumeRound */
    {"UNDS U1 player shot (state)",
      "4c 8d 45 88 4c 8d 8d 78 ff ff ff bf 64 8f f1 b0 e8 ?? ?? ?? ?? 31 f6 4c 89 f7 ?? ?? ?? ?? ?? "
      "49 8b 04 24 48 3b 45 d8 75 0d 48 83 c4 70 5b 41 5c 41 5e 41 5f 5d c3",
      21, 5, state_shot, NULL},
    /* 0x261eea (pattern at 0x261ed0): Player::EventHandler damage branch,
     * vsubss xmm0, xmm0, [r15+0x14]; vmovss [r14+0x1428], xmm0 */
    {"UNDS U1 player damage (state)",
      "48 8d 15 ?? ?? ?? ?? 8a 12 0a 11 ?? 35 85 c0 75 31 c4 c1 7a 10 86 28 14 00 00 "
      "c4 c1 7a 5c 47 14 c4 c1 7a 11 86 28 14 00 00 49 8b be 48 02 00 00 48 8b 07 ff 50 28 "
      "48 8d 3d ?? ?? ?? ?? e8 ?? ?? ?? ??",
      26, 6, state_damage, NULL},
};

/* ------------------------------------------------------------------------------------------
 * U2 (big2-ps4_Shipping.elf of CUSA02320 1.02, sha256 c049a945...a1383b4ee8; image offsets)
 * [VERIFICADO by disassembly of that executable; each pattern below matches once in its
 * executable segments and none in U1 1.02]. The player's class (vtable 0x1010980) has its
 * EventHandler at +0x38 (0x7edd80: the port's health cheat jumps at 0x7f0143) and a per-frame
 * update at +0x100 (0x9717b0).
 *   0x971b64  in that update, right after HealthSystem::GetHealthFraction: r15 = the player,
 *             xmm0 = health / maximum, clamped to 0..1 (the value the game hands to
 *             [player+0x28f0]+0x28; the "Enable health effect test" option sets the health just
 *             below). +0x1f0: the HealthSystem (vtable 0x103f200; +0x48 GetHealth regenerates from
 *             +0x2c, the health after the last hit, its frame +0x20, delay +0x24 and rate +0x28,
 *             up to the maximum at +0x30). +0x2768: the WeaponController (the EventHandler and
 *             0x7172c2 use it; its +0x128 is the player). Locators (get-object-position 0xa12390):
 *             the parent's at +0xf0 (position) / +0x100 (rotation x y z w), the object's own
 *             relative to it at +0x110 / +0x120; the world one is parent * own (the parent is the
 *             identity off moving platforms) [VERIFICADO 2026-10-27: +0xf0 alone was 0 in 99 %
 *             of the hits].
 *   WeaponController (Fire 0x9614a0): current slot byte at +0x208, handles at +0x130, 16 bytes
 *             each (slot pointer, u32 id at +8; valid when (*slot)+0x44 == id, as Fire checks).
 *             +0x128: the owner.
 *   weapon    +0x550: u32 weapon type string id (Fire compares it; 0xff5403e6 is in U1 too).
 *             Ammunition per fire mode, 32 bytes at +0x620: i16 clip (ConsumeRound 0xaea020:
 *             dec word [w+mode*32+0x620]), i16 reserve at +0x622 (Reload 0xae9430 moves it into
 *             the clip), i16 clip size at +0x624 (0xae4570 sets it from the weapon's settings).
 *   0x714a70  a helper the player's controllers call (66 call sites; rdi+8 = the player): its
 *             calls go out as upc and, by default, as ups [HIPÓTESIS: like U1's player update,
 *             none during cutscenes, while the frame hook (upf) runs in them too];
 *             UNDS_U2_UPS=frame makes ups the frames instead.
 *   0x961ac3  WeaponController::Fire: xor esi, esi; mov rdi, r14; call ConsumeRound (the port's
 *             ammo cheat NOPs that call at 0x961ac8): r14 = the weapon, r12 = the controller.
 *   0x7f06dc  the EventHandler applies damage (past the god-mode checks): r12 = HealthSystem,
 *             r13 = the damage (+0x24 i32 amount in health points, +8 source handle: pointer,
 *             u32 id at +0x10, valid when (*pointer)+0x44 == id), r15 = the player.
 * Grenades are not mapped yet (gr -1). */
#define U2_HEALTH_SYSTEM 0x1f0
#define U2_HEALTH_MAX 0x30
#define U2_CONTROLLER 0x2768
#define U2_LOCATOR 0xf0          /* parent position, rotation; own position, rotation: 4 vec4 */
#define U2_CONTROLLER_SLOT 0x208
#define U2_CONTROLLER_HANDLES 0x130
#define U2_CONTROLLER_OWNER 0x128
#define U2_HANDLES 8
#define U2_PROCESS_ID 0x44
#define U2_WEAPON_TYPE 0x550
#define U2_WEAPON_AMMO 0x620
#define U2_DAMAGE_AMOUNT 0x24
#define U2_DAMAGE_SOURCE 0x8


/* The world locator {x y z w, qx qy qz qw} of a U2 object: parent * own (get-object-position). */
static int u2_world_locator(uintptr_t object, float out[8]) {
    float l[16];
    if (!peek(object+U2_LOCATOR,l,sizeof(l))) return 0;
    const float *pp=l, *pq=l+4, *lp=l+8, *lq=l+12;
    /* v' = v + 2w (q x v) + 2 q x (q x v) */
    const float tx=2.0f*(pq[1]*lp[2]-pq[2]*lp[1]), ty=2.0f*(pq[2]*lp[0]-pq[0]*lp[2]), tz=2.0f*(pq[0]*lp[1]-pq[1]*lp[0]);
    out[0]=pp[0]+lp[0]+pq[3]*tx+(pq[1]*tz-pq[2]*ty);
    out[1]=pp[1]+lp[1]+pq[3]*ty+(pq[2]*tx-pq[0]*tz);
    out[2]=pp[2]+lp[2]+pq[3]*tz+(pq[0]*ty-pq[1]*tx);
    out[3]=1.0f;
    out[4]=pq[3]*lq[0]+pq[0]*lq[3]+pq[1]*lq[2]-pq[2]*lq[1];
    out[5]=pq[3]*lq[1]-pq[0]*lq[2]+pq[1]*lq[3]+pq[2]*lq[0];
    out[6]=pq[3]*lq[2]+pq[0]*lq[1]-pq[1]*lq[0]+pq[2]*lq[3];
    out[7]=pq[3]*lq[3]-pq[0]*lq[0]-pq[1]*lq[1]-pq[2]*lq[2];
    return 1;
}
/* {pointer to a slot, u32 id at +8}: the process in the slot while its id is still `id`. */
static uintptr_t u2_process(uintptr_t handle) {
    uint64_t pointer=0, process=0; uint32_t id=0, check=0;
    if (!peek(handle,&pointer,8) || !peek(handle+8,&id,4) || !pointer || !id ||
        !peek((uintptr_t)pointer,&process,8) || !process ||
        !peek((uintptr_t)process+U2_PROCESS_ID,&check,4) || check!=id) return 0;
    return (uintptr_t)process;
}
static void u2_read_weapon(uintptr_t controller, WeaponState *w) {
    w->ws=-1; w->w=0; w->clip=w->res=w->cap=-1; w->gr=w->grr=-1;
    uint8_t slot=0;
    if (!peek(controller+U2_CONTROLLER_SLOT,&slot,1) || slot>=U2_HANDLES) return;
    const uintptr_t weapon=u2_process(controller+U2_CONTROLLER_HANDLES+16u*slot);
    int16_t ammo[3]; uint32_t type;
    if (weapon && peek(weapon+U2_WEAPON_TYPE,&type,4) && peek(weapon+U2_WEAPON_AMMO,ammo,sizeof(ammo))) {
        w->ws=slot; w->w=type; w->clip=ammo[0]; w->res=ammo[1]; w->cap=ammo[2];
    }
}
/* The player's controller, or 0 if [player+0x2768] is not a controller of the player. */
static uintptr_t u2_controller(uintptr_t player) {
    uint64_t controller=0, owner=0;
    if (!peek(player+U2_CONTROLLER,&controller,8) || !controller ||
        !peek((uintptr_t)controller+U2_CONTROLLER_OWNER,&owner,8) || owner!=player) return 0;
    return (uintptr_t)controller;
}
/* Once a frame: r15 = the player, xmm0 = its health fraction. */
static void __attribute__((ms_abi)) state_u2_frame(uint64_t rdi, uint64_t rsi, uint64_t image, uint64_t r14,
                                                    const RuntimeGuestRegs *regs) {
    (void)rdi; (void)rsi; (void)r14;
    if (!atomic_load_explicit(&active,memory_order_relaxed) || !regs || !regs->r15) return;
    const uintptr_t player=(uintptr_t)regs->r15;
    const float fraction=runtime_guest_xmm(regs,0)[0];
    atomic_fetch_add_explicit(&gs_frame_calls,1,memory_order_relaxed);
    if (!atomic_load_explicit(&ups_from_control,memory_order_relaxed))
        atomic_fetch_add_explicit(&gs_player_calls,1,memory_order_relaxed);
    if (atomic_exchange_explicit(&gs_player,player,memory_order_relaxed)!=player) {
        state_log("U2 player object %#" PRIxPTR " (image %#" PRIx64 "), health %.3f\n",player,image,(double)fraction);
        pthread_mutex_lock(&state_lock);
        gs_valid=0; /* a new player (level load): the next state goes out at once */
        pthread_mutex_unlock(&state_lock);
    }
    WeaponState w;
    const uintptr_t controller=u2_controller(player);
    if (controller) {
        atomic_fetch_add_explicit(&gs_controller_calls,1,memory_order_relaxed);
        if (atomic_exchange_explicit(&gs_controller,controller,memory_order_relaxed)!=controller)
            state_log("U2 weapon controller %#" PRIxPTR "\n",controller);
        u2_read_weapon(controller,&w);
    } else {
        w.ws=-1; w.w=0; w.clip=w.res=w.cap=-1; w.gr=w.grr=-1;
    }
    const uint64_t now=now_us();
    w.us=now;
    pthread_mutex_lock(&state_lock);
    gs_hp=health_milli(fraction); gs_inj=0; gs_have_player=1; gs_weapon=w;
    pthread_mutex_unlock(&state_lock);
    publish_state(now);
}
/* rdi+8 = the player when one of the player's controllers runs. */
static void __attribute__((ms_abi)) state_u2_control(uint64_t rdi, uint64_t rsi, uint64_t image, uint64_t r14,
                                                      const RuntimeGuestRegs *regs) {
    (void)rsi; (void)image; (void)r14; (void)regs;
    if (!atomic_load_explicit(&active,memory_order_relaxed) || !rdi) return;
    uint64_t object=0;
    const uintptr_t player=atomic_load_explicit(&gs_player,memory_order_relaxed);
    if (player && peek((uintptr_t)rdi+8,&object,8) && (uintptr_t)object==player) {
        atomic_fetch_add_explicit(&gs_control_calls,1,memory_order_relaxed);
        if (atomic_load_explicit(&ups_from_control,memory_order_relaxed))
            atomic_fetch_add_explicit(&gs_player_calls,1,memory_order_relaxed);
    }
}
/* r14 = the weapon Fire takes a round from, r12 = its controller (the player's only). */
static void __attribute__((ms_abi)) state_u2_shot(uint64_t rdi, uint64_t rsi, uint64_t image, uint64_t weapon,
                                                   const RuntimeGuestRegs *regs) {
    (void)rdi; (void)rsi; (void)image;
    if (!atomic_load_explicit(&active,memory_order_relaxed) || !weapon || !regs) return;
    const uintptr_t player=atomic_load_explicit(&gs_player,memory_order_relaxed);
    uint64_t owner=0;
    if (!player || !peek((uintptr_t)regs->r12+U2_CONTROLLER_OWNER,&owner,8) || (uintptr_t)owner!=player) return;
    uint32_t type=0; int16_t ammo[3]={-1,-1,-1};
    if (!peek((uintptr_t)weapon+U2_WEAPON_TYPE,&type,4) || !peek((uintptr_t)weapon+U2_WEAPON_AMMO,ammo,sizeof(ammo))) return;
    atomic_fetch_add_explicit(&gs_shots,1,memory_order_relaxed);
    emit("shot",",\"w\":\"0x%" PRIx32 "\",\"clip\":%d,\"res\":%d,\"cap\":%d",type,ammo[0],ammo[1],ammo[2]);
}
/* r15 = the player, r13 = the damage, r12 = the player's HealthSystem. */
static void __attribute__((ms_abi)) state_u2_damage(uint64_t rdi, uint64_t rsi, uint64_t image, uint64_t r14,
                                                     const RuntimeGuestRegs *regs) {
    (void)rdi; (void)rsi; (void)image; (void)r14;
    if (!atomic_load_explicit(&active,memory_order_relaxed) || !regs || !regs->r15 || !regs->r13) return;
    const uintptr_t player=(uintptr_t)regs->r15, damage=(uintptr_t)regs->r13, health=(uintptr_t)regs->r12;
    if (player!=atomic_load_explicit(&gs_player,memory_order_relaxed)) return;
    int32_t amount=0, maximum=0; uint32_t type=0;
    if (!peek(damage+U2_DAMAGE_AMOUNT,&amount,4)) return;
    peek(damage,&type,4);
    const long milli=peek(health+U2_HEALTH_MAX,&maximum,4) && maximum>0 ? lround(1000.0*(double)amount/(double)maximum) : -1;
    atomic_fetch_add_explicit(&gs_damages,1,memory_order_relaxed);
    float loc[8]={0,0,0,0,0,0,0,1}, src[8]={0,0,0,0,0,0,0,1};
    const int have_loc=u2_world_locator(player,loc);
    const uintptr_t source=u2_process(damage+U2_DAMAGE_SOURCE);
    const int have_src=source && u2_world_locator(source,src);
    pthread_mutex_lock(&state_lock);
    const int32_t hp=gs_hp;
    pthread_mutex_unlock(&state_lock);
    emit("dmg",",\"ty\":%" PRIu32 ",\"amt\":%ld,\"hp\":%" PRId32 ",\"src\":%d,\"sx\":%ld,\"sy\":%ld,\"sz\":%ld,"
         "\"loc\":%d,\"px\":%ld,\"py\":%ld,\"pz\":%ld,\"qx\":%ld,\"qy\":%ld,\"qz\":%ld,\"qw\":%ld,\"hpt\":%" PRId32,
         type,milli,hp,have_src,scaled(src[0],1000.0f),scaled(src[1],1000.0f),scaled(src[2],1000.0f),
         have_loc,scaled(loc[0],1000.0f),scaled(loc[1],1000.0f),scaled(loc[2],1000.0f),
         scaled(loc[4],10000.0f),scaled(loc[5],10000.0f),scaled(loc[6],10000.0f),scaled(loc[7],10000.0f),amount);
}
static const RuntimeGuestCallback u2_state_callbacks[]={
    /* 0x971b64 (pattern at 0x971b3a): mov rdi, [r15+0x1f0]; call [vtable+0x80] (health fraction);
     * clamp to 0..1; here: mov rdi, [r15+0x28f0]; ... */
    {"UNDS U2 player frame (state)",
      "49 8b bf f0 01 00 00 48 8b 07 ff 90 80 00 00 00 c5 f8 28 c8 c5 fa 10 05 ?? ?? ?? ?? c5 f8 2e c8 77 ?? "
      "c5 f8 57 c0 c5 f2 5f c0 49 8b bf f0 28 00 00 48 8b 07 ff 50 28 4c 89 ff e8 ?? ?? ?? ?? 4c 89 ff "
      "e8 ?? ?? ?? ?? 4c 89 ff e8 ?? ?? ?? ?? 4c 89 ff e8 ?? ?? ?? ?? 41 f6 87 5d 2c 00 00 02 74 ??",
      42, 7, state_u2_frame, NULL},
    /* 0x714a70: push rbp; mov rbp, rsp; push r15 ...; sub rsp, 0x4b8; mov r14, rdi; ...; mov rsi, [r14+8];
     * cmp byte [rsi+0x3e4], 0 */
    {"UNDS U2 player control (state)",
      "55 48 89 e5 41 57 41 56 41 55 41 54 53 48 81 ec b8 04 00 00 49 89 fe 48 8b 05 ?? ?? ?? ?? 48 8b 00 "
      "48 89 45 d0 49 8b 76 08 80 be e4 03 00 00 00 0f 84 ?? ?? ?? ?? 48 8d 05 ?? ?? ?? ?? 8a 00 84 c0 "
      "0f 85 ?? ?? ?? ??",
      0, 6, state_u2_control, NULL},
    /* 0x961ac3 (pattern at 0x961a96): Fire, mov esi, 0xb0f18f64; call; cmp [r14+0x30], 0xc3792632; ...;
     * xor esi, esi; mov rdi, r14; call ConsumeRound (or the ammo cheat's NOPs) */
    {"UNDS U2 player shot (state)",
      "be 64 8f f1 b0 e8 ?? ?? ?? ?? 41 81 7e 30 32 26 79 c3 75 ?? 48 8d 05 ?? ?? ?? ?? 83 b8 a4 48 00 00 00 "
      "75 ?? 83 3d ?? ?? ?? ?? 02 7c ?? 31 f6 4c 89 f7 ?? ?? ?? ?? ?? 31 f6 4c 89 f7 e8 ?? ?? ?? ?? 84 c0",
      45, 5, state_u2_shot, NULL},
    /* 0x7f06dc (pattern at 0x7f06b9): EventHandler, mov rax, [r12]; mov esi, [r13+0x24]; mov rdi, r12;
     * call [rax+8] (TakeDamage) */
    {"UNDS U2 player damage (state)",
      "49 8b 07 49 8b 4f 70 48 8b 80 e8 01 00 00 8b 49 0c 89 8d 00 5a ff ff 48 8d b5 00 5a ff ff 4c 89 ff "
      "ff d0 49 8b 04 24 41 8b 75 24 4c 89 e7 ff 50 08 48 8d 05 ?? ?? ?? ?? 80 38 00 74 ?? 8b 05 ?? ?? ?? ?? "
      "83 f8 02 7c ?? 48 8d 0d ?? ?? ?? ?? 8a 89 7c 49 00 00 84 c9 75 ??",
      35, 8, state_u2_damage, NULL},
};

const RuntimeGuestCallback *runtime_unds_guest_callbacks(size_t *count) {
    *count=0;
    const char *events=getenv("UNDS_EVENTS"), *state=getenv("UNDS_STATE"), *debug=getenv("UNDS_STATE_DEBUG");
    if (!events || !*events || (state && !strcmp(state,"0"))) return u1_state_callbacks;
    exe_names();
    const RuntimeGuestCallback *table=NULL;
    if (!strcmp(exe_id,"u1")) { table=u1_state_callbacks; *count=sizeof(u1_state_callbacks)/sizeof(*u1_state_callbacks); }
    else if (!strcmp(exe_id,"u2")) { table=u2_state_callbacks; *count=sizeof(u2_state_callbacks)/sizeof(*u2_state_callbacks); }
    else return u1_state_callbacks; /* U3: not mapped yet */
    offsets=&u1_100;
    atomic_store(&control_enabled,table==u2_state_callbacks);
    const char *u2_ups=getenv("UNDS_U2_UPS");
    atomic_store(&ups_from_control,!(u2_ups && !strcmp(u2_ups,"frame")));
    atomic_store(&state_enabled,1);
    atomic_store(&state_debug,debug && !strcmp(debug,"1"));
    return table;
}

void runtime_unds_report(void) {
    if (!atomic_load(&configured)) return;
    if (atomic_load(&state_enabled))
        printf("Runtime: UNDS state: player updates=%" PRIu64 ", weapon controller updates=%" PRIu64 " (other objects %" PRIu64
               "), U2 frames=%" PRIu64 " controls=%" PRIu64 ", shots=%" PRIu64 ", damage=%" PRIu64 ", failed reads=%" PRIu64 "\n",
               atomic_load(&gs_player_calls),atomic_load(&gs_controller_calls),atomic_load(&gs_controller_foreign),
               atomic_load(&gs_frame_calls),atomic_load(&gs_control_calls),atomic_load(&gs_shots),atomic_load(&gs_damages),
               atomic_load(&gs_peek_failures));
    printf("Runtime: UNDS events sent=%" PRIu64 ", dropped=%" PRIu64 ", errors=%" PRIu64 " (last %d), "
           "longest sendto %" PRIu64 " us, sends over 1 ms=%" PRIu64 ", scePadSetVibration calls=%" PRIu64 "\n",
           atomic_load(&sent),atomic_load(&dropped),atomic_load(&errors),atomic_load(&last_error),
           atomic_load(&send_max_ns_total)/1000u,atomic_load(&slow_sends),atomic_load(&vibration_calls));
    if (atomic_load(&fx_on))
        printf("Runtime: UNDS fx: commands=%" PRIu64 ", rejected=%" PRIu64 ", SDL failures=%" PRIu64 ", rewritten after a "
               "reconnect=%" PRIu64 ", watchdog releases=%" PRIu64 "\n",
               atomic_load(&fx_commands),atomic_load(&fx_rejected),atomic_load(&fx_failures),atomic_load(&fx_reapplied),
               atomic_load(&fx_timeouts));
}
