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
 *          rumble(1 SDL rumble on) sent drop err smax(longest sendto since last hello, us)
 *          vib(scePadSetVibration calls) ports("id:type:channels:frames,...")
 *   bye    reason
 *   rumble l s (0-255) re(1 refresh)
 *   led    r g b re(1 refresh)
 *   pad    b(button mask, BTN_* of runtime_pad.c) lx ly rx ry l2 r2 (0-255)
 *   aopen  p(port) ty(type) ch fr(frames per buffer)
 *   audio  p ty f(first frame of the window, port timeline) n(frames) rms pk lf
 *          levels x10000 of full scale over the window: rms of L/R (8 channels folded like the
 *          stereo output: centre and surrounds at -3 dB, LFE dropped), peak |L|,|R|, and rms
 *          of (L+R)/2 low-passed at 150 Hz (one pole). Digital silence (pk 0) is sent once and
 *          then refreshed every 1 s instead of every window. */
#define _GNU_SOURCE
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
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
#define HEARTBEAT_TICK_NS 250000000u
#define SLOW_SEND_NS 1000000u

static _Atomic int active;     /* sending */
static _Atomic int configured; /* UNDS_EVENTS was set (the report prints) */
static _Atomic int rumble_on=1;
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
typedef struct { uint32_t b; uint8_t a[6]; } PadState; /* lx ly rx ry l2 r2 */
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
    char text[512];
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
    char ports[256];
    pthread_mutex_lock(&state_lock);
    ports_text(ports,sizeof(ports));
    pthread_mutex_unlock(&state_lock);
    const uint64_t smax=atomic_exchange_explicit(&send_max_ns,0,memory_order_relaxed);
    emit("hello",",\"boot\":%d,\"exe\":\"%s\",\"bin\":\"%s\",\"pid\":%lu,\"base\":\"0x%" PRIxPTR "\",\"ver\":\"%s\","
         "\"up\":%" PRIu64 ",\"rumble\":%d,\"sent\":%" PRIu64 ",\"drop\":%" PRIu64 ",\"err\":%" PRIu64 ","
         "\"smax\":%" PRIu64 ",\"vib\":%" PRIu64 ",\"ports\":\"%s\"",
         boot,exe_id,exe_file,process_id,image_base,UNDS_VERSION,(now_us()-start_us)/1000u,
         atomic_load_explicit(&rumble_on,memory_order_relaxed),
         atomic_load_explicit(&sent,memory_order_relaxed),atomic_load_explicit(&dropped,memory_order_relaxed),
         atomic_load_explicit(&errors,memory_order_relaxed),smax/1000u,
         atomic_load_explicit(&vibration_calls,memory_order_relaxed),ports);
}

static void send_pad(const PadState *s) {
    emit("pad",",\"b\":%" PRIu32 ",\"lx\":%u,\"ly\":%u,\"rx\":%u,\"ry\":%u,\"l2\":%u,\"r2\":%u",
         s->b,s->a[0],s->a[1],s->a[2],s->a[3],s->a[4],s->a[5]);
}
/* The pending pad state, when the rate limit allows it now (called with state_lock held). */
static int take_pad(uint64_t now, PadState *out) {
    if (!pad_pending || now-pad_sent_us<PAD_MIN_INTERVAL_US) return 0;
    *out=pad_now; pad_sent=pad_now; pad_pending=0; pad_sent_us=now;
    return 1;
}
static int pad_differs(const PadState *a, const PadState *b) {
    if (a->b!=b->b || a->a[4]!=b->a[4] || a->a[5]!=b->a[5]) return 1;
    for (int i=0;i<4;++i) { int d=(int)a->a[i]-(int)b->a[i]; if (d>=STICK_DEADBAND || d<=-STICK_DEADBAND) return 1; }
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
#ifdef WSA_FLAG_NO_HANDLE_INHERIT
    sock=WSASocketW(AF_INET,SOCK_DGRAM,IPPROTO_UDP,NULL,0,WSA_FLAG_NO_HANDLE_INHERIT);
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

void runtime_unds_start(uintptr_t image) {
    const char *rumble=getenv("UNDS_RUMBLE");
    if (rumble && !strcmp(rumble,"0")) {
        atomic_store(&rumble_on,0);
        puts("Runtime: UNDS: SDL rumble off (UNDS_RUMBLE=0); scePadSetVibration is still published");
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
    atomic_store(&active,1);
    printf("Runtime: UNDS: events to %u.%u.%u.%u:%u (protocol %d, %s, %s)\n",ip>>24,(ip>>16)&255u,(ip>>8)&255u,ip&255u,
           port,UNDS_PROTOCOL,UNDS_VERSION,exe_id);
    send_hello(1);
    pthread_t thread;
    if (pthread_create(&thread,NULL,heartbeat,NULL)) puts("Runtime: UNDS: no heartbeat thread (pthread_create failed)");
    else pthread_detach(thread);
}

int runtime_unds_rumble_enabled(void) { return atomic_load_explicit(&rumble_on,memory_order_relaxed); }

void runtime_unds_bye(const char *reason) {
    if (!atomic_load_explicit(&active,memory_order_relaxed)) return;
    if (atomic_exchange(&bye_done,1)) return;
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

void runtime_unds_pad(uint32_t buttons, uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry, uint8_t l2, uint8_t r2) {
    if (!atomic_load_explicit(&active,memory_order_relaxed)) return;
    const uint64_t now=now_us();
    const PadState state={buttons,{lx,ly,rx,ry,l2,r2}};
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

void runtime_unds_report(void) {
    if (!atomic_load(&configured)) return;
    printf("Runtime: UNDS events sent=%" PRIu64 ", dropped=%" PRIu64 ", errors=%" PRIu64 " (last %d), "
           "longest sendto %" PRIu64 " us, sends over 1 ms=%" PRIu64 ", scePadSetVibration calls=%" PRIu64 "\n",
           atomic_load(&sent),atomic_load(&dropped),atomic_load(&errors),atomic_load(&last_error),
           atomic_load(&send_max_ns_total)/1000u,atomic_load(&slow_sends),atomic_load(&vibration_calls));
}
