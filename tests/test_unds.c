/* UNDS event bus: endpoint parsing, change detection, pad rate limit, audio levels, protocol
 * fields, seq continuity and "bye". Sends to a UDP socket of its own on 127.0.0.1.
 * Linux:            cc -std=c11 -Wall -Wextra -Werror -pthread -Isrc tests/test_unds.c -lm -o out/unds-test
 * MSYS2 CLANG64:    clang -std=c11 -Wall -Wextra -Werror -pthread -Isrc tests/test_unds.c src/compat_win.c -lws2_32 -lbcrypt -o out/unds-test.exe */
#include "../src/runtime_unds.c"
#undef NDEBUG /* the checks below have side effects */
#include <assert.h>

static unds_socket_t rx=UNDS_INVALID_SOCKET;
static uint32_t expect_seq;

static void open_receiver(char *endpoint, size_t size) {
#ifdef _WIN32
    WSADATA data; assert(!WSAStartup(MAKEWORD(2,2),&data));
#endif
    rx=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    assert(rx!=UNDS_INVALID_SOCKET);
    struct sockaddr_in a; memset(&a,0,sizeof(a));
    a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(0x7f000001u); a.sin_port=0;
    assert(!bind(rx,(struct sockaddr *)&a,sizeof(a)));
    socklen_t len=sizeof(a);
    assert(!getsockname(rx,(struct sockaddr *)&a,&len));
    snprintf(endpoint,size,"127.0.0.1:%u",(unsigned)ntohs(a.sin_port));
}
/* Next datagram within `ms`, or 0. */
static int receive(char *out, size_t size, int ms) {
    fd_set set; FD_ZERO(&set); FD_SET(rx,&set);
    struct timeval timeout={ms/1000,(ms%1000)*1000};
    if (select((int)rx+1,&set,NULL,NULL,&timeout)<=0) return 0;
    int n=(int)recv(rx,out,(int)size-1,0);
    assert(n>0);
    out[n]=0;
    return 1;
}
static long long field(const char *json, const char *key) {
    char pattern[64]; snprintf(pattern,sizeof(pattern),"\"%s\":",key);
    const char *at=strstr(json,pattern);
    assert(at && "missing field");
    return strtoll(at+strlen(pattern),NULL,10);
}
static int has(const char *json, const char *text) { return strstr(json,text)!=NULL; }
/* Next datagram that is not a heartbeat (hello, refreshes); checks v and seq continuity. */
static int next_event(char *out, size_t size, int ms) {
    for (;;) {
        if (!receive(out,size,ms)) return 0;
        assert(field(out,"v")==1);
        assert((uint32_t)field(out,"seq")==expect_seq);
        ++expect_seq;
        if (has(out,"\"e\":\"hello\"") || has(out,"\"re\":1")) continue;
        return 1;
    }
}
static void no_event(int ms) {
    char text[512];
    assert(!next_event(text,sizeof(text),ms));
}
static void set_env(const char *name, const char *value) {
#ifdef _WIN32
    _putenv_s(name,value);
#else
    setenv(name,value,1);
#endif
}

static void test_parse(void) {
    uint32_t ip; uint16_t port;
    assert(!parse_endpoint("127.0.0.1:47111",&ip,&port) && ip==0x7f000001u && port==47111);
    assert(!parse_endpoint("localhost:9",&ip,&port) && ip==0x7f000001u && port==9);
    assert(!parse_endpoint("47111",&ip,&port) && ip==0x7f000001u && port==47111);
    assert(!parse_endpoint("192.168.1.20:65535",&ip,&port) && ip==0xc0a80114u && port==65535);
    const char *bad[]={"", ":1", "127.0.0.1:", "127.0.0.1:0", "127.0.0.1:65536", "127.0.0:1", "127.0.0.1.5:1",
                       "256.0.0.1:1", "host.lan:1", "127.0.0.1:12a", "1270.0.0.1:1", "::1:5", "127.0.0.1 :1"};
    for (size_t i=0;i<sizeof(bad)/sizeof(*bad);++i) assert(parse_endpoint(bad[i],&ip,&port));
}

static void test_audio(void) {
    char e[512];
    runtime_unds_audio_open(0,0,8,1024);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"aopen\"") && field(e,"p")==0 && field(e,"ch")==8 && field(e,"fr")==1024);
    /* 1.5 kHz (16 whole periods per window), 0.5 peak on FL only (8 ch float): rms over L/R 0.25, peak 0.5 */
    static float pcm[1024*8];
    for (int f=0;f<1024;++f) pcm[f*8]=0.5f*sinf(2.0f*3.14159265f*1500.0f*(float)f/48000.0f);
    runtime_unds_audio(0,0,pcm,1024,8,1);
    for (int w=0;w<2;++w) {
        assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"audio\""));
        assert(field(e,"p")==0 && field(e,"ty")==0 && field(e,"n")==512 && field(e,"f")==w*512);
        assert(llabs(field(e,"rms")-2500)<=2 && llabs(field(e,"pk")-5000)<=2);
        assert(field(e,"lf")<300); /* 1.5 kHz is well above the 150 Hz low-pass */
    }
    /* 93.75 Hz (one period per window) on FL and FR: rms 0.3536, mostly passes the low-pass (|H| 0.85) */
    for (int f=0;f<1024;++f) pcm[f*8]=pcm[f*8+1]=0.5f*sinf(2.0f*3.14159265f*93.75f*(float)f/48000.0f);
    runtime_unds_audio(0,0,pcm,1024,8,1);
    assert(next_event(e,sizeof(e),500));
    assert(next_event(e,sizeof(e),500) && field(e,"f")==1536 && llabs(field(e,"rms")-3536)<=2 && field(e,"lf")>2800 && field(e,"lf")<3100);
    /* Silence: one window is sent, the following ones are held back for 1 s */
    memset(pcm,0,sizeof(pcm));
    runtime_unds_audio(0,0,pcm,1024,8,1);
    runtime_unds_audio(0,0,pcm,1024,8,1);
    assert(next_event(e,sizeof(e),500) && field(e,"pk")==0 && field(e,"rms")==0 && field(e,"f")==2048);
    no_event(100);
    /* s16 mono (pad speaker port): full-scale square of +-16384 -> rms 0.5, peak 0.5 */
    runtime_unds_audio_open(17,4,1,256);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"aopen\""));
    static int16_t mono[256];
    for (int f=0;f<256;++f) mono[f]=(int16_t)(f&1 ? 16384 : -16384);
    runtime_unds_audio(17,4,mono,256,1,0);
    no_event(50); /* half a window */
    runtime_unds_audio(17,4,mono,256,1,0);
    assert(next_event(e,sizeof(e),500) && field(e,"p")==17 && field(e,"ty")==4 && field(e,"rms")==5000 && field(e,"pk")==5000);
}

int main(void) {
    test_parse();
    set_env("BB_EXE","big2-ps4_Shipping.elf");
    /* No UNDS_EVENTS: nothing is active, every call returns at once. */
    runtime_unds_start(0x1234);
    assert(!atomic_load(&active) && !atomic_load(&configured) && runtime_unds_rumble_enabled());
    runtime_unds_rumble(1,2); runtime_unds_pad(0,1,2,3,4,5,6); runtime_unds_bye("x");
    assert(!atomic_load(&bye_done) && !atomic_load(&sent));

    char endpoint[64], e[512];
    open_receiver(endpoint,sizeof(endpoint));
    set_env("UNDS_EVENTS",endpoint);
    set_env("UNDS_RUMBLE","0");
    runtime_unds_start(0xb400000);
    assert(atomic_load(&active) && !runtime_unds_rumble_enabled());
    assert(receive(e,sizeof(e),500) && has(e,"\"e\":\"hello\"") && field(e,"seq")==0 && field(e,"boot")==1);
    assert(has(e,"\"exe\":\"u2\"") && has(e,"\"bin\":\"big2-ps4_Shipping.elf\"") && has(e,"\"base\":\"0xb400000\""));
    assert(field(e,"rumble")==0 && field(e,"pid")>0);
    expect_seq=1;

    /* rumble and LED: only changes */
    runtime_unds_rumble(10,20);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"rumble\"") && field(e,"l")==10 && field(e,"s")==20);
    runtime_unds_rumble(10,20);
    no_event(30);
    runtime_unds_rumble(0,0);
    assert(next_event(e,sizeof(e),500) && field(e,"l")==0 && field(e,"s")==0);
    runtime_unds_led(0,0,255); runtime_unds_led(0,0,255);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"led\"") && field(e,"b")==255);
    no_event(30);

    /* pad: first state, stick noise below the deadband, buttons through the limit, analog held */
    runtime_unds_pad(0,128,128,128,128,0,0);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"pad\"") && field(e,"lx")==128);
    runtime_unds_pad(0,129,127,128,128,0,0);
    no_event(30);
    runtime_unds_pad(0x200,128,128,128,128,0,255);           /* R2 pressed: sent at once */
    assert(next_event(e,sizeof(e),500) && field(e,"b")==0x200 && field(e,"r2")==255);
    runtime_unds_pad(0x200,128,128,128,128,0,200);           /* analog only, <8 ms later: held */
    runtime_unds_pad(0x200,128,128,128,128,0,180);
    assert(next_event(e,sizeof(e),400) && field(e,"r2")==180); /* sent by the heartbeat */
    runtime_unds_pad(0,128,128,128,128,0,0);                 /* release: at once */
    assert(next_event(e,sizeof(e),500) && field(e,"b")==0 && field(e,"r2")==0);

    test_audio();

    /* heartbeat: hello every second with counters, refreshes carry re:1 */
    int hello=0, refresh=0;
    for (int i=0;i<40 && (!hello || !refresh);++i) {
        if (!receive(e,sizeof(e),100)) continue;
        assert((uint32_t)field(e,"seq")==expect_seq); ++expect_seq;
        if (has(e,"\"e\":\"hello\"")) { hello=1; assert(field(e,"boot")==0 && field(e,"sent")>0 && has(e,"\"ports\":\"0:0:8:1024,17:4:1:256\"")); }
        if (has(e,"\"re\":1")) refresh=1;
    }
    assert(hello && refresh);

    runtime_unds_bye("loadexec");
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"bye\"") && has(e,"\"reason\":\"loadexec\""));
    runtime_unds_bye("again");
    runtime_unds_rumble(1,1);
    char extra[512];
    assert(!receive(extra,sizeof(extra),600)); /* nothing after bye, not even the heartbeat */
    runtime_unds_report();
    assert(!atomic_load(&errors) && !atomic_load(&dropped));
    puts("unds-test: ok");
    return 0;
}
