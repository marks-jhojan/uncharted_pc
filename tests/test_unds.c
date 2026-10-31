/* UNDS event bus: endpoint parsing, change detection, pad rate limit, audio levels, protocol
 * fields, seq continuity, "bye", UNDS_LED and UNDS_TRIGGER. Sends to a UDP socket of its own on 127.0.0.1.
 * Linux:            cc -std=c11 -Wall -Wextra -Werror -pthread -Isrc tests/test_unds.c -lm -o out/unds-test
 * MSYS2 CLANG64:    clang -std=c11 -Wall -Wextra -Werror -pthread -Isrc tests/test_unds.c src/compat_win.c -lws2_32 -lbcrypt -o out/unds-test.exe */
#include "../src/runtime_unds.c"
#undef NDEBUG /* the checks below have side effects */
#include <assert.h>
#include <ctype.h>

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

static void test_thresholds(void) {
    int t[2]={0,0};
    assert(!parse_thresholds("30,30",t) && t[0]==30 && t[1]==30);
    assert(!parse_thresholds("1,254",t) && t[0]==1 && t[1]==254);
    assert(!parse_thresholds("40,140",t) && t[0]==40 && t[1]==140);
    const char *bad[]={"", "30", "30,", ",30", "0,30", "30,255", "30;30", "30,30,30", "a,30", "30, 30", "1000,30", "-1,30"};
    for (size_t i=0;i<sizeof(bad)/sizeof(*bad);++i) assert(parse_thresholds(bad[i],t));
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

/* Game state: the callbacks on objects laid out like U1 1.00's (player, weapon controller,
 * weapons, handle slots, a damage record). */
static void test_state(void) {
    char e[512];
    size_t count=0;
    const RuntimeGuestCallback *c=runtime_unds_guest_callbacks(&count);
    assert(count==4 && atomic_load(&state_enabled));
    assert(c[0].callback==state_player && c[1].callback==state_controller && c[2].callback==state_shot && c[3].callback==state_damage);
    assert(c[0].skip==0 && c[0].moved==6 && c[1].skip==0 && c[1].moved==6 && c[2].skip==21 && c[2].moved==5 &&
           c[3].skip==26 && c[3].moved==6);
    for (size_t i=0;i<count;++i) {
        /* the loader's pattern syntax: "xx" or "??" separated by single spaces; the hook and the
           moved bytes lie inside the pattern and are not wildcards (they are copied to the thunk) */
        const char *t=c[i].pattern;
        size_t n=0, len=strlen(t);
        assert(len%3==2 && !strncmp(c[i].name,"UNDS U1 ",8) && !c[i].patch);
        for (size_t k=0;k<len;k+=3,++n) {
            assert((isxdigit((unsigned char)t[k]) && isxdigit((unsigned char)t[k+1])) || (t[k]=='?' && t[k+1]=='?'));
            assert(k+2==len || t[k+2]==' ');
            if (n>=c[i].skip && n<c[i].skip+c[i].moved) assert(t[k]!='?');
        }
        assert(c[i].skip+c[i].moved<=n);
    }
    static _Alignas(16) unsigned char player[0x1500], controller[0x200], rifle[0x500], grenade[0x500], damage[0x40], source[0x40];
    static _Alignas(16) float locator[12];
    static uint64_t rifle_slot, grenade_slot;
    float f; uint32_t u; int32_t ammo[3]; uint64_t p;
    RuntimeGuestRegs damage_regs={0};
    damage_regs.r15=(uint64_t)(uintptr_t)damage;
    /* player: full health, locator at (1, 2, 3) with the identity rotation */
    f=1.0f; memcpy(player+0x1428,&f,4); player[0x1430]=0;
    p=(uint64_t)(uintptr_t)locator; memcpy(player+0xf0,&p,8);
    locator[4]=1.0f; locator[5]=2.0f; locator[6]=3.0f; locator[11]=1.0f;
    /* before any player: the controller alone publishes nothing */
    p=(uint64_t)(uintptr_t)player; memcpy(controller+0x128,&p,8);
    state_controller((uint64_t)(uintptr_t)controller,0,0,0,NULL);
    no_event(30);
    state_player((uint64_t)(uintptr_t)player,0,0x400000,0,NULL);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"gs\"") && field(e,"hp")==1000 && field(e,"inj")==0);
    assert(field(e,"ws")==-1 && has(e,"\"w\":\"0x0\"") && field(e,"clip")==-1 && field(e,"gr")==-1);
    state_player((uint64_t)(uintptr_t)player,0,0x400000,0,NULL);
    no_event(30);
    /* a rifle in slot 2 (handle: slot pointer, id 77 = the process's +0x64), grenades in slot 4 */
    u=0xa1b2c3d4u; memcpy(rifle+0x2e4,&u,4); u=77; memcpy(rifle+0x64,&u,4);
    ammo[0]=30; ammo[1]=90; ammo[2]=30; memcpy(rifle+0x440,ammo,12);
    rifle_slot=(uint64_t)(uintptr_t)rifle; p=(uint64_t)(uintptr_t)&rifle_slot;
    memcpy(controller+0x130+16*2,&p,8); memcpy(controller+0x130+16*2+8,&u,4);
    u=5; memcpy(grenade+0x64,&u,4); ammo[0]=2; ammo[1]=1; memcpy(grenade+0x440,ammo,12);
    grenade_slot=(uint64_t)(uintptr_t)grenade; p=(uint64_t)(uintptr_t)&grenade_slot;
    memcpy(controller+0x130+16*4,&p,8); memcpy(controller+0x130+16*4+8,&u,4);
    u=2; memcpy(controller+0x198,&u,4);
    state_controller((uint64_t)(uintptr_t)controller,0,0,0,NULL);
    assert(atomic_load(&owner_confirmed));
    assert(next_event(e,sizeof(e),500) && field(e,"ws")==2 && has(e,"\"w\":\"0xa1b2c3d4\"") && field(e,"clip")==30 &&
           field(e,"res")==90 && field(e,"cap")==30 && field(e,"gr")==2 && field(e,"grr")==1);
    /* a shot: the round before ConsumeRound takes it, then the new clip */
    state_shot(0,0,0,(uint64_t)(uintptr_t)rifle,NULL);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"shot\"") && has(e,"\"w\":\"0xa1b2c3d4\"") && field(e,"clip")==30);
    ammo[0]=29; ammo[1]=90; ammo[2]=30; memcpy(rifle+0x440,ammo,12);
    state_controller((uint64_t)(uintptr_t)controller,0,0,0,NULL);
    assert(next_event(e,sizeof(e),500) && field(e,"clip")==29);
    /* stale handle (id changed: the weapon was dropped): no weapon */
    u=78; memcpy(rifle+0x64,&u,4);
    state_controller((uint64_t)(uintptr_t)controller,0,0,0,NULL);
    assert(next_event(e,sizeof(e),500) && field(e,"ws")==-1 && field(e,"clip")==-1 && field(e,"gr")==2);
    u=77; memcpy(rifle+0x64,&u,4);
    state_controller((uint64_t)(uintptr_t)controller,0,0,0,NULL);
    assert(next_event(e,sizeof(e),500) && field(e,"ws")==2);
    /* a controller of another object is ignored once the player's has been seen */
    static unsigned char other[0x200];
    memcpy(other,controller,sizeof(other)); p=1234; memcpy(other+0x128,&p,8); u=0; memcpy(other+0x198,&u,4);
    state_controller((uint64_t)(uintptr_t)other,0,0,0,NULL);
    no_event(30);
    assert(atomic_load(&gs_controller_foreign)==1);
    /* damage: 0.25 from a source at (4, 2, 3), health 1.0 before; then the new health */
    u=3; memcpy(damage,&u,4); p=(uint64_t)(uintptr_t)source; memcpy(damage+8,&p,8);
    f=0.25f; memcpy(damage+0x14,&f,4);
    f=4.0f; memcpy(source+0x10,&f,4); f=2.0f; memcpy(source+0x14,&f,4); f=3.0f; memcpy(source+0x18,&f,4);
    state_damage(0,0,0,(uint64_t)(uintptr_t)player,&damage_regs);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"dmg\"") && field(e,"ty")==3 && field(e,"amt")==250 && field(e,"hp")==1000);
    assert(field(e,"src")==1 && field(e,"sx")==4000 && field(e,"sz")==3000 && field(e,"loc")==1 && field(e,"px")==1000 &&
           field(e,"py")==2000 && field(e,"qw")==10000 && field(e,"qx")==0);
    f=0.75f; memcpy(player+0x1428,&f,4); player[0x1430]=1;
    state_player((uint64_t)(uintptr_t)player,0,0x400000,0,NULL);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"gs\"") && field(e,"hp")==750 && field(e,"inj")==1);
    /* regeneration: +0.002 steps wait 100 ms, +0.005 goes at once, full health at once */
    f=0.752f; memcpy(player+0x1428,&f,4);
    state_player((uint64_t)(uintptr_t)player,0,0x400000,0,NULL);
    no_event(30);
    f=0.757f; memcpy(player+0x1428,&f,4);
    state_player((uint64_t)(uintptr_t)player,0,0x400000,0,NULL);
    assert(next_event(e,sizeof(e),500) && field(e,"hp")==757);
    f=1.0f; memcpy(player+0x1428,&f,4); player[0x1430]=0;
    state_player((uint64_t)(uintptr_t)player,0,0x400000,0,NULL);
    assert(next_event(e,sizeof(e),500) && field(e,"hp")==1000);
    /* no source (a fall): src 0; a value that cannot be health: -1 */
    memset(damage+8,0,8);
    state_damage(0,0,0,(uint64_t)(uintptr_t)player,&damage_regs);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"dmg\"") && field(e,"src")==0 && field(e,"sx")==0);
    f=37.0f; memcpy(player+0x1428,&f,4);
    state_player((uint64_t)(uintptr_t)player,0,0x400000,0,NULL);
    assert(next_event(e,sizeof(e),500) && field(e,"hp")==-1);
    f=0.0f; memcpy(player+0x1428,&f,4);
    state_player((uint64_t)(uintptr_t)player,0,0x400000,0,NULL);
    assert(next_event(e,sizeof(e),500) && field(e,"hp")==0);
    /* unmapped memory: a read fails instead of faulting */
    char byte;
    assert(!peek(0x10,&byte,1) && !peek(UINTPTR_MAX-2,&byte,8));
}

int main(void) {
    test_parse();
    test_thresholds();
    set_env("BB_EXE","big2-ps4_Shipping.elf");
    /* No UNDS_* variables: defaults, nothing is active, every call returns at once. */
    size_t callbacks=1;
    runtime_unds_guest_callbacks(&callbacks);
    assert(!callbacks && !atomic_load(&state_enabled)); /* no game-state hooks without UNDS_EVENTS */
    runtime_unds_start(0x1234);
    assert(!atomic_load(&active) && !atomic_load(&configured) && runtime_unds_rumble_enabled() && runtime_unds_led_enabled());
    assert(runtime_unds_trigger_threshold(0)==30 && runtime_unds_trigger_threshold(1)==30);
    runtime_unds_rumble(1,2); runtime_unds_pad(0,1,2,3,4,5,6,7,8); runtime_unds_bye("x");
    assert(!atomic_load(&bye_done) && !atomic_load(&sent));
    /* UNDS_LED and UNDS_TRIGGER work without UNDS_EVENTS; a malformed UNDS_TRIGGER changes nothing */
    set_env("UNDS_TRIGGER","30,300");
    runtime_unds_start(0x1234);
    assert(runtime_unds_trigger_threshold(0)==30 && runtime_unds_trigger_threshold(1)==30 && !atomic_load(&active));
    set_env("UNDS_LED","0");
    set_env("UNDS_TRIGGER","40,140");
    runtime_unds_start(0x1234);
    assert(!runtime_unds_led_enabled() && runtime_unds_trigger_threshold(0)==40 && runtime_unds_trigger_threshold(1)==140);
    assert(!atomic_load(&active) && !atomic_load(&sent));

    char endpoint[64], e[512];
    open_receiver(endpoint,sizeof(endpoint));
    set_env("UNDS_EVENTS",endpoint);
    set_env("UNDS_STATE","0");
    runtime_unds_guest_callbacks(&callbacks);
    assert(!callbacks); /* UNDS_STATE=0: events without game-state hooks */
    set_env("UNDS_STATE","");
    set_env("UNDS_RUMBLE","0");
    runtime_unds_start(0xb400000);
    assert(atomic_load(&active) && !runtime_unds_rumble_enabled());
    assert(receive(e,sizeof(e),500) && has(e,"\"e\":\"hello\"") && field(e,"seq")==0 && field(e,"boot")==1);
    assert(has(e,"\"exe\":\"u2\"") && has(e,"\"bin\":\"big2-ps4_Shipping.elf\"") && has(e,"\"base\":\"0xb400000\""));
    assert(field(e,"rumble")==0 && field(e,"led")==0 && has(e,"\"trig\":\"40,140\"") && field(e,"pid")>0);
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
    runtime_unds_pad(0,128,128,128,128,0,0,0,0);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"pad\"") && field(e,"lx")==128 && field(e,"r2a")==0);
    runtime_unds_pad(0,129,127,128,128,0,0,1,1);             /* stick and trigger noise below the deadband */
    no_event(30);
    runtime_unds_pad(0,128,128,128,128,0,0,0,60);            /* R2 travel under the threshold: a change */
    assert(next_event(e,sizeof(e),500) && field(e,"b")==0 && field(e,"r2")==0 && field(e,"r2a")==60 && field(e,"l2a")==0);
    runtime_unds_pad(0x200,128,128,128,128,0,255,0,150);     /* R2 pressed: sent at once */
    assert(next_event(e,sizeof(e),500) && field(e,"b")==0x200 && field(e,"r2")==255 && field(e,"r2a")==150);
    runtime_unds_pad(0x200,128,128,128,128,0,200,0,150);     /* analog only, <8 ms later: held */
    runtime_unds_pad(0x200,128,128,128,128,0,180,0,170);
    assert(next_event(e,sizeof(e),400) && field(e,"r2")==180 && field(e,"r2a")==170); /* sent by the heartbeat */
    runtime_unds_pad(0,128,128,128,128,0,0,0,0);             /* release: at once */
    assert(next_event(e,sizeof(e),500) && field(e,"b")==0 && field(e,"r2")==0);

    test_audio();
    runtime_unds_guest_callbacks(&callbacks);
    assert(!callbacks && !atomic_load(&state_enabled)); /* U2: the U1 game-state hooks are not handed out */
    set_env("BB_EXE","eboot.bin");
    test_state();

    /* heartbeat: hello every second with counters, refreshes carry re:1 (game state: ups/wps) */
    int hello=0, refresh=0, state_refresh=0;
    for (int i=0;i<40 && (!hello || !refresh || !state_refresh);++i) {
        if (!receive(e,sizeof(e),100)) continue;
        assert((uint32_t)field(e,"seq")==expect_seq); ++expect_seq;
        if (has(e,"\"e\":\"hello\"")) {
            hello=1;
            assert(field(e,"boot")==0 && field(e,"sent")>0 && has(e,"\"ports\":\"0:0:8:1024,17:4:1:256\""));
            assert(field(e,"gsp")>=8 && field(e,"gss")==1 && field(e,"gsd")==2);
        }
        if (has(e,"\"re\":1")) refresh=1;
        if (has(e,"\"e\":\"gs\"") && has(e,"\"re\":1")) { state_refresh=1; assert(field(e,"hp")==0 && field(e,"ups")>=0); }
    }
    assert(hello && refresh && state_refresh);

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
