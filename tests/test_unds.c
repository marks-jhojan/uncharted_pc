/* UNDS event bus: endpoint parsing, change detection, pad rate limit, audio levels, protocol
 * fields, seq continuity, "bye", UNDS_LED and UNDS_TRIGGER, and the fx control channel (reports
 * handed to a stand-in of runtime_pad.c). Sends to a UDP socket of its own on 127.0.0.1.
 * Linux:            cc -std=c11 -Wall -Wextra -Werror -pthread -Isrc tests/test_unds.c -lm -o out/unds-test
 * MSYS2 CLANG64:    clang -std=c11 -Wall -Wextra -Werror -pthread -Isrc tests/test_unds.c src/compat_win.c -lws2_32 -lbcrypt -o out/unds-test.exe */
#include "../src/runtime_unds.c"
#undef NDEBUG /* the checks below have side effects */
#include <assert.h>
#include <ctype.h>
#include <signal.h>

static unds_socket_t rx=UNDS_INVALID_SOCKET;
/* seq: every datagram exactly once. Three threads send (the caller, the heartbeat, the fx
 * receiver) and each takes its seq before its sendto, so two sends at the same moment can arrive
 * swapped: the order is not checked, losses and repeats are (check_no_gaps at the end). */
static uint8_t seen_seq[8192];
static uint32_t max_seq, reordered;
static size_t longest_datagram;
static void note_seq(const char *json);

/* runtime_pad.c stand-ins: the DS5EffectsState_t reports the control channel hands to SDL. */
static pthread_mutex_t pad_stub=PTHREAD_MUTEX_INITIALIZER;
static uint8_t pad_report[UNDS_DS5_EFFECTS_SIZE], game_led_rgb[3];
static int pad_reports, pad_result=1, game_led_calls;
static unsigned pad_generation;
int runtime_pad_unds_effect(const uint8_t *effects, int size) {
    assert(size==UNDS_DS5_EFFECTS_SIZE);
    pthread_mutex_lock(&pad_stub);
    memcpy(pad_report,effects,sizeof(pad_report)); ++pad_reports;
    const int result=pad_result;
    pthread_mutex_unlock(&pad_stub);
    return result;
}
unsigned runtime_pad_unds_generation(void) {
    pthread_mutex_lock(&pad_stub);
    const unsigned generation=pad_generation;
    pthread_mutex_unlock(&pad_stub);
    return generation;
}
void runtime_pad_unds_game_led(uint8_t r, uint8_t g, uint8_t b) {
    pthread_mutex_lock(&pad_stub);
    game_led_rgb[0]=r; game_led_rgb[1]=g; game_led_rgb[2]=b; ++game_led_calls;
    pthread_mutex_unlock(&pad_stub);
}
static int reports(void) { pthread_mutex_lock(&pad_stub); const int n=pad_reports; pthread_mutex_unlock(&pad_stub); return n; }
static void last_report(uint8_t out[UNDS_DS5_EFFECTS_SIZE]) {
    pthread_mutex_lock(&pad_stub); memcpy(out,pad_report,UNDS_DS5_EFFECTS_SIZE); pthread_mutex_unlock(&pad_stub);
}

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
    static char buffer[4096]; /* Windows refuses a datagram longer than the buffer (WSAEMSGSIZE) */
    int n=(int)recv(rx,buffer,(int)sizeof(buffer),0);
    assert(n>0);
    if ((size_t)n>longest_datagram) longest_datagram=(size_t)n;
    assert((size_t)n<size && "datagram longer than the test's buffer");
    memcpy(out,buffer,(size_t)n);
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
static void note_seq(const char *json) {
    const long long seq=field(json,"seq");
    assert(seq>=0 && seq<(long long)sizeof(seen_seq));
    if (seen_seq[seq]) { printf("unds-test: seq %lld twice\n",seq); assert(0 && "repeated seq"); }
    seen_seq[seq]=1;
    if ((uint32_t)seq<max_seq) ++reordered;
    else max_seq=(uint32_t)seq;
}
static void check_no_gaps(void) {
    for (uint32_t s=0;s<=max_seq;++s)
        if (!seen_seq[s]) { printf("unds-test: datagram seq %u never arrived (last seq %u)\n",s,max_seq); assert(0 && "lost datagram"); }
}
/* Next datagram that is not a heartbeat (hello, refreshes); checks v and seq continuity. */
/* The last datagram next_event saw: printed when an assertion fails (SIGABRT). */
static char last_event[1024]="(none)";
static void print_last_event(int signal_number) {
    (void)signal_number;
    fprintf(stderr,"unds-test: last datagram seen: %s\n",last_event);
    fflush(stderr);
}
static int next_event(char *out, size_t size, int ms) {
    for (;;) {
        if (!receive(out,size,ms)) { snprintf(last_event,sizeof(last_event),"(none within %d ms)",ms); return 0; }
        snprintf(last_event,sizeof(last_event),"%s",out);
        assert(field(out,"v")==1);
        note_seq(out);
        if (has(out,"\"e\":\"hello\"") || has(out,"\"re\":1")) continue;
        return 1;
    }
}
static void no_event(int ms) {
    char text[512];
    if (next_event(text,sizeof(text),ms)) { printf("unds-test: unexpected datagram: %s\n",text); fflush(stdout); assert(0 && "no event expected"); }
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
/* the loader's pattern syntax: "xx" or "??" separated by single spaces; the hook and the moved
   bytes lie inside the pattern and are not wildcards (they are copied to the thunk) */
static void check_table(const RuntimeGuestCallback *c, size_t count, const char *prefix) {
    for (size_t i=0;i<count;++i) {
        const char *t=c[i].pattern;
        size_t n=0, len=strlen(t);
        assert(len%3==2 && !strncmp(c[i].name,prefix,strlen(prefix)) && !c[i].patch && c[i].moved>=5);
        for (size_t k=0;k<len;k+=3,++n) {
            assert((isxdigit((unsigned char)t[k]) && isxdigit((unsigned char)t[k+1])) || (t[k]=='?' && t[k+1]=='?'));
            assert(k+2==len || t[k+2]==' ');
            if (n>=c[i].skip && n<c[i].skip+c[i].moved) assert(t[k]!='?');
        }
        assert(c[i].skip+c[i].moved<=n);
    }
}
static void test_state(void) {
    char e[512];
    size_t count=0;
    const RuntimeGuestCallback *c=runtime_unds_guest_callbacks(&count);
    assert(count==4 && atomic_load(&state_enabled) && !atomic_load(&control_enabled));
    assert(c[0].callback==state_player && c[1].callback==state_controller && c[2].callback==state_shot && c[3].callback==state_damage);
    assert(c[0].skip==0 && c[0].moved==6 && c[1].skip==0 && c[1].moved==6 && c[2].skip==21 && c[2].moved==5 &&
           c[3].skip==26 && c[3].moved==6);
    check_table(c,count,"UNDS U1 ");
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

/* U2: the frame hook (health fraction in the thunk's saved xmm0, weapon through the player's
 * controller), the controller helper (upc), shots from the player's controller only, damage. */
static void test_state_u2(void) {
    char e[512];
    size_t count=0;
    set_env("BB_EXE","big2-ps4_Shipping.elf");
    const RuntimeGuestCallback *c=runtime_unds_guest_callbacks(&count);
    assert(count==4 && atomic_load(&state_enabled) && atomic_load(&control_enabled));
    assert(c[0].callback==state_u2_frame && c[1].callback==state_u2_control && c[2].callback==state_u2_shot &&
           c[3].callback==state_u2_damage);
    assert(c[0].skip==42 && c[0].moved==7 && c[1].skip==0 && c[1].moved==6 && c[2].skip==45 && c[2].moved==5 &&
           c[3].skip==35 && c[3].moved==8);
    check_table(c,count,"UNDS U2 ");
    static _Alignas(16) unsigned char player[0x3000], controller[0x300], pistol[0x700], other[0x300], health[0x40],
                                      damage[0x40], source[0x200], helper[0x20], frame[0x200], frame2[0x200];
    static uint64_t pistol_slot, source_slot;
    float f; uint32_t u; int16_t ammo[3]; uint64_t p; int32_t i;
    /* the thunk's frame: RuntimeGuestRegs at +0x100 (16-aligned), xmm0 saved at +0x100-0xb0+0x30 */
    RuntimeGuestRegs *regs=(RuntimeGuestRegs *)(void *)(frame+0x100);
    memset(regs,0,sizeof(*regs));
    regs->r15=(uint64_t)(uintptr_t)player;
    assert((const unsigned char *)runtime_guest_xmm(regs,0)==frame+0x80 && (const unsigned char *)runtime_guest_xmm(regs,2)==frame+0xa0);
    /* player at (1, 2, 3), identity rotation; its controller (+0x128 = the player) holds a pistol in slot 3 */
    /* player's own locator at (1, 2, 3), identity rotation, on an identity parent */
    f=1.0f; memcpy(player+0x10c,&f,4); memcpy(player+0x12c,&f,4);
    f=1.0f; memcpy(player+0x110,&f,4); f=2.0f; memcpy(player+0x114,&f,4); f=3.0f; memcpy(player+0x118,&f,4);
    p=(uint64_t)(uintptr_t)controller; memcpy(player+0x2768,&p,8);
    p=(uint64_t)(uintptr_t)player; memcpy(controller+0x128,&p,8);
    u=0xff5403e6u; memcpy(pistol+0x550,&u,4); u=91; memcpy(pistol+0x44,&u,4);
    ammo[0]=15; ammo[1]=45; ammo[2]=15; memcpy(pistol+0x620,ammo,6);
    pistol_slot=(uint64_t)(uintptr_t)pistol; p=(uint64_t)(uintptr_t)&pistol_slot;
    memcpy(controller+0x130+16*3,&p,8); memcpy(controller+0x130+16*3+8,&u,4);
    controller[0x208]=3;
    f=0.8f; memcpy(frame+0x80,&f,4);
    state_u2_frame(0,0,0x400000,0,regs);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"gs\"") && field(e,"hp")==800 && field(e,"ws")==3);
    assert(has(e,"\"w\":\"0xff5403e6\"") && field(e,"clip")==15 && field(e,"res")==45 && field(e,"cap")==15 && field(e,"gr")==-1);
    state_u2_frame(0,0,0x400000,0,regs);
    no_event(30);
    /* the helper of the player's controllers counts only with the player at +8 */
    const uint64_t controls=atomic_load(&gs_control_calls);
    p=(uint64_t)(uintptr_t)player; memcpy(helper+8,&p,8);
    state_u2_control((uint64_t)(uintptr_t)helper,0,0,0,NULL);
    p=1234; memcpy(helper+8,&p,8);
    state_u2_control((uint64_t)(uintptr_t)helper,0,0,0,NULL);
    assert(atomic_load(&gs_control_calls)==controls+1 && atomic_load(&ups_from_control));
    /* a shot of the player's controller (the round before ConsumeRound takes it); another owner's is ignored */
    RuntimeGuestRegs *shot=(RuntimeGuestRegs *)(void *)(frame2+0x100);
    memset(shot,0,sizeof(*shot));
    shot->r12=(uint64_t)(uintptr_t)controller;
    state_u2_shot(0,0,0,(uint64_t)(uintptr_t)pistol,shot);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"shot\"") && has(e,"\"w\":\"0xff5403e6\"") && field(e,"clip")==15 &&
           field(e,"res")==45 && field(e,"cap")==15);
    memcpy(other,controller,sizeof(other)); p=1234; memcpy(other+0x128,&p,8);
    shot->r12=(uint64_t)(uintptr_t)other;
    state_u2_shot(0,0,0,(uint64_t)(uintptr_t)pistol,shot);
    no_event(30);
    ammo[0]=14; memcpy(pistol+0x620,ammo,6);
    state_u2_frame(0,0,0x400000,0,regs);
    assert(next_event(e,sizeof(e),500) && field(e,"clip")==14);
    /* stale handle (the id changed): no weapon; a controller of another object: no weapon either */
    u=92; memcpy(pistol+0x44,&u,4);
    state_u2_frame(0,0,0x400000,0,regs);
    assert(next_event(e,sizeof(e),500) && field(e,"ws")==-1 && has(e,"\"w\":\"0x0\"") && field(e,"clip")==-1);
    u=91; memcpy(pistol+0x44,&u,4);
    state_u2_frame(0,0,0x400000,0,regs);
    assert(next_event(e,sizeof(e),500) && field(e,"ws")==3);
    p=(uint64_t)(uintptr_t)other; memcpy(player+0x2768,&p,8);
    state_u2_frame(0,0,0x400000,0,regs);
    assert(next_event(e,sizeof(e),500) && field(e,"ws")==-1);
    p=(uint64_t)(uintptr_t)controller; memcpy(player+0x2768,&p,8);
    state_u2_frame(0,0,0x400000,0,regs);
    assert(next_event(e,sizeof(e),500) && field(e,"ws")==3);
    /* damage: 25 of 100 health points from a source at (4, 2, 3) (handle at +8, id at +0x10) */
    RuntimeGuestRegs *hit=(RuntimeGuestRegs *)(void *)(frame2+0x100);
    memset(hit,0,sizeof(*hit));
    hit->r15=(uint64_t)(uintptr_t)player; hit->r13=(uint64_t)(uintptr_t)damage; hit->r12=(uint64_t)(uintptr_t)health;
    i=100; memcpy(health+0x30,&i,4); i=25; memcpy(damage+0x24,&i,4);
    u=7; memcpy(source+0x44,&u,4); memcpy(damage+0x10,&u,4);
    source_slot=(uint64_t)(uintptr_t)source; p=(uint64_t)(uintptr_t)&source_slot; memcpy(damage+8,&p,8);
    /* the source on a platform at (10, 0, 0) turned 90 degrees about y: own (0, 2, 6) -> world (16, 2, 0)... */
    f=10.0f; memcpy(source+0xf0,&f,4); f=0.70710678f; memcpy(source+0x104,&f,4); memcpy(source+0x10c,&f,4);
    f=0.0f; memcpy(source+0x110,&f,4); f=2.0f; memcpy(source+0x114,&f,4); f=6.0f; memcpy(source+0x118,&f,4);
    f=1.0f; memcpy(source+0x12c,&f,4);
    state_u2_damage(0,0,0,0,hit);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"dmg\"") && field(e,"amt")==250 && field(e,"hpt")==25 && field(e,"hp")==800);
    assert(field(e,"src")==1 && field(e,"sx")==16000 && field(e,"sy")==2000 && llabs(field(e,"sz"))<=1 && field(e,"loc")==1 &&
           field(e,"px")==1000 && field(e,"py")==2000 && field(e,"pz")==3000 && field(e,"qw")==10000 && field(e,"qx")==0);
    f=0.55f; memcpy(frame+0x80,&f,4);
    state_u2_frame(0,0,0x400000,0,regs);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"gs\"") && field(e,"hp")==550);
    /* no source (a fall): src 0; damage to another object: nothing */
    memset(damage+8,0,8);
    state_u2_damage(0,0,0,0,hit);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"dmg\"") && field(e,"src")==0 && field(e,"sx")==0);
    hit->r15=(uint64_t)(uintptr_t)other;
    state_u2_damage(0,0,0,0,hit);
    no_event(30);
    /* death: the fraction reaches 0 */
    f=0.0f; memcpy(frame+0x80,&f,4);
    state_u2_frame(0,0,0x400000,0,regs);
    assert(next_event(e,sizeof(e),500) && field(e,"hp")==0);
    /* refreshes carry upc for U2 */
    int upc=0;
    for (int k=0;k<40 && !upc;++k) {
        if (!receive(e,sizeof(e),100)) continue;
        note_seq(e);
        if (has(e,"\"e\":\"gs\"") && has(e,"\"re\":1")) { upc=1; assert(field(e,"upc")>=0 && field(e,"upf")>=0 && field(e,"ups")>=0 && field(e,"hp")==0); }
    }
    assert(upc);
}

/* The control channel: datagrams from `from` to the port's fx socket. */
static void fx_send(unds_socket_t from, int port, int flags, uint32_t command, const uint8_t *right, const uint8_t *left,
                    const uint8_t *rgb, size_t size, uint8_t version) {
    uint8_t d[64]; memset(d,0,sizeof(d));
    memcpy(d,"UNDC",4); d[4]=version; d[5]=(uint8_t)flags;
    d[8]=(uint8_t)command; d[9]=(uint8_t)(command>>8); d[10]=(uint8_t)(command>>16); d[11]=(uint8_t)(command>>24);
    if (right) memcpy(d+12,right,11);
    if (left) memcpy(d+23,left,11);
    if (rgb) memcpy(d+34,rgb,3);
    struct sockaddr_in to; memset(&to,0,sizeof(to));
    to.sin_family=AF_INET; to.sin_addr.s_addr=htonl(0x7f000001u); to.sin_port=htons((uint16_t)port);
    assert(sendto(from,(const char *)d,(int)size,0,(const struct sockaddr *)&to,(int)sizeof(to))==(int)size);
}
static void fx_cmd(int port, int flags, uint32_t command, const uint8_t *right, const uint8_t *left, const uint8_t *rgb) {
    fx_send(rx,port,flags,command,right,left,rgb,UNDS_FX_SIZE,1);
}
/* The fx event answering command `command`. */
static void fx_ack(char *e, size_t size, uint32_t command, int flags, int ok, const char *why) {
    assert(next_event(e,size,500) && has(e,"\"e\":\"fx\""));
    char text[32]; snprintf(text,sizeof(text),"\"why\":\"%s\"",why);
    assert((uint32_t)field(e,"c")==command && field(e,"f")==flags && field(e,"ok")==ok && has(e,text));
    assert(field(e,"q")>=0 && field(e,"q")<100000);
}
static void test_fx(int port) {
    char e[512];
    uint8_t r[UNDS_DS5_EFFECTS_SIZE];
    /* WEAPON 3 4 8 and FEEDBACK 4 1 as unds_hid.py encodes them */
    const uint8_t weapon[11]={0x25,0x18,0x00,0x07}, feedback[11]={0x21,0xf0,0x03}, rgb[3]={128,128,0};
    assert(atomic_load(&fx_on) && port>0 && !runtime_unds_fx_led_owned());
    /* R2 alone: enable bit 0x04 of the first byte, the effect at offset 10, nothing else */
    fx_cmd(port,UNDS_FX_RIGHT,1,weapon,NULL,NULL);
    fx_ack(e,sizeof(e),1,UNDS_FX_RIGHT,1,"cmd");
    last_report(r);
    assert(reports()==1 && r[0]==0x04 && r[1]==0 && !memcmp(r+10,weapon,11) && r[21]==0 && r[44]==0);
    assert(atomic_load(&fx_held)==UNDS_FX_RIGHT);
    /* L2 and the light bar in one report; the companion holds the light bar from now on */
    fx_cmd(port,UNDS_FX_LEFT|UNDS_FX_LED,2,NULL,feedback,rgb);
    fx_ack(e,sizeof(e),2,UNDS_FX_LEFT|UNDS_FX_LED,1,"cmd");
    last_report(r);
    assert(reports()==2 && r[0]==0x08 && r[1]==0x04 && !memcmp(r+21,feedback,11) && r[10]==0 && !memcmp(r+44,rgb,3));
    assert(runtime_unds_fx_led_owned() && atomic_load(&fx_held)==(UNDS_FX_RIGHT|UNDS_FX_LEFT|UNDS_FX_LED));
    /* never the rumble-emulation bits (they turn the audio haptics off) */
    assert(!(r[0]&0x03));
    /* the game's colour is still published (the companion builds its light bar from it) */
    runtime_unds_led(64,0,0);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"led\"") && field(e,"r")==64);
    /* rejected: an unknown trigger mode, a short datagram, another version, another host */
    const uint8_t bogus[11]={0x99};
    fx_cmd(port,UNDS_FX_RIGHT,3,bogus,NULL,NULL);
    fx_send(rx,port,UNDS_FX_RIGHT,4,weapon,NULL,NULL,UNDS_FX_SIZE-1,1);
    fx_send(rx,port,UNDS_FX_RIGHT,5,weapon,NULL,NULL,UNDS_FX_SIZE,2);
    unds_socket_t stranger=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    struct sockaddr_in a; memset(&a,0,sizeof(a));
    a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(0x7f000002u);
    int strangers=0;
    if (stranger!=UNDS_INVALID_SOCKET && !bind(stranger,(struct sockaddr *)&a,sizeof(a))) {
        /* only where the source address survives (Wine reports 127.0.0.2 as 127.0.0.1) */
        struct sockaddr_in me, from; socklen_t length=sizeof(me);
        assert(!getsockname(rx,(struct sockaddr *)&me,&length));
        assert(sendto(stranger,"probe",5,0,(const struct sockaddr *)&me,(int)sizeof(me))==5);
        fd_set set; FD_ZERO(&set); FD_SET(rx,&set);
        struct timeval timeout={0,500000};
        assert(select((int)rx+1,&set,NULL,NULL,&timeout)==1);
        char probe[16]; length=sizeof(from);
        assert(recvfrom(rx,probe,(int)sizeof(probe),0,(struct sockaddr *)&from,&length)==5);
        if (from.sin_addr.s_addr==a.sin_addr.s_addr) {
            fx_send(stranger,port,UNDS_FX_RIGHT,6,weapon,NULL,NULL,UNDS_FX_SIZE,1);
            strangers=1;
        } else puts("unds-test: this system reports 127.0.0.2 as another address, the source check is not exercised");
    } else puts("unds-test: no 127.0.0.2 here, the source check is not exercised");
    no_event(150);
    assert(reports()==2 && atomic_load(&fx_rejected)==(uint64_t)(3+strangers));
    /* a keep-alive (no part): answered, nothing written */
    fx_cmd(port,0,7,NULL,NULL,NULL);
    fx_ack(e,sizeof(e),7,0,1,"cmd");
    assert(reports()==2);
    /* the controller reconnected: everything held, in one report */
    pthread_mutex_lock(&pad_stub); ++pad_generation; pthread_mutex_unlock(&pad_stub);
    fx_ack(e,sizeof(e),0,UNDS_FX_RIGHT|UNDS_FX_LEFT|UNDS_FX_LED,1,"reconnect");
    last_report(r);
    assert(reports()==3 && r[0]==0x0c && r[1]==0x04 && !memcmp(r+10,weapon,11) && !memcmp(r+21,feedback,11) && !memcmp(r+44,rgb,3));
    /* not a DualSense: the result is answered and counted */
    pthread_mutex_lock(&pad_stub); pad_result=-1; pthread_mutex_unlock(&pad_stub);
    fx_cmd(port,UNDS_FX_RIGHT,8,weapon,NULL,NULL);
    fx_ack(e,sizeof(e),8,UNDS_FX_RIGHT,-1,"cmd");
    assert(atomic_load(&fx_failures)==1);
    pthread_mutex_lock(&pad_stub); pad_result=1; pthread_mutex_unlock(&pad_stub);
    /* release: both triggers off and the game's colour back (SDL may show it: UNDS_LED not 0) */
    atomic_store(&led_on,1);
    fx_cmd(port,UNDS_FX_RELEASE|UNDS_FX_RIGHT,9,weapon,NULL,NULL);   /* the other bits are ignored */
    fx_ack(e,sizeof(e),9,UNDS_FX_RELEASE|UNDS_FX_RIGHT,1,"release");
    last_report(r);
    assert(r[0]==0x0c && r[10]==0x05 && r[21]==0x05 && r[1]==0x04 && r[44]==64 && r[45]==0 && r[46]==0);
    assert(game_led_calls==1 && game_led_rgb[0]==64 && !runtime_unds_fx_led_owned() && !atomic_load(&fx_held));
    /* watchdog: held state and no datagram for fx_timeout_us -> triggers off (the light bar was not held) */
    fx_timeout_us=300000u;
    fx_cmd(port,UNDS_FX_RIGHT,10,weapon,NULL,NULL);
    fx_ack(e,sizeof(e),10,UNDS_FX_RIGHT,1,"cmd");
    const int before=reports();
    assert(next_event(e,sizeof(e),900) && has(e,"\"e\":\"fx\"") && has(e,"\"why\":\"timeout\"") && field(e,"f")==UNDS_FX_RELEASE);
    last_report(r);
    assert(reports()==before+1 && r[0]==0x0c && r[10]==0x05 && r[21]==0x05 && r[1]==0 && game_led_calls==1);
    assert(atomic_load(&fx_timeouts)==1 && !atomic_load(&fx_held));
    no_event(400); /* nothing held: the watchdog stays quiet */
    fx_timeout_us=5000000u;
    /* held again for the bye test in main() */
    fx_cmd(port,UNDS_FX_RIGHT,11,weapon,NULL,NULL);
    fx_ack(e,sizeof(e),11,UNDS_FX_RIGHT,1,"cmd");
#ifdef _WIN32
    if (stranger!=UNDS_INVALID_SOCKET) closesocket(stranger);
#else
    if (stranger!=UNDS_INVALID_SOCKET) close(stranger);
#endif
}

int main(void) {
    signal(SIGABRT,print_last_event);
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
    assert(field(e,"fx")==1 && field(e,"fxp")>0 && field(e,"fxc")==0 && field(e,"fxo")==0);
    const int fx_udp=(int)field(e,"fxp");
    note_seq(e);

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
    const uint64_t pressed_us=now_us();
    runtime_unds_pad(0x200,128,128,128,128,0,255,0,150);     /* R2 pressed: sent at once */
    runtime_unds_pad(0x200,128,128,128,128,0,200,0,150);     /* analog only, <8 ms later: held */
    runtime_unds_pad(0x200,128,128,128,128,0,180,0,170);
    const uint64_t held_us=now_us()-pressed_us;
    assert(next_event(e,sizeof(e),500) && field(e,"b")==0x200 && field(e,"r2")==255 && field(e,"r2a")==150);
    /* The newest analog state comes with the heartbeat. Only if this thread was held up for 8 ms
     * or more between the calls (a busy Windows PC) may the middle one have gone out at once. */
    assert(next_event(e,sizeof(e),400) && has(e,"\"e\":\"pad\""));
    if (field(e,"r2")==200) {
        printf("unds-test: the pad calls took %" PRIu64 " us: the middle state went out at once\n",held_us);
        assert(held_us>=PAD_MIN_INTERVAL_US && "an analog change inside 8 ms was not held back");
        assert(next_event(e,sizeof(e),400) && has(e,"\"e\":\"pad\""));
    }
    assert(field(e,"r2")==180 && field(e,"r2a")==170); /* sent by the heartbeat */
    runtime_unds_pad(0,128,128,128,128,0,0,0,0);             /* release: at once */
    assert(next_event(e,sizeof(e),500) && field(e,"b")==0 && field(e,"r2")==0);

    test_audio();
    set_env("BB_EXE","big3-ps4_Shipping.elf");
    runtime_unds_guest_callbacks(&callbacks);
    assert(!callbacks && !atomic_load(&state_enabled)); /* U3: no game-state hooks yet */
    set_env("BB_EXE","eboot.bin");
    test_state();

    /* heartbeat: hello every second with counters, refreshes carry re:1 (game state: ups/wps) */
    int hello=0, refresh=0, state_refresh=0;
    for (int i=0;i<40 && (!hello || !refresh || !state_refresh);++i) {
        if (!receive(e,sizeof(e),100)) continue;
        note_seq(e);
        if (has(e,"\"e\":\"hello\"")) {
            hello=1;
            assert(field(e,"boot")==0 && field(e,"sent")>0 && has(e,"\"ports\":\"0:0:8:1024,17:4:1:256\""));
            assert(field(e,"gsp")>=8 && field(e,"gss")==1 && field(e,"gsd")==2);
        }
        if (has(e,"\"re\":1")) refresh=1;
        if (has(e,"\"e\":\"gs\"") && has(e,"\"re\":1")) { state_refresh=1; assert(field(e,"hp")==0 && field(e,"ups")>=0); }
    }
    assert(hello && refresh && state_refresh);
    test_state_u2();

    test_fx(fx_udp);
    /* bye with R2 held: the triggers go off before the bye, and nothing is taken after it */
    const int before_bye=reports();
    runtime_unds_bye("loadexec");
    uint8_t r[UNDS_DS5_EFFECTS_SIZE];
    last_report(r);
    assert(reports()==before_bye+1 && r[0]==0x0c && r[10]==0x05 && r[21]==0x05);
    assert(next_event(e,sizeof(e),500) && has(e,"\"e\":\"bye\"") && has(e,"\"reason\":\"loadexec\""));
    const uint8_t weapon[11]={0x25,0x18,0x00,0x07};
    fx_cmd(fx_udp,UNDS_FX_RIGHT,12,weapon,NULL,NULL);
    runtime_unds_bye("again");
    runtime_unds_rumble(1,1);
    char extra[512];
    assert(!receive(extra,sizeof(extra),600)); /* nothing after bye, not even the heartbeat or an fx answer */
    assert(reports()==before_bye+1);
    runtime_unds_report();
    assert(!atomic_load(&errors) && !atomic_load(&dropped));
    /* No send may wait on another thread (the fx receiver sits in select() on the same socket). */
    const uint64_t longest=atomic_load(&send_max_ns_total);
    printf("unds-test: longest sendto %" PRIu64 " us\n",longest/1000u);
    assert(longest<250000000u && "a sendto waited 250 ms or more");
    check_no_gaps();
    printf("unds-test: %u datagrams, %u arrived ahead of an earlier one, longest %zu bytes\n",max_seq+1,reordered,longest_datagram);
    puts("unds-test: ok");
    return 0;
}
