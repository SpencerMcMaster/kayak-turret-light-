// Host test for button handling: lift chord, brightness click on release,
// pan lockout, debounce. Compiles the REAL button/UpdateInputs() source,
// extracted from kayak_working.ino by tools/test/run_input_test.sh.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#define HIGH 1
#define LOW 0
static int pinState[8] = {1,1,1,1,1,1,1,1};
static int digitalRead(uint8_t p){ return pinState[p]; }
enum { PIN_PB_BRIGHTNESS=0, PIN_PB_MODE=1, PIN_PB_TURN_LEFT=2, PIN_PB_TURN_RIGHT=3 };
static const uint16_t BUTTON_DEBOUNCE_MS = 25;
static bool g_turnLeftHeld=false, g_turnRightHeld=false, g_liftUpHeld=false, g_liftDownHeld=false;
static bool g_brightnessPressedEdge=false, g_modePressedEdge=false;
static bool g_liftChordActive=false, g_brightnessChordUsed=false, g_panLockout=false;
#include "ex.inc"
static uint32_t t=0; static int clicks=0;
struct Out { bool pl, pr, up, dn; };
static Out step(int ms){ Out o{}; for(int i=0;i<ms;i++){ UpdateInputs(t++); if(g_brightnessPressedEdge) clicks++; } o={g_turnLeftHeld,g_turnRightHeld,g_liftUpHeld,g_liftDownHeld}; return o; }
static int fails=0;
#define EXPECT(cond,msg) do{ bool c_=(cond); printf("%-62s %s\n",msg,c_?"PASS":"FAIL"); if(!c_) fails++; }while(0)
int main(){
  Out o;
  // 1. brightness click alone -> exactly one click on release, not on press
  pinState[0]=0; o=step(60); EXPECT(clicks==0, "BRIGHTNESS press: no brightness change yet");
  pinState[0]=1; o=step(60); EXPECT(clicks==1, "BRIGHTNESS release: one brightness step");
  // 2. LEFT alone -> pan left
  pinState[2]=0; o=step(60); EXPECT(o.pl && !o.pr && !o.up && !o.dn, "LEFT held: pan left only");
  pinState[2]=1; o=step(60); EXPECT(!o.pl, "LEFT released: pan stops");
  // 3. chord BRIGHTNESS then LEFT -> lift down, no pan, no click
  int c0=clicks; pinState[0]=0; step(60); pinState[2]=0; o=step(60);
  EXPECT(o.dn && !o.up && !o.pl && !o.pr, "BRIGHTNESS+LEFT: lift DOWN, pan suppressed");
  pinState[2]=1; step(60); pinState[0]=1; o=step(60);
  EXPECT(clicks==c0 && !o.dn, "chord released: no brightness step, lift stops");
  // 4. chord BRIGHTNESS + RIGHT -> lift up
  pinState[0]=0; step(60); pinState[3]=0; o=step(60);
  EXPECT(o.up && !o.dn && !o.pl && !o.pr, "BRIGHTNESS+RIGHT: lift UP, pan suppressed");
  // 5. release BRIGHTNESS first while RIGHT still held -> no pan (lockout)
  pinState[0]=1; o=step(60);
  EXPECT(!o.up && !o.pr && !o.pl, "BRIGHTNESS released first: lift stops, NO pan (lockout)");
  pinState[3]=1; step(60); pinState[3]=0; o=step(60);
  EXPECT(o.pr, "RIGHT released and pressed again: pan right works");
  pinState[3]=1; step(60);
  // 6. pan LEFT in progress, then BRIGHTNESS pressed -> converts to lift down
  c0=clicks; pinState[2]=0; o=step(60); EXPECT(o.pl, "LEFT held: pan left");
  pinState[0]=0; o=step(60); EXPECT(o.dn && !o.pl, "then BRIGHTNESS: pan stops, lift DOWN");
  pinState[0]=1; pinState[2]=1; o=step(60); EXPECT(clicks==c0 && !o.dn && !o.pl, "all released: nothing moves, no click");
  // 7. BRIGHTNESS + LEFT + RIGHT -> nothing moves
  pinState[0]=0; pinState[2]=0; pinState[3]=0; o=step(60);
  EXPECT(!o.up && !o.dn && !o.pl && !o.pr, "BRIGHTNESS+LEFT+RIGHT: nothing moves");
  pinState[0]=pinState[2]=pinState[3]=1; step(60);
  // 8. LEFT+RIGHT (hazard) -> both flags (engine treats as no rotation)
  pinState[2]=0; pinState[3]=0; o=step(60); EXPECT(o.pl && o.pr && !o.up && !o.dn, "LEFT+RIGHT: hazard flags, no lift");
  pinState[2]=pinState[3]=1; step(60);
  // 9. bounce: 10 ms glitch on LEFT ignored
  pinState[2]=0; o=step(10); pinState[2]=1; o=step(60); EXPECT(!o.pl, "10 ms glitch on LEFT ignored (debounce)");
  printf(fails? "%d FAILED\n" : "ALL PASS\n", fails); return fails?1:0; }
