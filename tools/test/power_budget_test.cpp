// Host test for the LED power budget (HARD SAFETY CONSTRAINT).
// Compiles the REAL EnforcePowerBudget()/ShowAllStrips() source, extracted
// from kayak_working.ino by tools/test/run_power_budget_test.sh, against a
// mock strip that records exactly what would be transmitted.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
typedef uint16_t neoPixelType;
#define NEO_GRBW ((3 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_KHZ800 0x0000
struct Adafruit_NeoPixel {
  std::vector<uint8_t> buf; uint16_t n; double worst = 0; long shows = 0;
  std::vector<uint8_t> lastSent;
  Adafruit_NeoPixel(uint16_t c): buf(c*4,0), n(c) {}
  uint8_t* getPixels() { return buf.data(); }
  uint16_t numPixels() const { return n; }
  void show();
};
// Replay log: after every single strip send, the budget term of what the
// strips are physically showing (part old frame, part new frame).
static uint32_t displayedPower[4] = {0,0,0,0};
static double   worstIntermediate = 0.0;
static const neoPixelType LED_PIXEL_TYPE = (neoPixelType)(NEO_GRBW + NEO_KHZ800);
static const uint16_t LED_COUNT_BOTTOM_LEFT=256, LED_COUNT_BOTTOM_RIGHT=256, LED_COUNT_COCKPIT_FRONT=44, LED_COUNT_COCKPIT_BACK=44;
static const uint16_t LED_TOTAL_PIXELS=600, LED_MAX_STRIP_PIXELS=256;
static uint8_t g_brightnessValue = 96;
Adafruit_NeoPixel a(256), b(256), c(44), d(44);
Adafruit_NeoPixel* LED_STRIPS[] = {&a,&b,&c,&d};
static const uint8_t LED_STRIP_COUNT = 4;
#include "extracted.inc"
void Adafruit_NeoPixel::show() {
  lastSent = buf; shows++;
  // lit count of the FULL-SCALE frame being sent (held in g_showScratch now)
  uint32_t lit = 0;
  for (int p = 0; p < n; p++) { bool any=false; for (int k=0;k<4;k++) any |= g_showScratch[p*4+k]!=0; lit += any; }
  int idx = 0; for (int i=0;i<4;i++) if (LED_STRIPS[i]==this) idx=i;
  displayedPower[idx] = lit * g_budget.applied;
  double sum = 0; for (int i=0;i<4;i++) sum += displayedPower[i];
  double prod = sum / (600.0 * 255.0);
  if (prod > worstIntermediate) worstIntermediate = prod;
}
// Independent check of what was transmitted: lit pixels x applied/255
static void check(const char* name, uint8_t req) {
  g_brightnessValue = req;
  std::vector<std::vector<uint8_t>> before; for (auto s: LED_STRIPS) before.push_back(s->buf);
  EnforcePowerBudget(); ShowAllStrips();
  int lit=0; bool restored=true, scaledOK=true;
  for (int i=0;i<4;i++){ auto s=LED_STRIPS[i];
    if (s->buf!=before[i]) restored=false;
    for (int p=0;p<s->n;p++){ bool any=false; for(int k=0;k<4;k++){ any|=before[i][p*4+k]!=0;
        unsigned exp=(before[i][p*4+k]*g_budget.applied)/255; if (s->lastSent[p*4+k]!=exp) scaledOK=false; }
      lit+=any; } }
  double prod=(lit/600.0)*(g_budget.applied/255.0);
  bool pass = prod<=0.20+1e-9 && restored && scaledOK && g_budget.litPixels==lit && g_budget.applied<=req
              && (!g_budget.limited || ((lit/600.0)*((g_budget.applied+1)/255.0) > 0.20)); // applied is the max passing value
  printf("%-34s req=%3u lit=%3d applied=%3u product=%.4f limited=%d restored=%d %s\n",name,req,lit,g_budget.applied,prod,g_budget.limited,restored,pass?"PASS":"FAIL");
  if(!pass) exit(1);
}
static void fill(int nlit, uint8_t v){ for(auto s:LED_STRIPS) std::fill(s->buf.begin(),s->buf.end(),0);
  int k=0; for(auto s:LED_STRIPS) for(int p=0;p<s->n && k<nlit;p++,k++) s->buf[p*4+3]=v; }
int main(){
  fill(0,0);   check("all dark", 220);
  fill(600,255); check("nav/flood all lit, min idx", 16);
  fill(600,255); check("all lit, max idx 220", 220);
  fill(600,255); check("all lit, 255", 255);
  fill(120,255); check("20% lit, 255 (exactly at limit)", 255);
  fill(121,255); check("20.2% lit, 255", 255);
  fill(14,255);  check("knight-rider-ish 14 px, 220", 220);
  fill(600,1);   check("all lit at value 1 (still counts)", 220);
  fill(600,200); for(int i=0;i<50;i++){ EnforcePowerBudget(); ShowAllStrips(); }
                 check("50 repeat frames: no decay", 220);
  // ShowAllStrips without EnforcePowerBudget must still enforce
  fill(600,255); g_brightnessValue=255; g_budgetFresh=false; g_budget.applied=255; ShowAllStrips();
  { int bad=0; for(auto s:LED_STRIPS) for(auto v:s->lastSent) if(v> (255*51)/255) bad++;
    printf("%-34s %s\n","bypass attempt (no Enforce call)", bad?"FAIL":"PASS"); if(bad) return 1; }
  // Transient: part-old/part-new frames while strips are sent one by one
  fill(0,0); c.buf.assign(c.buf.size(),255); d.buf.assign(d.buf.size(),255);   // cockpit only, full
  check("transient A: cockpit-only @255", 255);
  fill(600,255); worstIntermediate=0; check("transient B: -> all lit (applied 51)", 255);
  printf("%-34s worst part-sent frame = %.4f %s\n","transient A->B intermediate", worstIntermediate, worstIntermediate<=0.20+1e-9?"PASS":"FAIL");
  if (worstIntermediate > 0.20+1e-9) return 1;
  fill(0,0); c.buf.assign(c.buf.size(),255); d.buf.assign(d.buf.size(),255);
  worstIntermediate=0; check("transient C: -> cockpit-only @255", 255);
  printf("%-34s worst part-sent frame = %.4f %s\n","transient B->C intermediate", worstIntermediate, worstIntermediate<=0.20+1e-9?"PASS":"FAIL");
  if (worstIntermediate > 0.20+1e-9) return 1;
  puts("ALL PASS"); }
