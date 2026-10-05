// chladni5 - the Chladni panel on the Waveshare ESP32-S3-Touch-LCD-5
//
// For the alchemy firmware `chladni`, running on both labs.  Built on
// leibniz5 (same bus, addresses, frames, shadow-frame canvas, Wi-Fi
// updates, knob panels on the outside); what is different is the column
// on the inner side of each lab:
//
//       header        LAB n, address, link
//       NOTE          note name and cents, material
//       STRIKE  BOW   a flash on each strike; the bow's state - CLEAN
//                     (Helmholtz, one slip a period), SLIP (slipping too
//                     often: thin, scratchy), HARD (raucous) - and level
//       BODY          string or bar: its shape in slow motion (a strobe at
//                     about one cycle a second) inside a faint band of its
//                     whole swing.  Plate: a Chladni figure - sand where the
//                     plate is still, from the live mode amplitudes
//       MODES         24 bars, each at its mode's real frequency (log), the
//                     mode's energy in dB
//       CONTACT       the contact point's phase portrait: velocity against
//                     displacement over one cycle; clean bowing draws a
//                     closed loop, a strike spirals in
//       COUPLE IN     how hard the other lab's bridge is shaking this one
//
// Nothing is relayed: the two labs couple through patch cables (BRIDGE ->
// COUPLE), not the bus.
//
// The body is rebuilt here from each mode's amplitude and phase, so the
// mode shapes and frequency ratios are recomputed from the same formulas as
// src/chladni/chladni_dsp.cpp Rebuild() (plate table, bar betas,
// stiffness stretch, exciter point) - change one, change both.
//
// Event payloads (src/chladni/chladni.cpp is the authority):
//   0x50 STAT   flags hz_lo hz_hi env bowq couple level
//               flags: b0 struck, b1 bow force on, b2 bow requested,
//                      b3 latched, b4-5 material; hz * 16; bowq = slips
//                      per period * 32; level = 255 + 60 log10(Amax/0.01)
//   0x51 MODES  chunk|seq<<3, (amp, phase) x 3   eight chunks = 24 modes
//               amp = 255 + 60 log10(A/Amax); phase = (phi_n - n phi_1)/2pi*256
//   0x52 GEOM   material pos pickup stiff box size hiloss
//
// WIRING, BOARD, LIBS, WIFI: see "The panel" in ../../README.md.  Never
// Arduino Wire; no flash writes while running.
// SERIAL  115200 on the board's own USB: one status line a second, also
//         at http://<panel>/status.

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <driver/i2c.h>
#include <atomic>
#include "src/gfx/Adafruit_GFX.h"   // vendored core only - the library pulls in Wire
#include "wifi_ota.h"               // Wi-Fi + over-the-air updates with rollback

using namespace esp_panel::drivers;
using namespace esp_panel::board;

/* A GFX canvas that draws OFF-SCREEN, into a shadow copy of the frame in
 * PSRAM, and records the box it touched; flush() copies just that box to
 * the panel's frame buffer, so the glass never shows a half-drawn part.
 * See duet5 for the history. */
class FbCanvas : public GFXcanvas16 {
 public:
  FbCanvas(uint16_t *shadow, uint16_t *fb, uint16_t w, uint16_t h)
      : GFXcanvas16(w, h, false), fb_(fb) { buffer = shadow; }
  void drawPixel(int16_t x, int16_t y, uint16_t c) override {
    touch(x, y, 1, 1); GFXcanvas16::drawPixel(x, y, c);
  }
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) override {
    touch(x, y, w, 1); GFXcanvas16::drawFastHLine(x, y, w, c);
  }
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c) override {
    touch(x, y, 1, h); GFXcanvas16::drawFastVLine(x, y, h, c);
  }
  void fillScreen(uint16_t c) override {
    touch(0, 0, width(), height()); GFXcanvas16::fillScreen(c);
  }
  void flush() {
    if (x1_ < x0_) return;
    const size_t n = (size_t)(x1_ - x0_ + 1) * sizeof(uint16_t);
    for (int y = y0_; y <= y1_; y++)
      memcpy(fb_ + (size_t)y * width() + x0_, buffer + (size_t)y * width() + x0_, n);
    x0_ = y0_ = INT16_MAX; x1_ = y1_ = -1;
  }
 private:
  void touch(int x, int y, int w, int h) {
    if (w < 0) { x += w + 1; w = -w; }
    if (h < 0) { y += h + 1; h = -h; }
    const int xa = max(x, 0), ya = max(y, 0);
    const int xb = min(x + w - 1, (int)width() - 1), yb = min(y + h - 1, (int)height() - 1);
    if (xb < xa || yb < ya) return;
    x0_ = min(x0_, xa); y0_ = min(y0_, ya); x1_ = max(x1_, xb); y1_ = max(y1_, yb);
  }
  uint16_t *fb_;
  int x0_ = INT16_MAX, y0_ = INT16_MAX, x1_ = -1, y1_ = -1;
};
static FbCanvas *tft = nullptr;
static const int SCREEN_W = 800, SCREEN_H = 480;

#define NUM_SOURCES  3       /* 0 unused, 1 = 0x42 (right), 2 = 0x43 (left) */
#define MAX_PAYLOAD  32

#define MSG_TEXT   0x11
#define MSG_STATE  0x12
#define MSG_COMMIT 0x13
#define MSG_EVENT  0x14

#define EVT_STAT   0x50
#define EVT_MODES  0x51
#define EVT_GEOM   0x52

const uint32_t kBaud = 115200ul;

/* ── I2C ─────────────────────────────────────────────────────────────── */

const i2c_port_t kPort = I2C_NUM_0;        /* installed by Board::begin() */
const uint8_t  kI2cAddr[NUM_SOURCES] = {0, 0x42, 0x43};

const uint8_t  CMD_ANNOUNCE   = 0x01;
const uint8_t  kReadLen       = 32;
const uint32_t kPollMs        = 4ul;
const uint32_t kAbsentPollMs  = 250ul;
const uint32_t kAnnounceAskMs = 500ul;
const int        kStretchExp = 20;          /* ~26 ms clock-stretch limit */
const TickType_t kXferTicks  = pdMS_TO_TICKS(50);

struct I2cSource {
  bool     present;
  uint32_t next_poll;
  uint32_t next_ask;
  uint32_t reads, frames, errors;
};
static I2cSource i2c[NUM_SOURCES];

/* ── Text slots (the module's announce) ──────────────────────────────── */

#define LBL        12
#define NUM_KNOBS   6
#define NUM_BTNS    3
#define NUM_JACKS  10
#define NUM_SLOTS  21

#define SLOT_KNOB(i) (i)
#define SLOT_BTN(i)  (6 + (i))
#define SLOT_PAGE    9
#define SLOT_NAME    10
#define SLOT_JACK(i) (11 + (i))

struct Module {
  bool     seen;
  bool     ready;
  bool     dirty;
  bool     demo;
  char     text[NUM_SLOTS][LBL + 1];
  uint8_t  page, n_pages, rgb[3];
  uint8_t  val[NUM_KNOBS];
  uint8_t  btn_flags;
  uint32_t last_activity;
  uint32_t last_frame;
};
static Module mod[NUM_SOURCES];

struct View {
  int      ox, oy;
  bool     drawn, waiting, need_full, need_lbls;
  uint8_t  page;
  uint32_t knob_sig[NUM_KNOBS];
  uint8_t  flags;
  char     txt[NUM_SLOTS][LBL + 1];
  uint16_t page_c;
};
static View view[NUM_SOURCES];

/* ── Chladni state per lab ───────────────────────────────────────────── */

#define NUM_MODES     24
#define NUM_MATERIALS  3
#define CHUNKS         8          /* MODES chunks per snapshot, 3 modes each */

struct Cl {
  bool     have;                  /* a STAT has arrived */
  uint32_t last_stat;
  uint8_t  flags, env, bowq, cpl, level;
  float    hz;
  uint32_t strike_ms;
  bool     have_geom;
  uint8_t  material, pos, pickup, stiff, box, size, hiloss;
  uint8_t  amp_in[NUM_MODES], ph_in[NUM_MODES];
  uint8_t  got, seq_in;           /* chunks received of snapshot seq_in */
  uint8_t  amp[NUM_MODES], ph[NUM_MODES];
  uint32_t snaps, stats;          /* totals */
  uint32_t snap_rate, stat_rate;  /* per second */
};
static Cl cl[NUM_SOURCES];

/* What is on the glass in a lab's column. */
struct ClView {
  int      x;                     /* column left edge; width kColW */
  bool     drawn;                 /* the furniture is up */
  int8_t   link;                  /* -1 never, 0 waiting, 1 live, 2 lost, 3 self test, 4 no data */
  int16_t  note_q;                /* note * 4 + cents bucket, drawn */
  int8_t   material;
  int8_t   strike_lit, bow_state;
  int16_t  env_w, cpl_w;
  uint32_t snaps_drawn;           /* the snapshot the static parts were drawn from */
  uint32_t geom_sig;
};
static ClView colv[NUM_SOURCES];

struct Parser {
  uint8_t state, type, len, idx, crc;
  uint8_t buf[MAX_PAYLOAD];
};
static Parser parser[NUM_SOURCES];

/* Bytes read by the bus task, waiting for the loop (see pollTask). */
#define RX_RING 4096
struct RxRing {
  uint8_t buf[RX_RING];
  std::atomic<uint16_t> head{0}, tail{0};     /* task writes head, loop writes tail */
  uint32_t dropped = 0;
};
static RxRing rx[NUM_SOURCES];

static bool selftest = true;

const uint32_t kLinkTimeoutMs  = 3000ul;
const uint32_t kNoDataMs       = 2000ul;

/* ── Palette ─────────────────────────────────────────────────────────── */

#define RGB(r, g, b) ((uint16_t)(((r) & 0xF8) << 8 | ((g) & 0xFC) << 3 | (b) >> 3))

const uint16_t C_BG      = RGB(  0,   0,   0);
const uint16_t C_PANEL   = RGB( 12,  12,  16);
const uint16_t C_TEXT    = RGB(255, 255, 255);
const uint16_t C_DIM     = RGB(140, 138, 132);
const uint16_t C_KNOB    = RGB( 52,  52,  60);
const uint16_t C_KNOBRIM = RGB(215, 215, 225);
const uint16_t C_POINTER = RGB(255, 255, 255);
const uint16_t C_LED_OFF = RGB( 60,  58,  52);
const uint16_t C_IND_OFF = RGB( 34,  34,  40);
const uint16_t C_JACK    = RGB(215, 215, 225);
const uint16_t C_JACKIN  = RGB(  0,   0,   0);
const uint16_t C_IDLE    = RGB( 70,  70,  78);
const uint16_t C_LOST    = RGB(255,  60,  40);
const uint16_t C_LIVE    = RGB( 70, 210, 190);
const uint16_t C_RULE    = RGB( 40,  40,  48);

static uint16_t C_PAGE = RGB(255, 168, 48);



const uint16_t C_STRIKE   = RGB(255, 255, 255);
const uint16_t C_CLEAN    = RGB( 48, 255,  80);
const uint16_t C_SLIP     = RGB(255, 160,  16);
const uint16_t C_HARD     = RGB(255,  32,  16);
const uint16_t C_DOWN     = RGB( 16,  96,  32);
const uint16_t C_SHAPE    = RGB(255, 255, 255);
const uint16_t C_SWING    = RGB( 36,  52,  64);
const uint16_t C_AXIS     = RGB( 40,  40,  48);
const uint16_t C_SAND     = RGB(255, 226, 160);
const uint16_t C_PLATE    = RGB( 22,  26,  34);
const uint16_t C_DOT      = RGB(255,  80, 144);
const uint16_t C_PICK     = RGB( 48, 224, 255);
const uint16_t C_LOOP     = RGB(255, 168,  32);
const uint16_t C_COUPLE   = RGB( 48, 224, 255);

/* One hue per material, for the mode bars. */
const uint8_t MAT_RGB[NUM_MATERIALS][3] = {
  {255, 168,  32},   /* String amber */
  { 48, 224, 255},   /* Bar    cyan  */
  {255,  80, 144},   /* Plate  rose  */
};
const uint8_t BAR_OFF[3] = {24, 24, 30};
static uint16_t lerpRGB(const uint8_t *a, const uint8_t *b, uint8_t t, uint8_t of) {
  uint8_t c[3];
  for (uint8_t k = 0; k < 3; k++) c[k] = (uint8_t)(a[k] + ((int16_t)b[k] - a[k]) * t / of);
  return RGB(c[0], c[1], c[2]);
}

/* ── Panel geometry, mm (as duet5) ───────────────────────────────────── */

const float PANEL_W = 61.0, PANEL_H = 128.5;
const float TITLE_FROM_TOP = 4.5;
const float RULE_TOP = 7.5, RULE_BOT = 7.0, BRAND_FROM_BOT = 3.0;
const float ROW_FROM_TOP[3] = {21.5, 49.0, 77.5};
const float SIDE_X_FROM_EDGE = 13.0;
const float KNOB_D = 21.6, RING_OD = 20.6, CAP_D = 15.0;
const float CENTRE_BTN_D = 8.5, IND_OFFSET = 6.5, IND_W = 4.5, IND_H = 2.5;
const float USB_W = 11.0, USB_H = 4.25, USB_BOTTOM = 35.25;
const uint8_t JACK_COLS = 5;
const float JACK_X_PITCH = 11.5, JACK_Y_PITCH = 13.0, JACK_X_SHIFT = 0.4;
const float JACK_D = 7.0, JACK_Y_LOW = 15.75;
const float LBL_JACK_GAP = 1.2, LBL_SIDE_GAP = 1.2, LBL_CENTRE_GAP = 1.2;
const float CENTRE_CUT_H = 16.5;

static float gScale;
static int   gOx, gOy;

static inline int MM (float v)  { return (int)(v * gScale + 0.5f); }
static inline int PX (float mm) { return gOx + (int)(mm * gScale + 0.5f); }
static inline int PYT(float mm) { return gOy + (int)(mm * gScale + 0.5f); }
static inline int PYB(float mm) { return gOy + (int)((PANEL_H - mm) * gScale + 0.5f); }

static void use(uint8_t src) {
  gOx = view[src].ox;
  gOy = view[src].oy;
  const Module &m = mod[src];
  C_PAGE = m.ready ? RGB(m.rgb[0], m.rgb[1], m.rgb[2]) : C_IDLE;
}

static void PumpAll(void);

/* ── Text ────────────────────────────────────────────────────────────── */

static void textAt(int cx, int cy, const char *s, uint8_t size, uint16_t color) {
  int16_t x1, y1; uint16_t w, h;
  tft->setTextSize(size);
  tft->setTextColor(color);
  tft->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  tft->setCursor(cx - w / 2 - x1, cy - h / 2 - y1);
  tft->print(s);
}

static void textCell(int cx, int cy, int halfw, int halfh,
                     const char *s, uint8_t size, uint16_t color, uint16_t bg = C_PANEL) {
  tft->fillRect(cx - halfw, cy - halfh, halfw * 2, halfh * 2, bg);
  if (s && s[0]) textAt(cx, cy, s, size, color);
}

/* Left-aligned, in a cleared box: for the word columns. Same GFX print
 * path as textAt, which is the path known to draw on this kind of glass. */
static void textLeft(int x, int y, int w, const char *s, uint8_t size, uint16_t color,
                     uint16_t bg = C_BG) {
  const int h = 8 * size;
  tft->fillRect(x, y, w, h, bg);
  tft->setTextSize(size);
  tft->setTextColor(color);
  tft->setCursor(x, y);
  tft->print(s);
}

static void fillRing(int cx, int cy, int rOuter, int rInner, uint16_t color) {
  for (int r = rInner; r <= rOuter; r++) tft->drawCircle(cx, cy, r, color);
}

/* ── Panel parts (duet5's) ───────────────────────────────────────────── */

static float knobXmm(uint8_t i) { return (i & 1) ? PANEL_W - SIDE_X_FROM_EDGE
                                                 : SIDE_X_FROM_EDGE; }
static float knobYmm(uint8_t i) { return ROW_FROM_TOP[i >> 1]; }
static float jackXmm(uint8_t c) { return PANEL_W / 2 + JACK_X_SHIFT
                                       + (c - (JACK_COLS - 1) / 2.0f) * JACK_X_PITCH; }

static void drawKnobFace(uint8_t i, uint8_t v) {
  const int cx = PX(knobXmm(i)), cy = PYT(knobYmm(i));
  const int rRing = MM(RING_OD / 2), rCap = MM(CAP_D / 2);
  const float value = v / 255.0f;
  for (uint8_t s = 0; s < 17; s++) {
    float t = (float)s / 16.0f;
    float a = (-150.0f + 300.0f * t) * DEG_TO_RAD;
    tft->fillCircle(cx + (int)(sin(a) * rRing), cy - (int)(cos(a) * rRing), 2,
                    t <= value + 0.001f ? C_PAGE : C_LED_OFF);
  }
  tft->fillCircle(cx, cy, rCap, C_KNOB);
  tft->drawCircle(cx, cy, rCap, C_KNOBRIM);
  tft->drawCircle(cx, cy, rCap - 1, C_KNOBRIM);
  float a = (-150.0f + 300.0f * value) * DEG_TO_RAD;
  int px = cx + (int)(sin(a) * (rCap - 2)), py = cy - (int)(cos(a) * (rCap - 2));
  tft->drawLine(cx, cy, px, py, C_POINTER);
  tft->drawLine(cx + 1, cy, px + 1, py, C_POINTER);
}

static uint32_t knobSig(uint8_t i, uint8_t v) {
  const int cx = PX(knobXmm(i)), cy = PYT(knobYmm(i)), rCap = MM(CAP_D / 2);
  const float value = v / 255.0f;
  uint8_t lit = 0;
  for (uint8_t s = 0; s < 17; s++) if ((float)s / 16.0f <= value + 0.001f) lit++;
  const float a = (-150.0f + 300.0f * value) * DEG_TO_RAD;
  const int px = cx + (int)(sin(a) * (rCap - 2)), py = cy - (int)(cos(a) * (rCap - 2));
  return ((uint32_t)lit << 24) | ((uint32_t)(px & 0xFFF) << 12) | (uint32_t)(py & 0xFFF);
}

static void drawKnobLabel(uint8_t i, const char *s) {
  textCell(PX(knobXmm(i)),
           PYT(knobYmm(i)) + MM(KNOB_D / 2 + LBL_SIDE_GAP + 1.4f),
           40, 4, s, 1, C_TEXT);
}

static void drawButtonFace(uint8_t i, bool lit) {
  const int cx = PX(PANEL_W / 2), cy = PYT(ROW_FROM_TOP[i]);
  const int r = MM(CENTRE_BTN_D / 2);
  const int iw = MM(IND_W), ih = MM(IND_H), io = MM(IND_OFFSET);
  tft->fillCircle(cx, cy, r, C_KNOB);
  tft->drawCircle(cx, cy, r, C_KNOBRIM);
  tft->fillRect(cx - iw / 2, cy - io - ih / 2, iw, ih, lit ? C_PAGE : C_IND_OFF);
  tft->fillRect(cx - iw / 2, cy + io - ih / 2, iw, ih, C_IND_OFF);
}

static void drawButtonLabel(uint8_t i, const char *s) {
  textCell(PX(PANEL_W / 2),
           PYT(ROW_FROM_TOP[i]) - MM(CENTRE_CUT_H / 2 + LBL_CENTRE_GAP + 1.4f),
           28, 4, s, 1, C_TEXT);
}

static void drawJack(float xmm, float ymm_from_bottom) {
  const int cx = PX(xmm), cy = PYB(ymm_from_bottom), r = MM(JACK_D / 2);
  fillRing(cx, cy, r, r - 2, C_JACK);
  tft->fillCircle(cx, cy, r - 3, C_JACKIN);
  tft->drawCircle(cx, cy, r - 3, C_KNOBRIM);
}

static bool textChanged(View &v, uint8_t slot, const char *s, bool force) {
  if (!force && strcmp(v.txt[slot], s) == 0) return false;
  strncpy(v.txt[slot], s, LBL);
  v.txt[slot][LBL] = '\0';
  return true;
}

static void drawLabels(uint8_t src, bool force) {
  View &v = view[src];
  const Module &m = mod[src];
  for (uint8_t i = 0; i < NUM_KNOBS; i++)
    if (textChanged(v, SLOT_KNOB(i), m.text[SLOT_KNOB(i)], force)) drawKnobLabel(i, m.text[SLOT_KNOB(i)]);
  for (uint8_t i = 0; i < NUM_BTNS; i++)
    if (textChanged(v, SLOT_BTN(i), m.text[SLOT_BTN(i)], force)) drawButtonLabel(i, m.text[SLOT_BTN(i)]);
  const bool recolour = force || C_PAGE != v.page_c;
  if (recolour) {
    tft->drawFastHLine(PX(3), PYT(RULE_TOP), MM(PANEL_W - 6), C_PAGE);
    tft->drawFastHLine(PX(3), PYB(RULE_BOT), MM(PANEL_W - 6), C_PAGE);
  }
  if (textChanged(v, SLOT_PAGE, m.text[SLOT_PAGE], recolour))
    textCell(PX(17), PYB(BRAND_FROM_BOT), 46, 5, m.text[SLOT_PAGE], 1, C_PAGE);
  v.page_c = C_PAGE;
}

static void drawValues(uint8_t src, bool force) {
  View &v = view[src];
  const Module &m = mod[src];
  for (uint8_t i = 0; i < NUM_KNOBS; i++) {
    const uint32_t sig = knobSig(i, m.val[i]);
    if (force || sig != v.knob_sig[i]) { drawKnobFace(i, m.val[i]); v.knob_sig[i] = sig; }
  }
  if (force || m.btn_flags != v.flags) {
    for (uint8_t i = 0; i < NUM_BTNS; i++) drawButtonFace(i, (m.btn_flags >> i) & 1);
    v.flags = m.btn_flags;
  }
}

static void drawAll(uint8_t src) {
  View &v = view[src];
  const Module &m = mod[src];
  tft->fillRect(gOx, gOy, MM(PANEL_W), MM(PANEL_H), C_PANEL);
  tft->drawRect(gOx, gOy, MM(PANEL_W), MM(PANEL_H), C_IDLE);
  textAt(PX(PANEL_W / 2), PYT(TITLE_FROM_TOP),
         m.text[SLOT_NAME][0] ? m.text[SLOT_NAME] : "no name", 2, C_TEXT);
  textAt(PX(PANEL_W - 16), PYB(BRAND_FROM_BOT), "Hermetic Modular", 1, C_DIM);
  tft->drawRect(PX(PANEL_W / 2 - USB_W / 2), PYB(USB_BOTTOM + USB_H),
                MM(USB_W), MM(USB_H), C_IDLE);
  const float jy[2] = {JACK_Y_LOW, JACK_Y_LOW + JACK_Y_PITCH};
  const float lblUpY  = (jy[0] + jy[1]) / 2;
  const float lblLowY = jy[0] - JACK_D / 2 - LBL_JACK_GAP - 1.2f;
  for (uint8_t c = 0; c < JACK_COLS; c++) {
    drawJack(jackXmm(c), jy[0]);
    drawJack(jackXmm(c), jy[1]);
    textAt(PX(jackXmm(c)), PYB(lblUpY),  m.text[SLOT_JACK(c)],     1, C_TEXT);
    textAt(PX(jackXmm(c)), PYB(lblLowY), m.text[SLOT_JACK(c + 5)], 1, C_TEXT);
  }
  PumpAll();
  drawLabels(src, true);
  drawValues(src, true);
  v.page = m.page;
  v.drawn = true;
  v.waiting = false;
}

static void drawWaiting(uint8_t src) {
  View &v = view[src];
  tft->fillRect(gOx, gOy, MM(PANEL_W), MM(PANEL_H), C_BG);
  tft->drawRect(gOx, gOy, MM(PANEL_W), MM(PANEL_H), C_IDLE);
  char s[24];
  snprintf(s, sizeof s, "waiting for 0x%02X", kI2cAddr[src]);
  textAt(PX(PANEL_W / 2), PYT(PANEL_H / 2), s, 1, C_DIM);
  v.drawn = false;
  v.waiting = true;
  memset(v.txt, 0, sizeof v.txt);
}

/* ── Mode physics: the module's formulas (chladni_dsp.cpp Rebuild) ───── */

static const float kAspect = 1.13f;              /* plate Ly / Lx */
static uint8_t plate_m[NUM_MODES], plate_n[NUM_MODES];
static float   plate_ratio[NUM_MODES];
static int8_t  plate_idx[13][13];               /* (m, n) -> mode index, or -1 */
static const char *kMaterialNames[NUM_MATERIALS] = { "String", "Bar", "Plate" };

static void buildPlateTable(void) {
  struct P { uint8_t m, n; float v; };
  static P all[144];
  int k = 0;
  for (int m = 1; m <= 12; m++)
    for (int n = 1; n <= 12; n++)
      all[k++] = { (uint8_t)m, (uint8_t)n, (float)(m * m) + (float)(n * n) / (kAspect * kAspect) };
  for (int i = 1; i < k; i++) {
    P t = all[i];
    int j = i - 1;
    while (j >= 0 && all[j].v > t.v) { all[j + 1] = all[j]; j--; }
    all[j + 1] = t;
  }
  memset(plate_idx, -1, sizeof plate_idx);
  for (int i = 0; i < NUM_MODES; i++) {
    plate_m[i] = all[i].m; plate_n[i] = all[i].n; plate_ratio[i] = all[i].v / all[0].v;
    plate_idx[all[i].m][all[i].n] = (int8_t)i;
  }
}

static float barBeta(int i) {
  static const float b[5] = { 4.73004f, 7.85320f, 10.99561f, 14.13717f, 17.27876f };
  return i < 5 ? b[i] : (2.f * (float)i + 3.f) * (float)PI * 0.5f;
}

static float barShape(int i, float x) {
  const float be = barBeta(i), sgn = (i & 1) ? -1.f : 1.f;
  return 0.5f * (cosf(be * x) - sinf(be * x) + expf(-be * x) + sgn * expf(-be * (1.f - x)));
}

static float modeRatio(const Cl &z, int i) {
  const float n = (float)(i + 1), s = z.stiff / 255.f, B = 0.02f * s * s;
  float r = n;
  if (z.material == 1) { const float q = barBeta(i) / barBeta(0); r = q * q; }
  else if (z.material == 2) r = plate_ratio[i];
  return r * sqrtf(1.f + B * n * n);
}

/* Mode i's shape at a point: x along the string/bar, (x, y) on the plate. */
static float modeShape(uint8_t mat, int i, float x, float y) {
  if (mat == 1) return barShape(i, x);
  if (mat == 2) return sinf(plate_m[i] * (float)PI * x) * sinf(plate_n[i] * (float)PI * y);
  return sinf((float)(i + 1) * (float)PI * x);
}

static float exciterX(const Cl &z) { return 0.5f * z.pos / 255.f; }
static float pickupX(const Cl &z)  { return 0.02f + 0.96f * z.pickup / 255.f; }
static void plateExciter(float p, float &x, float &y) { x = p; y = 0.38f + 0.5f * p; }
static void platePickup (float p, float &x, float &y) { x = p; y = 1.f - 0.6f * p; }

static float relAmp(uint8_t a) { return a ? powf(10.f, ((int)a - 255) / 60.f) : 0.f; }
static float phRad(uint8_t p)  { return p * (6.28318530718f / 256.f); }

/* How loud the body is, 0..1, for scaling the pictures: silent draws flat. */
static float visLevel(const Cl &z) {
  const float v = (z.level - 60.f) / 120.f;
  return v < 0.f ? 0.f : v > 1.f ? 1.f : v;
}

/* ── The column ──────────────────────────────────────────────────────── */

const int kColW = 152;
const int Y_HEAD = 10, Y_NOTE = 34, Y_LAMPS = 56, Y_ENV = 76;
const int Y_BODY = 86, BODY_H = 140;
const int Y_MODES_L = 232, Y_BARS = 244, BARS_H = 54;
const int Y_CONT_L = 304, Y_PORT = 316, PORT = 88;
const int Y_CPL_L = 412, Y_CPL = 424;
const int Y_END = 432;

const int GRID = 46, CELL = 3;                   /* Chladni grid, 138 px */
static float sinTab[13][GRID];                   /* sin(m pi x) at cell centres */

/* Per lab, rebuilt when a snapshot lands: each mode's amplitude times its
 * shape across the body, and the body's whole swing. */
static float (*shapeTab)[NUM_MODES][kColW] = nullptr;   /* [NUM_SOURCES], in PSRAM (setup) */
static float swingTab[NUM_SOURCES][kColW];
static float swingMax[NUM_SOURCES];
static float ratioTab[NUM_SOURCES][NUM_MODES];
static float ampTab[NUM_SOURCES][NUM_MODES], phTab[NUM_SOURCES][NUM_MODES];

static uint32_t anim_ms[NUM_SOURCES];

static void clHeader(uint8_t src, bool force) {
  ClView &v = colv[src];
  const Module &m = mod[src];
  const Cl &z = cl[src];
  const int8_t link = m.demo ? 3
                    : !m.seen ? 0
                    : (millis() - m.last_frame > kLinkTimeoutMs) ? 2
                    : (!z.have || millis() - z.last_stat > kNoDataMs) ? 4 : 1;
  if (!force && link == v.link) return;
  v.link = link;
  textLeft(v.x, Y_HEAD, 64, src == 1 ? "LAB 1" : "LAB 2", 2, C_TEXT);
  char addr[8];
  snprintf(addr, sizeof addr, "0x%02X", kI2cAddr[src]);
  textLeft(v.x + 70, Y_HEAD, 82, addr, 1, C_DIM);
  static const char *kLink[] = {"WAITING", "LIVE", "NO LINK", "SELF TEST", "NO DATA"};
  const uint16_t kc[] = {C_DIM, C_LIVE, C_LOST, C_DIM, C_LOST};
  textLeft(v.x + 70, Y_HEAD + 10, 82, kLink[link], 1, kc[link]);
}

static void clFurniture(uint8_t src) {
  ClView &v = colv[src];
  tft->fillRect(v.x, Y_NOTE, kColW, Y_END - Y_NOTE, C_BG);
  textLeft(v.x, Y_MODES_L, 40, "MODES", 1, C_DIM);
  textLeft(v.x, Y_CONT_L, 50, "CONTACT", 1, C_DIM);
  textLeft(v.x, Y_CPL_L, 60, "COUPLE IN", 1, C_DIM);
  tft->drawRect(v.x, Y_CPL - 1, kColW, 8, C_RULE);
  v.note_q = -32768; v.material = -1; v.strike_lit = -1; v.bow_state = -1;
  v.env_w = -1; v.cpl_w = -1; v.snaps_drawn = 0xFFFFFFFFu; v.geom_sig = 0xFFFFFFFFu;
  v.drawn = true;
}

static void clNote(uint8_t src) {
  ClView &v = colv[src];
  const Cl &z = cl[src];
  static const char *kNames[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  int16_t q = -1;
  int note = 0, cents = 0;
  if (z.hz > 1.f) {
    const float midi = 69.f + 12.f * log2f(z.hz / 440.f);
    note = (int)lroundf(midi);
    cents = (int)lroundf((midi - note) * 100.f);
    q = (int16_t)(note * 128 + cents + 64);
  }
  if (q != v.note_q) {
    char s[16] = "--";
    if (q >= 0) snprintf(s, sizeof s, "%s%d %+dc", kNames[((note % 12) + 12) % 12], note / 12 - 1, cents);
    textLeft(v.x, Y_NOTE, 104, s, 2, C_TEXT);
    v.note_q = q;
  }
  if (z.material != v.material) {
    textLeft(v.x + 108, Y_NOTE + 4, 44, z.material < NUM_MATERIALS ? kMaterialNames[z.material] : "?",
             1, RGB(MAT_RGB[z.material % 3][0], MAT_RGB[z.material % 3][1], MAT_RGB[z.material % 3][2]));
    v.material = z.material;
  }
}

static void lamp(int x, int w, const char *s, uint16_t bg, uint16_t fg) {
  tft->fillRect(x, Y_LAMPS, w, 16, bg);
  tft->setTextSize(1);
  tft->setTextColor(fg);
  int16_t x1, y1; uint16_t tw, th;
  tft->getTextBounds(s, 0, 0, &x1, &y1, &tw, &th);
  tft->setCursor(x + (w - (int)tw) / 2 - x1, Y_LAMPS + 4);
  tft->print(s);
}

static void clLamps(uint8_t src) {
  ClView &v = colv[src];
  const Cl &z = cl[src];
  const int8_t lit = (millis() - z.strike_ms < 150ul && z.strike_ms) ? 1 : 0;
  if (lit != v.strike_lit) {
    lamp(v.x, 62, "STRIKE", lit ? C_STRIKE : C_IND_OFF, lit ? C_BG : C_DIM);
    v.strike_lit = lit;
  }
  int8_t bs = 0;
  if (z.flags & 4) {
    const float q = z.bowq / 32.f;
    bs = q < 0.05f ? 1 : q > 1.3f ? 3 : q < 0.7f ? 4 : 2;
  }
  if (bs != v.bow_state) {
    static const char *kBow[] = {"BOW", "BOW DOWN", "CLEAN", "SLIP", "HARD"};
    const uint16_t bg[] = {C_IND_OFF, C_DOWN, C_CLEAN, C_SLIP, C_HARD};
    lamp(v.x + 68, kColW - 68, kBow[bs], bg[bs], bs ? C_BG : C_DIM);
    v.bow_state = bs;
  }
  const int16_t ew = (int16_t)(z.env * kColW / 255);
  if (ew != v.env_w) {
    tft->fillRect(v.x, Y_ENV, kColW, 4, C_IND_OFF);
    tft->fillRect(v.x, Y_ENV, ew, 4, C_LIVE);
    v.env_w = ew;
  }
  const int16_t cw = (int16_t)(z.cpl * (kColW - 2) / 255);
  if (cw != v.cpl_w) {
    tft->fillRect(v.x + 1, Y_CPL, kColW - 2, 6, C_BG);
    tft->fillRect(v.x + 1, Y_CPL, cw, 6, C_COUPLE);
    v.cpl_w = cw;
  }
}

/* A snapshot landed: rebuild this lab's tables. */
static void clRebuild(uint8_t src) {
  const Cl &z = cl[src];
  const uint8_t mat = z.material;
  for (int i = 0; i < NUM_MODES; i++) {
    ratioTab[src][i] = modeRatio(z, i);
    ampTab[src][i]   = relAmp(z.amp[i]);
    phTab[src][i]    = phRad(z.ph[i]);
  }
  if (mat == 2) return;
  for (int i = 0; i < NUM_MODES; i++)
    for (int x = 0; x < kColW; x++)
      shapeTab[src][i][x] = ampTab[src][i] * modeShape(mat, i, (x + 0.5f) / kColW, 0.f);
  /* The swing: the most each point reaches over 24 phases of the slow
   * cycle - the faint band the shape moves inside. */
  float mx = 0.f;
  for (int x = 0; x < kColW; x++) swingTab[src][x] = 0.f;
  for (int k = 0; k < 24; k++) {
    const float th = k * (6.28318530718f / 24.f);
    float c[NUM_MODES];
    for (int i = 0; i < NUM_MODES; i++) c[i] = cosf(ratioTab[src][i] * th + phTab[src][i]);
    for (int x = 0; x < kColW; x++) {
      float y = 0.f;
      for (int i = 0; i < NUM_MODES; i++) y += shapeTab[src][i][x] * c[i];
      y = fabsf(y);
      if (y > swingTab[src][x]) swingTab[src][x] = y;
    }
  }
  for (int x = 0; x < kColW; x++) if (swingTab[src][x] > mx) mx = swingTab[src][x];
  swingMax[src] = mx;
}

/* String or bar: the strobe. Redrawn every ~33 ms. */
static void clStrobe(uint8_t src) {
  ClView &v = colv[src];
  const Cl &z = cl[src];
  const int cy = Y_BODY + BODY_H / 2, half = BODY_H / 2 - 6;
  tft->fillRect(v.x, Y_BODY, kColW, BODY_H, C_BG);
  tft->drawFastHLine(v.x, cy, kColW, C_AXIS);
  const float vis = visLevel(z);
  const float g = swingMax[src] > 0.f ? vis * half / swingMax[src] : 0.f;
  for (int x = 0; x < kColW; x++) {
    const int h = (int)(swingTab[src][x] * g + 0.5f);
    if (h > 0) tft->drawFastVLine(v.x + x, cy - h, 2 * h + 1, C_SWING);
  }
  /* The slow cycle: about one turn of the fundamental a second. */
  const float th = (millis() % 100000ul) * 0.001f * 6.28318530718f * 0.8f;
  float c[NUM_MODES];
  for (int i = 0; i < NUM_MODES; i++) c[i] = cosf(ratioTab[src][i] * th + phTab[src][i]);
  int py = cy;
  for (int x = 0; x < kColW; x++) {
    float y = 0.f;
    for (int i = 0; i < NUM_MODES; i++) y += shapeTab[src][i][x] * c[i];
    const int yy = cy - (int)(y * g);
    if (x) { tft->drawLine(v.x + x - 1, py, v.x + x, yy, C_SHAPE); tft->drawLine(v.x + x - 1, py + 1, v.x + x, yy + 1, C_SHAPE); }
    py = yy;
  }
  /* Where it is played and where it is heard. */
  const int ex = v.x + (int)(exciterX(z) * kColW), pk = v.x + (int)(pickupX(z) * kColW);
  tft->fillRect(ex - 1, Y_BODY + BODY_H - 8, 3, 8, C_DOT);
  tft->fillRect(pk - 1, Y_BODY + BODY_H - 8, 3, 8, C_PICK);
}

/* Plate: the Chladni figure.
 *
 * A real Chladni figure is a plate driven at ONE frequency: sand piles on
 * that mode's nodal lines. A struck or bowed plate rings many modes, and
 * averaged over time their motions fill in each other's nodes - drawing
 * "sand where the plate is still" from all of them (the first version)
 * showed only the always-still edges. And the lowest mode, (1,1), which
 * dominates every strike, has no nodal lines inside the plate at all.
 *
 * So this draws the figure of the strongest OVERTONE - by velocity, the
 * fundamental left out - together with its mirror partner (n,m), added in
 * proportion to how strongly each rings. A mode alone gives a grid of
 * straight lines; the pair together gives the curved figures Chladni drew
 * (diagonals, S-curves, rings), and the figure morphs as the balance
 * between the two shifts. Sand fades out near the edges, which are always
 * still on this simply supported plate. Prototyped against engine
 * snapshots of strikes and bowing before it went on the panel. */
static void clChladni(uint8_t src) {
  ClView &v = colv[src];
  const Cl &z = cl[src];
  const int ox = v.x + (kColW - GRID * CELL) / 2, oy = Y_BODY + (BODY_H - GRID * CELL) / 2;
  int k = -1;
  float vk = 0.f;
  for (int i = 1; i < NUM_MODES; i++) {
    const float vel = ampTab[src][i] * ratioTab[src][i];
    if (vel > vk) { vk = vel; k = i; }
  }
  int p = -1;
  float vp = 0.f;
  if (k >= 0) {
    p = plate_idx[plate_n[k]][plate_m[k]];
    if (p == k) p = -1;
    if (p >= 0) vp = ampTab[src][p] * ratioTab[src][p];
  }
  static float w[GRID][GRID];
  float mx = 0.f;
  for (int j = 0; j < GRID; j++)
    for (int i = 0; i < GRID; i++) {
      float a = 0.f;
      if (k >= 0) a = vk * sinTab[plate_m[k]][i] * sinTab[plate_n[k]][j];
      if (p >= 0) a += vp * sinTab[plate_m[p]][i] * sinTab[plate_n[p]][j];
      a = fabsf(a);
      w[j][i] = a;
      if (a > mx) mx = a;
    }
  float vs = (z.level - 60.f) / 60.f;                 /* sand shows once it rings at all */
  vs = vs < 0.f ? 0.f : vs > 1.f ? 1.f : vs;
  static const uint8_t kPlate[3] = {22, 26, 34}, kSand[3] = {255, 226, 160};
  for (int j = 0; j < GRID; j++)
    for (int i = 0; i < GRID; i++) {
      float s = 0.f;
      if (mx > 0.f) {
        s = 1.f - (w[j][i] / mx) / 0.2f;              /* sand within 20 % of a node */
        s = s < 0.f ? 0.f : s * sqrtf(s);
        const int e = min(min(i, GRID - 1 - i), min(j, GRID - 1 - j));
        if (e < 3) s *= (e + 0.5f) / 3.f;             /* edges are always still: fade them */
      }
      tft->fillRect(ox + i * CELL, oy + j * CELL, CELL, CELL,
                    lerpRGB(kPlate, kSand, (uint8_t)(s * vs * 255.f), 255));
    }
  tft->drawRect(ox - 1, oy - 1, GRID * CELL + 2, GRID * CELL + 2, C_RULE);
  float x, y;
  plateExciter(exciterX(z), x, y);
  tft->fillCircle(ox + (int)(x * GRID * CELL), oy + (int)(y * GRID * CELL), 3, C_DOT);
  platePickup(pickupX(z), x, y);
  tft->fillCircle(ox + (int)(x * GRID * CELL), oy + (int)(y * GRID * CELL), 2, C_PICK);
}

/* The mode bars: each at its real frequency on a log axis, its energy in dB. */
static void clBars(uint8_t src) {
  ClView &v = colv[src];
  const Cl &z = cl[src];
  tft->fillRect(v.x, Y_BARS, kColW, BARS_H, C_BG);
  tft->drawFastHLine(v.x, Y_BARS + BARS_H - 1, kColW, C_AXIS);
  float vmax = 0.f, rmax = 1.f, e[NUM_MODES];
  for (int i = 0; i < NUM_MODES; i++) {
    e[i] = ampTab[src][i] * ratioTab[src][i];          /* velocity amplitude */
    if (e[i] > vmax) vmax = e[i];
    if (ratioTab[src][i] > rmax) rmax = ratioTab[src][i];
  }
  const float vis = visLevel(z);
  const float span = log2f(rmax);
  const uint8_t *hue = MAT_RGB[z.material % 3];
  for (int i = 0; i < NUM_MODES; i++) {
    if (e[i] <= 0.f || vmax <= 0.f) continue;
    const float db = 20.f * log10f(e[i] / vmax);
    float f = (db + 60.f) / 60.f * vis;
    if (f <= 0.f) continue;
    if (f > 1.f) f = 1.f;
    const int h = (int)(f * (BARS_H - 2));
    const int x = v.x + 1 + (int)((span > 0.f ? log2f(ratioTab[src][i]) / span : 0.f) * (kColW - 5));
    tft->fillRect(x, Y_BARS + BARS_H - 1 - h, 3, h, lerpRGB(BAR_OFF, hue, (uint8_t)(64 + 191 * f), 255));
  }
}

/* The contact point over one slow cycle: displacement across, velocity up. */
static void clPortrait(uint8_t src) {
  ClView &v = colv[src];
  const Cl &z = cl[src];
  const int ox = v.x, oy = Y_PORT;
  tft->fillRect(ox, oy, PORT, PORT, C_BG);
  tft->drawRect(ox, oy, PORT, PORT, C_RULE);
  tft->drawFastHLine(ox + 1, oy + PORT / 2, PORT - 2, C_AXIS);
  tft->drawFastVLine(ox + PORT / 2, oy + 1, PORT - 2, C_AXIS);
  float a[NUM_MODES];
  float ex = exciterX(z), ey = 0.f;
  if (z.material == 2) plateExciter(ex, ex, ey);
  for (int i = 0; i < NUM_MODES; i++) a[i] = ampTab[src][i] * modeShape(z.material, i, ex, ey);
  const int N = 96;
  const float turns = z.material == 0 ? 1.f : 2.f;
  static float px[96], pv[96];
  float mx = 0.f, mv = 0.f;
  for (int k = 0; k < N; k++) {
    const float th = k * (6.28318530718f * turns / (N - 1));
    float xs = 0.f, vs = 0.f;
    for (int i = 0; i < NUM_MODES; i++) {
      const float ph = ratioTab[src][i] * th + phTab[src][i];
      xs += a[i] * cosf(ph);
      vs -= a[i] * ratioTab[src][i] * sinf(ph);
    }
    px[k] = xs; pv[k] = vs;
    mx = fmaxf(mx, fabsf(xs)); mv = fmaxf(mv, fabsf(vs));
  }
  if (mx <= 0.f || mv <= 0.f || visLevel(z) <= 0.f) return;
  const float hx = (PORT / 2 - 4) / mx, hv = (PORT / 2 - 4) / mv;
  const int cx = ox + PORT / 2, cy = oy + PORT / 2;
  for (int k = 1; k < N; k++)
    tft->drawLine(cx + (int)(px[k - 1] * hx), cy - (int)(pv[k - 1] * hv),
                  cx + (int)(px[k] * hx),     cy - (int)(pv[k] * hv), C_LOOP);
}

static void clGeom(uint8_t src) {
  ClView &v = colv[src];
  const Cl &z = cl[src];
  const uint32_t sig = (uint32_t)z.pos | ((uint32_t)z.pickup << 8) | ((uint32_t)z.stiff << 16)
                     | ((uint32_t)z.hiloss << 24);
  if (sig == v.geom_sig) return;
  v.geom_sig = sig;
  const int x = v.x + PORT + 8, w = kColW - PORT - 8;
  char s[20];
  snprintf(s, sizeof s, "POS  %.2f", exciterX(z));
  textLeft(x, Y_PORT + 4, w, s, 1, C_DOT);
  snprintf(s, sizeof s, "PICK %.2f", pickupX(z));
  textLeft(x, Y_PORT + 18, w, s, 1, C_PICK);
  snprintf(s, sizeof s, "STIF %.2f", z.stiff / 255.f);
  textLeft(x, Y_PORT + 32, w, s, 1, C_DIM);
  snprintf(s, sizeof s, "LOSS %.2f", z.hiloss / 255.f);
  textLeft(x, Y_PORT + 46, w, s, 1, C_DIM);
  snprintf(s, sizeof s, "BOX  %.2f", z.box / 255.f);
  textLeft(x, Y_PORT + 60, w, s, 1, C_DIM);
}

static void clMessage(uint8_t src, const char *s) {
  ClView &v = colv[src];
  tft->fillRect(v.x, Y_NOTE, kColW, Y_END - Y_NOTE, C_BG);
  textAt(v.x + kColW / 2, (Y_NOTE + Y_END) / 2, s, 1, C_DIM);
  v.drawn = false;
}

static void drawColumn(uint8_t src) {
  ClView &v = colv[src];
  const Cl &z = cl[src];
  clHeader(src, v.link < 0);
  const bool live = v.link == 1 || v.link == 3;
  if (!live) {
    static int8_t shown[NUM_SOURCES] = {-1, -1, -1};
    if (shown[src] != v.link || v.drawn) {
      shown[src] = v.link;
      clMessage(src, v.link == 4 ? "no Chladni data" : v.link == 2 ? "link lost" : "");
    }
    return;
  }
  if (!v.drawn) clFurniture(src);
  clNote(src);
  clLamps(src);
  clGeom(src);
  const bool fresh = z.snaps != v.snaps_drawn;
  if (fresh) {
    v.snaps_drawn = z.snaps;
    clRebuild(src);
    PumpAll();
    clBars(src);
    clPortrait(src);
    if (z.material == 2) clChladni(src);
    PumpAll();
  }
  if (z.material != 2 && millis() - anim_ms[src] >= 33ul) {
    anim_ms[src] = millis();
    clStrobe(src);
  }
}

/* ── Wi-Fi line, bottom centre (duet5's) ─────────────────────────────── */

static int8_t wifi_drawn = -1;

static void drawWifi(void) {
  int8_t &drawn = wifi_drawn;
  static String drawn_ip;
  const int8_t st = (int8_t)wifi_ota::state();
  const String ip = wifi_ota::address();
  if (st == drawn && ip == drawn_ip) return;
  drawn = st; drawn_ip = ip;
  static const char *kSt[] = {"NO WI-FI CONFIGURED", "WI-FI JOINING", "WI-FI ONLINE", "UPDATING"};
  const uint16_t kc[] = {C_IDLE, C_DIM, C_LIVE, RGB(255, 168, 48)};
  textCell(SCREEN_W / 2, SCREEN_H - 34, 118, 6, kSt[st], 1, kc[st], C_BG);
  char line[48] = "";
  if (ip.length()) snprintf(line, sizeof line, "%s.local  %s", wifi_ota::kHost, ip.c_str());
  textCell(SCREEN_W / 2, SCREEN_H - 18, 118, 6, line, 1, C_DIM, C_BG);
}

static void forceRepaint(void) {
  tft->fillScreen(C_BG);
  for (uint8_t i = 1; i < NUM_SOURCES; i++) {
    view[i].drawn = view[i].waiting = false;
    view[i].need_full = true;
    colv[i].drawn = false;
    colv[i].link = -1;
  }
  wifi_drawn = -1;
}

static void onUpdateStart(void) {
  tft->fillRect(SCREEN_W / 2 - 120, SCREEN_H / 2 - 40, 240, 80, C_BG);
  tft->drawRect(SCREEN_W / 2 - 120, SCREEN_H / 2 - 40, 240, 80, RGB(255, 168, 48));
  textAt(SCREEN_W / 2, SCREEN_H / 2 - 14, "UPDATING", 3, RGB(255, 168, 48));
  textAt(SCREEN_W / 2, SCREEN_H / 2 + 22, "keep the power on", 1, C_TEXT);
  tft->flush();
}
static void onUpdateFailed(void) { forceRepaint(); }
static void onUpdateProgress(uint8_t pct) {
  char s[8];
  snprintf(s, sizeof s, "%u%%", pct);
  textCell(SCREEN_W / 2, SCREEN_H / 2 + 8, 30, 6, s, 1, C_TEXT, C_BG);
  tft->flush();
}

/* ── Frame handling ──────────────────────────────────────────────────── */

static uint8_t crc8(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  return crc;
}

static bool copyField(char *dst, const uint8_t *src, uint8_t n) {
  char tmp[LBL + 1];
  if (n > LBL) n = LBL;
  memcpy(tmp, src, n);
  tmp[n] = '\0';
  for (int8_t i = (int8_t)n - 1; i >= 0 && (tmp[i] == ' ' || tmp[i] == '\0'); i--)
    tmp[i] = '\0';
  if (strcmp(tmp, dst) == 0) return false;
  strcpy(dst, tmp);
  return true;
}

static void onEvent(uint8_t src, const uint8_t *p, uint8_t len) {
  Cl &z = cl[src];
  switch (p[0]) {
    case EVT_STAT: {
      if (len < 8) return;
      z.flags = p[1];
      z.hz    = (float)((uint16_t)p[2] | ((uint16_t)p[3] << 8)) / 16.f;
      z.env = p[4]; z.bowq = p[5]; z.cpl = p[6]; z.level = p[7];
      const uint8_t mat = (p[1] >> 4) & 3;
      if (mat < NUM_MATERIALS) z.material = mat;
      if (p[1] & 1) z.strike_ms = millis();
      z.have = true;
      z.last_stat = millis();
      z.stats++;
      break;
    }
    case EVT_MODES: {
      if (len < 8) return;
      const uint8_t chunk = p[1] & 7, seq = p[1] >> 3;
      if (seq != z.seq_in) { z.seq_in = seq; z.got = 0; }
      for (uint8_t i = 0; i < 3; i++) {
        z.amp_in[chunk * 3 + i] = p[2 + 2 * i];
        z.ph_in[chunk * 3 + i]  = p[3 + 2 * i];
      }
      z.got |= (uint8_t)(1u << chunk);
      if (z.got == 0xFF) {        /* all eight chunks of one snapshot */
        memcpy(z.amp, z.amp_in, NUM_MODES);
        memcpy(z.ph, z.ph_in, NUM_MODES);
        z.snaps++;
        z.got = 0;
      }
      break;
    }
    case EVT_GEOM:
      if (len < 8) return;
      if (p[1] < NUM_MATERIALS) z.material = p[1];
      z.pos = p[2]; z.pickup = p[3]; z.stiff = p[4];
      z.box = p[5]; z.size = p[6]; z.hiloss = p[7];
      z.have_geom = true;
      break;
  }
}

static void clearDemo(uint8_t i) {
  memset(&mod[i], 0, sizeof mod[i]);
  memset(&cl[i], 0, sizeof cl[i]);
  view[i].drawn = view[i].waiting = false;
  colv[i].drawn = false;
  colv[i].link = -1;
}

static void applyFrame(uint8_t src, uint8_t type, const uint8_t *p, uint8_t len) {
  Module &m = mod[src];
  if (m.demo) clearDemo(src);
  m.seen = true;
  m.last_frame = millis();
  i2c[src].frames++;
  if (selftest) {
    selftest = false;
    for (uint8_t i = 1; i < NUM_SOURCES; i++) if (mod[i].demo) clearDemo(i);
  }
  if (type == MSG_TEXT && len >= 1) {
    uint8_t slot = p[0];
    if (slot < NUM_SLOTS && copyField(m.text[slot], p + 1, len - 1)) m.dirty = true;
  }
  else if (type == MSG_COMMIT && len >= 6) {
    m.page    = p[1];
    m.n_pages = p[2];
    m.rgb[0]  = p[3]; m.rgb[1] = p[4]; m.rgb[2] = p[5];
    const bool fresh = !m.ready || m.dirty;
    m.ready = true;
    m.dirty = false;
    if (!p[0])      view[src].need_lbls = true;
    else if (fresh) view[src].need_full = true;
  }
  else if (type == MSG_STATE && len >= 3 + NUM_KNOBS) {
    m.page      = p[0];
    m.btn_flags = p[1];
    if (p[2]) m.last_activity = millis();
    for (uint8_t i = 0; i < NUM_KNOBS; i++) m.val[i] = p[3 + i];
  }
  else if (type == MSG_EVENT && len >= 1) {
    onEvent(src, p, len);
  }
}

static void feed(uint8_t src, uint8_t b) {
  Parser &ps = parser[src];
  switch (ps.state) {
    case 0: if (b == 0xAA) ps.state = 1; break;
    case 1: ps.state = (b == 0x55) ? 2 : (b == 0xAA ? 1 : 0); break;
    case 2: ps.type = b; ps.crc = crc8(0, b); ps.state = 3; break;
    case 3:
      ps.len = b; ps.crc = crc8(ps.crc, b); ps.idx = 0;
      ps.state = (ps.len > MAX_PAYLOAD) ? 0 : (ps.len ? 4 : 5);
      break;
    case 4:
      ps.buf[ps.idx++] = b; ps.crc = crc8(ps.crc, b);
      if (ps.idx >= ps.len) ps.state = 5;
      break;
    case 5:
      if (b == ps.crc) applyFrame(src, ps.type, ps.buf, ps.len);
      ps.state = 0;
      break;
  }
}

/* ── I2C polling and the relay ───────────────────────────────────────── */

static esp_err_t busWrite(uint8_t addr, const uint8_t *d, size_t n) {
  return i2c_master_write_to_device(kPort, addr, d, n, kXferTicks);
}


/* The bus is polled by its own task on core 0, not from the drawing loop:
 * each 32-byte read blocks for ~3 ms at 100 kHz, and polling between draws
 * (as leibniz5 does) starved the bus once the strobe redrew 30 times a
 * second - 35 reads/s a lab, STAT at 19/s, 2-3 snapshots/s.  The task only
 * moves raw bytes into a ring per lab; the frame parser and every piece of
 * display state stay on the loop's thread (drainRings), so nothing is
 * shared but the rings and the plain counters in i2c[]. */

static void ringPut(RxRing &r, const uint8_t *d, uint8_t n) {
  uint16_t h = r.head.load(std::memory_order_relaxed);
  const uint16_t t = r.tail.load(std::memory_order_acquire);
  for (uint8_t i = 0; i < n; i++) {
    const uint16_t nx = (uint16_t)((h + 1) % RX_RING);
    if (nx == t) { r.dropped += n - i; break; }
    r.buf[h] = d[i];
    h = nx;
  }
  r.head.store(h, std::memory_order_release);
}

/* One poll of one lab, in the task.  Returns true if the read came back
 * full (more is waiting). */
static bool pollSource(uint8_t src) {
  I2cSource &s = i2c[src];
  const uint32_t now = millis();
  if ((int32_t)(now - s.next_poll) < 0) return false;
  uint8_t buf[kReadLen];
  const esp_err_t e = i2c_master_read_from_device(kPort, kI2cAddr[src], buf, kReadLen, kXferTicks);
  if (e != ESP_OK) {
    if (e != ESP_FAIL) s.errors++;
    s.present   = false;
    s.next_poll = now + kAbsentPollMs;
    return false;
  }
  s.reads++;
  uint8_t n = buf[0];
  if (n > kReadLen - 1) n = 0;
  ringPut(rx[src], buf + 1, n);
  if (!s.present) {
    s.present  = true;
    s.next_ask = now;
  }
  if (!mod[src].ready && (int32_t)(now - s.next_ask) >= 0) {
    const uint8_t ask = CMD_ANNOUNCE;
    busWrite(kI2cAddr[src], &ask, 1);
    s.next_ask = now + kAnnounceAskMs;
  }
  const bool full = n == kReadLen - 1;
  s.next_poll = full ? now : now + kPollMs;
  return full;
}

static void pollTask(void *) {
  for (;;) {
    bool more = false;
    for (uint8_t i = 1; i < NUM_SOURCES; i++) more |= pollSource(i);
    if (!more) vTaskDelay(1);
  }
}

/* The loop's side: parse whatever the task has read. */
static void drainRings(void) {
  for (uint8_t src = 1; src < NUM_SOURCES; src++) {
    RxRing &r = rx[src];
    uint16_t t = r.tail.load(std::memory_order_relaxed);
    const uint16_t h = r.head.load(std::memory_order_acquire);
    while (t != h) {
      feed(src, r.buf[t]);
      t = (uint16_t)((t + 1) % RX_RING);
    }
    r.tail.store(t, std::memory_order_release);
  }
}

static void PumpAll(void) {
  tft->flush();
  while (Serial.available()) Serial.read();
  drainRings();
}

static bool probe(uint8_t addr) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (uint8_t)(addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  const esp_err_t e = i2c_master_cmd_begin(kPort, cmd, kXferTicks);
  i2c_cmd_link_delete(cmd);
  return e == ESP_OK;
}

/* ── Self test: Chladni labels and a made-up instrument, until a real lab speaks ── */

static const char *kStJack[NUM_JACKS] = {
  "Couple In","Pitch","Bow Speed","Envelope","Out",
  "Strike","Bow","Position","Brightness","Bridge" };
static const char *kStPage[3] = {"Play","Body","Box"};
static const uint8_t kStRgb[3][3] = { {0xff,0xa8,0x20},{0x30,0xe0,0xff},{0xff,0x50,0x90} };
static const char *kStKnob[3][NUM_KNOBS] = {
  {"Pitch","Pressure","Bow Speed","Position","Hardness","Velocity"},
  {"Material","Stiffness","Decay","High Loss","Pickup","Couple"},
  {"Box","Box Size","Rosin","Level","",""} };

static void selftestTick(void) {
  static uint32_t next_page = 0, next_snap = 0;
  static uint8_t  pg = 0;
  const bool turn = (int32_t)(millis() - next_page) >= 0;
  if (turn) next_page = millis() + 4000ul;
  const bool snap = (int32_t)(millis() - next_snap) >= 0;
  if (snap) next_snap = millis() + 66ul;
  for (uint8_t src = 1; src < NUM_SOURCES; src++) {
    Module &m = mod[src];
    if (m.seen && !m.demo) continue;
    if (!m.ready) {
      m.seen = m.ready = m.demo = true;
      strncpy(m.text[SLOT_NAME], "SELF TEST", LBL);
      for (uint8_t i = 0; i < NUM_JACKS; i++) strncpy(m.text[SLOT_JACK(i)], kStJack[i], LBL);
      m.n_pages = 3;
      view[src].need_full = true;
    }
    if (turn) {
      const uint8_t p = (pg + src) % 3;
      m.page = p;
      strncpy(m.text[SLOT_PAGE], kStPage[p], LBL);
      memcpy(m.rgb, kStRgb[p], 3);
      for (uint8_t i = 0; i < NUM_KNOBS; i++) strncpy(m.text[SLOT_KNOB(i)], kStKnob[p][i], LBL);
      strncpy(m.text[SLOT_BTN(0)], "PAGE", LBL);
      strncpy(m.text[SLOT_BTN(1)], "STRIKE", LBL);
      strncpy(m.text[SLOT_BTN(2)], "BOW", LBL);
      m.btn_flags = 1;
      view[src].need_lbls = true;
    }
    const float t = millis() / 1000.0f + src * 7;
    for (uint8_t i = 0; i < NUM_KNOBS; i++)
      m.val[i] = (uint8_t)(127.5f + 127.0f * sin(t * (0.3f + 0.11f * i) + i));
    if (snap) {   /* lab 1 a bowed string (1/n^2, in phase), lab 2 a plate */
      Cl &z = cl[src];
      z.have = z.have_geom = true; z.last_stat = millis();
      z.material = src == 1 ? 0 : 2;
      z.pos = 51; z.pickup = 196; z.stiff = 0; z.hiloss = 230; z.box = 255;
      z.hz = src == 1 ? 110.f : 220.f;
      z.flags = (uint8_t)((src == 1 ? 6u : 0u) | (z.material << 4));
      z.bowq = 32; z.env = 160; z.level = 200; z.cpl = (uint8_t)(64 + 63 * sinf(t));
      for (int i = 0; i < NUM_MODES; i++) {
        const float a = src == 1 ? 1.f / ((i + 1) * (i + 1))
                                 : 0.5f + 0.5f * sinf(t * (0.2f + 0.05f * i) + i);
        const float b = 255.f + 60.f * log10f(a);
        z.amp[i] = b < 0.f ? 0 : (uint8_t)b;
        z.ph[i]  = src == 1 ? 0 : (uint8_t)(i * 37);
      }
      z.snaps++;
    }
  }
  if (turn) pg = (pg + 1) % 3;
}

/* ── Once a second ───────────────────────────────────────────────────── */

static char status_line[640];
static const char *statusText(void) { return status_line; }

static void secondTick(void) {
  static uint32_t last_snaps[NUM_SOURCES], last_stats[NUM_SOURCES];
  for (uint8_t i = 1; i < NUM_SOURCES; i++) {
    cl[i].snap_rate = cl[i].snaps - last_snaps[i]; last_snaps[i] = cl[i].snaps;
    cl[i].stat_rate = cl[i].stats - last_stats[i]; last_stats[i] = cl[i].stats;
  }
  int n = snprintf(status_line, sizeof status_line, "t=%lu internal_free=%u dma_free=%u\n",
                   (unsigned long)(millis() / 1000ul),
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
  for (uint8_t i = 1; i < NUM_SOURCES && n < (int)sizeof status_line; i++) {
    const Module &m = mod[i];
    const Cl &z = cl[i];
    n += snprintf(status_line + n, sizeof status_line - n,
                  "0x%02X %s %s page=%s reads=%lu frames=%lu err=%lu drop=%lu | stat=%lu/s snaps=%lu/s"
                  " hz=%.2f mat=%s flags=%02X bowq=%.2f env=%u level=%u couple=%u pos=%u age=%ldms\n",
                  kI2cAddr[i], i2c[i].present ? "up" : "down", m.text[SLOT_NAME], m.text[SLOT_PAGE],
                  (unsigned long)i2c[i].reads, (unsigned long)i2c[i].frames, (unsigned long)i2c[i].errors,
                  (unsigned long)rx[i].dropped, (unsigned long)z.stat_rate, (unsigned long)z.snap_rate, z.hz,
                  kMaterialNames[z.material % 3], z.flags, z.bowq / 32.f, z.env, z.level, z.cpl, z.pos,
                  z.have ? (long)(millis() - z.last_stat) : -1L);
  }
  Serial.print(status_line);
}

/* ── Main ────────────────────────────────────────────────────────────── */

/* Every way setup() can stop goes through here.  A new image that cannot
 * bring the board up never reaches wifi_ota::tick(), so it would never roll
 * itself back - restart instead, and the bootloader boots the previous one. */
static void halt(const __FlashStringHelper *why) {
  Serial.println(why);
  while (true) {
    if (wifi_ota::pendingVerify() && millis() > wifi_ota::kVerifyMs) esp_restart();
    delay(1000);
  }
}

void setup() {
  Serial.begin(kBaud);
  delay(1200);
  Serial.println(F("chladni5 starting"));
  buildPlateTable();
  for (int m = 0; m < 13; m++)
    for (int i = 0; i < GRID; i++) sinTab[m][i] = sinf(m * (float)PI * (i + 0.5f) / GRID);

  Board *board = new Board();
  if (!board || !board->init()) halt(F("board init failed"));
  LCD *lcd = board->getLCD();
  lcd->configFrameBufferNumber(1);
  /* 20 rows, not duet5's 10.  With 10, short streaks of stale pixels
   * showed at the LEFT edge, every 10 rows (photo 2026-10-04): the refill
   * of a bounce buffer from the PSRAM frame started late, so the start of
   * each strip went out with the strip from 20 rows above.  duet5 had black
   * there, so it never showed.  Twice the rows = twice the refill deadline;
   * costs another 32 KB of internal DMA RAM. */
  static_cast<BusRGB *>(lcd->getBus())->configRGB_BounceBufferSize(SCREEN_W * 20);
  if (!board->begin()) {
    Serial.println(F("board begin failed"));
    i2c_config_t c = {};
    c.mode = I2C_MODE_MASTER; c.sda_io_num = 8; c.scl_io_num = 9;
    c.sda_pullup_en = c.scl_pullup_en = GPIO_PULLUP_ENABLE; c.master.clk_speed = 100000;
    i2c_param_config(kPort, &c);
    i2c_driver_install(kPort, I2C_MODE_MASTER, 0, 0, 0);
    while (true) {
      Serial.printf("lines idle: SDA=%d SCL=%d   answers:", gpio_get_level(GPIO_NUM_8), gpio_get_level(GPIO_NUM_9));
      for (uint8_t a = 0x08; a <= 0x77; a++) if (probe(a)) Serial.printf(" 0x%02X", a);
      Serial.println(F("   (board: CH422G 0x20-0x27/0x30-0x3F, RTC 0x51, touch 0x5D or 0x14; labs 0x42/0x43)"));
      delay(2000);
      if (wifi_ota::pendingVerify() && millis() > wifi_ota::kVerifyMs) esp_restart();   /* roll back */
    }
  }
  uint16_t *fb = (uint16_t *)lcd->getFrameBufferByIndex(0);
  if (!fb) halt(F("no frame buffer"));
  uint16_t *shadow = (uint16_t *)heap_caps_malloc((size_t)SCREEN_W * SCREEN_H * 2, MALLOC_CAP_SPIRAM);
  if (!shadow) halt(F("no PSRAM for the shadow frame"));
  shapeTab = (float (*)[NUM_MODES][kColW])heap_caps_calloc(NUM_SOURCES, sizeof *shapeTab, MALLOC_CAP_SPIRAM);
  if (!shapeTab) halt(F("no PSRAM for the shape tables"));
  tft = new FbCanvas(shadow, fb, SCREEN_W, SCREEN_H);
  tft->setTextWrap(false);

  if (i2c_set_timeout(kPort, kStretchExp) != ESP_OK)
    Serial.println(F("i2c_set_timeout refused - using the driver default"));
  Serial.printf("bus: 0x42 %s, 0x43 %s\n", probe(0x42) ? "answers" : "silent",
                probe(0x43) ? "answers" : "silent");

  /* Lab #2 (0x43) on the left, lab #1 (0x42) on the right; knob panels on
   * the outside, Chladni columns either side of the centre. */
  const int margin = 8;
  gScale = (SCREEN_H - 2.0f * margin) / PANEL_H;
  const int pw = (int)(PANEL_W * gScale + 0.5f);
  view[2].ox = margin;                  view[2].oy = margin;
  view[1].ox = SCREEN_W - margin - pw;  view[1].oy = margin;
  colv[2].x = margin + pw + 12;                /* 240 */
  colv[1].x = SCREEN_W / 2 + 8;                /* 408 */
  for (uint8_t i = 1; i < NUM_SOURCES; i++) colv[i].link = -1;

  tft->fillScreen(C_BG);
  tft->drawFastVLine(SCREEN_W / 2, margin, Y_END - margin, C_RULE);
  xTaskCreatePinnedToCore(pollTask, "i2cpoll", 4096, nullptr, 2, nullptr, 0);
  wifi_ota::onStatus(statusText);
  wifi_ota::begin("chladni5", onUpdateStart, onUpdateProgress, onUpdateFailed);
  Serial.println(F("chladni5 ready"));
}

void loop() {
  PumpAll();
  if (selftest) selftestTick();

  static uint32_t next_sec = 1000ul;
  if ((int32_t)(millis() - next_sec) >= 0) { next_sec += 1000ul; secondTick(); }

  for (uint8_t src = 1; src < NUM_SOURCES; src++) {
    View &v = view[src];
    const Module &m = mod[src];
    use(src);
    if (!m.ready) {
      if (!v.waiting) drawWaiting(src);
    }
    else if (!v.drawn || v.need_full) {
      v.need_full = v.need_lbls = false;
      drawAll(src);
    }
    else if (m.page != v.page || v.need_lbls) {
      const bool recolour = C_PAGE != v.page_c;
      v.page = m.page; v.need_lbls = false;
      drawLabels(src, false);
      drawValues(src, recolour);
    }
    else {
      drawValues(src, false);
    }
    PumpAll();
    drawColumn(src);
    PumpAll();
  }
  drawWifi();
  tft->flush();
  wifi_ota::tick();
}
