/**
 * chladni.cpp — A struck and bowed modal body for the Alchemy Lab.
 *
 * One instrument per lab: a mallet and a bow acting at one point of a
 * 24-mode string, bar or plate, solved with zero delay (see chladni_dsp.h).
 * Two labs patched BRIDGE -> COUPLE both ways are two instruments sharing a
 * soundboard: strike one and the other answers on the partials they share.
 *
 * The sound lives in chladni_dsp.* (no libDaisy, rendered and checked by
 * tools/chladni_render). This file owns the board: knobs, jacks, buttons,
 * LEDs, the manual and the panel link.
 *
 * Three pages on B1: **Play**, **Body**, **Box**. B2 strikes. B3 is the
 * bow: hold it to bow while held, tap it to latch the bow down, tap again
 * to lift. Its light is the bow's state — green Helmholtz, amber slipping
 * too often (thin, scratchy), red raucous.
 *
 * Jacks (hw.cv[i] is J3+i):
 *
 *   J1 COUPLE  audio in: the other lab's BRIDGE   J2 STRIKE  trigger; peak = velocity
 *   J3 PITCH   1 V/oct                            J4 BOW     gate / pressure CV
 *   J5 SPEED   bow speed, +/-5 V                  J6 POS     strike/bow point
 *   J7 ENV     OUTPUT: envelope, 0-8 V            J8 BRIGHT  OUTPUT: centroid, 1 V/oct over f0
 *   J9 OUT     the pickup / box                   J10 BRIDGE for the other lab's COUPLE
 *
 * BOW (J4): over 0.3 V puts the bow down, and the voltage scales the
 * Pressure knob — 5 V is the knob's value, so a gate bows at the knob's
 * pressure and an envelope swells it. With B3 down and J4 unpatched the
 * knob's pressure is used as it stands.
 *
 * ── Panel bus ─────────────────────────────────────────────────────────
 * The panel (panel/chladni5) is the I2C master; nothing goes down.
 * EVENT payloads up, at most 8 bytes each:
 *
 *   0x50 STAT   flags hz_lo hz_hi env bowq couple level        every other frame
 *               flags: b0 struck since last STAT, b1 bow force on,
 *                      b2 bow requested, b3 bow latched, b4-5 material
 *               hz: fundamental * 16. env, couple: 0..255.
 *               bowq: slips per period * 32 (32 = Helmholtz), 0 unbowed
 *               level: 255 + 60 log10(Amax / 0.01), the loudest mode
 *   0x51 MODES  chunk|seq<<3, (amp, phase) x 3                 8 chunks
 *               amp: 255 + 60 log10(A / Amax), 0 = silent
 *               phase: (phi_n - n phi_1) / 2pi * 256 — the mode's phase
 *               against the fundamental's, so a periodic motion's shape
 *               stands still and the panel can strobe it
 *   0x52 GEOM   material pos pickup stiff box size hiloss      on change
 *               pos 0..0.5 as 0..255; the rest 0..1 as 0..255
 *
 * One snapshot (all eight MODES chunks) comes from one audio block, so the
 * shape the panel rebuilds is consistent. Everything waits while
 * panel_i2c's ring has a backlog, so the knob STATE frames always get out.
 *
 * Mirror any control change into tools/descriptor_probe/mirrors/chladni.cpp,
 * then run tools/descriptor_probe/run.sh chladni.
 *
 * The Pager is load-bearing and must stay managed and attached — see
 * README.md, "Checking the build without hardware".
 */

#include "daisy_seed.h"
#include "alchemy/hw/alchemy_lab.h"
#include "alchemy/host_link/host.h"
#include "alchemy/surface/control_loop.h"
#include "alchemy/surface/jack.h"
#include "alchemy/surface/manual.h"
#include "alchemy/surface/page.h"
#include "alchemy/surface/pager.h"
#include "alchemy/surface/presets.h"
#include "alchemy/surface/virtual_knob.h"

#include <cmath>
#include <cstring>

#include "common/panel_i2c.h"
#include "chladni_dsp.h"
#include "chladni_palette.h"

using namespace alchemy;
namespace cd = chladni_dsp;

/* ── Ranges ──────────────────────────────────────────────────────────── */

static constexpr float kPitchMinHz = 27.5f;     /* A0 */
static constexpr float kPitchMaxHz = 1760.f;    /* A6 */
static constexpr float kPitchCvMinV = -5.f;
static constexpr float kPitchCvMaxV =  5.f;
/* Retune only when the pitch moves this many cents: CV noise would
 * otherwise rebuild the modes on every block. */
static constexpr float kRetuneCents = 0.3f;

/* Pressure 1 is Schelleng's maximum (chladni_dsp.h). Helmholtz motion
 * holds from about 0.3 to 0.5; above 0.6 it turns raucous. The knob tops
 * out just past that so the scratch is reachable but not easy to fall in. */
static constexpr float kPressureMax = 0.7f;

static constexpr float kPosMin = 0.02f, kPosMax = 0.5f;
static constexpr float kPosPerVolt = 0.05f;      /* +/-5 V = +/-0.25 */

static constexpr float kDecayMinS = 0.05f, kDecayMaxS = 20.f;

/* ── Jacks ───────────────────────────────────────────────────────────── */

static constexpr uint8_t kCvPitch  = 0;   /* J3  1 V/oct                 */
static constexpr uint8_t kCvBow    = 1;   /* J4  gate / pressure         */
static constexpr uint8_t kCvSpeed  = 2;   /* J5  bipolar                 */
static constexpr uint8_t kCvPos    = 3;   /* J6  bipolar                 */
static constexpr uint8_t kCvEnv    = 4;   /* J7  OUTPUT                  */
static constexpr uint8_t kCvBright = 5;   /* J8  OUTPUT                  */
static_assert(kNumCvInputs > kCvBright, "chladni needs all six CV jacks");

static constexpr float kBowOnV      = 0.3f;
static constexpr float kBowFullV    = 5.f;    /* this many volts = the knob's pressure */
static constexpr float kBowMaxScale = 1.4f;
static constexpr float kEnvOutV     = 8.f;
static constexpr float kBrightMaxV  = 8.f;

/* J2 strike, as handpan reads its trigger: codec units are +/-5 V = +/-1,
 * the edge arms at 1 V and the peak over the next millisecond is the
 * velocity, a 4 V trigger being full. */
static constexpr float kJackFullScaleV = 5.f;
static constexpr float kTrigThresh     = 1.0f / kJackFullScaleV;
static constexpr float kFullVelVolts   = 4.f;
static constexpr float kPeakWindowMs   = 1.f;
static constexpr float kRefractoryMs   = 6.f;
static constexpr float kMinVel         = 0.05f;

/* B3: released before this it was a tap (latch); after, a momentary hold. */
static constexpr uint32_t kBowTapMs = 300u;
static constexpr uint32_t kStrikeFlashMs = 60u;

/* ── Pages ───────────────────────────────────────────────────────────── */

enum : uint8_t { kPagePlay = 0, kPageBody, kPageBox, kNumPages };

/* ── Play ────────────────────────────────────────────────────────────── */

static VirtualKnob k_pitch = VirtualKnob(kPotTopLeft, "Pitch")
    .Exp(kPitchMinHz, kPitchMaxHz).Ident("play.pitch").Unit("Hz")
    .Ring(Level(kPlayOn).Passive(kPlayOff));

static VirtualKnob k_pressure = VirtualKnob(kPotTopRight, "Pressure")
    .Linear(0.f, kPressureMax).Ident("play.pressure")
    .Ring(Level(kPlayOn).Passive(kPlayOff));

static VirtualKnob k_speed = VirtualKnob(kPotMiddleLeft, "Bow Speed")
    .Linear(-1.f, 1.f).Ident("play.speed")
    .Ring(Bipolar(kPlayOn, kPlayOn, kPlayOff));

static VirtualKnob k_pos = VirtualKnob(kPotMiddleRight, "Position")
    .Linear(kPosMin, kPosMax).Ident("play.position")
    .Ring(Level(kPlayOn).Passive(kPlayOff));

static VirtualKnob k_hard = VirtualKnob(kPotBottomLeft, "Hardness")
    .Linear(0.f, 1.f).Ident("play.hardness")
    .Ring(Level(kPlayOn).Passive(kPlayOff));

static VirtualKnob k_vel = VirtualKnob(kPotBottomRight, "Velocity")
    .Linear(0.05f, 1.f).Ident("play.velocity")
    .Ring(Level(kPlayOn).Passive(kPlayOff));

/* ── Body ────────────────────────────────────────────────────────────── */

static const char* const kMaterialLabels[cd::kNumMaterials] = { "String", "Bar", "Plate" };

static VirtualKnob k_material = VirtualKnob(kPotTopLeft, "Material")
    .Selector(cd::kNumMaterials).Labels(kMaterialLabels).Ident("body.material")
    .Ring(SelectorRing(kBodyOn, kBodyOff, cd::kNumMaterials));

static VirtualKnob k_stiff = VirtualKnob(kPotTopRight, "Stiffness")
    .Linear(0.f, 1.f).Ident("body.stiffness")
    .Ring(Level(kBodyOn).Passive(kBodyOff));

static VirtualKnob k_decay = VirtualKnob(kPotMiddleLeft, "Decay")
    .Exp(kDecayMinS, kDecayMaxS).Ident("body.decay").Unit("s")
    .Ring(Level(kBodyOn).Passive(kBodyOff));

static VirtualKnob k_hiloss = VirtualKnob(kPotMiddleRight, "High Loss")
    .Linear(0.f, 1.f).Ident("body.hiloss")
    .Ring(Level(kBodyOn).Passive(kBodyOff));

static VirtualKnob k_pickup = VirtualKnob(kPotBottomLeft, "Pickup")
    .Linear(0.02f, 0.98f).Ident("body.pickup")
    .Ring(Level(kBodyOn).Passive(kBodyOff));

static VirtualKnob k_couple = VirtualKnob(kPotBottomRight, "Couple")
    .Linear(0.f, 1.f).Ident("body.couple")
    .Ring(Level(kBodyOn).Passive(kBodyOff));

/* ── Box ─────────────────────────────────────────────────────────────── */

static VirtualKnob k_box = VirtualKnob(kPotTopLeft, "Box")
    .Linear(0.f, 1.f).Ident("box.amount")
    .Ring(Level(kBoxOn).Passive(kBoxOff));

static VirtualKnob k_size = VirtualKnob(kPotTopRight, "Box Size")
    .Linear(0.f, 1.f).Ident("box.size")
    .Ring(Level(kBoxOn).Passive(kBoxOff));

static VirtualKnob k_rosin = VirtualKnob(kPotMiddleLeft, "Rosin")
    .Linear(0.f, 1.f).Ident("box.rosin")
    .Ring(Level(kBoxOn).Passive(kBoxOff));

static VirtualKnob k_level = VirtualKnob(kPotMiddleRight, "Level")
    .Linear(0.f, 1.f).Ident("box.level")
    .Ring(Level(kBoxOn).Passive(kBoxOff));

static Page page_play = Page(kPagePlay).Name("Play").Color("#ffa820")
    .Knobs(k_pitch, k_pressure, k_speed, k_pos, k_hard, k_vel);

static Page page_body = Page(kPageBody).Name("Body").Color("#30e0ff")
    .Knobs(k_material, k_stiff, k_decay, k_hiloss, k_pickup, k_couple);

static Page page_box = Page(kPageBox).Name("Box").Color("#ff5090")
    .Knobs(k_box, k_size, k_rosin, k_level);

static Page* const kPages[] = { &page_play, &page_body, &page_box };

/* ── Jacks ───────────────────────────────────────────────────────────── */

static Jack j_couple("COUPLE", "Couple In", JackSig::AudioIn);
static Jack j_strike("STRIKE", "Strike",    JackSig::Trig);
static Jack j_pitch ("PITCH",  "Pitch",     JackSig::Voct);
static Jack j_bow   ("BOW",    "Bow",       JackSig::Gate);
static Jack j_speed ("SPEED",  "Bow Speed", JackSig::CvBi);
static Jack j_pos   ("POS",    "Position",  JackSig::CvBi);
static Jack j_env   ("ENV",    "Envelope",  JackSig::CvUni);
static Jack j_bright("BRIGHT", "Brightness",JackSig::CvUni);
static Jack j_out   ("OUT",    "Out",       JackSig::AudioOut);
static Jack j_bridge("BRIDGE", "Bridge",    JackSig::AudioOut);

/* Upper row J1 J3 J5 J7 J9, then J2 J4 J6 J8 J10. */
static const Jack* const kJackOrder[] = {
    &j_couple, &j_pitch, &j_speed, &j_env,    &j_out,
    &j_strike, &j_bow,   &j_pos,   &j_bright, &j_bridge,
};

static const char* const kBtnLabels[kNumPages * 3] = {
    "PAGE", "STRIKE", "BOW",
    "PAGE", "STRIKE", "BOW",
    "PAGE", "STRIKE", "BOW",
};

/* ── SDK surfaces ────────────────────────────────────────────────────── */

static AlchemyLab  hw;
static ControlLoop loop(hw);
static Pager       pager(kNumPages, kNumPots);
static Presets     presets(hw.seed.qspi);

/* To debug the panel link, pass panel_i2c::Diag() instead of "i2c". */
static hostlink::Host host(presets, "chladni", "Chladni", "0.1.1", "i2c");

static cd::Instrument inst;

/* ── Shared state ────────────────────────────────────────────────────── */

/* B2's strike, as a request counter the audio callback acknowledges, so the
 * engine's strike has exactly one producer (the audio thread). */
static volatile uint32_t strike_req_ = 0;
static volatile float    strike_req_vel_ = 0.5f;
static uint32_t          strike_ack_ = 0;     /* audio thread */

/* Strike detector on J2. Audio thread only. */
static bool    trig_armed_ = false;
static bool    trig_hi_    = false;
static float   trig_peak_  = 0.f;
static int32_t trig_count_ = 0;
static int32_t trig_refr_  = 0;
static int32_t peak_win_   = 48;
static int32_t refr_len_   = 288;

/* Peak |COUPLE in| since the control thread last took it. */
static volatile float couple_peak_ = 0.f;

/* Control thread only from here down. */
static bool     bow_latched_   = false;
static bool     bow_pressing_  = false;
static bool     bow_unlatching_= false;
static uint32_t bow_press_ms_  = 0;
static bool     bow_requested_ = false;
static float    pitch_set_hz_  = 0.f;
static uint32_t strikes_seen_  = 0;
static uint32_t strike_flash_until_ = 0;
static float    bowq_ = 0.f;              /* slips per period, last frame */
static uint32_t slips_seen_ = 0;
static uint32_t slips_ms_   = 0;
static float    pos_now_    = 0.1f;      /* knob + POS CV, for the panel */
static float inst_pos_now() { return pos_now_ / kPosMax; }

/* ── Audio ───────────────────────────────────────────────────────────── */

static void Audio(daisy::AudioHandle::InputBuffer  in,
                  daisy::AudioHandle::OutputBuffer out,
                  size_t                           n)
{
    float cpk = couple_peak_;
    for (size_t i = 0; i < n; i++)
    {
        const float c = fabsf(in[0][i]);
        if (c > cpk) cpk = c;

        const float x = in[1][i];
        if (trig_refr_ > 0) trig_refr_--;
        const bool hi = x > kTrigThresh;
        if (trig_armed_)
        {
            if (x > trig_peak_) trig_peak_ = x;
            if (--trig_count_ <= 0)
            {
                float v = (trig_peak_ * kJackFullScaleV) / kFullVelVolts;
                v = v < kMinVel ? kMinVel : v > 1.f ? 1.f : v;
                inst.Strike(v);
                trig_armed_ = false;
                trig_refr_  = refr_len_;
            }
        }
        else if (hi && !trig_hi_ && trig_refr_ == 0)
        {
            trig_armed_ = true;
            trig_peak_  = x;
            trig_count_ = peak_win_;
        }
        trig_hi_ = hi;
    }
    couple_peak_ = cpk;

    const uint32_t req = strike_req_;
    if (req != strike_ack_)
    {
        strike_ack_ = req;
        inst.Strike(strike_req_vel_);
    }

    inst.Process(in[0], out[0], out[1], n);
}

/* ── Panel bus ───────────────────────────────────────────────────────── */

static constexpr uint8_t kEvtStat  = 0x50;
static constexpr uint8_t kEvtModes = 0x51;
static constexpr uint8_t kEvtGeom  = 0x52;

/* Send only onto a quiet ring: panel_i2c queues STATE under 64 bytes. */
static constexpr uint16_t kStatGate  = 32;
static constexpr uint16_t kChunkGate = 48;
static constexpr uint8_t  kChunksPerFrame = 2;
static constexpr uint8_t  kModesPerChunk  = 3;
static constexpr uint8_t  kNumChunks      = cd::kNumModes / kModesPerChunk;   /* 8 */
static_assert(cd::kNumModes % kModesPerChunk == 0, "MODES chunks must tile the modes");
static constexpr uint32_t kGeomRefreshMs  = 1000u;

static uint8_t  snap_amp_[cd::kNumModes];
static uint8_t  snap_ph_[cd::kNumModes];
static uint8_t  snap_level_ = 0;
static uint8_t  snap_seq_   = 0;
static uint8_t  chunk_      = kNumChunks;    /* = take a new snapshot */
static uint8_t  geom_sent_[8];
static uint32_t geom_sent_ms_ = 0;

static constexpr float kTwoPi = 6.28318530718f;

static uint8_t LogByte(float ratio)
{
    if (ratio <= 0.f) return 0;
    const float b = 255.f + 60.f * log10f(ratio);
    return b <= 0.f ? 0 : b >= 255.f ? 255 : (uint8_t)(b + 0.5f);
}

static void TakeSnapshot(const cd::Telemetry& t)
{
    float amp[cd::kNumModes], ph[cd::kNumModes], amax = 0.f;
    for (uint8_t i = 0; i < cd::kNumModes; i++)
    {
        const float x = t.mode_x[i], y = t.mode_y[i];
        amp[i] = sqrtf(x * x + y * y);
        ph[i]  = atan2f(-y, x);
        if (amp[i] > amax) amax = amp[i];
    }
    for (uint8_t i = 0; i < cd::kNumModes; i++)
    {
        snap_amp_[i] = amax > 0.f ? LogByte(amp[i] / amax) : 0;
        float p = ph[i] - (float)(i + 1) * ph[0];
        p -= kTwoPi * floorf(p / kTwoPi);
        snap_ph_[i] = (uint8_t)((int)(p * (256.f / kTwoPi)) & 0xFF);
    }
    snap_level_ = LogByte(amax / 0.01f);
    snap_seq_ = (uint8_t)((snap_seq_ + 1u) & 0x1Fu);
    chunk_ = 0;
}

static uint8_t Byte01(float u)
{
    return u <= 0.f ? 0 : u >= 1.f ? 255 : (uint8_t)(u * 255.f + 0.5f);
}

/* The bus carries about two 31-byte reads per lab per 16 ms frame at
 * 100 kHz (measured with chladni5: ~117 reads/s). STAT plus two chunks
 * every frame (~53 bytes with framing) outran that, the ring backed up and
 * whole frames were skipped (STAT 27/s, 5-6 snapshots/s). STAT goes every
 * other frame instead; the chunks keep going every frame. */
static uint8_t frame_parity_ = 0;

static void SendChunks(const cd::Telemetry& t)
{
    for (uint8_t k = 0; k < kChunksPerFrame; k++)
    {
        if (chunk_ >= kNumChunks) TakeSnapshot(t);
        if (panel_i2c::Backlog() >= kChunkGate) return;
        const uint8_t base = (uint8_t)(chunk_ * kModesPerChunk);
        uint8_t b[8] = { kEvtModes, (uint8_t)(chunk_ | (snap_seq_ << 3)) };
        for (uint8_t i = 0; i < kModesPerChunk; i++)
        {
            b[2 + 2 * i] = snap_amp_[base + i];
            b[3 + 2 * i] = snap_ph_[base + i];
        }
        if (!panel_i2c::SendEvent(b, sizeof b)) return;
        chunk_++;
    }
}

static void SendDisplay(uint32_t now, const cd::Telemetry& t)
{
    frame_parity_ ^= 1u;
    if (frame_parity_) { SendChunks(t); return; }
    if (panel_i2c::Backlog() >= kStatGate) return;

    const uint8_t material = static_cast<uint8_t>(k_material.Value());
    const bool struck = t.strikes != strikes_seen_;
    const uint8_t flags = (uint8_t)((struck ? 1u : 0u) | (t.bowing ? 2u : 0u)
                        | (bow_requested_ ? 4u : 0u) | (bow_latched_ ? 8u : 0u)
                        | ((material & 3u) << 4));
    float cpk = couple_peak_;
    couple_peak_ = 0.f;
    const uint16_t hz16 = (uint16_t)fminf(t.pitch_hz * 16.f, 65535.f);
    const float bq = t.bowing ? bowq_ * 32.f : 0.f;
    const uint8_t stat[8] = { kEvtStat, flags, (uint8_t)(hz16 & 0xFF), (uint8_t)(hz16 >> 8),
                              Byte01(t.envelope), (uint8_t)(bq > 255.f ? 255 : bq),
                              Byte01(cpk), snap_level_ };
    if (!panel_i2c::SendEvent(stat, sizeof stat)) return;
    strikes_seen_ = t.strikes;

    /* Geometry, when it changes (or as a refresh for a panel that booted
     * after the module). */
    const uint8_t geom[8] = { kEvtGeom, material,
                              Byte01(inst_pos_now()), Byte01(k_pickup.Value()),
                              Byte01(k_stiff.Value()), Byte01(k_box.Value()),
                              Byte01(k_size.Value()), Byte01(k_hiloss.Value()) };
    if ((memcmp(geom, geom_sent_, sizeof geom) != 0 || now - geom_sent_ms_ >= kGeomRefreshMs)
        && panel_i2c::Backlog() < kChunkGate && panel_i2c::SendEvent(geom, sizeof geom))
    {
        memcpy(geom_sent_, geom, sizeof geom);
        geom_sent_ms_ = now;
        return;   /* that was this frame's chunk budget, near enough */
    }

    SendChunks(t);
}

/* ── Control ─────────────────────────────────────────────────────────── */

static float Clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }

/*
 * 1 ms: pitch, the bow (B3 and J4), speed and position CV, B2, the CV
 * outputs. Pitch and the bow gate are here and not on the 16 ms frame
 * because a sequencer moves them together; a frame late is a wrong note.
 */
static void OnPoll(uint32_t now)
{
    /* Pitch, with a little hysteresis so CV noise does not retune the
     * modes every block. */
    const float v  = Clampf(hw.cv[kCvPitch].Volts(), kPitchCvMinV, kPitchCvMaxV);
    const float hz = k_pitch.Value() * exp2f(v);
    if (pitch_set_hz_ <= 0.f || fabsf(1200.f * log2f(hz / pitch_set_hz_)) > kRetuneCents)
    {
        pitch_set_hz_ = hz;
        inst.SetPitchHz(hz);
    }

    /* B3: hold to bow, tap to latch, tap again to lift. */
    IButton& b3 = hw.buttons[kButtonB3];
    if (b3.RisingEdge())
    {
        if (bow_latched_) { bow_latched_ = false; bow_unlatching_ = true; }
        else              { bow_pressing_ = true; bow_press_ms_ = now; }
    }
    if (b3.FallingEdge())
    {
        if (bow_pressing_ && (now - bow_press_ms_) < kBowTapMs) bow_latched_ = true;
        bow_pressing_ = false;
        bow_unlatching_ = false;
    }

    /* J4: gate and pressure. */
    const float bv = hw.cv[kCvBow].Volts();
    const bool  jack_on = bv > kBowOnV;
    const bool  button_on = bow_latched_ || bow_pressing_;
    bow_requested_ = jack_on || button_on;
    float p = 0.f;
    if (bow_requested_)
    {
        const float scale = jack_on ? Clampf(bv / kBowFullV, 0.f, kBowMaxScale) : 1.f;
        p = k_pressure.Value() * scale;
    }
    inst.SetBowPressure(p);
    inst.SetBowSpeed(Clampf(k_speed.Value() + hw.cv[kCvSpeed].Volts() / 5.f, -1.f, 1.f));

    pos_now_ = Clampf(k_pos.Value() + hw.cv[kCvPos].Volts() * kPosPerVolt, kPosMin, kPosMax);
    inst.SetPosition(pos_now_);

    /* B2 strikes at the Velocity knob. */
    if (hw.buttons[kButtonB2].RisingEdge())
    {
        strike_req_vel_ = k_vel.Value();
        strike_req_ = strike_req_ + 1;
    }

    /* Outputs: the envelope, and brightness as octaves over the fundamental. */
    const cd::Telemetry& t = inst.Snapshot();
    hw.cv[kCvEnv].SetVolts(Clampf(t.envelope, 0.f, 1.f) * kEnvOutV);
    const float br = t.centroid_hz > 0.f && t.pitch_hz > 0.f
                   ? log2f(t.centroid_hz / t.pitch_hz) : 0.f;
    hw.cv[kCvBright].SetVolts(Clampf(br, 0.f, kBrightMaxV));
}

/* B2 and B3 lights. On the render hook, after the loop paints B1's page
 * indicator, so nothing overwrites them. */
static void OnRender(uint32_t now)
{
    hw.leds.SetButtonPair(kButtonB2,
        (int32_t)(strike_flash_until_ - now) > 0 ? kStrikeFlash : kStrikeIdle);

    Rgb b3 = kBowIdle;
    if (bow_requested_)
    {
        if (bowq_ < 0.05f)      b3 = kBowLatch;     /* down, not speaking yet */
        else if (bowq_ > 1.3f)  b3 = kBowMulti;
        else if (bowq_ < 0.7f)  b3 = kBowRaucous;
        else                    b3 = kBowClean;
    }
    hw.leds.SetButtonPair(kButtonB3, b3);
}

static void OnFrame()
{
    inst.SetMaterial(static_cast<uint8_t>(k_material.Value()));
    inst.SetStiffness(k_stiff.Value());
    inst.SetDecay(k_decay.Value());
    inst.SetHighLoss(k_hiloss.Value());
    inst.SetPickup(k_pickup.Value());
    inst.SetCouple(k_couple.Value());
    inst.SetHardness(k_hard.Value());
    inst.SetBody(k_box.Value());
    inst.SetBodySize(k_size.Value());
    inst.SetRosin(k_rosin.Value());
    inst.SetLevel(k_level.Value());

    const uint32_t now = daisy::System::GetNow();
    const cd::Telemetry t = inst.Snapshot();

    /* Bow quality: slips per period over the last frame. */
    const uint32_t dms = now - slips_ms_;
    if (dms >= 16u)
    {
        const float periods = (float)dms * 0.001f * t.pitch_hz;
        bowq_ = periods > 0.f ? (float)(t.slips - slips_seen_) / periods : 0.f;
        slips_seen_ = t.slips;
        slips_ms_   = now;
    }
    if (t.strikes != strikes_seen_) strike_flash_until_ = now + kStrikeFlashMs;

    panel_i2c::Tick();
    SendDisplay(now, t);
}

/* ── Manual ──────────────────────────────────────────────────────────── */

static Manual manual =
    Manual()
        .Tagline("A struck and bowed string, bar or plate")
        .Preamble(
            "**Chladni** is one physical instrument: a mallet and a bow "
            "acting at one point on a body of 24 vibrating modes. The bow's "
            "friction and the mallet's contact are solved against the body "
            "on every sample, so a bow settles into real Helmholtz motion "
            "and a mallet bounces the way it would.\n\n"
            "**B2** strikes. **B3** is the bow: hold it to bow, tap it to "
            "leave the bow down, tap again to lift. Its light says what the "
            "bow is doing — **green** is a clean tone, **amber** is slipping "
            "too often (too light, or still finding the note), **red** is "
            "pressed too hard.\n\n"
            "Run it on two labs and patch each **BRIDGE** into the other's "
            "**COUPLE**: they become two instruments on one soundboard, and "
            "a note on one wakes the partials it shares with the other.")
        .Section("play", "Play",
                 "**Pitch** with 1 V/oct on PITCH. **Pressure** and **Bow "
                 "Speed** are the bow; speed is bipolar, its sign is the bow's "
                 "direction. Pressure holds a clean tone from about a third to "
                 "two thirds of the knob, wherever the bow is and however "
                 "fast it moves. **Position** is where the bow and mallet "
                 "touch, from the very end (bright, glassy) to the middle "
                 "(round, hollow). **Hardness** runs from felt to steel, "
                 "**Velocity** is how hard B2 strikes.")
        .Section("body", "Body",
                 "**Material**: a **String** (harmonic), a **Bar** (a "
                 "free-free beam, 1 : 2.76 : 5.40 ...) or a **Plate** (a "
                 "slightly rectangular plate whose mode pairs beat). "
                 "**Stiffness** stretches the partials sharp, from harmonic "
                 "towards metal — keep it low for bowing a string. **Decay** "
                 "is how long the fundamental rings, **High Loss** how much "
                 "faster the upper partials die; bowing likes it high. "
                 "**Pickup** is where the dry output listens. **Couple** is "
                 "how strongly the COUPLE input shakes this body's bridge; "
                 "its top is capped so two coupled labs can never feed back "
                 "into a howl, whatever the cable.")
        .Section("box", "Box",
                 "**Box** crossfades the dry pickup into the bridge heard "
                 "through a wooden box — thirteen violin-family resonances. "
                 "**Box Size** scales it from violin to cello. **Rosin** adds "
                 "the grain of real bow hair and a player's unsteady hand. "
                 "**Level** is the output level.")
        .Section("jacks", "Jacks",
                 "**STRIKE**: a trigger whose height is the velocity (4 V is "
                 "full). **BOW**: over 0.3 V puts the bow down and the voltage "
                 "scales Pressure — a 5 V gate bows at the knob's pressure, an "
                 "envelope swells it. **SPEED** and **POS** add to their "
                 "knobs. **ENV** puts out the envelope, 0-8 V; **BRIGHT** the "
                 "spectral centroid as octaves above the fundamental, 1 V "
                 "each. **OUT** is the instrument; **BRIDGE** is what the "
                 "other lab's COUPLE wants.");

static void DescribeManual()
{
    page_play.Help("The exciter: pitch, the bow, where and how hard it is struck.");
    page_body.Help("The body: material, stiffness, decay, pickup, coupling.");
    page_box.Help("The wooden box, rosin, and output level.");

    k_pitch.Help("Fundamental. PITCH adds 1 V/oct.").SeeAlso(j_pitch);
    k_pressure.Help("Bow pressure. Clean from about a third to two thirds.").SeeAlso(j_bow);
    k_speed.Help("Bow speed; the sign is its direction. SPEED adds.").SeeAlso(j_speed);
    k_pos.Help("Where the bow and mallet touch: end (bright) to middle (hollow).").SeeAlso(j_pos);
    k_hard.Help("Mallet: felt (4 ms contact) to steel (0.15 ms).");
    k_vel.Help("How hard B2 strikes.");

    k_material.Help("String, Bar or Plate.");
    k_stiff.Help("Stretches the partials sharp. Keep low to bow a string.");
    k_decay.Help("Ring time of the fundamental.");
    k_hiloss.Help("Extra loss for the upper partials. High suits bowing.");
    k_pickup.Help("Where the dry output listens.");
    k_couple.Help("How strongly COUPLE shakes this body. Capped: never howls.").SeeAlso(j_couple);

    k_box.Help("Dry pickup to the bridge through a wooden box.");
    k_size.Help("Box from violin to cello.");
    k_rosin.Help("Bow-hair grain and an unsteady hand.");
    k_level.Help("Output level.").SeeAlso(j_out);

    j_couple.Help("The other lab's BRIDGE.").SeeAlso(k_couple);
    j_strike.Help("Trigger; its height is the velocity, 4 V full.");
    j_pitch.Help("1 V/oct.").SeeAlso(k_pitch);
    j_bow.Help("Over 0.3 V the bow is down; volts scale Pressure, 5 V = the knob.").SeeAlso(k_pressure);
    j_speed.Help("Adds to Bow Speed, +/-5 V = full.").SeeAlso(k_speed);
    j_pos.Help("Adds to Position.").SeeAlso(k_pos);
    j_env.Help("Output: the envelope, 0-8 V.");
    j_bright.Help("Output: spectral centroid, octaves over the fundamental.");
    j_out.Help("The instrument.").SeeAlso(k_box);
    j_bridge.Help("Patch to the other lab's COUPLE.").SeeAlso(j_couple);

    host.Jacks(j_couple, j_strike, j_pitch, j_bow, j_speed, j_pos, j_env,
               j_bright, j_out, j_bridge)
        .Attach(manual);
}

/* ── Glue ────────────────────────────────────────────────────────────── */

int main()
{
    hw.Init();
    inst.Init(hw.SampleRate());

    const float sr = hw.SampleRate();
    peak_win_ = (int32_t)(kPeakWindowMs * 0.001f * sr);
    refr_len_ = (int32_t)(kRefractoryMs * 0.001f * sr);
    if (peak_win_ < 1) peak_win_ = 1;

    pager.Cycle(hw.buttons[kButtonB1], kPagePlay, kPageBody, kPageBox);

    /* J7 and J8 become outputs — the STM32's own DAC, the fast pair. */
    hw.cv[kCvEnv].EnableCvOutput();
    hw.cv[kCvBright].EnableCvOutput();

    DescribeManual();

    presets.Manage(pager);
    presets.Init();
    presets.BootLoad();

    static panel_i2c::Config disp_cfg;
    disp_cfg.hw          = &hw;
    disp_cfg.pager       = &pager;
    disp_cfg.pages       = kPages;
    disp_cfg.num_pages   = kNumPages;
    disp_cfg.module_name = "Chladni";
    disp_cfg.jacks       = kJackOrder;
    disp_cfg.num_jacks   = (uint8_t)(sizeof(kJackOrder) / sizeof(kJackOrder[0]));
    disp_cfg.btn_labels  = kBtnLabels;
    disp_cfg.address     = panel_i2c::kDefaultAddress;   /* make PANEL_I2C_ADDR= */
    panel_i2c::Init(disp_cfg);

    OnFrame();
    hw.StartAudio(Audio);

    loop.Use(pager)
        .Use(host)
        .Use(page_play)
        .Use(page_body)
        .Use(page_box)
        .OnPoll(OnPoll)
        .OnFrame(OnFrame)
        .OnRender(OnRender)
        .OnPageChange(panel_i2c::PageChanged);

    for (;;) loop.Tick();
}
