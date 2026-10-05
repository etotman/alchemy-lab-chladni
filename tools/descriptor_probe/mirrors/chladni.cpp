/**
 * mirrors/chladni.cpp — Render src/chladni's HostLink descriptor.
 *
 * Nothing here touches hardware: the Alchemy SDK's descriptor layer is
 * board-free and compiles natively against lib/alchemy-sdk/stubs.  Run it
 * with tools/descriptor_probe/run.sh, which also validates the output.
 *
 * MAINTENANCE: the block between the MIRROR markers is a copy of
 * src/chladni/chladni.cpp's surfaces (knobs without their rings, pages,
 * jacks, the full manual and every Help line) — it is not compiled from
 * src/, so it can drift.  Update it whenever you add or rename a control.
 *
 * Three pages on B1, 16 knobs: the Pager is built with three pages.
 */

#include <cstdio>
#include <cstring>

#include "alchemy/host_link/describe.h"
#include "alchemy/host_link/descriptor.h"
#include "alchemy/hw/alchemy_lab.h"
#include "alchemy/surface/jack.h"
#include "alchemy/surface/manual.h"
#include "alchemy/surface/page.h"
#include "alchemy/surface/pager.h"
#include "alchemy/surface/presets.h"
#include "alchemy/surface/virtual_knob.h"

using namespace alchemy;
using namespace alchemy::hostlink;

static daisy::QSPIHandle g_qspi;   /* stub — never touched on the host */

/* ─────────────────── MIRROR: firmware surfaces begin ─────────────────── */

namespace cd {
constexpr uint8_t kNumMaterials = 3;
}

static const DescriptorBuilder::ModuleInfo kInfo = {
    "chladni", "Chladni", "0.1.2", "i2c", "host-probe", "v2"
};

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
    .Exp(kPitchMinHz, kPitchMaxHz).Ident("play.pitch").Unit("Hz");

static VirtualKnob k_pressure = VirtualKnob(kPotTopRight, "Pressure")
    .Linear(0.f, kPressureMax).Ident("play.pressure");

static VirtualKnob k_speed = VirtualKnob(kPotMiddleLeft, "Bow Speed")
    .Linear(-1.f, 1.f).Ident("play.speed");

static VirtualKnob k_pos = VirtualKnob(kPotMiddleRight, "Position")
    .Linear(kPosMin, kPosMax).Ident("play.position");

static VirtualKnob k_hard = VirtualKnob(kPotBottomLeft, "Hardness")
    .Linear(0.f, 1.f).Ident("play.hardness");

static VirtualKnob k_vel = VirtualKnob(kPotBottomRight, "Velocity")
    .Linear(0.05f, 1.f).Ident("play.velocity");

/* ── Body ────────────────────────────────────────────────────────────── */

static const char* const kMaterialLabels[cd::kNumMaterials] = { "String", "Bar", "Plate" };

static VirtualKnob k_material = VirtualKnob(kPotTopLeft, "Material")
    .Selector(cd::kNumMaterials).Labels(kMaterialLabels).Ident("body.material");

static VirtualKnob k_stiff = VirtualKnob(kPotTopRight, "Stiffness")
    .Linear(0.f, 1.f).Ident("body.stiffness");

static VirtualKnob k_decay = VirtualKnob(kPotMiddleLeft, "Decay")
    .Exp(kDecayMinS, kDecayMaxS).Ident("body.decay").Unit("s");

static VirtualKnob k_hiloss = VirtualKnob(kPotMiddleRight, "High Loss")
    .Linear(0.f, 1.f).Ident("body.hiloss");

static VirtualKnob k_pickup = VirtualKnob(kPotBottomLeft, "Pickup")
    .Linear(0.02f, 0.98f).Ident("body.pickup");

static VirtualKnob k_couple = VirtualKnob(kPotBottomRight, "Couple")
    .Linear(0.f, 1.f).Ident("body.couple");

/* ── Box ─────────────────────────────────────────────────────────────── */

static VirtualKnob k_box = VirtualKnob(kPotTopLeft, "Box")
    .Linear(0.f, 1.f).Ident("box.amount");

static VirtualKnob k_size = VirtualKnob(kPotTopRight, "Box Size")
    .Linear(0.f, 1.f).Ident("box.size");

static VirtualKnob k_rosin = VirtualKnob(kPotMiddleLeft, "Rosin")
    .Linear(0.f, 1.f).Ident("box.rosin");

static VirtualKnob k_level = VirtualKnob(kPotMiddleRight, "Level")
    .Linear(0.f, 1.f).Ident("box.level");

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

static Pager      pager(kNumPages, kNumPots);
static Presets    presets(g_qspi);

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

static void Compose()
{
    presets.Manage(pager);

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

}

static const Page* kPageRefs[] = { &page_play, &page_body, &page_box };
static const Jack* kJackRefs[] = {
    &j_couple, &j_strike, &j_pitch, &j_bow, &j_speed, &j_pos, &j_env,
    &j_bright, &j_out, &j_bridge,
};

/* ──────────────────── MIRROR: firmware surfaces end ──────────────────── */

int main()
{
    Compose();

    const PageSet pages{kPageRefs, sizeof kPageRefs / sizeof kPageRefs[0]};
    const uint8_t num_jacks = sizeof kJackRefs / sizeof kJackRefs[0];

    static char buf[24 * 1024];   /* same capacity the SDK gives the target */
    const uint32_t len = RenderDescriptor(
        buf, sizeof buf, kInfo, presets, &pages,
        nullptr, 0,               /* describe overrides */
        nullptr, 0,               /* buttons            */
        nullptr, 0,               /* root fragments     */
        kJackRefs, num_jacks, &manual,
        nullptr, 0);              /* factory defaults   */

    if (len == 0u)
    {
        std::fprintf(stderr,
                     "probe: RenderDescriptor returned 0 — the buffer could "
                     "not hold even a minimal error descriptor.\n");
        return 2;
    }

    std::fwrite(buf, 1, len, stdout);
    std::fputc('\n', stdout);
    return 0;
}
