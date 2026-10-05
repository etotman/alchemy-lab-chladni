/**
 * chladni_dsp.h — A struck and bowed modal body, solved with zero delay.
 *
 * One instrument per lab: an exciter (a Hunt-Crossley mallet and a
 * stick-slip bow, both acting at one point) driving a body of 24 modes
 * (string, bar or plate). Two labs patched bridge-out to coupling-in
 * become two instruments sharing a soundboard: each one's bridge shakes
 * the other's.
 *
 * Why the exciter and the body live in the same callback
 * ──────────────────────────────────────────────────────
 * A bow or a mallet needs the body's velocity at the contact point in the
 * *same sample* it computes the force from it. Each mode is a Simper/
 * Zavalishin state-variable filter, whose response to this sample's input
 * splits into a part fixed by its state and a part linear in the input:
 *
 *     v_contact = v_free + Y * F        y_contact = y_free + D * F
 *
 * Y and D are known the moment the modes are built, so the friction and
 * contact laws become scalar equations in F, solved per sample by a
 * bracketed Newton iteration. There is no delay anywhere in the loop, which
 * is what lets a bow lock into Helmholtz motion and a mallet bounce
 * correctly. Splitting exciter and body across two labs (an analog loop of
 * a few milliseconds) cannot do either.
 *
 * Why the coupling between labs cannot run away
 * ─────────────────────────────────────────────
 * The cable loop between two labs (bridge out -> coupling in, both ways)
 * is delayed and its gain is whatever the patch makes it, so stability is
 * guaranteed by the small-gain theorem rather than by a clipper: each lab
 * caps its own gain from coupling-in to bridge-out at kCoupleCeiling (< 1)
 * at its most resonant frequency. The product around the loop is then
 * below 1 whatever the other lab is doing and whatever the delay. A tanh
 * on the input bounds amplitude as well, but it is the cap, not the tanh,
 * that stops the pair singing on its own.
 *
 * The DSP knows nothing about libDaisy, so the same source builds natively
 * into tools/chladni_render and is judged by ear before it is flashed.
 *
 * Threading: Set*() and Strike() are control-thread calls that write plain
 * words; Process() and everything it touches belong to the audio thread.
 * Strike() is a request counter the audio thread acknowledges, so it is
 * safe from one producer thread.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace chladni_dsp {

#ifndef CHLADNI_MODES
#define CHLADNI_MODES 24
#endif
constexpr uint8_t kNumModes = CHLADNI_MODES;

/** Most of its own coupling-in -> bridge-out gain a lab may use. The loop
 *  through two labs is then at most kCoupleCeiling^2 < 1, so a ring always
 *  dies away. (Raising it while bowing, to pull two bows into tune, was
 *  tried at 3x: by ear the result was unusable. Don't retry.) */
#ifndef CHLADNI_COUPLE_CEILING
#define CHLADNI_COUPLE_CEILING 0.85f
#endif
constexpr float kCoupleCeiling = CHLADNI_COUPLE_CEILING;

/** Bow force follows bow speed (so one pressure setting stays playable at
 *  any speed), but through a fast-rise, slow-release follower: a player
 *  keeps pressing through a bow change, so the grip must not collapse as
 *  the speed passes through zero. CHLADNI_FN_LAW 0 = instantaneous speed
 *  (the old law), 1 = follower, 2 = fixed reference speed. */
#ifndef CHLADNI_FN_LAW
#define CHLADNI_FN_LAW 1
#endif
#ifndef CHLADNI_FN_RELEASE
#define CHLADNI_FN_RELEASE 0.25f
#endif

/** Fixed body (soundbox) resonances, driven by the bridge. */
constexpr uint8_t kBodyModes = 13;

enum Material : uint8_t {
    kString = 0,  /* f_n = n f0 sqrt(1 + B n^2), shapes sin(n pi x)       */
    kBar,         /* free-free beam, f_n ~ beta_n^2, 1 : 2.76 : 5.40 ...   */
    kPlate,       /* simply supported, slightly rectangular, (m, n) pairs  */
    kNumMaterials
};

const char* MaterialName(uint8_t m);

/** Telemetry for the panel and the CV outputs. Written by the audio thread
 *  once per block; read it with Instrument::Snapshot(). */
struct Telemetry {
    float   pitch_hz;          /* fundamental after 1 V/oct and stiffness   */
    float   contact_force;     /* exciter force this block (peak |F|)       */
    float   displacement;      /* body displacement at the contact point    */
    float   velocity;          /* body velocity at the contact point        */
    float   mode_energy[kNumModes]; /* per-mode energy, block end           */
    /* Each mode's state at block end in displacement units: q = A cos(phi)
     * is mode_x, -A sin(phi) is mode_y (velocity / omega). The panel
     * rebuilds the body's shape and the contact point's phase portrait
     * from these. */
    float   mode_x[kNumModes];
    float   mode_y[kNumModes];
    float   mode_hz[kNumModes];
    uint8_t mode_m[kNumModes];  /* plate (m, n); string/bar: (n, 0)          */
    uint8_t mode_n[kNumModes];
    float   centroid_hz;       /* energy-weighted mean mode frequency       */
    float   envelope;          /* output peak follower, 0..~1               */
    bool    in_contact;        /* mallet touching the body                  */
    bool    sticking;          /* bow stuck to the body                     */
    bool    bowing;            /* bow force on                              */
    uint32_t slips;            /* stick -> slip transitions, ever           */
    uint32_t strikes;          /* mallet impacts, ever                      */
};

class Instrument {
public:
    void Init(float sample_rate);

    /* ── Body ───────────────────────────────────────────────────────── */
    void SetMaterial(uint8_t m);
    void SetPitchHz(float hz);          /* f of mode 1                    */
    void SetStiffness(float u);         /* 0 harmonic .. 1 metallic       */
    void SetDecay(float t60_s);         /* T60 of mode 1                  */
    void SetHighLoss(float u);          /* 0 .. 1: extra loss above 1 kHz */
    void SetPosition(float p);          /* exciter point 0.02 .. 0.5      */
    void SetPickup(float p);            /* output pickup point 0.02 .. 0.98 */

    /* ── Mallet ─────────────────────────────────────────────────────── */
    void SetHardness(float u);          /* 0 felt (4 ms) .. 1 steel (0.15 ms) */
    void Strike(float velocity);        /* 0 .. 1, control thread          */

    /* ── Bow ────────────────────────────────────────────────────────── */
    void SetBowPressure(float p);       /* 0 = off; 1 = Schelleng's maximum */
    void SetBowSpeed(float v);          /* -1 .. 1 (sign = bow direction) */

    /* ── Coupling to the other lab ──────────────────────────────────── */
    void SetCouple(float u);            /* 0 .. 1 of kCoupleCeiling       */

    /* ── Player and box ─────────────────────────────────────────────── */
    void SetRosin(float u);             /* 0 clean .. 1: friction noise + bow drift */
    void SetBody(float u);              /* 0 pickup only .. 1 bridge through the box */
    void SetBodySize(float u);          /* 0 violin .. 1 cello (box scaled 0.42x)    */

    void SetLevel(float u);             /* output level, 0 .. 1           */

    /** in = coupling input (the other lab's bridge), out = pickup,
     *  bridge = this lab's bridge velocity for the other lab. Any of the
     *  pointers may alias nothing else; n is the block length. */
    void Process(const float* in, float* out, float* bridge, size_t n);

    Telemetry Snapshot() const { return tel_; }

    /* For tools: the gain bound actually in force, and the loop it allows. */
    float CoupleGain() const { return kappa_; }
    float BridgePeakAdmittance() const { return y_bridge_max_; }

private:
    void Rebuild();
    float SolveMallet(float y_free);
    float SolveBow(float v_free);

    float sr_ = 48000.f, T_ = 1.f / 48000.f;

    /* Parameters (control thread writes, audio thread reads). */
    volatile uint8_t material_ = kString;
    volatile float pitch_ = 110.f, stiff_ = 0.f, decay_ = 3.f, hiloss_ = 0.3f;
    volatile float pos_ = 0.13f, pickup_ = 0.77f;
    volatile float hard_ = 0.5f;
    volatile float bow_p_ = 0.f, bow_v_ = 0.f;
    volatile float couple_ = 0.f, level_ = 0.7f;
    volatile float rosin_ = 0.f, body_ = 0.f, body_size_ = 0.f;
    volatile uint32_t dirty_ = 1, built_ = 0;
    volatile uint32_t strike_req_ = 0;
    volatile float strike_vel_ = 0.f;
    uint32_t strike_ack_ = 0;

    /* Modes. */
    float g_[kNumModes], k1_[kNumModes], h_[kNumModes];
    float inv_w_[kNumModes], inv_w2_[kNumModes];
    float a_[kNumModes], b_[kNumModes], c_[kNumModes];   /* exciter, pickup, bridge */
    float s1_[kNumModes], s2_[kNumModes];
    float hz_[kNumModes];
    uint8_t mm_[kNumModes], mn_[kNumModes];
    float Y_ = 0.f, D_ = 0.f, torsion_y_ = 0.f;          /* contact admittance (v, y per F) */
    float y_bridge_max_ = 1.f, kappa_ = 0.f, kappa_s_ = 0.f;
    float out_gain_ = 1.f, bridge_gain_ = 1.f, bow_fscale_ = 1.f;

    /* Mallet state. */
    bool  mallet_live_ = false;
    float mh_ = 0.05f, mk_ = 1.f, mp_ = 1.5f, mlam_ = 0.f;
    float yh_ = 0.f, uh_ = 0.f, delta_prev_ = 0.f, mallet_age_ = 0.f;
    bool  touching_ = false;

    /* Bow state. */
    float dv_prev_ = 0.f, bow_fn_s_ = 0.f, bow_v_s_ = 0.f;
    bool  stuck_ = false;

    float env_ = 0.f;

    /* Rosin, drift, and the box. */
    uint32_t rng_ = 0x9E3779B9u;
    float rosin_lp_ = 0.f, drift_p_ = 0.f, drift_v_ = 0.f, bow_vref_ = 0.f;
    bool  bow_on_ = false; float bow_dir_ = 0.f, bite_ = 0.f;
    float sd1_[kNumModes], sd2_[kNumModes];   /* seed still to blend in */
    int   seed_left_ = 0;
    void SeedHelmholtz(float v);
    float body_s_ = 0.f;
    float bb0_[kBodyModes], ba1_[kBodyModes], ba2_[kBodyModes], bg_[kBodyModes];
    float bx1_ = 0.f, bx2_ = 0.f, by1_[kBodyModes], by2_[kBodyModes];
    float body_gain_ = 1.f, rosin_mul_ = 1.f;
    Telemetry tel_ = {};
};

} // namespace chladni_dsp
