/* chladni_dsp.cpp — see chladni_dsp.h for the model and why it is shaped
 * this way. Units are a self-consistent toy system: every mode has unit
 * modal mass, velocities are O(0.1..1), and the output gains below map
 * that onto the codec's ±1. */

#include "chladni_dsp.h"

#include <cmath>

#ifndef CHLADNI_OUT_TRIM
#define CHLADNI_OUT_TRIM 6.f       /* pickup velocity -> codec, measured   */
#endif
#ifndef CHLADNI_BODY_TRIM
#define CHLADNI_BODY_TRIM 15.f     /* bridge velocity through the box: measured, matches pickup +-4 dB */
#endif
#ifndef CHLADNI_BRIDGE_TRIM
#define CHLADNI_BRIDGE_TRIM 20.f   /* bridge velocity -> codec, measured   */
#endif

namespace chladni_dsp {

namespace {

constexpr float kPi     = 3.14159265358979f;
constexpr float kLn1000 = 6.90775527898f;

/* Plate: lowest 24 (m, n) of a simply supported plate of aspect kAspect
 * (Ly / Lx). Slightly off square so the degenerate (m,n)/(n,m) pairs split
 * into close, beating pairs instead of sitting on one frequency. */
constexpr float kAspect = 1.13f;
uint8_t  plate_m[kNumModes], plate_n[kNumModes];
float    plate_ratio[kNumModes];
bool     plate_built = false;

void BuildPlateTable()
{
    if (plate_built) return;
    struct P { uint8_t m, n; float v; } all[144];
    int k = 0;
    for (int m = 1; m <= 12; m++)
        for (int n = 1; n <= 12; n++)
            all[k++] = { (uint8_t)m, (uint8_t)n,
                         (float)(m * m) + (float)(n * n) / (kAspect * kAspect) };
    for (int i = 1; i < k; i++) {           /* insertion sort, once */
        P t = all[i];
        int j = i - 1;
        while (j >= 0 && all[j].v > t.v) { all[j + 1] = all[j]; j--; }
        all[j + 1] = t;
    }
    for (int i = 0; i < kNumModes; i++) {
        plate_m[i]     = all[i].m;
        plate_n[i]     = all[i].n;
        plate_ratio[i] = all[i].v / all[0].v;
    }
    plate_built = true;
}

/* Free-free beam: beta_n L. The first five are the roots of
 * cos(b) cosh(b) = 1; past that (2n+3) pi/2 is exact to 1e-7. */
float BarBeta(int i)
{
    static const float b[5] = { 4.73004f, 7.85320f, 10.99561f, 14.13717f, 17.27876f };
    return i < 5 ? b[i] : (2.f * (float)i + 3.f) * kPi * 0.5f;
}

/* Free-free beam shape, asymptotic form (max error 0.026 on mode 1, under
 * 1e-3 from mode 2 on), scaled to about ±1. */
float BarShape(int i, float x)
{
    const float be  = BarBeta(i);
    const float sgn = (i & 1) ? -1.f : 1.f;
    return 0.5f * (cosf(be * x) - sinf(be * x) + expf(-be * x)
                   + sgn * expf(-be * (1.f - x)));
}

/* Plate points. One Position knob walks the exciter from near the corner
 * (bright, every mode) towards the middle (odd modes only). */
inline void PlateExciter(float p, float& x, float& y) { x = p; y = 0.38f + 0.5f * p; }
inline void PlatePickup (float p, float& x, float& y) { x = p; y = 1.f - 0.6f * p; }
constexpr float kPlateBridgeX = 0.07f, kPlateBridgeY = 0.31f;
constexpr float kStringBridge = 0.03f;   /* near the termination: slope */
constexpr float kBarBridge    = 0.10f;

/* tanh, rational, |error| < 0.03, slope exactly 1 at 0 and never above 1. */
inline float SoftClip(float x)
{
    if (x > 3.f)  return 1.f;
    if (x < -3.f) return -1.f;
    const float x2 = x * x;
    return x * (27.f + x2) / (27.f + 9.f * x2);
}

/* Transparent below 0.6, u/(1+u) above, slope 1 at the knee. */
inline float Knee(float x)
{
    const float a = fabsf(x);
    if (a <= 0.6f) return x;
    const float u = (a - 0.6f) * 2.5f;
    const float y = 0.6f + 0.4f * u / (1.f + u);
    return x < 0.f ? -y : y;
}

/* Friction: a regularised Coulomb law with a Stribeck drop from static to
 * dynamic. mu(dv) is odd; it rises through zero over ~kEps and falls from
 * mu_s towards mu_d over ~kVs. The falling branch is the negative
 * resistance that lets a bow pump energy into the body. */
constexpr float kMuS = 0.8f, kMuD = 0.35f;
constexpr float kBowMaxV = 0.1f;     /* SetBowSpeed(1)               */
#ifndef CHLADNI_EPS
#define CHLADNI_EPS 3.3e-4f
#endif
#ifndef CHLADNI_VS
#define CHLADNI_VS 0.01f
#endif
#ifndef CHLADNI_TORSION
#define CHLADNI_TORSION 0.f
#endif
constexpr float kEps = CHLADNI_EPS;  /* stick regularisation width  */
constexpr float kVs  = CHLADNI_VS;   /* Stribeck velocity           */
constexpr float kBowFloor = 0.005f;  /* pressure scale at zero speed */

inline void Friction(float dv, float rosin, float& mu, float& dmu)
{
    const float x   = dv / kEps;
    const float r   = 1.f / sqrtf(1.f + x * x);
    const float sg  = x * r;                       /* smooth sign */
    const float dsg = r * r * r / kEps;
    const float z   = dv / kVs;
    const float q   = 1.f / (1.f + z * z);
    /* Rosin flicker rides on the sliding friction only: the static grip
     * (the q term at dv = 0) is untouched, so noise cannot knock the bow
     * loose mid-stick - it roughens the slip, which is where real bow
     * noise comes from. */
    const float m   = kMuD * rosin + (kMuS - kMuD) * q;
    const float dm  = -(kMuS - kMuD) * 2.f * dv / (kVs * kVs) * q * q;
    mu  = sg * m;
    dmu = dsg * m + sg * dm;
}

/* The box: violin-family body resonances at violin size, from the usual
 * signature modes (A0 air, CBR, B1-/B1+, the dense wood modes, then the
 * broad "bridge hill" near 2.5 kHz and the fall above it). Alternating
 * signs put anti-resonances between peaks, which is much of why a real
 * body sounds uneven rather than like an EQ. {Hz, Q, gain}. */
struct BodyMode { float hz, q, g; };
constexpr BodyMode kViolin[kBodyModes] = {
    {  275.f, 15.f,  0.60f }, {  405.f, 20.f, -0.30f }, {  470.f, 25.f,  0.90f },
    {  540.f, 25.f, -1.00f }, {  700.f, 20.f,  0.50f }, {  870.f, 18.f, -0.45f },
    { 1050.f, 18.f,  0.50f }, { 1300.f, 15.f, -0.40f }, { 1600.f, 12.f,  0.40f },
    { 2000.f,  6.f,  0.60f }, { 2500.f,  4.f,  0.70f }, { 3200.f,  4.f, -0.40f },
    { 4500.f,  3.f,  0.20f },
};
constexpr float kBodyDirect = 0.15f;

inline uint32_t XorShift(uint32_t& s) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
inline float Uni(uint32_t& s) { return (float)(int32_t)XorShift(s) * (1.f / 2147483648.f); } /* -1..1 */

} // namespace

const char* MaterialName(uint8_t m)
{
    switch (m) {
    case kString: return "String";
    case kBar:    return "Bar";
    case kPlate:  return "Plate";
    default:      return "?";
    }
}

/* ── Setters: write a word, bump dirty only on a real change ─────────── */

#define CHLADNI_SET(field, value)                     \
    do {                                              \
        const auto v_ = (value);                      \
        if (field != v_) { field = v_; dirty_++; }    \
    } while (0)

static inline float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : x > hi ? hi : x;
}

void Instrument::SetMaterial(uint8_t m)  { CHLADNI_SET(material_, (uint8_t)(m < kNumMaterials ? m : 0)); }
void Instrument::SetPitchHz(float hz)    { CHLADNI_SET(pitch_, Clamp(hz, 16.f, 4000.f)); }
void Instrument::SetStiffness(float u)   { CHLADNI_SET(stiff_, Clamp(u, 0.f, 1.f)); }
void Instrument::SetDecay(float t)       { CHLADNI_SET(decay_, Clamp(t, 0.02f, 30.f)); }
void Instrument::SetHighLoss(float u)    { CHLADNI_SET(hiloss_, Clamp(u, 0.f, 1.f)); }
void Instrument::SetPosition(float p)    { CHLADNI_SET(pos_, Clamp(p, 0.02f, 0.5f)); }
void Instrument::SetPickup(float p)      { CHLADNI_SET(pickup_, Clamp(p, 0.02f, 0.98f)); }
void Instrument::SetHardness(float u)    { CHLADNI_SET(hard_, Clamp(u, 0.f, 1.f)); }
void Instrument::SetCouple(float u)      { CHLADNI_SET(couple_, Clamp(u, 0.f, 1.f)); }
void Instrument::SetBowPressure(float p) { bow_p_ = Clamp(p, 0.f, 8.f); }
void Instrument::SetBowSpeed(float v)    { bow_v_ = Clamp(v, -1.f, 1.f); }
void Instrument::SetLevel(float u)       { level_ = Clamp(u, 0.f, 1.f); }
void Instrument::SetRosin(float u)       { rosin_ = Clamp(u, 0.f, 1.f); }
void Instrument::SetBody(float u)        { body_ = Clamp(u, 0.f, 1.f); }
void Instrument::SetBodySize(float u)    { CHLADNI_SET(body_size_, Clamp(u, 0.f, 1.f)); }

void Instrument::Strike(float velocity)
{
    strike_vel_ = Clamp(velocity, 0.f, 1.f);
    strike_req_ = strike_req_ + 1;
}

/* ── Init / Rebuild ─────────────────────────────────────────────────── */

void Instrument::Init(float sample_rate)
{
    sr_ = sample_rate;
    T_  = 1.f / sample_rate;
    BuildPlateTable();
    for (int i = 0; i < kNumModes; i++) s1_[i] = s2_[i] = 0.f;
    mallet_live_ = touching_ = stuck_ = false;
    yh_ = uh_ = delta_prev_ = dv_prev_ = 0.f;
    bow_fn_s_ = bow_v_s_ = kappa_s_ = 0.f;
    env_ = 0.f;
    rosin_lp_ = drift_p_ = drift_v_ = bow_vref_ = body_s_ = bite_ = 0.f;
    bow_on_ = false; bow_dir_ = 0.f; seed_left_ = 0;
    bx1_ = bx2_ = 0.f;
    for (int i = 0; i < kBodyModes; i++) by1_[i] = by2_[i] = 0.f;
    strike_ack_ = strike_req_;
    tel_ = Telemetry{};
    dirty_++;
    Rebuild();
    built_ = dirty_;
}

void Instrument::Rebuild()
{
    const uint8_t mat = material_;
    const float   f0  = pitch_;
    const float   B   = 0.02f * stiff_ * stiff_;
    const float   sig1 = kLn1000 / decay_;
    const float   hl   = hiloss_;
    const float   hl_k = 60.f * hl * hl;           /* extra sigma at 1 kHz */
    const float   fmax = 0.45f * sr_;

    float ex = pos_, ey = 0.f, px = pickup_, py = 0.f;
    if (mat == kPlate) { PlateExciter(pos_, ex, ey); PlatePickup(pickup_, px, py); }

    float Y = 0.f, D = 0.f, ainv = 0.f, ab2 = 0.f, wtop = 0.f;
    float sig[kNumModes], w[kNumModes];

    for (int i = 0; i < kNumModes; i++) {
        const float n = (float)(i + 1);
        float ratio, a, b, c;
        switch (mat) {
        case kBar: {
            const float r0 = BarBeta(0);
            ratio = (BarBeta(i) / r0) * (BarBeta(i) / r0);
            a = BarShape(i, ex); b = BarShape(i, px); c = BarShape(i, kBarBridge);
            mm_[i] = (uint8_t)(i + 1); mn_[i] = 0;
        } break;
        case kPlate: {
            const float m = plate_m[i], k = plate_n[i];
            ratio = plate_ratio[i];
            a = sinf(m * kPi * ex) * sinf(k * kPi * ey);
            b = sinf(m * kPi * px) * sinf(k * kPi * py);
            c = sinf(m * kPi * kPlateBridgeX) * sinf(k * kPi * kPlateBridgeY);
            mm_[i] = plate_m[i]; mn_[i] = plate_n[i];
        } break;
        default: {
            ratio = n;
            a = sinf(n * kPi * ex); b = sinf(n * kPi * px); c = sinf(n * kPi * kStringBridge);
            mm_[i] = (uint8_t)(i + 1); mn_[i] = 0;
        } break;
        }
        ratio *= sqrtf(1.f + B * n * n);

        float f = f0 * ratio;
        float sigma;
        if (f >= fmax) {
            /* Above Nyquist's comfort: park it, heavily damped, silent. */
            f = fmax; a = b = c = 0.f;
            sigma = 2.f * kPi * f * 0.5f;
        } else {
            const float fk = f * 0.001f;
            sigma = sig1 + hl_k * fk * fk;
        }
        hz_[i] = f;

        const float g  = tanf(kPi * f * T_);
        const float wa = 2.f * sr_ * g;            /* prewarped omega */
        const float R  = fmaxf(sigma / (2.f * kPi * f), 1e-6f);
        g_[i]   = g;
        k1_[i]  = 2.f * R + g;
        h_[i]   = 1.f / (1.f + 2.f * R * g + g * g);
        inv_w_[i]  = 1.f / wa;
        inv_w2_[i] = inv_w_[i] * inv_w_[i];
        a_[i] = a; b_[i] = b; c_[i] = c;
        Y += a * a * g * h_[i] * inv_w_[i];
        D += a * a * g * g * h_[i] * inv_w2_[i];
        ainv += a * a;
        ab2  += a * a * b * b;
        if (a != 0.f) wtop = 2.f * kPi * f;
        sig[i] = sigma;
        w[i]   = 2.f * kPi * f;
    }
    Y_ = fmaxf(Y, 1e-12f);
    D_ = D;

    /* Bow pressure scale (Schelleng). The band-averaged real admittance of
     * unit-mass modes is (pi/2) sum(a^2) / omega_top, i.e. 1/(2Z) for an
     * ideal string; the maximum bow force for Helmholtz motion is then
     * 2 Z v_b / (beta (mu_s - mu_d)). Pressure 1 = that force, so the
     * playable window stays put as speed, position and pitch move. */
    const float y_band = ainv > 1e-9f && wtop > 0.f ? 0.5f * kPi * ainv / wtop : 1e-3f;
    const float z_eff  = 0.5f / y_band;
    /* Schelleng's window runs from F_min ~ 1/beta^2 to F_max ~ 1/beta, so
     * scaling by 1/beta alone lets it creep upward as the bow nears the
     * end. Scaling by beta^-1.5 (calibrated at beta 0.13) pins the window's
     * geometric centre, so one pressure setting plays at any position. */
    torsion_y_  = CHLADNI_TORSION * 2.f * y_band;   /* rolling under the bow */
    bow_fscale_ = 2.f * z_eff / (pos_ * (kMuS - kMuD)) * sqrtf(0.13f / pos_);

    /* Bridge peak admittance, bounded from above: at each mode's frequency,
     * the sum of every mode's |velocity per force| there. Triangle
     * inequality makes it an upper bound on |Y_bridge| near that peak. */
    float ymax = 1e-9f;
    for (int k = 0; k < kNumModes; k++) {
        if (c_[k] == 0.f) continue;
        const float wk = w[k];
        float s = 0.f;
        for (int i = 0; i < kNumModes; i++) {
            if (c_[i] == 0.f) continue;
            const float d  = w[i] * w[i] - wk * wk;
            const float dd = 2.f * sig[i] * wk;
            s += c_[i] * c_[i] * wk / sqrtf(d * d + dd * dd);
        }
        if (s > ymax) ymax = s;
    }
    y_bridge_max_ = ymax;
    bridge_gain_  = CHLADNI_BRIDGE_TRIM;
    /* Normalise by how well the exciter and pickup points share modes,
     * so moving either does not swing the level by 20 dB (a pickup on a
     * node of mode 1 is still thinner, just not quieter). Boost <= 4x. */
    const float ab = sqrtf(ab2);
    out_gain_ = CHLADNI_OUT_TRIM / fmaxf(ab, 0.25f);

    /* Mallet: contact time from hardness, stiffness to match it. */
    const float hard = hard_;
    const float tau  = 0.004f * powf(0.15f / 4.f, hard);    /* 4 ms .. 0.15 ms */
    const float m_body = ainv > 1e-6f ? 1.f / ainv : 1.f;
    mh_  = Clamp(0.5f * m_body, 0.005f, 0.5f);   /* half the body's mass there */
    mp_  = 2.5f - hard;                                     /* felt 2.5 .. steel 1.5 */
    const float m_star = 1.f / (1.f / mh_ + 1.f / m_body);
    const float vref = 0.5f;
    mk_  = m_star * (kPi / tau) * (kPi / tau) * powf(kPi / (vref * tau), mp_ - 1.f);
    mlam_ = 0.05f + 0.6f * (1.f - hard);

    /* The box: constant-peak-gain two-pole resonators, scaled from violin
     * (size 0) to cello (size 1, 0.42x). */
    const float bscale = 1.f - 0.58f * body_size_;
    for (int i = 0; i < kBodyModes; i++) {
        const float f = fminf(kViolin[i].hz * bscale, 0.45f * sr_);
        const float r = expf(-kPi * f / (kViolin[i].q * sr_));
        ba1_[i] = 2.f * r * cosf(2.f * kPi * f * T_);
        ba2_[i] = r * r;
        bb0_[i] = 0.5f * (1.f - r * r);
        bg_[i]  = kViolin[i].g;
    }
    body_gain_ = CHLADNI_BODY_TRIM;

    tel_.pitch_hz = hz_[0];
    for (int i = 0; i < kNumModes; i++) {
        tel_.mode_hz[i] = hz_[i];
        tel_.mode_m[i]  = mm_[i];
        tel_.mode_n[i]  = mn_[i];
    }
}

/* ── Exciters ───────────────────────────────────────────────────────── */

/* Mallet: Hunt-Crossley, F = k d^p (1 + lam d'), d = compression.
 * Implicit in F: the mallet's own update and the body's instantaneous
 * response both depend on it, d = d0 - C F, and F - HC(d0 - C F) is
 * increasing in F, so the root in [0, d0/C] is unique. */
float Instrument::SolveMallet(float y_free)
{
    const float d0 = yh_ + uh_ * T_ - y_free;
    const float C  = T_ * T_ / mh_ + D_;
    float F = 0.f;
    if (d0 > 0.f) {
        float lo = 0.f, hi = d0 / C;
        F = 0.5f * hi;
        for (int it = 0; it < 30; it++) {
            const float d    = d0 - C * F;
            const float ddot = (d - delta_prev_) * sr_;
            float hc = 0.f, dhc = 0.f;
            if (d > 0.f) {
                const float dp = powf(d, mp_);
                const float damp = 1.f + mlam_ * ddot;
                if (damp > 0.f) {
                    hc  = mk_ * dp * damp;
                    dhc = mk_ * (mp_ * dp / d * damp + dp * mlam_ * sr_);
                }
            }
            const float phi = F - hc;
            if (fabsf(phi) <= 1e-5f * (F + hc) + 1e-12f) break;
            if (phi < 0.f) lo = F; else hi = F;
            if (hi - lo < 1e-6f * hi) break;
            float Fn = F - phi / (1.f + C * dhc);
            if (!(Fn > lo && Fn < hi)) Fn = 0.5f * (lo + hi);
            F = Fn;
        }
    }
    uh_ -= F * T_ / mh_;
    yh_ += uh_ * T_;
    const float d = yh_ - (y_free + D_ * F);
    if (F > 0.f && !touching_) tel_.strikes++;
    touching_ = F > 0.f;
    delta_prev_ = d;
    mallet_age_ += T_;
    /* The hand pulls the mallet away once it has left the surface and is
     * moving off, or after 60 ms whatever happens. */
    if ((d < 0.f && uh_ < 0.f && mallet_age_ > 0.0005f) || mallet_age_ > 0.06f) {
        mallet_live_ = false;
        touching_ = false;
    }
    return F;
}

/* Bow: F = Fn mu(dv), dv = v_bow - v_contact = dv0 - Y F. Solve
 * G(dv) = dv - dv0 + Y Fn mu(dv) = 0 by Newton inside the bracket
 * [dv0 - Y Fn mu_s, dv0 + Y Fn mu_s], starting from last sample's dv. Where
 * the friction curve allows several roots (the stick/slip hysteresis),
 * starting from the previous one keeps the bow on its branch until that
 * branch ceases to exist, which is exactly when a real bow slips. */
float Instrument::SolveBow(float v_free)
{
    const float fn  = bow_fn_s_;
    const float yfn = (Y_ + torsion_y_) * fn;
    const float dv0 = bow_v_s_ - v_free;
    float lo = dv0 - yfn * kMuS, hi = dv0 + yfn * kMuS;
    float x = Clamp(dv_prev_, lo, hi);
    float mu = 0.f, dmu = 0.f;
    /* Tolerance relative to the problem's own scale: a fixed 1e-9 sits
     * below float resolution here and ran every iteration, every sample. */
    const float tol = 1e-5f * (fabsf(dv0) + yfn * kMuS) + 1e-9f;
    for (int it = 0; it < 24; it++) {
        Friction(x, rosin_mul_, mu, dmu);
        const float G = x - dv0 + yfn * mu;
        if (fabsf(G) < tol) break;
        if (G < 0.f) lo = x; else hi = x;
        if (hi - lo < tol) break;
        const float dG = 1.f + yfn * dmu;
        float xn = dG > 0.f ? x - G / dG : 0.5f * (lo + hi);
        if (!(xn > lo && xn < hi)) xn = 0.5f * (lo + hi);
        x = xn;
    }
    Friction(x, rosin_mul_, mu, dmu);
    dv_prev_ = x;
    /* Stick/slip with hysteresis, so ripple riding on the stick phase is
     * not counted as a slip: stuck below 3 eps, slipping only once the
     * relative velocity is a real fraction of the bow's. */
    const float ax = fabsf(x);
    if (stuck_) {
        if (ax > 3.f * kEps + 0.25f * fabsf(bow_v_s_)) { stuck_ = false; tel_.slips++; }
    } else if (ax < 3.f * kEps) {
        stuck_ = true;
    }
    return fn * mu;
}

/* Bow onsets (the bow lands, or reverses). A friction-curve bow left to
 * itself spends 100-300 ms double-slipping before Helmholtz motion forms -
 * the "scratch" at every note start. Two things fix it, and only together
 * (measured over landings, starts from rest and bow changes at G2/D3,
 * beta 0.10/0.13: median settle 245 ms -> 24 ms, 26/32 attacks under
 * 60 ms; seed alone or bite alone does little):
 *  - seed: the string is given the ideal Helmholtz shape for the new
 *    direction, at CHLADNI_SEED x half the full bow speed, so it starts
 *    with one corner instead of having to find one;
 *  - bite: bow force +kBite at the onset, decaying over kBiteTau, as a
 *    player bites into a stroke.
 * Starts from rest with a very fast (20 ms) acceleration still scratch;
 * per Guettler they need more force than the window allows. */
#ifndef CHLADNI_SEED
#define CHLADNI_SEED 1.f
#endif
constexpr float kBite = 0.5f, kBiteTau = 0.08f;

/* Ideal Helmholtz motion for bow speed v at beta = pos_, just after a slip
 * (corner leaving the bow): q'_n = -(2v / (n pi beta)) cos(n pi beta),
 * q_n = -(2v / (n pi beta n w_n)) sin(n pi beta). From the Fourier series
 * of the bow-point velocity (v while stuck, -v(1-beta)/beta while slipping).
 * String only. The difference from the present state is not applied at
 * once - that clicks on the pickup (up to 6.6x the steady sample step) -
 * but blended in over kSeedRamp samples while it rings freely with the
 * modes: by linearity that is exactly a crossfade from the old motion to
 * the seeded one. */
constexpr int kSeedRamp = 144;   /* 3 ms */
void Instrument::SeedHelmholtz(float v)
{
    if (material_ != kString) return;
    const float beta = pos_;
    for (int i = 0; i < kNumModes; i++) {
        if (a_[i] == 0.f) { sd1_[i] = sd2_[i] = 0.f; continue; }
        const float nn = (float)(i + 1);
        const float amp = -2.f * v / (nn * kPi * beta);
        const float qd  = amp * cosf(nn * kPi * beta);
        const float q   = amp * sinf(nn * kPi * beta) * inv_w_[i];
        sd1_[i] = qd / inv_w_[i] - s1_[i];
        sd2_[i] = q / inv_w2_[i] - s2_[i];
    }
    seed_left_ = kSeedRamp;
}

/* ── Audio ──────────────────────────────────────────────────────────── */

void Instrument::Process(const float* in, float* out, float* bridge, size_t n)
{
    if (built_ != dirty_) { built_ = dirty_; Rebuild(); }

    if (strike_req_ != strike_ack_) {
        strike_ack_ = strike_req_;
        /* Start just touching the surface, moving in. */
        float yc = 0.f;
        for (int i = 0; i < kNumModes; i++) yc += a_[i] * s2_[i] * inv_w2_[i];
        yh_ = yc;
        uh_ = 0.05f + 0.95f * strike_vel_;
        delta_prev_ = 0.f;
        mallet_age_ = 0.f;
        mallet_live_ = true;
        touching_ = false;
    }

    /* Player drift: two slow (~2 Hz) mean-reverting walks on bow pressure
     * and speed, std up to 6 % and 4 % at Rosin 1. Nobody holds a bow
     * perfectly still, and a perfectly still one is a large part of what
     * makes a bowed model sound like a machine. */
    const float rosin = rosin_;
    const float dt    = (float)n * T_;
    const float theta = 2.f * kPi * 2.f;
    const float ou    = sqrtf(2.f * theta * dt) * 1.7320508f;
    drift_p_ += -theta * drift_p_ * dt + 0.06f * ou * Uni(rng_);
    drift_v_ += -theta * drift_v_ * dt + 0.04f * ou * Uni(rng_);

    const float kappa_t = couple_ * kCoupleCeiling / (bridge_gain_ * y_bridge_max_);
    const float bv_t    = bow_v_ * kBowMaxV * (1.f + rosin * drift_v_);

    /* The speed the force is scaled by (see CHLADNI_FN_LAW). */
    const float abv = fabsf(bv_t);
#if CHLADNI_FN_LAW == 0
    bow_vref_ = abv;
#elif CHLADNI_FN_LAW == 1
    if (abv > bow_vref_) bow_vref_ = abv;
    else bow_vref_ += (abv - bow_vref_) * (1.f - expf(-dt / CHLADNI_FN_RELEASE));
#else
    bow_vref_ = 0.5f * kBowMaxV;
#endif
    /* Onsets: the bow lands, or reverses. */
    if (CHLADNI_SEED > 0.f) {
        const bool on = bow_p_ > 0.05f;
        const float dir = bv_t > 0.02f * kBowMaxV ? 1.f : bv_t < -0.02f * kBowMaxV ? -1.f : 0.f;
        if (on && dir != 0.f && (!bow_on_ || (bow_dir_ != 0.f && dir != bow_dir_))) {
            SeedHelmholtz(dir * CHLADNI_SEED * 0.5f * kBowMaxV);
            bite_ = 1.f;
        }
        bow_on_ = on;
        if (dir != 0.f) bow_dir_ = dir;
    }

    bite_ *= expf(-dt / kBiteTau);
    const float fn_t    = bow_p_ * bow_fscale_ * (bow_vref_ + kBowFloor) * (1.f + kBite * bite_)
                        * (1.f + rosin * drift_p_);
    const float rosin_k = 0.25f * rosin;
    const float body_t  = body_;
    const float sm      = 1.f - expf(-T_ / 0.005f);   /* 5 ms smoothing */
    const float env_rel = expf(-T_ / 0.15f);

    float bp0[kNumModes], lp0[kNumModes];
    float fpk = 0.f, vc = 0.f, yc = 0.f;

    for (size_t s = 0; s < n; s++) {
        kappa_s_  += (kappa_t - kappa_s_) * sm;
        bow_fn_s_ += (fn_t - bow_fn_s_) * sm;
        bow_v_s_  += (bv_t - bow_v_s_) * sm;

        const float fc = in ? kappa_s_ * SoftClip(in[s]) : 0.f;

        /* Rosin: sliding friction flickers at audio rate (white noise,
         * 4 kHz one-pole, ~7 % std at Rosin 1). */
        rosin_lp_ += (Uni(rng_) - rosin_lp_) * 0.41f;
        rosin_mul_ = 1.f + rosin_k * rosin_lp_;

        /* Seed blend: add 1/kSeedRamp of the freely ringing difference. */
        if (seed_left_ > 0) {
            const float f = 1.f / (float)kSeedRamp;
            for (int i = 0; i < kNumModes; i++) {
                s1_[i] += f * sd1_[i];
                s2_[i] += f * sd2_[i];
                const float hp = (-k1_[i] * sd1_[i] - sd2_[i]) * h_[i];
                const float bp = g_[i] * hp + sd1_[i];
                const float lp = g_[i] * bp + sd2_[i];
                sd1_[i] = 2.f * bp - sd1_[i];
                sd2_[i] = 2.f * lp - sd2_[i];
            }
            seed_left_--;
        }

        /* Pass 1: free response (state + coupling force only). */
        float vf = 0.f, yf = 0.f;
        for (int i = 0; i < kNumModes; i++) {
            const float x  = c_[i] * fc;
            const float hp = (x - k1_[i] * s1_[i] - s2_[i]) * h_[i];
            const float bp = g_[i] * hp + s1_[i];
            const float lp = g_[i] * bp + s2_[i];
            bp0[i] = bp; lp0[i] = lp;
            vf += a_[i] * bp * inv_w_[i];
            yf += a_[i] * lp * inv_w2_[i];
        }

        /* Exciters, solved against the instantaneous response. */
        float F = 0.f, fb = 0.f;
        if (mallet_live_) {
            const float fm = SolveMallet(yf);
            vf += Y_ * fm; yf += D_ * fm; F += fm;
        }
        if (bow_fn_s_ > 1e-6f) {
            fb = SolveBow(vf);
            F += fb;
        } else {
            stuck_ = false;
            dv_prev_ = 0.f;
        }
        if (fabsf(F) > fpk) fpk = fabsf(F);

        /* Pass 2: commit, by superposition on pass 1. */
        float o = 0.f, br = 0.f;
        vc = vf + Y_ * fb;
        yc = yf + D_ * fb;
        for (int i = 0; i < kNumModes; i++) {
            const float gh = g_[i] * h_[i] * a_[i] * F;
            const float bp = bp0[i] + gh;
            const float lp = lp0[i] + g_[i] * gh;
            s1_[i] = 2.f * bp - s1_[i];
            s2_[i] = 2.f * lp - s2_[i];
            const float v = bp * inv_w_[i];
            o  += b_[i] * v;
            br += c_[i] * v;
        }

        /* The box, driven by the bridge. */
        const float bx = br;
        float bo = kBodyDirect * bx;
        for (int i = 0; i < kBodyModes; i++) {
            const float yb = bb0_[i] * (bx - bx2_) + ba1_[i] * by1_[i] - ba2_[i] * by2_[i];
            by2_[i] = by1_[i]; by1_[i] = yb;
            bo += bg_[i] * yb;
        }
        bx2_ = bx1_; bx1_ = bx;
        body_s_ += (body_t - body_s_) * sm;

        const float y = Knee(((1.f - body_s_) * o * out_gain_ + body_s_ * bo * body_gain_)
                             * level_);
        if (out)    out[s]    = y;
        if (bridge) bridge[s] = Knee(br * bridge_gain_);

        const float ay = fabsf(y);
        env_ = ay > env_ ? ay : env_ * env_rel;
    }

    /* Telemetry, once per block. */
    float etot = 0.f, ef = 0.f;
    for (int i = 0; i < kNumModes; i++) {
        const float e = 0.5f * (s1_[i] * s1_[i] + s2_[i] * s2_[i]) * inv_w2_[i];
        tel_.mode_energy[i] = e;
        tel_.mode_x[i] = s2_[i] * inv_w2_[i];
        tel_.mode_y[i] = s1_[i] * inv_w2_[i];
        etot += e;
        ef   += e * hz_[i];
    }
    tel_.centroid_hz   = etot > 1e-20f ? ef / etot : hz_[0];
    tel_.contact_force = fpk;
    tel_.velocity      = vc;
    tel_.displacement  = yc;
    tel_.envelope      = env_;
    tel_.in_contact    = touching_;
    tel_.sticking      = stuck_ && bow_fn_s_ > 1e-6f;
    tel_.bowing        = bow_fn_s_ > 1e-6f;
    kappa_ = kappa_s_;
}

} // namespace chladni_dsp
