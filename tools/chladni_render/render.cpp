/* Offline renderer and checks for chladni_dsp — same source the firmware
 * compiles. See README.md. */
#include "chladni_dsp.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace chladni_dsp;

static const float  SR  = 48000.f;
static const size_t BLK = 24;              /* the module's audio block */
static int g_fail = 0;

static void WriteWav(const std::string& path, const std::vector<float>& l,
                     const std::vector<float>& r)
{
    const uint32_t nf = (uint32_t)l.size(), data = nf * 6;
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) { perror(path.c_str()); return; }
    auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
    auto u16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
    fwrite("RIFF", 1, 4, f); u32(36 + data); fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f); u32(16); u16(1); u16(2);
    u32((uint32_t)SR); u32((uint32_t)SR * 6); u16(6); u16(24);
    fwrite("data", 1, 4, f); u32(data);
    for (uint32_t i = 0; i < nf; i++)
        for (int c = 0; c < 2; c++) {
            float v = c ? r[i] : l[i];
            v = v > 1.f ? 1.f : v < -1.f ? -1.f : v;
            int32_t s = (int32_t)lrintf(v * 8388607.f);
            uint8_t b[3] = { (uint8_t)s, (uint8_t)(s >> 8), (uint8_t)(s >> 16) };
            fwrite(b, 1, 3, f);
        }
    fclose(f);
    printf("  wrote %-34s %6.2f s\n", path.c_str(), nf / SR);
}

static void Defaults(Instrument& x, uint8_t mat = kString, float hz = 110.f)
{
    x.Init(SR);
    x.SetMaterial(mat);
    x.SetPitchHz(hz);
    x.SetStiffness(0.15f);
    x.SetDecay(4.f);
    x.SetHighLoss(0.6f);
    x.SetPosition(0.13f);
    x.SetPickup(0.77f);
    x.SetHardness(0.5f);
    x.SetBowPressure(0.f);
    x.SetBowSpeed(0.5f);
    x.SetCouple(0.f);
    x.SetLevel(0.7f);
}

/* Run one instrument for `secs`, calling `ctl(t)` at every block (the
 * control thread's view). Returns mono pickup and bridge. */
struct Run {
    std::vector<float> out, bridge;
    float peak = 0.f;
    bool  finite = true;
};
static Run Render(Instrument& x, float secs, std::function<void(float)> ctl,
                  const std::vector<float>* in = nullptr)
{
    Run r;
    const size_t N = (size_t)(secs * SR);
    r.out.resize(N); r.bridge.resize(N);
    for (size_t s = 0; s < N; s += BLK) {
        const size_t n = std::min(BLK, N - s);
        ctl(s / SR);
        x.Process(in ? in->data() + s : nullptr, r.out.data() + s, r.bridge.data() + s, n);
    }
    for (float v : r.out) {
        if (!std::isfinite(v)) r.finite = false;
        r.peak = std::max(r.peak, fabsf(v));
    }
    return r;
}

static float RmsDb(const std::vector<float>& v, size_t a, size_t b)
{
    double e = 0;
    b = std::min(b, v.size());
    for (size_t i = a; i < b; i++) e += (double)v[i] * v[i];
    e /= std::max<size_t>(1, b - a);
    return 10.f * log10f((float)e + 1e-30f);
}

/* Period estimate by normalised autocorrelation over [a, a+len). */
static float PitchHz(const std::vector<float>& v, size_t a, size_t len,
                     float fmin = 40.f, float fmax = 1500.f)
{
    const int lmin = (int)(SR / fmax), lmax = (int)(SR / fmin);
    if (a + len + lmax > v.size()) return 0.f;
    float best = 0.f; int bl = 0;
    for (int L = lmin; L <= lmax; L++) {
        double num = 0, d1 = 0, d2 = 0;
        for (size_t i = a; i < a + len; i++) {
            num += v[i] * v[i + L]; d1 += v[i] * v[i]; d2 += v[i + L] * v[i + L];
        }
        const float c = (float)(num / sqrt(d1 * d2 + 1e-30));
        if (c > best) { best = c; bl = L; }
    }
    if (!(best > 0.5f && bl)) return 0.f;
    /* Parabolic interpolation around the peak: whole-sample lags are ~4
     * cents apart at 110 Hz, too coarse to see a bow pulling into tune. */
    auto corr = [&](int L) {
        double num = 0, d1 = 0, d2 = 0;
        for (size_t i = a; i < a + len; i++) {
            num += v[i] * v[i + L]; d1 += v[i] * v[i]; d2 += v[i + L] * v[i + L];
        }
        return num / sqrt(d1 * d2 + 1e-30);
    };
    const double ym = corr(bl - 1), y0 = best, yp = corr(bl + 1);
    const double den = ym - 2 * y0 + yp;
    const double off = den < 0 ? 0.5 * (ym - yp) / den : 0.0;
    return (float)(SR / (bl + off));
}

static float Clamp01(float x) { return x < 0.f ? 0.f : x > 1.f ? 1.f : x; }

static void Check(bool ok, const char* what)
{
    printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) g_fail++;
}

/* ── Scenes ─────────────────────────────────────────────────────────── */

static void SceneStrike()
{
    printf("strike: one phrase per material, soft/hard mallet\n");
    static const float notes[] = { 130.81f, 164.81f, 196.f, 261.63f, 196.f, 98.f };
    for (uint8_t mat = 0; mat < kNumMaterials; mat++) {
        Instrument x;
        Defaults(x, mat, notes[0]);
        int step = -1;
        int contact_samples = 0;
        Run r = Render(x, 12.f, [&](float t) {
            const int k = (int)(t / 1.f);
            if (k != step && k < 12) {
                step = k;
                x.SetPitchHz(notes[k % 6]);
                x.SetHardness(k < 6 ? 0.25f : 0.85f);
                x.Strike(k % 3 == 2 ? 1.f : 0.55f);
            }
            if (x.Snapshot().in_contact && t < 0.02f) contact_samples += BLK;
        });
        char fn[64];
        snprintf(fn, sizeof fn, "out/strike_%s.wav", MaterialName(mat));
        WriteWav(fn, r.out, r.out);
        printf("    %-6s peak %.2f  rms(0-1s) %.1f dB  strikes %u\n",
               MaterialName(mat), r.peak, RmsDb(r.out, 0, 48000), x.Snapshot().strikes);
        Check(r.finite, "finite");
    }
}

/* Contact time vs hardness: the mallet is meant to span 4 ms .. 0.15 ms. */
static void SceneContact()
{
    printf("contact: mallet contact time vs hardness (string, 110 Hz, vel 0.6)\n");
    for (float h : { 0.f, 0.25f, 0.5f, 0.75f, 1.f }) {
        Instrument x; Defaults(x);
        x.SetHardness(h);
        x.Strike(0.6f);
        int first = -1, last = -1; float fpk = 0.f;
        std::vector<float> o(1);
        for (int s = 0; s < 960; s++) {      /* 20 ms, one sample per call */
            float out, br;
            x.Process(nullptr, &out, &br, 1);
            const Telemetry t = x.Snapshot();
            if (t.in_contact) { if (first < 0) first = s; last = s; }
            fpk = std::max(fpk, t.contact_force);
        }
        printf("    hardness %.2f  contact %.3f ms  peak F %.3g  impacts %u\n", h,
               first < 0 ? 0.f : (last - first + 1) * 1000.f / SR, fpk, x.Snapshot().strikes);
    }
}

/* Bow: pressure sweep at fixed speed. Helmholtz motion = one slip per
 * period and a steady pitch at the string's fundamental. */
static void SceneBow()
{
    const float hl = getenv("HL") ? atof(getenv("HL")) : 0.6f;
    printf("bow: pressure sweep, string 110 Hz, speed 0.5, high loss %.2f\n", hl);
    printf("    %8s %8s %9s %9s %9s\n", "pressure", "rms dB", "pitch Hz", "slips/per", "stick %");
    std::vector<float> L, R;
    for (float p : { 0.05f, 0.1f, 0.2f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f, 0.6f, 0.7f }) {
        Instrument x; Defaults(x);
        x.SetStiffness(0.f);
        x.SetHighLoss(hl);
        x.SetBowSpeed(0.5f);
        x.SetBowPressure(p);
        uint32_t sl0 = 0; int stick = 0, blocks = 0;
        Run r = Render(x, 2.f, [&](float t) {
            if (t >= 1.f) {
                if (!sl0) sl0 = x.Snapshot().slips + 1;
                stick += x.Snapshot().sticking; blocks++;
            }
        });
        const float hz = PitchHz(r.out, 48000, 4800);
        const float slips = (float)(x.Snapshot().slips - (sl0 - 1));
        printf("    %8.2f %8.1f %9.1f %9.2f %9.0f\n", p, RmsDb(r.out, 48000, 96000), hz,
               hz > 0 ? slips / hz : 0.f, 100.f * stick / std::max(1, blocks));
        L.insert(L.end(), r.out.begin(), r.out.end());
        R.insert(R.end(), r.out.begin(), r.out.end());
        if (!r.finite) Check(false, "bow finite");
    }
    WriteWav("out/bow_pressure.wav", L, R);

    /* A playable phrase: bow in, swell, change notes, lift. */
    Instrument x; Defaults(x, kString, 98.f);
    Run r = Render(x, 10.f, [&](float t) {
        static const float hz[] = { 98.f, 110.f, 130.81f, 146.83f, 130.81f };
        const int k = std::min(4, (int)(t / 2.f));
        x.SetPitchHz(hz[k]);
        const float ph = fmodf(t, 2.f);
        x.SetBowPressure(ph < 1.8f ? 0.35f : 0.f);
        x.SetBowSpeed((k & 1 ? -1.f : 1.f) * (0.3f + 0.3f * sinf(ph * 1.6f)));
    });
    Check(r.finite, "bow phrase finite");
}

/* A cellist, roughly: G2 A2 C3 D3 E3 D3 C3, one bow per note, alternating.
 * Each stroke: pressure accent then settle and a taper into the bow
 * change; speed ramps in and eases off; vibrato arrives after the onset.
 * The module gets this from knobs and CV; here it stands in for a player,
 * so the engine is judged on what it does with a human input. */
static void PlayerPhrase(Instrument& x, float t)
{
    static const float hz[] = { 98.f, 110.f, 130.81f, 146.83f, 164.81f, 146.83f, 130.81f };
    const float len = 1.4f;
    const int   k   = (int)(t / len);
    if (k >= 7) { x.SetBowPressure(0.f); return; }
    const float u   = t - k * len;                    /* time in this stroke */
    const float dir = (k & 1) ? -1.f : 1.f;
    float p;
    /* No accent here: the engine bites at every onset by itself. */
    if (u < 0.02f)              p = 0.42f * u / 0.02f;
    else if (u > len - 0.12f)   p = 0.42f - 0.10f * (u - (len - 0.12f)) / 0.12f;
    else                        p = 0.42f;
    float v;
    if (u < 0.1f)               v = 0.55f * u / 0.1f;
    else if (u > len - 0.1f)    v = 0.45f * (len - u) / 0.1f + 0.05f;
    else                        v = 0.55f - 0.1f * (u - 0.1f) / (len - 0.2f);
    const float vib_amt = Clamp01((u - 0.25f) / 0.3f) * 15.f;   /* cents */
    const float vib     = vib_amt * sinf(2.f * 3.14159265f * 5.4f * t + 0.3f * sinf(1.3f * t));
    x.SetPitchHz(hz[k] * exp2f(vib / 1200.f));
    x.SetBowPressure(p);
    x.SetBowSpeed(dir * v);
}

static void ScenePhrase()
{
    printf("phrase: cumulative variants of one bowed phrase (cello range)\n");
    struct V { const char* tag; float rosin, body, size; } vs[] = {
        { "B_player", 0.f,  0.f, 1.f },
        { "C_rosin",  0.6f, 0.f, 1.f },
        { "D_body",   0.6f, 1.f, 1.f },
    };
    const char* suffix = getenv("SUFFIX") ? getenv("SUFFIX") : "";
    for (const V& v : vs) {
        if (*suffix && strcmp(v.tag, "D_body")) continue;   /* mode-count builds: final variant only */
        Instrument x; Defaults(x, kString, 98.f);
        x.SetStiffness(0.f);
        x.SetHighLoss(0.9f);
        x.SetPosition(0.1f);
        x.SetRosin(v.rosin); x.SetBody(v.body); x.SetBodySize(v.size);
        Run r = Render(x, 11.5f, [&](float t) { PlayerPhrase(x, t); });
        char fn[80]; snprintf(fn, sizeof fn, "out/bow_phrase_%s%s.wav", v.tag, suffix);
        WriteWav(fn, r.out, r.out);
        printf("    %-9s rms %.1f dB peak %.2f\n", v.tag, RmsDb(r.out, 0, r.out.size()), r.peak);
        Check(r.finite, "finite");
    }
}

/* Settle time per note: ms from each onset to the first run of 10
 * periods with exactly one slip (Helmholtz motion). slips[] holds the
 * time of every slip, onsets[]/hz[] the notes. -1 = never. */
static std::vector<float> SettleMs(const std::vector<double>& slips,
                                   const std::vector<double>& onsets,
                                   const std::vector<float>& hz, double span)
{
    std::vector<float> r;
    for (size_t k = 0; k < onsets.size(); k++) {
        const double t0 = onsets[k], P = 1.0 / hz[k];
        int run = 0; double first = 0; float res = -1.f;
        for (double t = t0; t < t0 + span; t += P) {
            int c = 0;
            for (double sl : slips) if (sl >= t && sl < t + P) c++;
            if (c == 1) { if (!run) first = t; if (++run >= 10) { res = (float)((first - t0) * 1000); break; } }
            else run = 0;
        }
        r.push_back(res);
    }
    return r;
}

/* How the module is actually played: bow speed from a knob or CV, the note
 * started by putting the bow down (B3 or a gate). Landing: each note is a
 * gate on a steadily moving bow, direction alternating while lifted.
 * Legato: one long bow, the pitch moves under it (a slur). */
static void SceneModuleBow()
{
    printf("modbow: bowing the way the module plays it (rosin 0.6, cello box)\n");
    static const float hz[] = { 98.f, 110.f, 130.81f, 146.83f, 164.81f, 146.83f, 130.81f };
    for (int legato = 0; legato < 2; legato++) {
        Instrument x; Defaults(x, kString, 98.f);
        x.SetStiffness(0.f); x.SetHighLoss(0.9f); x.SetPosition(0.1f);
        x.SetRosin(0.6f); x.SetBody(1.f); x.SetBodySize(1.f);
        const float len = 1.4f;
        std::vector<double> slips, onsets; std::vector<float> notes;
        for (int k = 0; k < 7; k++) { onsets.push_back(k * len + (legato ? 0.0 : 0.0)); notes.push_back(hz[k]); }
        uint32_t last = 0;
        Run r = Render(x, 7 * len + 1.5f, [&](float t) {
            const int k = (int)(t / len);
            const float u = t - k * len;
            if (k >= 7) { x.SetBowPressure(0.f); return; }
            const float vib = 15.f * Clamp01((u - 0.3f) / 0.3f)
                            * sinf(2.f * 3.14159265f * 5.4f * t);
            x.SetPitchHz(hz[k] * exp2f(vib / 1200.f));
            if (legato) {
                x.SetBowSpeed(0.5f);
                x.SetBowPressure(t < 0.01f ? 42.f * t : 0.42f);
            } else {
                x.SetBowSpeed((k & 1) ? -0.5f : 0.5f);
                const float p = u < 0.01f ? 0.42f * u / 0.01f : u < len - 0.12f ? 0.42f : 0.f;
                x.SetBowPressure(p);
            }
            const uint32_t s = x.Snapshot().slips;
            if (s != last) { slips.push_back(t); last = s; }
        });
        const char* fn = legato ? "out/bow_module_legato.wav" : "out/bow_module_gated.wav";
        WriteWav(fn, r.out, r.out);
        std::vector<float> st = SettleMs(slips, onsets, notes, 1.2);
        printf("    %-7s settle ms per note:", legato ? "legato" : "gated");
        for (float v : st) printf(" %4.0f", v);
        printf("\n");
        Check(r.finite, "finite");
    }
}

static void ScenePosition()
{
    printf("position: strikes walking from the end to the middle (bar)\n");
    Instrument x; Defaults(x, kBar, 220.f);
    int step = -1;
    Run r = Render(x, 8.f, [&](float t) {
        const int k = (int)(t / 0.5f);
        if (k != step && k < 16) { step = k; x.SetPosition(0.02f + 0.48f * k / 15.f); x.Strike(0.7f); }
    });
    WriteWav("out/position_bar.wav", r.out, r.out);
    Check(r.finite, "finite");
}

/* Two labs, patched bridge -> coupling in both ways through a delay that
 * stands for codec + block latency. */
struct Pair {
    Instrument a, b;
    std::vector<float> da, db;      /* delay lines: a's bridge on its way to b */
    size_t w = 0;
    float  loop_trim = 1.f;         /* codec out -> codec in gain */
    Pair(size_t delay) : da(delay, 0.f), db(delay, 0.f) {}
    void Render(float secs, std::function<void(float)> ctl,
                std::vector<float>& L, std::vector<float>& R)
    {
        const size_t N = (size_t)(secs * SR);
        L.assign(N, 0.f); R.assign(N, 0.f);
        float ina[BLK], inb[BLK], bra[BLK], brb[BLK];
        for (size_t s = 0; s < N; s += BLK) {
            ctl(s / SR);
            const size_t n = std::min(BLK, N - s);
            const size_t D = da.size();
            for (size_t i = 0; i < n; i++) {
                ina[i] = loop_trim * db[(w + i) % D];
                inb[i] = loop_trim * da[(w + i) % D];
            }
            a.Process(ina, L.data() + s, bra, n);
            b.Process(inb, R.data() + s, brb, n);
            for (size_t i = 0; i < n; i++) { da[(w + i) % D] = bra[i]; db[(w + i) % D] = brb[i]; }
            w = (w + n) % D;
        }
    }
};

static void SceneCouple()
{
    const size_t kDelay = 168;   /* 3.5 ms */
    printf("couple: two labs, %.1f ms cable loop\n", kDelay * 1000.f / SR);

    /* 1. Sympathy: strike A, B silent. B tuned unison, fifth, unrelated. */
    for (float ratio : { 1.f, 1.5f, 1.0595f * 1.0595f * 1.0595f }) {
        Pair p(kDelay);
        Defaults(p.a, kString, 110.f); Defaults(p.b, kString, 110.f * ratio);
        p.a.SetCouple(1.f); p.b.SetCouple(1.f);
        std::vector<float> L, R;
        bool hit = false;
        p.Render(4.f, [&](float) { if (!hit) { p.a.Strike(0.8f); hit = true; } }, L, R);
        printf("    B at %.3f x A: A %.1f dB  B %.1f dB (0.5-2 s)\n", ratio,
               RmsDb(L, 24000, 96000), RmsDb(R, 24000, 96000));
        if (ratio == 1.f) WriteWav("out/couple_sympathy_unison.wav", L, R);
        if (ratio == 1.5f) WriteWav("out/couple_sympathy_fifth.wav", L, R);
    }

    /* 2. Stability: worst case - unison, longest decay, coupling full, loop
     *    gain above unity on the cable (trim 1.15). The pair must decay no
     *    slower than... well, it must decay. Compare 20-25 s to 2-7 s. */
    for (float trim : { 1.f, 1.15f }) {
        for (uint8_t mat = 0; mat < kNumMaterials; mat++) {
            Pair p(kDelay);
            p.loop_trim = trim;
            Defaults(p.a, mat, 110.f); Defaults(p.b, mat, 110.f);
            p.a.SetDecay(30.f); p.b.SetDecay(30.f);
            p.a.SetHighLoss(0.f); p.b.SetHighLoss(0.f);
            p.a.SetCouple(1.f); p.b.SetCouple(1.f);
            std::vector<float> L, R;
            bool hit = false;
            p.Render(26.f, [&](float) { if (!hit) { p.a.Strike(1.f); p.b.Strike(1.f); hit = true; } }, L, R);
            const float e1 = RmsDb(L, 2 * 48000, 7 * 48000) , e2 = RmsDb(L, 20 * 48000, 25 * 48000);
            char what[96];
            snprintf(what, sizeof what, "%-6s trim %.2f: decays %.1f dB from 2-7 s to 20-25 s (T60 30 s ~ -30)",
                     MaterialName(mat), trim, e2 - e1);
            Check(std::isfinite(e2) && e2 < e1 - 10.f, what);
        }
    }

    /* 3. Bowed and coupled, then lifted: the pair must still die away.
     *    Long decay, so a sustained loop would show. */
    for (uint8_t mat = 0; mat < kNumMaterials; mat++) {
        Pair p(kDelay);
        Defaults(p.a, mat, 110.f); Defaults(p.b, mat, 110.f);
        for (Instrument* x : { &p.a, &p.b }) {
            x->SetDecay(30.f); x->SetHighLoss(0.f); x->SetCouple(1.f); x->SetBowSpeed(0.5f);
        }
        std::vector<float> L, R;
        p.Render(26.f, [&](float t) {
            const float pr = t < 3.f ? 0.35f : 0.f;
            p.a.SetBowPressure(pr); p.b.SetBowPressure(pr);
        }, L, R);
        const float e1 = RmsDb(L, 5 * 48000, 10 * 48000), e2 = RmsDb(L, 20 * 48000, 25 * 48000);
        char what[96];
        snprintf(what, sizeof what, "%-6s bowed then lifted: decays %.1f dB from 5-10 s to 20-25 s",
                 MaterialName(mat), e2 - e1);
        Check(std::isfinite(e2) && e2 < e1 - 10.f, what);
    }
}

/* Every control jumping at random for a minute: nothing may go non-finite
 * or stick at the rails. */
static void SceneStress()
{
    printf("stress: random controls every 50 ms, 60 s, coupled pair\n");
    Pair p(168);
    Defaults(p.a); Defaults(p.b);
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> U(0.f, 1.f);
    auto jiggle = [&](Instrument& x) {
        x.SetMaterial((uint8_t)(U(rng) * 3) % 3);
        x.SetPitchHz(20.f * powf(2.f, U(rng) * 7.f));
        x.SetStiffness(U(rng)); x.SetDecay(0.02f + 30.f * U(rng) * U(rng));
        x.SetHighLoss(U(rng)); x.SetPosition(U(rng) * 0.5f); x.SetPickup(U(rng));
        x.SetHardness(U(rng)); x.SetCouple(U(rng));
        x.SetBowPressure(U(rng) < 0.5f ? 0.f : 1.5f * U(rng) * U(rng));
        x.SetBowSpeed(2.f * U(rng) - 1.f);
        x.SetRosin(U(rng)); x.SetBody(U(rng)); x.SetBodySize(U(rng));
        if (U(rng) < 0.3f) x.Strike(U(rng));
    };
    int step = -1;
    std::vector<float> L, R;
    p.Render(60.f, [&](float t) {
        const int k = (int)(t / 0.05f);
        if (k != step) { step = k; jiggle(p.a); jiggle(p.b); }
    }, L, R);
    bool fin = true; float pk = 0.f; size_t rail = 0;
    for (size_t i = 0; i < L.size(); i++) {
        if (!std::isfinite(L[i]) || !std::isfinite(R[i])) fin = false;
        pk = std::max(pk, std::max(fabsf(L[i]), fabsf(R[i])));
        if (fabsf(L[i]) > 0.99f) rail++;
    }
    Check(fin, "finite throughout");
    printf("    peak %.3f, %.2f %% of samples above 0.99\n", pk, 100.f * rail / L.size());
}

static void SceneBench()
{
    printf("bench: native cost (Apple core, NOT the M7 - see README)\n");
    Instrument x; Defaults(x);
    x.SetBowPressure(0.3f);
    std::vector<float> in(BLK, 0.1f), o(BLK), b(BLK);
    const int blocks = 20000;
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < blocks; i++) {
        if (i % 200 == 0) { x.Strike(0.8f); x.SetPitchHz(100.f + (i % 7)); }
        x.Process(in.data(), o.data(), b.data(), BLK);
    }
    auto t1 = std::chrono::steady_clock::now();
    const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count() / (blocks * BLK);
    printf("    %.1f ns/sample = %.0fx realtime (bow on, strike every 100 ms, retune every 100 ms)\n",
           ns, 1e9 / SR / ns);
}

int main(int argc, char** argv)
{
    const std::string which = argc > 1 ? argv[1] : "all";
    const bool all = which == "all";
    if (all || which == "contact")  SceneContact();
    if (all || which == "strike")   SceneStrike();
    if (all || which == "bow")      SceneBow();
    if (all || which == "phrase")   ScenePhrase();
    if (all || which == "modbow")   SceneModuleBow();
    if (all || which == "position") ScenePosition();
    if (all || which == "couple")   SceneCouple();
    if (all || which == "stress")   SceneStress();
    if (all || which == "bench")    SceneBench();
    if (g_fail) printf("%d check(s) FAILED\n", g_fail);
    return g_fail ? 1 : 0;
}
