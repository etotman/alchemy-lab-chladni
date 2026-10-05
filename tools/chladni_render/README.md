# chladni_render

Builds `src/chladni/chladni_dsp.cpp` natively (never a copy) and renders it
to WAV, so the instrument is judged by ear before a flash cycle — the same
idea as `tools/handpan_render`.

```sh
tools/chladni_render/run.sh            # every scene; non-zero exit on a failed check
tools/chladni_render/run.sh bow        # one scene
HL=0.9 tools/chladni_render/run.sh bow # bow sweep at another high-loss setting
```

| Scene | What it renders / checks |
| --- | --- |
| `contact` | mallet contact time vs hardness (target 4 ms felt .. 0.15 ms steel) |
| `strike` | a 12-strike phrase per material, soft then hard mallet (`strike_<Material>.wav`) |
| `modbow` | bowing as the module plays it - bow speed steady, notes gated (`bow_module_gated.wav`) or slurred under one bow (`bow_module_legato.wav`); prints each note's settle time (ms to Helmholtz motion; under ~50 ms is a good attack) |
| `phrase` | a cellist-style phrase with bow strokes from rest, cumulative variants (player / rosin / box) |
| `bow` | pressure sweep at fixed speed: pitch, slips per period (1.00 = Helmholtz), stick % (`bow_pressure.wav`); a bowed phrase (`bow_phrase.wav`) |
| `position` | bar struck at 16 points from the end to the middle |
| `couple` | two labs through a 3.5 ms cable loop: sympathy (A struck, B silent at unison / fifth / minor third), worst-case stability (T60 30 s, coupling full, cable gain 1.0 and 1.15 - must still decay), bowed-then-lifted pair must still decay |
| `stress` | coupled pair, every control random every 50 ms for 60 s: must stay finite. A crash test - writes no WAV |
| `bench` | native ns/sample. The M7 is NOT this core scaled by clock: handpan ran ~110x realtime here and 50-75 % on the module |

Single-instrument WAVs are the OUT jack on both channels. Coupled scenes are
lab A left, lab B right.

Level trims are measured, not guessed: build with
`EXTRA='-DCHLADNI_OUT_TRIM=1e-3f -DCHLADNI_BRIDGE_TRIM=1e-3f'` and read the
raw peaks back.
