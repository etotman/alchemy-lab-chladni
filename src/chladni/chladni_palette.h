/**
 * chladni_palette.h — Ring and button colours, one hue per page.
 *
 *   Play  amber    the exciter: how it is struck and bowed
 *   Body  cyan     what is struck: material, decay, coupling
 *   Box   rose     what it sounds through: box, rosin, level
 *
 * B2 flashes white on a strike. B3 is the bow, and its colour is the bow's
 * state: green while it holds Helmholtz motion (one slip a period), amber
 * while it slips more often (the thin, scratchy regime), red when it slips
 * less (pressed too hard, raucous).
 */

#pragma once

#include "alchemy/led/panel.h"

using Rgb = alchemy::LedPanel::Rgb;

constexpr Rgb kPlayOn   = {0xFF, 0xA8, 0x20};
constexpr Rgb kPlayOff  = {0x18, 0x0A, 0x01};

constexpr Rgb kBodyOn   = {0x30, 0xE0, 0xFF};
constexpr Rgb kBodyOff  = {0x02, 0x10, 0x14};

constexpr Rgb kBoxOn    = {0xFF, 0x50, 0x90};
constexpr Rgb kBoxOff   = {0x16, 0x04, 0x0A};

constexpr Rgb kStrikeFlash = {0xFF, 0xFF, 0xFF};
constexpr Rgb kStrikeIdle  = {0x10, 0x10, 0x10};

constexpr Rgb kBowIdle     = {0x04, 0x0C, 0x04};
constexpr Rgb kBowClean    = {0x30, 0xFF, 0x50};
constexpr Rgb kBowMulti    = {0xFF, 0xA0, 0x10};
constexpr Rgb kBowRaucous  = {0xFF, 0x20, 0x10};
constexpr Rgb kBowLatch    = {0x10, 0x60, 0x20};   /* latched, before it speaks */
