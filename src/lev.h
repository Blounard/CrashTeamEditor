#pragma once

#include "geo.h"

#include <vector>
#include <string>
#include <cstdint>

static constexpr size_t NUM_GRADIENT = 3;
static constexpr size_t NUM_DRIVERS = 8;
static constexpr size_t NUM_LEV_CONFIG_FLAGS = 3;
static constexpr size_t GHOST_DATA_FILESIZE = 0x3E00;
static constexpr size_t NUM_VERTICES_QUADBLOCK = 9;
static constexpr size_t OT_SIZE = 1024;
static constexpr size_t NUM_FRAME_OVERT = 28;

struct LevConfigFlags
{
	static constexpr uint32_t NONE = 0;
	static constexpr uint32_t ENABLE_SKYBOX_GRADIENT = 1 << 0;
	static constexpr uint32_t MASK_GRAB_UNDERWATER = 1 << 1;
	static constexpr uint32_t ANIMATE_WATER_VERTEX = 1 << 2;
};

struct Spawn
{
	Vec3 pos;
	Vec3 rot;
};

struct ColorGradient
{
	float posFrom;
	float posTo;
	Color colorFrom;
	Color colorTo;
};

struct Stars
{
    uint16_t numStars;
    bool spread;
    uint16_t seed;
    uint16_t zDepth;
};

static const std::vector<std::string> CTR_CHARACTERS = {
	"Crash Bandicoot", "Dr. Neo Cortex", "Tiny Tiger", "Coco Bandicoot",
	"N. Gin", "Dingodile", "Polar", "Pura", "Pinstripe", "Papu Papu",
	"Ripper Roo", "Komodo Joe", "N. Tropy", "Penta Penguin",
	"Fake Crash", "Nitrous Oxide"
};

struct WaterAnimParams
{
    float texelsPerUnit = 8.0f;   // tiling density: texels of the 64x64 map per world unit
    float flowDirX = 1.0f, flowDirZ = 0.3f; // flow direction in XZ
    int   flowLoopsU = 0;          // integer tile-widths crossed over the 28-frame loop (seamless)
    int   flowLoopsV = 0;

    float rippleAmpU = 20.0f, rippleAmpV = 20.0f; // texel amplitude of the wave distortion
    float rippleFreq = 0.15f;      // spatial frequency (1 / world units)
    int   rippleCyclesTime = 2;    // integer temporal cycles over the loop (seamless)

	float baseBrightness = 4.0f;
	float brightAmp = 2.5f;

	// 2D traveling wave, user-facing parameterization
	float brightWaveLength = 16.0f;  // world units between crests
	int	  brightWaveCycle = 1;   // world units per OVert frame (not per second — see note below)
	float brightWaveDirectionDeg = 0.0f; // propagation direction: 0 = +X, 90 = +Z
	float brightPhaseDeg = 0.0f;

    float seed = 0.0f;             // vary between separate, unconnected water bodies
};