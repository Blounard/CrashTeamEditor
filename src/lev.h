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

struct Weather
{
	Vec3 velocity;
	Color colorTop;
	Color colorBottom; 
	uint32_t fillMode; 
	int OTindex; 
};
enum class WeatherPreset { CUSTOM, RAIN, SNOW };

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

struct WaterAnimSettings
{
	float waveLength = 20.0f;  // world units 

	// For BaseUV (static tex part)
	float sizeTex = 20.0f;   // Size Of the full texture in world unit	

	// For ScrollUV (UV scrolling like a conveyer belt)
	int   ScrollULoops = 1; // Speed
	int   ScrollVLoops = 1;

	// For WaveUV  (Small perturbation)
	int waveCyclesTimeU = 3; // Speed
	int waveCyclesTimeV = 3;
	float waveAmplitude = 16.0f; // Size of the UV perturbation (in pixels)

	// For brightness :   
	float baseBrightness = 4.0f; // Base
	float brightAmp = 2.5f; // Amplitude
	int	  brightWaveCycle = 1;   //Speed
};
