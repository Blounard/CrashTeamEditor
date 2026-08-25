#pragma once

#include "geo.h"
#include "psx_types.h"
#include "texture.h"

#include <filesystem>
#include <string>
#include <cstdint>
#include <vector>

class Quadblock;

enum class MinimapOrientation : int
{
	RIGHT = 0,
	DOWN = 1, 
	LEFT = 2, 
	UP = 3
};

struct MinimapConfig
{
	
	float worldEndX = 0;
	float worldEndZ = 0;
	float worldStartX = 0;
	float worldStartZ = 0;
	int16_t driverDotStartX = 450;
	int16_t driverDotStartY = 180;
	MinimapOrientation orientationMode = MinimapOrientation::RIGHT;
	int16_t unk = 0;
	Texture texture;
	bool enabled = false;

	MinimapConfig() = default;
	MinimapConfig(const PSX::Map& map);
	void CalculateWorldBoundsFromQuadblocks(const std::vector<Quadblock>& quadblocks);
	PSX::Map Serialize() const;
	bool IsReady() const;
	bool RenderUI(const std::vector<Quadblock>& quadblocks, std::function<void(void)> refreshTextureStores);
	void Clear();
};
