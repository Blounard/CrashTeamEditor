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
	MinimapOrientation orientationMode = MinimapOrientation::RIGHT;
	int16_t unk = 0;
	Texture texture;
	bool enabled = false;

	MinimapConfig() = default;
	void LoadFromPSX(const PSX::Map& map);
	void CalculateWorldBoundsFromQuadblocks(const std::vector<Quadblock>& quadblocks);
	PSX::Map Serialize() const;
	bool IsReady() const;
	bool RenderUI(const std::vector<Quadblock>& quadblocks, std::function<void(void)> refreshTextureStores, const std::filesystem::path& parentDir);
	void Clear();
	bool GenerateMinimap(const std::vector<Quadblock>& quadblocks,
		const std::filesystem::path& outputDir,
		const std::string& textureName,
		int targetHeight = 87,
		MinimapOrientation orientation = MinimapOrientation::RIGHT);
};
