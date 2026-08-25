#pragma once

#include "geo.h"
#include "psx_types.h"
#include "texture.h"

#include <filesystem>
#include <string>
#include <cstdint>
#include <vector>

class Quadblock;

struct MinimapConfig
{
	float worldEndX = 0;
	float worldEndZ = 0;
	float worldStartX = 0;
	float worldStartZ = 0;

	// Screen position for driver dots (screen size is 512x252)
	int16_t driverDotStartX = 450;
	int16_t driverDotStartY = 180;

	// Orientation mode - determines minimap orientation relative to the world
	// 0 = Right, 1 = Down, 2 = Left, 3 = Up
	int16_t orientationMode = 0;
	
	// Unknown field - used for drawing, needed for some levels like Crash Cove
	int16_t unk = 0;

	Texture texture; // MAKE SURE THIS HAD EVEN HEIGHT!
	 
	// State flags
	bool enabled = false;

	// Calculate world bounds from all quadblocks in the level
	void CalculateWorldBoundsFromQuadblocks(const std::vector<Quadblock>& quadblocks);
	
	// Serialize to PSX Map struct
	PSX::Map Serialize() const;
	
	// Deserialize from PSX Map struct
	void Deserialize(const PSX::Map& map);
	
	// Check if minimap is ready for export
	bool IsReady() const;
	
	// Render the ImGui UI for minimap configuration
	// Returns true if world bounds were modified (for updating visualization)
	bool RenderUI(const std::vector<Quadblock>& quadblocks, std::function<void(void)> refreshTextureStores);
	
	// Clear all minimap data
	void Clear();
};
