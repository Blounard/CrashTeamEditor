#pragma once

#include "geo.h"
#include "psx_types.h"
#include "texture.h"

#include <filesystem>
#include <string>
#include <cstdint>
#include <vector>
#include <set>
#include <map>

enum class MinimapOrientation : int
{
	RIGHT = 0,
	DOWN = 1,
	LEFT = 2,
	UP = 3,
	AUTO = 4
};

struct Minimap
{
	BoundingBox worldBox;
	MinimapOrientation orientationMode = MinimapOrientation::RIGHT;
	Texture texture;
};

struct MinimapSettings
{
	int textureHeight = 87;
	MinimapOrientation orientation = MinimapOrientation::AUTO;
	std::set<std::string> materials = {};
	std::string previewMatName = "";
	bool checkpointQuads = true;
	bool checkpointPathableQuads = true;
};

static inline Minimap ConvertMinimap(const PSX::Minimap& map)
{
	Minimap out{};
	out.worldBox.min.x = ConvertFP(map.worldStartX, FP_ONE_GEO);
	out.worldBox.min.z = ConvertFP(map.worldStartZ, FP_ONE_GEO);
	out.worldBox.max.x = ConvertFP(map.worldEndX, FP_ONE_GEO);
	out.worldBox.max.z = ConvertFP(map.worldEndZ, FP_ONE_GEO);
	out.worldBox.min.y = -10.0f;
	out.worldBox.max.y = 10.0f;
	out.orientationMode = static_cast<MinimapOrientation>(map.orientationMode);
	out.texture.ClearTexture(); // filled elsewhere
	return out;
}

static inline PSX::Minimap ConvertMinimap(const Minimap& map)
{
	PSX::Minimap out{};
	out.worldStartX = ConvertFloat(map.worldBox.min.x, FP_ONE_GEO);
	out.worldStartZ = ConvertFloat(map.worldBox.min.z, FP_ONE_GEO);
	out.worldEndX = ConvertFloat(map.worldBox.max.x, FP_ONE_GEO);
	out.worldEndZ = ConvertFloat(map.worldBox.max.z, FP_ONE_GEO);
	out.orientationMode = static_cast<int16_t>(map.orientationMode);
	out.iconSizeX = static_cast<int16_t>(map.texture.GetWidth());
	out.iconSizeY = 1 + static_cast<int16_t>(map.texture.GetHeight() / 2);

	constexpr int16_t MinimapAnchorScreenX = 495;
	constexpr int16_t MinimapAnchorBaseScreenY = 193;
	out.driverDotStartX = MinimapAnchorScreenX;
	out.driverDotStartY = MinimapAnchorBaseScreenY + 16;

	float bboxSizeX = map.worldBox.AxisLength().x;
	float bboxSizeZ = map.worldBox.AxisLength().z;

	if (bboxSizeX < EPSILON || bboxSizeZ < EPSILON)
		return out;

	switch (map.orientationMode)
	{
	case MinimapOrientation::RIGHT: // 0 deg
		out.driverDotStartX -= static_cast<int16_t>(static_cast<float>(out.iconSizeX) * map.worldBox.max.x / bboxSizeX);
		out.driverDotStartY -= static_cast<int16_t>(static_cast<float>(out.iconSizeY * 2) * map.worldBox.max.z / bboxSizeZ);
		break;
	case MinimapOrientation::DOWN: // 90 deg
		out.driverDotStartX += static_cast<int16_t>(static_cast<float>(out.iconSizeX) * map.worldBox.min.z / bboxSizeZ);
		out.driverDotStartY -= static_cast<int16_t>(static_cast<float>(out.iconSizeY * 2) * map.worldBox.max.x / bboxSizeX);
		break;
	case MinimapOrientation::LEFT: // 180 deg
		out.driverDotStartX += static_cast<int16_t>(static_cast<float>(out.iconSizeX) * map.worldBox.min.x / bboxSizeX);
		out.driverDotStartY += static_cast<int16_t>(static_cast<float>(out.iconSizeY * 2) * map.worldBox.min.z / bboxSizeZ);
		break;
	case MinimapOrientation::UP: // 270 deg
		out.driverDotStartX -= static_cast<int16_t>(static_cast<float>(out.iconSizeX) * map.worldBox.max.z / bboxSizeZ);
		out.driverDotStartY += static_cast<int16_t>(static_cast<float>(out.iconSizeY * 2) * map.worldBox.min.x / bboxSizeX);
		break;
	}
	return out;
}
