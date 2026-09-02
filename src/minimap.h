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

	const int32_t bboxSizeX = static_cast<int32_t>(out.worldEndX) - static_cast<int32_t>(out.worldStartX);
	const int32_t bboxSizeZ = static_cast<int32_t>(out.worldEndZ) - static_cast<int32_t>(out.worldStartZ);
	if (bboxSizeX == 0 || bboxSizeZ == 0)
		return out;

	switch (map.orientationMode)
	{
	case MinimapOrientation::RIGHT: // 0 deg
		out.driverDotStartX -= static_cast<int16_t>((static_cast<int32_t>(out.worldEndX) * out.iconSizeX) / bboxSizeX);
		out.driverDotStartY -= static_cast<int16_t>((static_cast<int32_t>(out.worldEndZ) * out.iconSizeY * 2) / bboxSizeZ);
		break;
	case MinimapOrientation::DOWN: // 90 deg
		out.driverDotStartX += static_cast<int16_t>((static_cast<int32_t>(out.worldStartZ) * out.iconSizeX) / bboxSizeZ);
		out.driverDotStartY -= static_cast<int16_t>((static_cast<int32_t>(out.worldEndX) * out.iconSizeY * 2) / bboxSizeX);
		break;
	case MinimapOrientation::LEFT: // 180 deg
		out.driverDotStartX += static_cast<int16_t>((static_cast<int32_t>(out.worldStartX) * out.iconSizeX) / bboxSizeX);
		out.driverDotStartY += static_cast<int16_t>((static_cast<int32_t>(out.worldStartZ) * out.iconSizeY * 2) / bboxSizeZ);
		break;
	case MinimapOrientation::UP: // 270 deg
		out.driverDotStartX -= static_cast<int16_t>((static_cast<int32_t>(out.worldEndZ) * out.iconSizeX) / bboxSizeZ);
		out.driverDotStartY += static_cast<int16_t>((static_cast<int32_t>(out.worldStartX) * out.iconSizeY * 2) / bboxSizeX);
		break;
	}
	return out;
}