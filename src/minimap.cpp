#include "minimap.h"
#include "quadblock.h"
#include "texture.h"

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <portable-file-dialogs.h>
#include <algorithm>
#include <limits>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <vector>

#include "stb_image_write.h"



void MinimapConfig::LoadFromPSX(const PSX::Map& map)
{
	worldEndX = ConvertFP(map.worldEndX, FP_ONE_GEO);
	worldEndZ = ConvertFP(map.worldEndZ, FP_ONE_GEO);
	worldStartX = ConvertFP(map.worldStartX, FP_ONE_GEO);
	worldStartZ = ConvertFP(map.worldStartZ, FP_ONE_GEO);
	orientationMode = static_cast<MinimapOrientation>(map.orientationMode);
    const int32_t bboxSizeX = static_cast<int32_t>(map.worldEndX) - static_cast<int32_t>(map.worldStartX);
    const int32_t bboxSizeZ = static_cast<int32_t>(map.worldEndZ) - static_cast<int32_t>(map.worldStartZ);
}




PSX::Map MinimapConfig::Serialize() const
{
	PSX::Map map = {};
	map.worldEndX = ConvertFloat(worldEndX, FP_ONE_GEO);
	map.worldEndZ = ConvertFloat(worldEndZ, FP_ONE_GEO);
	map.worldStartX = ConvertFloat(worldStartX, FP_ONE_GEO);
	map.worldStartZ = ConvertFloat(worldStartZ, FP_ONE_GEO);
	// Icon size is the texture dimensions
	map.iconSizeX = static_cast<int16_t>(texture.GetWidth());
	map.iconSizeY = 1 + static_cast<int16_t>(texture.GetHeight()/2);
    map.orientationMode = static_cast<int16_t>(orientationMode);
    map.unk = 0;


    const int32_t bboxSizeX = map.worldEndX - map.worldStartX;
    const int32_t bboxSizeZ = map.worldEndZ - map.worldStartZ;
    if (bboxSizeX == 0 || bboxSizeZ == 0) { return map; }


    constexpr int32_t MinimapAnchorScreenX = 495;
    constexpr int32_t MinimapAnchorBaseScreenY = 193;


    int32_t driverX = 0;
    int32_t driverBaseY = 0;
    switch (orientationMode)
    {
    case MinimapOrientation::RIGHT: // 0 deg
        driverX = MinimapAnchorScreenX - (static_cast<int32_t>(map.worldEndX * map.iconSizeX)) / bboxSizeX;
        driverBaseY = MinimapAnchorBaseScreenY - static_cast<int32_t>((map.worldEndZ * map.iconSizeY * 2)) / bboxSizeZ;
        break;
    case MinimapOrientation::DOWN: // 90 deg
        driverX = MinimapAnchorScreenX + static_cast<int32_t>((map.worldStartZ * map.iconSizeX)) / bboxSizeZ;
        driverBaseY = MinimapAnchorBaseScreenY - static_cast<int32_t>((map.worldEndX * map.iconSizeY * 2)) / bboxSizeX;
        break;
    case MinimapOrientation::LEFT: // 180 deg
        driverX = MinimapAnchorScreenX + static_cast<int32_t>((map.worldStartX * map.iconSizeX)) / bboxSizeX;
        driverBaseY = MinimapAnchorBaseScreenY + static_cast<int32_t>((map.worldStartZ * map.iconSizeY * 2)) / bboxSizeZ;
        break;
    case MinimapOrientation::UP: // 270 deg
        driverX = MinimapAnchorScreenX - static_cast<int32_t>((map.worldEndZ * map.iconSizeX)) / bboxSizeZ;
        driverBaseY = MinimapAnchorBaseScreenY + static_cast<int32_t>((map.worldStartX * map.iconSizeY * 2)) / bboxSizeX;
        break;
    }

	map.driverDotStartX = static_cast<int16_t>(driverX);
	map.driverDotStartY = static_cast<int16_t>(driverBaseY + 16);
	return map;
}

