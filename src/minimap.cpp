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
	unk = map.unk;
	enabled = true;
    const int32_t bboxSizeX = static_cast<int32_t>(map.worldEndX) - static_cast<int32_t>(map.worldStartX);
    const int32_t bboxSizeZ = static_cast<int32_t>(map.worldEndZ) - static_cast<int32_t>(map.worldStartZ);
    if (bboxSizeX == 0 || bboxSizeZ == 0 || map.iconSizeX == 0 || map.iconSizeY == 0)
    {
        printf("MinimapConfig::LoadFromPSX: cannot back-solve anchor (degenerate bounds or icon size)\n");
        return;
    }

    const int32_t driverBaseY = static_cast<int32_t>(map.driverDotStartY) - 16;
    int32_t anchorX = 0;
    int32_t anchorBaseY = 0;
    switch (orientationMode)
    {
    case MinimapOrientation::RIGHT: // 0 deg
        anchorX = map.driverDotStartX + (static_cast<int32_t>(map.worldEndX) * map.iconSizeX) / bboxSizeX;
        anchorBaseY = driverBaseY + (static_cast<int32_t>(map.worldEndZ) * map.iconSizeY * 2) / bboxSizeZ;
        break;
    case MinimapOrientation::DOWN: // 90 deg
        anchorX = map.driverDotStartX - (static_cast<int32_t>(map.worldStartZ) * map.iconSizeX) / bboxSizeZ;
        anchorBaseY = driverBaseY + (static_cast<int32_t>(map.worldEndX) * map.iconSizeY * 2) / bboxSizeX;
        break;
    case MinimapOrientation::LEFT: // 180 deg
        anchorX = map.driverDotStartX - (static_cast<int32_t>(map.worldStartX) * map.iconSizeX) / bboxSizeX;
        anchorBaseY = driverBaseY - (static_cast<int32_t>(map.worldStartZ) * map.iconSizeY * 2) / bboxSizeZ;
        break;
    case MinimapOrientation::UP: // 270 deg
        anchorX = map.driverDotStartX + (static_cast<int32_t>(map.worldEndZ) * map.iconSizeX) / bboxSizeZ;
        anchorBaseY = driverBaseY - (static_cast<int32_t>(map.worldStartX) * map.iconSizeY * 2) / bboxSizeX;
        break;
    }

    printf("MinimapConfig::LoadFromPSX: back-solved anchor -> ScreenX=%d, BaseScreenY=%d "
        "(orientation=%d, iconSize=%dx%d, worldBounds=[%d,%d]-[%d,%d])\n",
        anchorX, anchorBaseY, static_cast<int>(orientationMode),
        map.iconSizeX, map.iconSizeY,
        map.worldStartX, map.worldStartZ, map.worldEndX, map.worldEndZ);
}


void MinimapConfig::CalculateWorldBoundsFromQuadblocks(const std::vector<Quadblock>& quadblocks)
{
	if (quadblocks.empty()) { return; }

	worldStartX = std::numeric_limits<float>::max();
	worldStartZ = std::numeric_limits<float>::max();
	worldEndX = std::numeric_limits<float>::lowest();
	worldEndZ = std::numeric_limits<float>::lowest();

    bool found = false;
    for (const Quadblock& qb : quadblocks)
    {
        // Only include quadblocks that have a checkpoint assigned
        if (qb.GetCheckpoint() >= 0)
        {
            const BoundingBox& bbox = qb.GetBoundingBox();
			worldStartX = std::min(worldStartX, bbox.min.x);
			worldStartZ = std::min(worldStartZ, bbox.min.z);
			worldEndX = std::max(worldEndX, bbox.max.x);
			worldEndZ = std::max(worldEndZ, bbox.max.z);
            found = true;
        }
    }
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
    map.unk = unk;


    const int16_t bboxSizeX = map.worldEndX - map.worldStartX;
    const int16_t bboxSizeZ = map.worldEndZ - map.worldStartZ;
    if (bboxSizeX == 0 || bboxSizeZ == 0) { return map; }


    constexpr int32_t kMinimapAnchorScreenX = 494;
    constexpr int32_t kMinimapAnchorBaseScreenY = 191;


    int32_t driverX = 0;
    int32_t driverBaseY = 0;
    switch (orientationMode)
    {
    case MinimapOrientation::RIGHT: // 0 deg
        driverX = kMinimapAnchorScreenX - (map.worldEndX * map.iconSizeX) / bboxSizeX;
        driverBaseY = kMinimapAnchorBaseScreenY - (map.worldEndZ * map.iconSizeY * 2) / bboxSizeZ;
        break;
    case MinimapOrientation::DOWN: // 90 deg
        driverX = kMinimapAnchorScreenX + (map.worldStartZ * map.iconSizeX) / bboxSizeZ;
        driverBaseY = kMinimapAnchorBaseScreenY - (map.worldEndX * map.iconSizeY * 2) / bboxSizeX;
        break;
    case MinimapOrientation::LEFT: // 180 deg
        driverX = kMinimapAnchorScreenX + (map.worldStartX * map.iconSizeX) / bboxSizeX;
        driverBaseY = kMinimapAnchorBaseScreenY + (map.worldStartZ * map.iconSizeY * 2) / bboxSizeZ;
        break;
    case MinimapOrientation::UP: // 270 deg
        driverX = kMinimapAnchorScreenX - (map.worldEndZ * map.iconSizeX) / bboxSizeZ;
        driverBaseY = kMinimapAnchorBaseScreenY + (map.worldStartX * map.iconSizeY * 2) / bboxSizeX;
        break;
    }

	map.driverDotStartX = static_cast<int16_t>(driverX);
	map.driverDotStartY = static_cast<int16_t>(driverBaseY + 16);
	return map;
}




bool MinimapConfig::IsReady() const
{
	return enabled && !texture.IsEmpty() && texture.GetHeight()%2 != 0;
}

void MinimapConfig::Clear()
{
	worldEndX = 0;
	worldEndZ = 0;
	worldStartX = 0;
	worldStartZ = 0;
	orientationMode = MinimapOrientation::RIGHT;
	unk = 0;
	texture.ClearTexture();
	enabled = false;
}

bool MinimapConfig::RenderUI(const std::vector<Quadblock>& quadblocks, std::function<void(void)> refreshTextureStores, const std::filesystem::path& parentDir)
{
	bool boundsChanged = false;
	
	ImGui::Checkbox("Enable Minimap", &enabled);

	if (!enabled) { return false; }

    ImGui::Separator();
    static int targetHeightMinimapGeneration = 87;
    static float aspectRatioMinimapGeneration = 1.6f;
    static MinimapOrientation orientationMiniampGeneration = MinimapOrientation::RIGHT;
    
    ImGui::InputInt("Target Height##minimap", &targetHeightMinimapGeneration);
    ImGui::InputFloat("Aspect Ratio##minimap", &aspectRatioMinimapGeneration);
    if (ImGui::Button("AutoGenerate##minimap"))
    {
        GenerateMinimap(quadblocks, parentDir, "minimap", targetHeightMinimapGeneration, orientationMiniampGeneration, aspectRatioMinimapGeneration);
    }

	ImGui::Separator();
	ImGui::Text("World Bounds:");
	
	if (ImGui::InputFloat("World Start X", &worldStartX, 1.0f, 10.0f, "%.2f"))
	{
		boundsChanged = true;
	}
	if (ImGui::InputFloat("World Start Y", &worldStartZ, 1.0f, 10.0f, "%.2f"))
	{
		boundsChanged = true;
	}
	if (ImGui::InputFloat("World End X", &worldEndX, 1.0f, 10.0f, "%.2f"))
	{
		boundsChanged = true;
	}
	if (ImGui::InputFloat("World End Y", &worldEndZ, 1.0f, 10.0f, "%.2f"))
	{
		boundsChanged = true;
	}

	if (ImGui::Button("Calculate from Quadblocks"))
	{
		CalculateWorldBoundsFromQuadblocks(quadblocks);
		boundsChanged = true;
	}
	ImGui::SetItemTooltip("(Experimental) Automatically calculate world bounds from quadblocks with checkpoints");

	ImGui::Separator();
	ImGui::Text("Minimap Orientation:");
	
	// Orientation mode dropdown
	const char* orientationModes[] = { "0°", "90°", "180°", "270°" };
	int currentOrientation = static_cast<int>(orientationMode);
	if (currentOrientation < 0 || currentOrientation > 3) { currentOrientation = 0; }
	if (ImGui::Combo("Relative rotation", &currentOrientation, orientationModes, 4))
	{
		orientationMode = static_cast<MinimapOrientation>(currentOrientation);
	}
	ImGui::SetItemTooltip("Determines minimap clockwise rotation relative to the world\n It doesnt affect texture orientation, it affects how the driver icon moves on the minimap");
	
	ImGui::Separator();
	ImGui::InputScalar("Unknown", ImGuiDataType_S16, &unk);
	ImGui::SetItemTooltip("???");

	ImGui::Separator();
	ImGui::Text("Textures (both halves must have the same dimensions):");

	std::vector<Quadblock> dummy;
	texture.RenderUI({}, dummy, refreshTextureStores);


	// Status display
	ImGui::Separator();
	ImGui::Text("Texture Size: %dx%d pixels", texture.GetWidth(), texture.GetHeight());
	if (IsReady())
	{
		ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Minimap ready!");
	}
	else if (!texture.IsEmpty())
	{
		ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Need an odd texture height");
	}
	else
	{
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "No minimap textures loaded");
	}
	
	return boundsChanged;
}















bool MinimapConfig::GenerateMinimap(const std::vector<Quadblock>& quadblocks,
    const std::filesystem::path& outputDir,
    const std::string& textureName,
    int targetHeight,
    MinimapOrientation orientation,
    float aspectRatio,
    int dotRadius,
    bool flipX,
    bool flipZ)
{
    Clear();

    if (targetHeight % 2 == 0)
    {
        printf("WARNING: MinimapConfig targetHeight (%d) must be odd, using %d instead\n", targetHeight, targetHeight + 1);
        targetHeight += 1;
    }

    // --- 1. Gather (x, z) centers of drivable quadblocks only. Height (y) is unused entirely. ---
    struct Point { float x, z; };
    std::vector<Point> worldPoints;
    worldPoints.reserve(quadblocks.size());
    for (const Quadblock& qb : quadblocks)
    {
        if (qb.GetCheckpoint() < 0) { continue; }
        const Vec3 c = qb.GetCenter();
        worldPoints.push_back({ c.x, c.z });
    }
    if (worldPoints.empty()) { return false; }

    // --- 2. Raw world bounds, BEFORE rotation/stretch/flip. This is what gets stored in the
    //        config fields - presumably what the game normalizes the live player (x, z) against,
    //        so it must stay in plain, untouched world space. ---
    float worldMinX = std::numeric_limits<float>::max(), worldMaxX = std::numeric_limits<float>::lowest();
    float worldMinZ = std::numeric_limits<float>::max(), worldMaxZ = std::numeric_limits<float>::lowest();
    for (const Point& p : worldPoints)
    {
        worldMinX = std::min(worldMinX, p.x); worldMaxX = std::max(worldMaxX, p.x);
        worldMinZ = std::min(worldMinZ, p.z); worldMaxZ = std::max(worldMaxZ, p.z);
    }
    worldStartX = worldMinX; worldEndX = worldMaxX;
    worldStartZ = worldMinZ; worldEndZ = worldMaxZ;

    // --- 3. World -> render space: rotate, then stretch X, then optionally flip either axis.
    //        Applied identically to every track point AND the spawn point below. ---
    constexpr float kPi = MATH_PI;
    const float rotationDeg = 90.0f * static_cast<int>(orientation);
    const float rad = rotationDeg * (kPi / 180.0f);
    const float cosR = std::cos(rad), sinR = std::sin(rad);
    auto toRenderSpace = [&](float x, float z, float& outX, float& outZ)
        {
            float rx = x * cosR - z * sinR;
            float rz = x * sinR + z * cosR;
            rx *= aspectRatio;
            if (flipX) { rx = -rx; }
            if (flipZ) { rz = -rz; }
            outX = rx;
            outZ = rz;
        };

    std::vector<Point> renderPoints(worldPoints.size());
    float meshMinX = std::numeric_limits<float>::max(), meshMaxX = std::numeric_limits<float>::lowest();
    float meshMinZ = std::numeric_limits<float>::max(), meshMaxZ = std::numeric_limits<float>::lowest();
    for (size_t i = 0; i < worldPoints.size(); i++)
    {
        toRenderSpace(worldPoints[i].x, worldPoints[i].z, renderPoints[i].x, renderPoints[i].z);
        meshMinX = std::min(meshMinX, renderPoints[i].x); meshMaxX = std::max(meshMaxX, renderPoints[i].x);
        meshMinZ = std::min(meshMinZ, renderPoints[i].z); meshMaxZ = std::max(meshMaxZ, renderPoints[i].z);
    }
    const float meshW = meshMaxX - meshMinX;
    const float meshH = meshMaxZ - meshMinZ;
    if (meshW <= 0.0f || meshH <= 0.0f) { return false; }

    // --- 4. One scale factor, derived purely from height. Width falls out of it - no fixed
    //        target box, so no letterboxing/offset math needed; the mesh bounding box maps
    //        exactly onto the image bounding box. ---
    const float scaleFactor = static_cast<float>(targetHeight) / meshH;
    const int targetWidth = std::max(1, static_cast<int>(std::lround(meshW * scaleFactor)));

    // --- 5. Plot each center as a small filled (blocky, non-antialiased) square. ---
    std::vector<uint8_t> gray(static_cast<size_t>(targetWidth) * targetHeight, 0);
    auto plot = [&](int px, int py)
        {
            for (int dy = -dotRadius; dy <= dotRadius; dy++)
            {
                const int y = py + dy;
                if (y < 0 || y >= targetHeight) { continue; }
                for (int dx = -dotRadius; dx <= dotRadius; dx++)
                {
                    const int x = px + dx;
                    if (x < 0 || x >= targetWidth) { continue; }
                    gray[static_cast<size_t>(y) * targetWidth + x] = 255;
                }
            }
        };
    for (const Point& p : renderPoints)
    {
        const int px = static_cast<int>(std::lround((p.x - meshMinX) * scaleFactor));
        const int py = static_cast<int>(std::lround((p.z - meshMinZ) * scaleFactor));
        plot(px, py);
    }

    // --- 6. Pure black / pure white, always fully opaque. Texture::ConvertColor already turns
    //        opaque (0,0,0,255) into the special "transparent under additive" 16-bit value, so
    //        nothing special is needed here. ---
    std::vector<uint8_t> rgba(gray.size() * 4);
    for (size_t i = 0; i < gray.size(); i++)
    {
        const uint8_t v = gray[i];
        rgba[i * 4 + 0] = 255 ; // was v
        rgba[i * 4 + 1] = v;
        rgba[i * 4 + 2] = v;
        rgba[i * 4 + 3] = 255;
    }

    std::error_code ec;
    std::filesystem::create_directories(outputDir, ec);
    const std::filesystem::path pngPath = outputDir / (textureName + ".png");
    if (!stbi_write_png(pngPath.string().c_str(), targetWidth, targetHeight, 4, rgba.data(), targetWidth * 4))
    {
        printf("ERROR: Failed to write minimap PNG for %s\n", textureName.c_str());
        return false;
    }

    texture = Texture(pngPath);
    if (texture.IsEmpty())
    {
        printf("ERROR: Failed to load generated minimap texture %s\n", pngPath.string().c_str());
        return false;
    }
    texture.SetBlendMode(static_cast<uint16_t>(PSX::BlendMode::ADDITIVE_TRANSLUCENT)); // Additive (1)

    //  7 : Orientation
    orientationMode = orientation;

    enabled = true;
    return true;
}