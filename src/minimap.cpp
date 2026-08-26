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
	driverDotStartX = map.driverDotStartX;
	driverDotStartY = map.driverDotStartY;
	orientationMode = static_cast<MinimapOrientation>(map.orientationMode);
	unk = map.unk;
	enabled = true;
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
	map.driverDotStartX = driverDotStartX;
	map.driverDotStartY = driverDotStartY;
	map.orientationMode = static_cast<int16_t>(orientationMode);
	map.unk = unk;
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
	driverDotStartX = 450;
	driverDotStartY = 180;
	orientationMode = MinimapOrientation::RIGHT;
	unk = 0;
	texture.ClearTexture();
	enabled = false;
}

bool MinimapConfig::RenderUI(const std::vector<Quadblock>& quadblocks, std::function<void(void)> refreshTextureStores, const std::filesystem::path& parentDir, const Vec3& spawnPos)
{
	bool boundsChanged = false;
	
	ImGui::Checkbox("Enable Minimap", &enabled);

	if (!enabled) { return false; }

    ImGui::Separator();
    static int targetHeightMinimapGeneration = 87;
    static float aspectRatioMinimapGeneration = 1.6f;
    
    ImGui::InputInt("Target Height##minimap", &targetHeightMinimapGeneration);
    ImGui::InputFloat("Aspect Ratio##minimap", &aspectRatioMinimapGeneration);
    if (ImGui::Button("AutoGenerate##minimap"))
    {
        GenerateMinimap(quadblocks, parentDir, "minimap", spawnPos, targetHeightMinimapGeneration, aspectRatioMinimapGeneration);
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
	ImGui::Text("Driver Icon Start Position (screen %d x %d):", PSX::SCREEN_WIDTH, PSX::SCREEN_HEIGHT);
	if (ImGui::InputScalar("Icon Start X", ImGuiDataType_S16, &driverDotStartX)) {
		Clamp(driverDotStartX, static_cast<int16_t>(0), PSX::SCREEN_WIDTH);
	}
	if (ImGui::InputScalar("Icon Start Y", ImGuiDataType_S16, &driverDotStartY)) {
		Clamp(driverDotStartY, static_cast<int16_t>(0), PSX::SCREEN_HEIGHT);
	}

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
    const Vec3& spawnPos,
    int targetHeight,
    float aspectRatio,
    float rotationDeg,
    int16_t mapPosX,
    int16_t mapPosY,
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
    constexpr float kPi = 3.14159265358979323846f;
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
        rgba[i * 4 + 0] = v;
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

    // --- 7. Driver dot: push the spawn position through the EXACT same pipeline as the track
    //        points, then offset by wherever the minimap image is actually drawn on screen. ---
    float spawnRx, spawnRz;
    toRenderSpace(spawnPos.x, spawnPos.z, spawnRx, spawnRz);
    const float spawnPxF = (spawnRx - meshMinX) * scaleFactor;
    const float spawnPyF = (spawnRz - meshMinZ) * scaleFactor;

    // Assumes mapPosX/mapPosY is the BOTTOM-RIGHT corner of the on-screen minimap (as in the
    // old python tool). If the dot is off by a constant amount in-game, try top-left instead:
    //   screenX = mapPosX + spawnPxF;  screenY = mapPosY + spawnPyF;
    const float screenX = (mapPosX - targetWidth) + spawnPxF;
    const float screenY = (mapPosY - targetHeight) + spawnPyF;
    driverDotStartX = static_cast<int16_t>(std::lround(screenX));
    driverDotStartY = static_cast<int16_t>(std::lround(screenY));

    int rotMode = static_cast<int>(std::lround(rotationDeg)) / 90 % 4;
    if (rotMode < 0) { rotMode += 4; }
    orientationMode = static_cast<MinimapOrientation>(rotMode);

    enabled = true;
    return true;
}