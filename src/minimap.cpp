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


    constexpr int32_t kMinimapAnchorScreenX = 495;
    constexpr int32_t kMinimapAnchorBaseScreenY = 193;


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

bool MinimapConfig::RenderUI(const std::vector<Quadblock>& quadblocks, std::function<void(void)> refreshTextureStores, const std::filesystem::path& parentDir, const std::map<std::string, std::vector<size_t>>& materialMap)
{
	bool boundsChanged = false;
	
	ImGui::Checkbox("Enable Minimap", &enabled);

	if (!enabled) { return false; }

    ImGui::Separator();
    static MinimapSettings minimapSettings;
    
    ImGui::InputInt("Target Height##minimap", &minimapSettings.textureHeight);
    ImGui::Checkbox("Use checkpoint quads##minimap", &minimapSettings.checkpointQuads);
    ImGui::Checkbox("Use checkpoint pathable quads##minimap", &minimapSettings.checkpointPathableQuads);
    const char* orientationModes[] = { "0°", "90°", "180°", "270°", "Auto"};
    int selectOrientation = static_cast<int>(minimapSettings.orientation);
    if (ImGui::Combo("Relative rotation##minimapsettings", &selectOrientation, orientationModes, 5))
    {
        minimapSettings.orientation = static_cast<MinimapOrientation>(selectOrientation);
    }
    if (ImGui::TreeNode("Materials##minimapsettings"))
    {

        if (ImGui::BeginCombo("##minimapmatcombo", minimapSettings.previewMatName.c_str()))
        {
            for (const auto& [material, indexes] : materialMap)
            {
                if (ImGui::Selectable(material.c_str()))
                {
                    minimapSettings.previewMatName = material;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Add Material##minimapsettinbgd"))
            minimapSettings.materials.insert(minimapSettings.previewMatName);


        std::vector<std::string> toDel;
        for (const std::string& matName : minimapSettings.materials)
        {
            ImGui::Text(matName.c_str());
            ImGui::SameLine();
            if (ImGui::Button("Delete##minimapsettingsmaterial"))
                toDel.push_back(matName);
        }
        for (const std::string& matName : toDel)
            minimapSettings.materials.erase(matName);
        ImGui::TreePop();
    }


    if (ImGui::Button("AutoGenerate##minimap"))
    {
        GenerateMinimap(quadblocks, parentDir, "minimap", minimapSettings);
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













namespace
{
    constexpr float kFullCoverageEpsilon = 0.01f;
    // "semi-transparent" flag after 16-bit VRAM
    // conversion - this exact number is arbitrary

    // One Sutherland-Hodgman clip pass against a single half-plane.
    template <typename InsideFn, typename IntersectFn>
    void ClipHalfPlane(std::vector<Vec2>& poly, InsideFn inside, IntersectFn intersect)
    {
        if (poly.empty()) { return; }
        std::vector<Vec2> out;
        out.reserve(poly.size() + 1);
        for (size_t i = 0; i < poly.size(); i++)
        {
            const Vec2& curr = poly[i];
            const Vec2& prev = poly[(i + poly.size() - 1) % poly.size()];
            const bool currIn = inside(curr);
            const bool prevIn = inside(prev);
            if (currIn)
            {
                if (!prevIn) { out.push_back(intersect(prev, curr)); }
                out.push_back(curr);
            }
            else if (prevIn)
            {
                out.push_back(intersect(prev, curr));
            }
        }
        poly = std::move(out);
    }

    // Exact area of a triangle clipped against axis-aligned pixel box [x0,x1] x [y0,y1].
    float ClipTriangleToBoxArea(Vec2 p0, Vec2 p1, Vec2 p2, float x0, float y0, float x1, float y1)
    {
        std::vector<Vec2> poly = { p0, p1, p2 };

        ClipHalfPlane(poly, [&](const Vec2& p) { return p.x >= x0; },
            [&](const Vec2& a, const Vec2& b) { const float t = (x0 - a.x) / (b.x - a.x); return Vec2{ x0, a.y + t * (b.y - a.y) }; });
        ClipHalfPlane(poly, [&](const Vec2& p) { return p.x <= x1; },
            [&](const Vec2& a, const Vec2& b) { const float t = (x1 - a.x) / (b.x - a.x); return Vec2{ x1, a.y + t * (b.y - a.y) }; });
        ClipHalfPlane(poly, [&](const Vec2& p) { return p.y >= y0; },
            [&](const Vec2& a, const Vec2& b) { const float t = (y0 - a.y) / (b.y - a.y); return Vec2{ a.x + t * (b.x - a.x), y0 }; });
        ClipHalfPlane(poly, [&](const Vec2& p) { return p.y <= y1; },
            [&](const Vec2& a, const Vec2& b) { const float t = (y1 - a.y) / (b.y - a.y); return Vec2{ a.x + t * (b.x - a.x), y1 }; });
        if (poly.size() < 3) { return 0.0; }
        float area2 = 0.0;
        for (size_t i = 0; i < poly.size(); i++)
        {
            const Vec2& a = poly[i];
            const Vec2& b = poly[(i + 1) % poly.size()];
            area2 += (a.x * b.y) - (b.x * a.y);
        }
        return std::fabs(area2) * 0.5f;
    }
}

bool MinimapConfig::GenerateMinimap(const std::vector<Quadblock>& quadblocks, const std::filesystem::path& outputDir, const std::string& textureName, const MinimapSettings settings)
{
    Clear();
    int targetHeight = settings.textureHeight;
    if (targetHeight % 2 == 0)
    {
        printf("WARNING: MinimapConfig targetHeight (%d) must be odd, using %d instead\n", targetHeight, targetHeight - 1);
        targetHeight -= 1;
    }
    if (targetHeight < 3)
    {
        printf("WARNING: MinimapConfig targetHeight (%d) too small once padding is reserved, using 3 instead\n", targetHeight);
        targetHeight = 3;
    }
    const int contentHeight = targetHeight - 1;

    // Build quad list to use for the minimap 
    std::vector<size_t> usedQuadIds;
    for (size_t i = 0; i < quadblocks.size(); i++)
    {
        if (settings.checkpointQuads && quadblocks[i].GetCheckpoint() != -1)
            usedQuadIds.push_back(i);
        else if (settings.checkpointPathableQuads && quadblocks[i].GetCheckpointPathable())
            usedQuadIds.push_back(i);
        else if (settings.materials.contains(quadblocks[i].GetMaterial()))
            usedQuadIds.push_back(i);
    }
    if (usedQuadIds.empty()) return false;

    // Build Triangle list
    std::vector<Tri> tris;
    for (size_t i : usedQuadIds)
    {
        for (const std::array<size_t, 3>& face : quadblocks[i].GetTriFacesIndexes())
        {
            const std::array<Vec3, 3> f = quadblocks[i].GetTriFace(face[0], face[1], face[2]);
            Tri t;
            for (int j = 0; j < 3; j++) { t.p[j].pos = f[j]; }
            tris.push_back(t);
        }
    }

    // Build Bounding Box
    BoundingBox worldBox = BoundingBox::Empty();
    for (const Tri& t : tris)
    {
        for (int i = 0; i < 3; i++)
            worldBox.Expand(t.p[i].pos);
    }
    worldStartX = worldBox.min.x;
    worldEndX = worldBox.max.x;
    worldStartZ = worldBox.min.z;
    worldEndZ = worldBox.max.z;

    const float spanX = worldEndX - worldStartX;
    const float spanZ = worldEndZ - worldStartZ;

    // World -> pixel mapping
    if (settings.orientation == MinimapOrientation::AUTO)
        if (spanX > spanZ)
            orientationMode = MinimapOrientation::DOWN;
        else
            orientationMode = MinimapOrientation::RIGHT;
    else
        orientationMode = settings.orientation;

    const bool swapped = (orientationMode == MinimapOrientation::DOWN || orientationMode == MinimapOrientation::UP);
    const float colSpanWorld = swapped ? spanZ : spanX;
    const float rowSpanWorld = swapped ? spanX : spanZ;
    constexpr float minimapStretchX = 1.6f;
    const int contentWidth = std::max(1, static_cast<int>(std::lround(contentHeight * (colSpanWorld * minimapStretchX) / rowSpanWorld)));
    const int targetWidth = contentWidth + 1; // One extra column reserved the same way as the padding row (see below).

    auto toPixelSpace = [&](const Vec3& worldPos) // Convert World Pos to Pixel coordinate on the image
        {
            float x = worldPos.x, z = worldPos.z;
            float colFrac = 0.0, rowFrac = 0.0;
            switch (orientationMode)
            {
            case MinimapOrientation::RIGHT: colFrac = (x - worldStartX) / spanX; rowFrac = (z - worldStartZ) / spanZ; break;
            case MinimapOrientation::DOWN:  colFrac = (worldEndZ - z) / spanZ;   rowFrac = (x - worldStartX) / spanX; break;
            case MinimapOrientation::LEFT:  colFrac = (worldEndX - x) / spanX;   rowFrac = (worldEndZ - z) / spanZ;   break;
            case MinimapOrientation::UP:    colFrac = (z - worldStartZ) / spanZ; rowFrac = (worldEndX - x) / spanX;   break;
            }
            // Multiplied by CONTENT dimensions, not the padded targetWidth/targetHeight, so real
            // geometry only ever lands in [0, contentWidth) x [0, contentHeight) - the last
            // column/row of the canvas stays architecturally empty, not just empty by luck.
            Vec2 res{};
            res.x = colFrac * contentWidth;
            res.y = rowFrac * contentHeight;
            return res;
        };

    // Coverage calculation
    std::vector<float> coverage(static_cast<size_t>(targetWidth) * targetHeight, 0.0);
    for (const Tri& t : tris)
    {
        Vec2 p0 = toPixelSpace(t.p[0].pos);
        Vec2 p1 = toPixelSpace(t.p[1].pos);
        Vec2 p2 = toPixelSpace(t.p[2].pos);

        const int pxMin = Clamp(static_cast<int>(std::floor(std::min({ p0.x, p1.x, p2.x }))), 0, contentWidth - 1);
        const int pxMax = Clamp(static_cast<int>(std::floor(std::max({ p0.x, p1.x, p2.x }))), 0, contentWidth - 1);
        const int pyMin = Clamp(static_cast<int>(std::floor(std::min({ p0.y, p1.y, p2.y }))), 0, contentHeight - 1);
        const int pyMax = Clamp(static_cast<int>(std::floor(std::max({ p0.y, p1.y, p2.y }))), 0, contentHeight - 1);
        for (int py = pyMin; py <= pyMax; py++)
        {
            for (int px = pxMin; px <= pxMax; px++)
            {
                const float area = ClipTriangleToBoxArea(p0, p1, p2, static_cast<float>(px), static_cast<float>(py), static_cast<float>(px + 1), static_cast<float>(py + 1));
                if (area > 0.0) { coverage[static_cast<size_t>(py) * targetWidth + px] += area; } // This assume quads don't overlap for the formula to be correct.
            }
        }
    }

    const float extCol = colSpanWorld / static_cast<float>(contentWidth);
    const float extRow = rowSpanWorld / static_cast<float>(contentHeight);
    switch (orientationMode)
    {
    case MinimapOrientation::RIGHT: worldEndX += extCol; worldEndZ += extRow; break;
    case MinimapOrientation::DOWN:  worldStartZ -= extCol; worldEndX += extRow; break;
    case MinimapOrientation::LEFT:  worldStartX -= extCol; worldStartZ -= extRow; break;
    case MinimapOrientation::UP:    worldEndZ += extCol; worldStartX -= extRow; break;
    }

    // Colors
    std::vector<uint8_t> rgba(coverage.size() * 4);
    for (size_t i = 0; i < coverage.size(); i++)
    {
        constexpr int colorCount = 16;
        int level = std::min(static_cast<int>(coverage[i] * colorCount), colorCount - 1);
        uint8_t color = static_cast<uint8_t>(Clamp(std::round(level * 255.0f / (colorCount - 1)), 0.0f, 255.0f));
        uint8_t r = color;
        uint8_t g = color;
        uint8_t b = color;
        uint8_t a;
        if (color == 0 || color == 255)
            a = 255;
        else
            a = 128;
        rgba[i * 4 + 0] = r;
        rgba[i * 4 + 1] = g;
        rgba[i * 4 + 2] = b;
        rgba[i * 4 + 3] = a;
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
    texture.SetBlendMode(static_cast<uint16_t>(PSX::BlendMode::ADDITIVE));

    enabled = true;
    return true;
}