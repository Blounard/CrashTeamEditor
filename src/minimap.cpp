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
        GenerateMinimap(quadblocks, parentDir, "minimap", targetHeightMinimapGeneration, orientationMiniampGeneration);//, aspectRatioMinimapGeneration);
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













namespace
{
    constexpr float kMinimapStretchX = 1.6f;  // fixed horizontal display compensation
    constexpr float kFullCoverageEpsilon = 0.01f;
    // "semi-transparent" flag after 16-bit VRAM
    // conversion - this exact number is arbitrary

    struct Pt { double x, y; };

    // One Sutherland-Hodgman clip pass against a single half-plane.
    template <typename InsideFn, typename IntersectFn>
    void ClipHalfPlane(std::vector<Pt>& poly, InsideFn inside, IntersectFn intersect)
    {
        if (poly.empty()) { return; }
        std::vector<Pt> out;
        out.reserve(poly.size() + 1);
        for (size_t i = 0; i < poly.size(); i++)
        {
            const Pt& curr = poly[i];
            const Pt& prev = poly[(i + poly.size() - 1) % poly.size()];
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
    double ClipTriangleToBoxArea(const Pt tri[3], double x0, double y0, double x1, double y1)
    {
        std::vector<Pt> poly = { tri[0], tri[1], tri[2] };
        ClipHalfPlane(poly, [&](const Pt& p) { return p.x >= x0; },
            [&](const Pt& a, const Pt& b) { const double t = (x0 - a.x) / (b.x - a.x); return Pt{ x0, a.y + t * (b.y - a.y) }; });
        ClipHalfPlane(poly, [&](const Pt& p) { return p.x <= x1; },
            [&](const Pt& a, const Pt& b) { const double t = (x1 - a.x) / (b.x - a.x); return Pt{ x1, a.y + t * (b.y - a.y) }; });
        ClipHalfPlane(poly, [&](const Pt& p) { return p.y >= y0; },
            [&](const Pt& a, const Pt& b) { const double t = (y0 - a.y) / (b.y - a.y); return Pt{ a.x + t * (b.x - a.x), y0 }; });
        ClipHalfPlane(poly, [&](const Pt& p) { return p.y <= y1; },
            [&](const Pt& a, const Pt& b) { const double t = (y1 - a.y) / (b.y - a.y); return Pt{ a.x + t * (b.x - a.x), y1 }; });
        if (poly.size() < 3) { return 0.0; }
        double area2 = 0.0;
        for (size_t i = 0; i < poly.size(); i++)
        {
            const Pt& a = poly[i];
            const Pt& b = poly[(i + 1) % poly.size()];
            area2 += (a.x * b.y) - (b.x * a.y);
        }
        return std::fabs(area2) * 0.5;
    }
}

bool MinimapConfig::GenerateMinimap(const std::vector<Quadblock>& quadblocks,
    const std::filesystem::path& outputDir,
    const std::string& textureName,
    int targetHeight,
    MinimapOrientation orientation)
{
    Clear();

    if (targetHeight % 2 == 0)
    {
        printf("WARNING: MinimapConfig targetHeight (%d) must be odd, using %d instead\n", targetHeight, targetHeight + 1);
        targetHeight += 1;
    }
    if (targetHeight <= 0) { return false; }

    // --- 1. Collect triangles (world X/Z only) from drivable quadblocks. ---
    struct Tri { float x[3], z[3]; };
    std::vector<Tri> tris;
    for (const Quadblock& qb : quadblocks)
    {
        if (qb.GetCheckpoint() < 0) { continue; }
        for (const std::array<size_t, 3>&face : qb.GetTriFacesIndexes())
        {
            const std::array<Vec3, 3> f = qb.GetTriFace(face[0], face[1], face[2]);
            Tri t;
            for (int i = 0; i < 3; i++) { t.x[i] = f[i].x; t.z[i] = f[i].z; }
            tris.push_back(t);
        }
    }
    if (tris.empty()) { return false; }

    // --- 2. Raw world bounds (unrotated, unstretched) - what Serialize()'s icon formula uses.
    //        Computed from FULL triangle vertices (not just centers), so the box tightly matches
    //        the actual road width, not just its centerline. Note: this is a slightly different
    //        bounding box than the earlier center-only version used - worth re-checking any
    //        anchor values derived against a center-based bbox if you compare across versions.
    float worldMinX = std::numeric_limits<float>::max(), worldMaxX = std::numeric_limits<float>::lowest();
    float worldMinZ = std::numeric_limits<float>::max(), worldMaxZ = std::numeric_limits<float>::lowest();
    for (const Tri& t : tris)
    {
        for (int i = 0; i < 3; i++)
        {
            worldMinX = std::min(worldMinX, t.x[i]); worldMaxX = std::max(worldMaxX, t.x[i]);
            worldMinZ = std::min(worldMinZ, t.z[i]); worldMaxZ = std::max(worldMaxZ, t.z[i]);
        }
    }
    worldStartX = worldMinX; worldEndX = worldMaxX;
    worldStartZ = worldMinZ; worldEndZ = worldMaxZ;
    const float spanX = worldEndX - worldStartX;
    const float spanZ = worldEndZ - worldStartZ;
    if (spanX <= 0.0f || spanZ <= 0.0f) { return false; }

    if (spanX > spanZ)
        orientation = MinimapOrientation::UP;

    // --- 3. World -> pixel mapping: exact per-orientation axis remap (see explanation above),
    //        NOT a generic float rotation - guarantees the image matches the runtime icon math's
    //        axis convention exactly, with no trig imprecision.
    const bool swapped = (orientation == MinimapOrientation::DOWN || orientation == MinimapOrientation::UP);
    const float colSpanWorld = swapped ? spanZ : spanX;
    const float rowSpanWorld = swapped ? spanX : spanZ;
    const int targetWidth = std::max(1, static_cast<int>(std::lround(targetHeight * (colSpanWorld * kMinimapStretchX) / rowSpanWorld)));

    auto toPixelSpace = [&](float x, float z, double& outPx, double& outPy)
        {
            double colFrac = 0.0, rowFrac = 0.0;
            switch (orientation)
            {
            case MinimapOrientation::RIGHT: colFrac = (x - worldStartX) / spanX; rowFrac = (z - worldStartZ) / spanZ; break;
            case MinimapOrientation::DOWN:  colFrac = (worldEndZ - z) / spanZ;   rowFrac = (x - worldStartX) / spanX; break;
            case MinimapOrientation::LEFT:  colFrac = (worldEndX - x) / spanX;   rowFrac = (worldEndZ - z) / spanZ;   break;
            case MinimapOrientation::UP:    colFrac = (z - worldStartZ) / spanZ; rowFrac = (worldEndX - x) / spanX;   break;
            }
            outPx = colFrac * targetWidth;
            outPy = rowFrac * targetHeight;
        };

    // --- 4. Transform every triangle into pixel space up front. ---
    struct PixelTri { Pt v[3]; };
    std::vector<PixelTri> pixelTris(tris.size());
    for (size_t i = 0; i < tris.size(); i++)
    {
        for (int j = 0; j < 3; j++) { toPixelSpace(tris[i].x[j], tris[i].z[j], pixelTris[i].v[j].x, pixelTris[i].v[j].y); }
    }

    // --- 5. Exact per-pixel coverage via polygon clipping. Each pixel box is a 1x1 unit square
    //        in this space, so the clipped area IS the coverage fraction. Clamped to 100% per
    //        pixel to handle overlapping quadblocks (see caveat in the message above this code). ---
    std::vector<double> coverage(static_cast<size_t>(targetWidth) * targetHeight, 0.0);
    for (const PixelTri& t : pixelTris)
    {
        const double minXf = std::min({ t.v[0].x, t.v[1].x, t.v[2].x });
        const double maxXf = std::max({ t.v[0].x, t.v[1].x, t.v[2].x });
        const double minYf = std::min({ t.v[0].y, t.v[1].y, t.v[2].y });
        const double maxYf = std::max({ t.v[0].y, t.v[1].y, t.v[2].y });
        const int pxMin = std::clamp(static_cast<int>(std::floor(minXf)), 0, targetWidth - 1);
        const int pxMax = std::clamp(static_cast<int>(std::floor(maxXf)), 0, targetWidth - 1);
        const int pyMin = std::clamp(static_cast<int>(std::floor(minYf)), 0, targetHeight - 1);
        const int pyMax = std::clamp(static_cast<int>(std::floor(maxYf)), 0, targetHeight - 1);
        for (int py = pyMin; py <= pyMax; py++)
        {
            for (int px = pxMin; px <= pxMax; px++)
            {
                const double area = ClipTriangleToBoxArea(t.v, px, py, px + 1, py + 1);
                if (area > 0.0) { coverage[static_cast<size_t>(py) * targetWidth + px] += area; }
            }
        }
    }

    // --- 6. Coverage fraction -> color: 0% = opaque black, 100% = opaque white, otherwise grey
    //        (scaled by coverage) at a fixed semi-transparent alpha. ---
    std::vector<uint8_t> rgba(coverage.size() * 4);
    for (size_t i = 0; i < coverage.size(); i++)
    {
        const double c = std::clamp(coverage[i], 0.0, 1.0);
        uint8_t r, g, b, a;
        if (c <= kFullCoverageEpsilon) { r = g = b = 0; a = 255; }
        else if (c >= 1.0 - kFullCoverageEpsilon) { r = g = b = 255; a = 255; }
        else { r = g = b = (static_cast<uint8_t>(std::lround(c * 255.0))/32) * 32 ; a = 128; }
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
    texture.SetBlendMode(static_cast<uint16_t>(PSX::BlendMode::ADDITIVE_TRANSLUCENT));

    orientationMode = orientation;
    enabled = true;
    return true;
}