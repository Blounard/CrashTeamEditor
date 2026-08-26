#include "minimap.h"
#include "quadblock.h"

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <portable-file-dialogs.h>
#include <algorithm>
#include <limits>

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

bool MinimapConfig::RenderUI(const std::vector<Quadblock>& quadblocks, std::function<void(void)> refreshTextureStores)
{
	bool boundsChanged = false;
	
	ImGui::Checkbox("Enable Minimap", &enabled);

	if (!enabled) { return false; }

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
