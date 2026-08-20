#include "geo.h"
#include "bsp.h"
#include "checkpoint.h"
#include "level.h"
#include "path.h"
#include "quadblock.h"
#include "vertex.h"
#include "utils.h"
#include "gui_render_settings.h"
#include "mesh.h"
#include "texture.h"
#include "ui.h"
#include "script.h"
#include "minimap.h"

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <portable-file-dialogs.h>

#include <string>
#include <chrono>
#include <functional>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <algorithm>

class ButtonUI
{
public:
	ButtonUI();
	ButtonUI(long long timeout);
	bool Show(const std::string& label, const std::string& message, bool unsavedChanges);

private:
	static constexpr long long DEFAULT_TIMEOUT = 1;
	long long m_timeout;
	std::string m_labelTriggered;
	std::chrono::time_point<std::chrono::high_resolution_clock> m_messageTimeoutStart;
};

ButtonUI::ButtonUI()
{
	m_timeout = DEFAULT_TIMEOUT;
	m_labelTriggered = std::string();
	m_messageTimeoutStart = std::chrono::high_resolution_clock::now();
}

ButtonUI::ButtonUI(long long timeout)
{
	m_timeout = timeout;
	m_labelTriggered = std::string();
	m_messageTimeoutStart = std::chrono::high_resolution_clock::now();
}

bool ButtonUI::Show(const std::string& label, const std::string& message, bool unsavedChanges)
{
	bool ret = false;
	if (ImGui::Button(label.c_str()))
	{
		m_labelTriggered = label;
		m_messageTimeoutStart = std::chrono::high_resolution_clock::now();
		ret = true;
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - m_messageTimeoutStart).count() < m_timeout
		&& m_labelTriggered == label)
	{
		ImGui::Text(message.c_str());
	}
	else if (unsavedChanges)
	{
		const ImVec4 redColor = {247.0f / 255.0f, 44.0f / 255.0f, 37.0f / 255.0f, 1.0f};
		ImGui::PushStyleColor(ImGuiCol_Text, redColor);
		ImGui::Text("Unsaved changes.");
		ImGui::PopStyleColor();
	}
	return ret;
}

template<typename T>
static bool UIFlagCheckbox(T& var, const T flag, const std::string& title)
{
	bool active = var & flag;
	if (ImGui::Checkbox(title.c_str(), &active))
	{
		if (active) { var |= flag; }
		else { var &= ~flag; }
		return true;
	}
	return false;
}


static bool ModelIdWidget(const char* label, ModelId* id)
{
	int16_t raw = static_cast<int16_t>(*id);
	ImGui::SetNextItemWidth(180.0f);
	bool changed = ImGui::InputScalar(label, ImGuiDataType_S16, &raw);
	if (changed)
		*id = static_cast<ModelId>(raw);

	ImGui::SameLine();

	auto it = ModelIdLabels.find(*id);
	const char* currentLabel = (it != ModelIdLabels.end()) ? it->second : "Unknown";

	ImGui::SetNextItemWidth(180.0f);
	if (ImGui::BeginCombo("##ModelIdCombo", currentLabel))
	{
		for (const auto& [entryId, entryLabel] : ModelIdLabels)
		{
			bool isSelected = (entryId == *id);
			if (ImGui::Selectable(entryLabel, isSelected))
			{
				*id = entryId;
				changed = true;
			}
			if (isSelected)
				ImGui::SetItemDefaultFocus();
		}
		ImGui::EndCombo();
	}

	return changed;
}

void BoundingBox::RenderUI() const
{
	ImGui::Text("Max:"); ImGui::SameLine();
	ImGui::BeginDisabled();
	ImGui::InputFloat3("##max", const_cast<float*>(&max.x));
	ImGui::EndDisabled();
	ImGui::Text("Min:"); ImGui::SameLine();
	ImGui::BeginDisabled();
	ImGui::InputFloat3("##min", const_cast<float*>(&min.x));
	ImGui::EndDisabled();
}

void BSP::RenderUI(const std::vector<Quadblock>& quadblocks)
{
	std::string title = GetType() + " " + std::to_string(m_id);
	if (ImGui::TreeNode(title.c_str()))
	{
		if (IsBranch()) { ImGui::Text(("Axis:  " + GetAxis()).c_str()); }
		ImGui::Text(("Quads: " + std::to_string(m_quadblockIndexes.size())).c_str());
		if (ImGui::TreeNode(("Quadblock List: (" + std::to_string(m_quadblockIndexes.size()) + ")").c_str()))
		{
			constexpr size_t QUADS_PER_LINE = 10;
			for (size_t i = 0; i < m_quadblockIndexes.size(); i++)
			{
				ImGui::Text((quadblocks[m_quadblockIndexes[i]].GetName() + ", ").c_str());
				if (((i + 1) % QUADS_PER_LINE) == 0 || i == m_quadblockIndexes.size() - 1) { continue; }
				ImGui::SameLine();
			}
			ImGui::TreePop();
		}
		ImGui::Text("Bounding Box:");
		m_bbox.RenderUI();
		if (IsBranch())
		{
			if (ImGui::Button("Merge"))
			{
				MergeBranch();
			}
			ImGui::SetItemTooltip("Transform this branch into a leaf by merging all it's child leaves together");
		}
		else
		{
			static AxisSplit BSPaxisSplit = AxisSplit::NONE;
			static float BSPmidpointSplit = 0.0f;
			ImGui::SetNextItemWidth(200.0f);
			if (ImGui::BeginCombo("Axis", AxisSplitNames[static_cast<int>(BSPaxisSplit)]))
			{
				for (int i = 0; i < 4; ++i)
				{
					AxisSplit currentAxis = static_cast<AxisSplit>(i);
					bool isSelected = (BSPaxisSplit == currentAxis);

					if (ImGui::Selectable(AxisSplitNames[i], isSelected))
					{
						BSPaxisSplit = currentAxis;
					}

					// Set initial focus to the currently selected item when opening the combo
					if (isSelected)
					{
						ImGui::SetItemDefaultFocus();
					}
				}
				ImGui::EndCombo();
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(200.0f);
			ImGui::InputFloat("Midpoint", &BSPmidpointSplit);
			ImGui::SameLine();
			if (ImGui::Button("Split"))
			{
				SplitLeafGeometry(quadblocks, BSPaxisSplit, BSPmidpointSplit);
			}
		}
		if (m_left) { m_left->RenderUI(quadblocks); }
		if (m_right) { m_right->RenderUI(quadblocks); }
		ImGui::TreePop();
	}
}

void Checkpoint::RenderUI(size_t numCheckpoints, const std::vector<Quadblock>& quadblocks)
{
	if (ImGui::TreeNode(("Checkpoint " + std::to_string(m_index)).c_str()))
	{
		ImGui::Text("Pos:       "); ImGui::SameLine(); ImGui::InputFloat3("##pos", m_pos.Data());
		ImGui::Text("Quad:      "); ImGui::SameLine();
		if (ImGui::BeginCombo("##quad", m_uiPosQuad.c_str()))
		{
			for (const Quadblock& quadblock : quadblocks)
			{
				if (ImGui::Selectable(quadblock.GetName().c_str()))
				{
					m_uiPosQuad = quadblock.GetName();
					m_pos = quadblock.GetCenter();
				}
			}
			ImGui::EndCombo();
		}
		ImGui::SetItemTooltip("Update checkpoint position by selecting a specific quadblock.");

		ImGui::Text("Distance:  "); ImGui::SameLine();
		if (ImGui::InputFloat("##dist", &m_distToFinish)) { m_distToFinish = m_distToFinish < 0.0f ? 0.0f : m_distToFinish; }
		ImGui::SetItemTooltip("Distance from checkpoint to the finish line.");

		auto LinkUI = [](int index, size_t numCheckpoints, int& dir, std::string& s, const std::string& title)
			{
				if (dir == NONE_CHECKPOINT_INDEX) { s = DEFAULT_UI_CHECKBOX_LABEL; }
				ImGui::Text(title.c_str()); ImGui::SameLine();
				if (ImGui::BeginCombo(("##" + title).c_str(), s.c_str()))
				{
					if (ImGui::Selectable(DEFAULT_UI_CHECKBOX_LABEL.c_str())) { dir = NONE_CHECKPOINT_INDEX; }
					for (int i = 0; i < numCheckpoints; i++)
					{
						if (i == index) { continue; }
						std::string str = "Checkpoint " + std::to_string(i);
						if (ImGui::Selectable(str.c_str()))
						{
							s = str;
							dir = i;
						}
					}
					ImGui::EndCombo();
				}
			};

		LinkUI(m_index, numCheckpoints, m_up, m_uiLinkUp, "Link up:   ");
		LinkUI(m_index, numCheckpoints, m_down, m_uiLinkDown, "Link down: ");
		LinkUI(m_index, numCheckpoints, m_left, m_uiLinkLeft, "Link left: ");
		LinkUI(m_index, numCheckpoints, m_right, m_uiLinkRight, "Link right:");

		if (ImGui::Button("Delete")) { m_delete = true; }
		ImGui::TreePop();
	}
}

void BotNode::RenderUI(int index, bool& deleteRequested)
{
	const std::string nodeLabel = "Node " + std::to_string(index);
	if (ImGui::TreeNode(nodeLabel.c_str()))
	{
		// Position
		float pos[3] = { m_pos.x, m_pos.y, m_pos.z };
		if (ImGui::DragFloat3("Position", pos, 1.0f))
		{
			m_pos.x = pos[0];
			m_pos.y = pos[1];
			m_pos.z = pos[2];
		}

		// Rotation
		ImGui::DragFloat("Yaw", &m_rot.y, 0.5f, -180.0f, 180.0f, "%.1f deg");
		ImGui::DragFloat("Pitch", &m_rot.x, 0.5f, -180.0f, 180.0f, "%.1f deg");
		ImGui::DragFloat("Roll", &m_rot.z, 0.5f, -180.0f, 180.0f, "%.1f deg");

		// Flags — one checkbox per named flag bit
		ImGui::SeparatorText("Flags");
		auto FlagCheckbox = [&](const char* label, uint16_t bit)
			{
				bool v = (m_flags & bit) != 0;
				if (ImGui::Checkbox(label, &v))
					m_flags = v ? (m_flags | bit) : (m_flags & ~bit);
			};
		FlagCheckbox("Turbo Pad (High)", BotNodeFlags::TURBO_PAD_HIGH);
		FlagCheckbox("Skidmarks Front", BotNodeFlags::SKIDMARKS_FRONT);
		FlagCheckbox("Skidmarks Back", BotNodeFlags::SKIDMARKS_BACK);
		FlagCheckbox("Turbo Pad (Low)", BotNodeFlags::TURBO_PAD_LOW);
		FlagCheckbox("Mask Grab STP", BotNodeFlags::MASK_GRAB_STP);
		FlagCheckbox("Jump", BotNodeFlags::JUMP);
		FlagCheckbox("Drift Left", BotNodeFlags::DRIFT_LEFT);
		FlagCheckbox("Drift Right", BotNodeFlags::DRIFT_RIGHT);
		FlagCheckbox("Engine Echo", BotNodeFlags::ENGINE_ECHO);
		FlagCheckbox("Mid Air", BotNodeFlags::MID_AIR);
		FlagCheckbox("Sink Kart", BotNodeFlags::SINK_KART);

		// Terrain dropdown — built from TerrainType::LABELS, sorted by value
		ImGui::SeparatorText("Terrain");
		// Build a sorted list once, reuse across frames
		static std::vector<std::pair<std::string, uint8_t>> terrainList;
		if (terrainList.empty())
		{
			for (const auto& [name, val] : TerrainType::LABELS)
				terrainList.emplace_back(name, val);
			std::sort(terrainList.begin(), terrainList.end(),
				[](const auto& a, const auto& b) { return a.second < b.second; });
		}
		// Find current terrain label
		std::string currentLabel = "Unknown";
		for (const auto& [name, val] : terrainList)
			if (val == m_terrain) { currentLabel = name; break; }

		if (ImGui::BeginCombo("Terrain Type", currentLabel.c_str()))
		{
			for (const auto& [name, val] : terrainList)
			{
				bool selected = (val == m_terrain);
				if (ImGui::Selectable(name.c_str(), selected))
					m_terrain = val;
				if (selected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		// Path change
		ImGui::SeparatorText("Path Change");
		ImGui::InputInt("Path Change OpCode", &m_pathChange);
		ImGui::InputInt("Path Change Index", &m_pathChangeIndex);

		// Misc
		ImGui::SeparatorText("Misc");
		int goBack = static_cast<int>(m_goBackCount);
		if (ImGui::InputInt("Go Back Count", &goBack))
			m_goBackCount = static_cast<uint8_t>(std::clamp(goBack, 0, 255));
		int special = static_cast<int>(m_specialBits);
		if (ImGui::InputInt("Special Bits", &special))
			m_specialBits = static_cast<uint8_t>(std::clamp(special, 0, 255));

		// Delete button at the bottom of each node
		ImGui::Spacing();
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
		if (ImGui::Button(("Delete Node##" + std::to_string(index)).c_str()))
			deleteRequested = true;
		ImGui::PopStyleColor(3);

		ImGui::TreePop();
	}
}

void BotPath::RenderUI(int pathIndex)
{
	const std::string pathLabel = "Nodes";
	if (ImGui::TreeNode(pathLabel.c_str()))
	{
		ImGui::Text("Nodes: %zu", m_nodes.size());

		std::vector<int> toDelete;
		for (int i = 0; i < static_cast<int>(m_nodes.size()); i++)
		{
			bool deleteRequested = false;
			// Push id to avoid TreeNode label collisions across paths
			ImGui::PushID(i);
			m_nodes[i].RenderUI(i, deleteRequested);
			ImGui::PopID();
			if (deleteRequested)
				toDelete.push_back(i);
		}

		// Process deletions in reverse to preserve indices
		if (!toDelete.empty())
		{
			for (int i = static_cast<int>(toDelete.size()) - 1; i >= 0; i--)
				m_nodes.erase(m_nodes.begin() + toDelete[i]);
		}

		if (ImGui::Button(("Add Node##path" + std::to_string(pathIndex)).c_str()))
		{
			// Default-construct a new node; position it at the last node's
			// position if available so it doesn't spawn at the world origin
			BotNode newNode;
			if (!m_nodes.empty())
				newNode.SetPos(m_nodes.back().GetPos());
			m_nodes.push_back(newNode);
		}

		ImGui::TreePop();
	}
}


bool Instance::RenderUI(bool& shouldDelete, bool& shouldDuplicate, int index, const std::unordered_map<size_t, InstanceModel>& modelInstances, Vec3& queryPoint, std::vector<Quadblock>& quadblocks)
{
	bool modelChanged = false;

	std::string headerLabel = m_name.empty() ? ("Instance " + std::to_string(index + 1)) : m_name;
	if (ImGui::CollapsingHeader((headerLabel + "###instHeader").c_str()))
	{
		// Model selection dropdown
		if (ImGui::BeginCombo("Model", modelInstances.at(m_modelKey).GetName().c_str()))
		{
			for (const auto& [key, model] : modelInstances)
			{
				bool isSelected = (m_modelKey == key);
				if (ImGui::Selectable(model.GetName().c_str(), isSelected))
				{
					m_modelKey = key;
					modelChanged = true;
				}
				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		// Instance name
		char nameBuffer[64] = {};
		m_name.copy(nameBuffer, sizeof(nameBuffer) - 1);
		if (ImGui::InputText("Name", nameBuffer, sizeof(nameBuffer)))
		{
			m_name = std::string(nameBuffer, strnlen(nameBuffer, sizeof(nameBuffer)));
		}

		// Model ID selector (behavior selector)
		{
			ModelId prevModelID = m_modelID;

			ModelIdWidget("Model ID", &m_modelID);

			// Auto-set hitbox when model ID changes to a crate or wumpa fruit
			if (m_modelID != prevModelID)
			{
				if (m_modelID == ModelId::WUMPA_FRUIT)
				{
					m_hitbox.enabled = true;
					m_hitbox.preset = InstanceHitbox::PICKUP;
					m_hitbox.yOffset = 0.0f;
				}
				else if (m_modelID == ModelId::EXPLOSIVE_CRATE || m_modelID == ModelId::FRUIT_CRATE ||
				         m_modelID == ModelId::RANDOM_CRATE || m_modelID == ModelId::TIME_CRATE_1 ||
				         m_modelID == ModelId::TIME_CRATE_2 || m_modelID == ModelId::TIME_CRATE_3)
				{
					m_hitbox.enabled = true;
					m_hitbox.preset = InstanceHitbox::PICKUP;
					m_hitbox.yOffset = 0.71875f;
				}
			}
		}

		ImGui::Text("Pos:");
		ImGui::SameLine();
		ImGui::DragFloat3("##pos", m_pos.Data(), 0.5f);
		ImGui::SameLine();
		if (ImGui::Button(("Set from selection##Instance")))
		{
			m_pos = queryPoint;
		}

		ImGui::Text("Rot:"); 
		ImGui::SameLine();
		if (ImGui::DragFloat3("##rot", m_rot.Data(), 1.0f, -360.0f, 360.0f))
		{
			m_rot.x = Clamp(m_rot.x, -360.0f, 360.0f);
			m_rot.y = Clamp(m_rot.y, -360.0f, 360.0f);
			m_rot.z = Clamp(m_rot.z, -360.0f, 360.0f);
		};
		ImGui::SameLine();
		if (ImGui::Button(("Snap to ground##Instance")))
		{
			std::vector<size_t> quadindexes;
			for (size_t j = 0; j < quadblocks.size(); j++)
			{
				if (quadblocks[j].GetFlags() & QuadFlags::GROUND)
					quadindexes.push_back(j);
			}
			SnapToClosestQuad(quadblocks, quadindexes, m_pos, m_rot, Vec3(0.0f, 1.0f, 0.0f), -1.0f, 1.0f);
		}

		ImGui::Text("Scale:");
		ImGui::SameLine();
		ImGui::InputFloat3("##scale", m_scale.Data());
		// Flags as checkboxes (retractable, retracted by default)
		if (ImGui::TreeNode("Flags"))
		{
			static const std::pair<const char*, InstanceFlag> flagInfos[] = {
				{"Draw Instance",        InstanceFlag::DRAW_INSTANCE},
				{"Animation: Loop",      InstanceFlag::ANIM_LOOP},
				{"Animation: Stop End",  InstanceFlag::ANIM_STOP_AT_END},
				{"Hide Model",           InstanceFlag::HIDE_MODEL},
				{"Pixel LOD",            InstanceFlag::PIXEL_LOD},
				{"Screenspace",          InstanceFlag::SCREENSPACE_INSTANCE},
				{"Billboard",            InstanceFlag::CUSTOM_MATRIX},
				{"Draw Transparent",     InstanceFlag::DRAW_TRANSPARENT},
				{"Use Instance Color",   InstanceFlag::USE_SPECULAR_LIGHT},
				{"Reflection",           InstanceFlag::REFLECTION_FUNC23},
				{"Depth Fade",           InstanceFlag::DEPTH_FADE},
				{"Visible In Gameplay",  InstanceFlag::VISIBLE_DURING_GAMEPLAY},
				{"Owner Pushbuf Gate",   InstanceFlag::OWNER_PUSHBUFFER_GATE},
				{"Draw Huge",            InstanceFlag::DRAW_HUGE},
				{"Hide Before Pause",    InstanceFlag::INVISIBLE_BEFORE_PAUSE},
				{"Hide During Pause",    InstanceFlag::INVISIBLE_DURING_PAUSE},
			};
			for (const auto& [label, bit] : flagInfos)
			{
				uint32_t val = static_cast<uint32_t>(bit);
				bool checked = (m_flags & val) != 0;
				if (ImGui::Checkbox(label, &checked))
				{
					if (checked)
						m_flags |= val;
					else
						m_flags &= ~val;
				}
			}
			ImGui::TreePop();
		}

		float colorModelData[3] = { m_color.Red(), m_color.Green(), m_color.Blue() };
		if (ImGui::ColorEdit3("##modelColor", colorModelData))
		{
			m_color = Color(static_cast<float>(colorModelData[0]), colorModelData[1], colorModelData[2]);
		}

		ImGui::InputScalar("Unk24", ImGuiDataType_U32, &m_unk24);
		ImGui::InputScalar("Unk28", ImGuiDataType_U32, &m_unk28);

		// BSP collision hitbox settings
		ImGui::Checkbox("BSP Collision Hitbox", &m_hitbox.enabled);
		ImGui::SetItemTooltip("Emit a collision hitbox into every BSP leaf overlapping this instance.\nRequired for the instance to be collidable/triggerable in-game.");
		if (m_hitbox.enabled)
		{
			static const char* presetNames[] = { "Pickup", "Solid Wall", "Static Decoration", "Custom" };
			if (ImGui::Combo("Hitbox Type", &m_hitbox.preset, presetNames, 4))
			{
				switch (m_hitbox.preset)
				{
				case InstanceHitbox::PICKUP: m_hitbox.flags = 0x000004C0; m_hitbox.halfExtent = 1.1875f; m_hitbox.yOffset = 0.71875f; break;
				case InstanceHitbox::SOLID_WALL: m_hitbox.flags = 0x026500A0; m_hitbox.halfExtent = 1.1875f; break;
				case InstanceHitbox::STATIC_DECORATION: m_hitbox.flags = 0x000000C0; m_hitbox.halfExtent = 1.1875f; break;
				}
			}
			ImGui::SetItemTooltip("Pickup: trigger-only, vanilla crate radius.\nSolid Wall: blocks the kart.\nStatic Decoration: small solid collider.\nCustom: edit the raw flags yourself.");

			float halfExtent = m_hitbox.halfExtent;
			if (ImGui::InputFloat("Half Extent", &m_hitbox.halfExtent))
			{
				// Note : trying to not clamp, and use 16384 for the squared value if above cap.
				// 
				// halfExtent^2 serialized must fit in int16, so cap at sqrt(2**15)/64
				// m_hitbox.halfExtent = Clamp(halfExtent, 0.0f, 2.828125f);
			}
			ImGui::SetItemTooltip("Hitbox radius around the instance position (vanilla crates use 1.18).\nMax 2.82 since the squared value must fit in 16 bits.");

			ImGui::InputFloat("Hitbox Y Offset", &m_hitbox.yOffset);
		
			
			ImGui::SetItemTooltip("Raises the hitbox center above the instance position.\nVanilla crates use ~0.71 so karts overlap at chest height.");

			ImGui::BeginDisabled(m_hitbox.preset != InstanceHitbox::CUSTOM);
			ImGui::InputScalar("Hitbox Flags", ImGuiDataType_U32, &m_hitbox.flags, nullptr, nullptr, "%08X", ImGuiInputTextFlags_CharsHexadecimal);
			ImGui::EndDisabled();
			ImGui::SetItemTooltip("Raw hitbox flags (editable with Custom type).\nBit 0x80: set = trigger-only (pass through), clear = solid collision.");
		}

		// Delete and Duplicate buttons
		if (ImGui::Button("Duplicate Instance"))
		{
			shouldDuplicate = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete Instance"))
		{
			shouldDelete = true;
		}
	}

	return modelChanged;
}

bool InstanceModelHeader::RenderUI(std::unordered_map<std::string, Texture>& materialToTexture)
{
	bool toDel = false;
	if (ImGui::TreeNodeEx((void*)this, ImGuiTreeNodeFlags_None, m_name.c_str()))
	{
		ImGui::InputText("Name", &m_name, 0x10);
		ImGui::SetNextItemWidth(200.0f);
		ImGui::DragFloat("Max visible distance##", &m_maxDistLOD, 0.5f, -1.0f, 1000.0f, "%.1f");
		ImGui::InputScalar("Flags", ImGuiDataType_U16, &m_flags, nullptr, nullptr, "%04X", ImGuiInputTextFlags_CharsHexadecimal);
		ImGui::Checkbox("Hardcoded scale", &m_hasScale);
		ImGui::BeginDisabled(!m_hasScale);
		ImGui::Text("Scale:"); ImGui::SameLine();
		ImGui::InputFloat3("##scale", m_scale.Data());
		ImGui::EndDisabled();
		ImGui::Checkbox("Start Banner Waving", &m_bannerWave);
		ImGui::Text(("Triangle count: " + std::to_string(m_animations[0].frames[0].size())).c_str());
		if (m_isAnimated)
			ImGui::Text(("is animated : yes"));
		else
			ImGui::Text(("is animated : no"));

		ImGui::Text(("AnimGroup: " + std::to_string(m_animations.size())).c_str());
		for (size_t i = 0 ; i < m_animations.size(); i++)
		{
			ModelAnimation& anim = m_animations[i];
			ImGui::Text(("Group: " + std::to_string(i) + ", Frames: " + std::to_string(anim.frames.size())).c_str());
		}

		if (ImGui::Button("Delete LOD"))
		{
			toDel = true;
		}

		ImGui::Text("\n");
		ImGui::TreePop();
	}
	return toDel;
}

bool InstanceModel::RenderUI(std::unordered_map<std::string, Texture>& materialToTexture, std::function<void(void)> refreshTextureStores)
{
	bool toDel = false;
	if (ImGui::TreeNodeEx((void*)this, ImGuiTreeNodeFlags_None, m_name.c_str()))
	{
		ImGui::InputText("Name", &m_name, 0x10);
		ModelIdWidget("Model ID", &m_id);

		//ImGui::Text("List of LOD");
		ImGui::SeparatorText("List of LOD");
		std::vector<size_t> headerToDel;
		for (size_t i = 0; i < m_headers.size() ; i++)
		{
			InstanceModelHeader& header = m_headers[i];
			ImGui::PushID(static_cast<int>(i));
			if (header.RenderUI(materialToTexture))
			{
				headerToDel.push_back(i);
			}
			ImGui::PopID();
			ImGui::Separator();
		}
		//ImGui::Separator();
		if (!headerToDel.empty())
		{
			for (int i = static_cast<int>(headerToDel.size()) - 1; i >= 0; i--)
				m_headers.erase(m_headers.begin() + headerToDel[i]);
		}
		if (ImGui::TreeNode("Textures##Model"))
		{
			std::unordered_set<std::string> texList;
			for (InstanceModelHeader& hd : m_headers)
			{
				for (Tri& tri : hd.GetGeometry())
				{
					texList.insert(tri.texture);
				}

			}
			for (std::string texName : texList)
			{
				if (ImGui::TreeNode((texName + "##modelListtexture").c_str()))
				{
					std::vector<Quadblock> dummy;
					materialToTexture[texName].RenderUI({}, dummy, refreshTextureStores);
					ImGui::TreePop();
				}
			}
			ImGui::TreePop();
		}
		if (ImGui::Button("Add LOD"))
		{
			auto selection = pfd::open_file("Model LOD File", Settings::m_lastOpenedModelFolder, { "CTR Model Files", "*.gltf" }, pfd::opt::force_path).result();
			if (!selection.empty())
			{
				InstanceModelHeader header;
				header.Clear();
				header.LoadGLTF(selection.front(), materialToTexture);
				if (!header.GetGeometry().empty())
					m_headers.push_back(header);
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Export Model"))
		{
			auto selection = pfd::select_folder("Model Folder", Settings::m_lastOpenedModelFolder, pfd::opt::force_path).result();
			if (!selection.empty())
			{
				const std::filesystem::path path = selection + "\\";
				Settings::m_lastOpenedModelFolder = path.string();
				Export(path, materialToTexture);
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete Model"))
		{
			toDel = true;
		}
		ImGui::Text("\n");
		ImGui::TreePop();
	}
	return toDel;
}

template<typename T, MaterialType M>
bool MaterialProperty<T, M>::RenderUI(const std::string& material, const std::vector<size_t>& quadblockIndexes, std::vector<Quadblock>& quadblocks)
{
	if constexpr (M == MaterialType::TERRAIN)
	{
		ImGui::Text("Terrain:"); ImGui::SameLine();
		if (ImGui::BeginCombo("##terrain", GetPreview(material).c_str()))
		{
			for (const auto& [label, terrain] : TerrainType::LABELS)
			{
				if (ImGui::Selectable(label.c_str()))
				{
					SetPreview(material, label);
				}
			}
			ImGui::EndCombo();
		} ImGui::SameLine();

		static ButtonUI terrainApplyButton = ButtonUI();
		if (terrainApplyButton.Show(("Apply##terrain" + material).c_str(), "Terrain type successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::QUAD_FLAGS)
	{
		if (ImGui::TreeNode("Quad Flags"))
		{
			for (const auto& [label, flag] : QuadFlags::LABELS)
			{
				UIFlagCheckbox(GetPreview(material), flag, label);
			}

			static ButtonUI quadFlagsApplyButton = ButtonUI();
			if (quadFlagsApplyButton.Show(("Apply##quadflags" + material).c_str(), "Quad flags successfully updated.", UnsavedChanges(material)))
			{
				Apply(material, quadblockIndexes, quadblocks);
				return true;
			}
			static ButtonUI killPlaneButton = ButtonUI();
			if (killPlaneButton.Show("Kill Plane##quadflags", "Modified quad flags to kill plane.", false))
			{
				SetPreview(material, QuadFlags::INVISIBLE_TRIGGER | QuadFlags::OUT_OF_BOUNDS | QuadFlags::MASK_GRAB | QuadFlags::WALL | QuadFlags::NO_COLLISION);
				Apply(material, quadblockIndexes, quadblocks);
				return true;
			}
			ImGui::TreePop();
		}
	}
	else if constexpr (M == MaterialType::DRAW_FLAGS)
	{
		T& preview = GetPreview(material);
		ImGui::Checkbox("Double Sided", &preview);
		ImGui::SameLine();
		static ButtonUI drawFlagsApplyButton = ButtonUI();
		if (drawFlagsApplyButton.Show(("Apply##drawflags" + material).c_str(), "Draw flags successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::DRAW_ORDER_HIGH)
	{
		T& preview = GetPreview(material);
		ImGui::Text("Z depth bias:"); ImGui::SameLine();
		if (ImGui::InputInt("##draworderHigh", &preview)) { preview = Clamp(preview, static_cast<int>(INT8_MIN), static_cast<int>(INT8_MAX)); }
		ImGui::SameLine();
		static ButtonUI drawOrderHighButton = ButtonUI();
		if (drawOrderHighButton.Show(("Apply##draworderHigh" + material).c_str(), "Draw Order High successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::CHECKPOINT)
	{
		T& preview = GetPreview(material);
		ImGui::Checkbox("Checkpoint", &preview);
		ImGui::SameLine();

		static ButtonUI checkpointApplyButton = ButtonUI();
		if (checkpointApplyButton.Show(("Apply##checkpoint" + material).c_str(), "Checkpoint status successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::TURBO_PAD)
	{
		T& trigger = GetPreview(material);
		ImGui::Text("Trigger:"); ImGui::SameLine();
		if (ImGui::RadioButton("None", trigger == QuadblockTrigger::NONE))
		{
			trigger = QuadblockTrigger::NONE;
		} ImGui::SameLine();
		if (ImGui::RadioButton("Turbo Pad", trigger == QuadblockTrigger::TURBO_PAD))
		{
			trigger = QuadblockTrigger::TURBO_PAD;
		} ImGui::SameLine();
		if (ImGui::RadioButton("Super Turbo Pad", trigger == QuadblockTrigger::SUPER_TURBO_PAD))
		{
			trigger = QuadblockTrigger::SUPER_TURBO_PAD;
		} ImGui::SameLine();
		static ButtonUI padApplyButton = ButtonUI();
		if (padApplyButton.Show(("Apply##pad" + material).c_str(), "Turbo pad status successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::SPEED_IMPACT)
	{
		T& preview = GetPreview(material);
		ImGui::Text("Downforce:"); ImGui::SameLine();
		if (ImGui::InputInt("##downforce", &preview)) { preview = Clamp(preview, static_cast<T>(INT8_MIN), static_cast<T>(INT8_MAX)); }
		ImGui::SameLine();
		static ButtonUI speedApplyButton = ButtonUI();
		if (speedApplyButton.Show(("Apply##downforce" + material).c_str(), "Downforce successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::WEATHER_INTENSITY)
	{
		T& preview = GetPreview(material);
		ImGui::Text("Weather intensity:"); ImGui::SameLine();
		if (ImGui::InputInt("##Weather intensity", &preview)) { preview = Clamp(preview, static_cast<T>(0), static_cast<T>(UINT8_MAX)); }
		ImGui::SameLine();
		static ButtonUI WeatherIntesityApplyButton = ButtonUI();
		if (WeatherIntesityApplyButton.Show(("Apply##Weather intensity" + material).c_str(), "Weather intensity successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::WEATHER_VANISH_RATE)
	{
		T& preview = GetPreview(material);
		ImGui::Text("Weather vanish rate:"); ImGui::SameLine();
		if (ImGui::InputInt("##Weather vanish rate", &preview)) { preview = Clamp(preview, static_cast<T>(0), static_cast<T>(UINT8_MAX)); }
		ImGui::SameLine();
		static ButtonUI WeathervanishRateApplyButton = ButtonUI();
		if (WeathervanishRateApplyButton.Show(("Apply##Weather vanish rate" + material).c_str(), "Weather vanish rate successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::CHECKPOINT_PATHABLE)
	{
		T& preview = GetPreview(material);
		ImGui::Checkbox("Checkpoint Pathable", &preview);
		ImGui::SameLine();

		static ButtonUI pathableApplyButton = ButtonUI();
		if (pathableApplyButton.Show(("Apply##pathable" + material).c_str(), "Checkpoint pathable status successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::VISTREE_TRANSPARENT)
	{
		T& preview = GetPreview(material);
		ImGui::Checkbox("VisTree Transparency", &preview);
		ImGui::SameLine();

		static ButtonUI visTreeTransparentApplyButton = ButtonUI();
		if (visTreeTransparentApplyButton.Show(("Apply##transparent" + material).c_str(), "VisTree transparency successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
	}
	else if constexpr (M == MaterialType::WATER)
	{
		T& preview = GetPreview(material);
		ImGui::Checkbox("Water", &preview);
		ImGui::SameLine();

		static ButtonUI waterApplyButton = ButtonUI();
		if (waterApplyButton.Show(("Apply##water" + material).c_str(), "Water successfully updated.", UnsavedChanges(material)))
		{
			Apply(material, quadblockIndexes, quadblocks);
			return true;
		}
		}
	return false;
}

void Level::RenderUI(Renderer& renderer)
{
	if (m_showLogWindow)
	{
		if (ImGui::Begin("Log", &m_showLogWindow))
		{
			if (!m_logMessage.empty()) { ImGui::Text(m_logMessage.c_str()); }
			if (!m_invalidQuadblocks.empty())
			{
				ImGui::Text("Error - the following quadblocks are not in the valid format:");
				for (size_t i = 0; i < m_invalidQuadblocks.size(); i++)
				{
					const std::string& quadblock = std::get<0>(m_invalidQuadblocks[i]);
					const std::string& errorMessage = std::get<1>(m_invalidQuadblocks[i]);
					if (ImGui::TreeNode((quadblock + "##" + std::to_string(i)).c_str()))
					{
						ImGui::Text(errorMessage.c_str());
						ImGui::TreePop();
					}
				}
			}
		}
		ImGui::End();
	}

	if (m_showHotReloadWindow)
	{
		if (ImGui::Begin("Hot Reload", &m_showHotReloadWindow))
		{
			std::string levPath = m_hotReloadLevPath.string();
			ImGui::Text("Lev Path"); ImGui::SameLine();
			ImGui::InputText("##levpath", &levPath, ImGuiInputTextFlags_ReadOnly);
			ImGui::SetItemTooltip(levPath.c_str()); ImGui::SameLine();
			if (ImGui::Button("...##levhotreload"))
			{
				auto selection = pfd::open_file("Lev File", m_parentPath.string(), {"Lev Files", "*.lev"}, pfd::opt::force_path).result();
				if (!selection.empty()) { m_hotReloadLevPath = selection.front(); }
			}

			std::string vrmPath = m_hotReloadVRMPath.string();
			ImGui::Text("Vrm Path"); ImGui::SameLine();
			ImGui::InputText("##vrmpath", &vrmPath, ImGuiInputTextFlags_ReadOnly);
			ImGui::SetItemTooltip(vrmPath.c_str()); ImGui::SameLine();
			if (ImGui::Button("...##vrmhotreload"))
			{
				auto selection = pfd::open_file("Vrm File", m_parentPath.string(), {"Vrm Files", "*.vrm"}, pfd::opt::force_path).result();
				if (!selection.empty()) { m_hotReloadVRMPath = selection.front(); }
			}
			if (ImGui::TreeNode("Settings##hotreload"))
			{
				ImGui::InputFloat("Relic Sapphire time", &m_hotReloadSettings.relicSapphire);
				ImGui::InputFloat("Relic Gold time", &m_hotReloadSettings.relicGold);
				ImGui::InputFloat("Relic Platinum time", &m_hotReloadSettings.relicPlatinum);
				ImGui::InputFloat("Crystal Challenge time", &m_hotReloadSettings.crystalTime);
				ImGui::Checkbox("Intro Cutscene", &m_hotReloadSettings.introCutscene);
				ImGui::Checkbox("Ghost", &m_hotReloadSettings.ghost);
				ImGui::TreePop();
			}

			const std::string successMessage = "Successfully hot reloaded.";
			const std::string failMessage = "Failed hot reloading.\nMake sure Duckstation is opened and that the game is unpaused.";

			bool disabled = levPath.empty();
			ImGui::BeginDisabled(disabled);
			static ButtonUI hotReloadButton = ButtonUI(5);
			static std::string hotReloadMessage;
			if (hotReloadButton.Show("Hot Reload##btn", hotReloadMessage, false))
			{
				if (HotReload(levPath, vrmPath, "duckstation")) { hotReloadMessage = successMessage; }
				else { hotReloadMessage = failMessage; }
			}
			ImGui::EndDisabled();
			if (disabled) { ImGui::SetItemTooltip("You must select the lev path before hot reloading."); }

			bool vrmDisabled = vrmPath.empty();
			ImGui::BeginDisabled(vrmDisabled);
			static ButtonUI vrmOnlyButton = ButtonUI(5);
			static std::string vrmOnlyMessage;
			if (vrmOnlyButton.Show("Vrm Only##btn", vrmOnlyMessage, false))
			{
				if (HotReload(std::string(), vrmPath, "duckstation")) { hotReloadMessage = successMessage; }
				else { hotReloadMessage = failMessage; }
			}
			ImGui::EndDisabled();
			if (vrmDisabled) { ImGui::SetItemTooltip("You must select the vrm path before hot reloading the vram."); }
		}
		ImGui::End();
	}


	if (!m_loaded) { return; }

	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::MenuItem("Spawn")) { Settings::w_spawn = !Settings::w_spawn; }
		if (ImGui::MenuItem("Level")) { Settings::w_level = !Settings::w_level; }
		if (!m_materialToQuadblocks.empty() && ImGui::MenuItem("Material")) { Settings::w_material = !Settings::w_material; }
		if (ImGui::MenuItem("Anim Tex")) { Settings::w_animtex = !Settings::w_animtex; }
		if (ImGui::MenuItem("Quadblocks")) { Settings::w_quadblocks = !Settings::w_quadblocks; }
		if (ImGui::MenuItem("Checkpoints")) { Settings::w_checkpoints = !Settings::w_checkpoints; }
		if (ImGui::MenuItem("BSP Tree")) { Settings::w_bsp = !Settings::w_bsp; }
		if (ImGui::MenuItem("Renderer")) { Settings::w_renderer = !Settings::w_renderer; }
		if (ImGui::MenuItem("Ghosts")) { Settings::w_ghost = !Settings::w_ghost; }
		if (ImGui::MenuItem("Python")) { Settings::w_python = !Settings::w_python; }
		if (ImGui::MenuItem("Model Importer")) { Settings::w_modelImporter = !Settings::w_modelImporter; }
		if (ImGui::MenuItem("Bot")) { Settings::w_bot = !Settings::w_bot; }
		ImGui::EndMainMenuBar();
	}

	if (Settings::w_spawn)
	{
		if (ImGui::Begin("Spawn", &Settings::w_spawn))
		{
			static std::string spawnButtonMessage;
			static ButtonUI generateSpawnButton = ButtonUI();
			static float spawnRowSpacing = 3.5f;
			static float spawnColSpacing = 3.0f;
			static float centerOffset = 0.0f;

			ImGui::SetNextItemWidth(200.0f);
			ImGui::DragFloat("Row Spacing##spawnRowSpace", &spawnRowSpacing, 0.1f, 0.1f, 30.0f, "%.1f");
			ImGui::SetNextItemWidth(200.0f);
			ImGui::DragFloat("Column Spacing##spawnColSpace", &spawnColSpacing, 0.1f, 0.1f, 30.0f, "%.1f");
			ImGui::SetNextItemWidth(200.0f);
			ImGui::DragFloat("Center Offset##spawnColSpace", &centerOffset, 0.1f, -30.0f, 30.0f, "%.1f");

			if (generateSpawnButton.Show("Generate from checkpoint", spawnButtonMessage, false))
			{
				if (GenerateSpawn(spawnColSpacing, spawnRowSpacing, centerOffset)) { spawnButtonMessage = "Successfully generated the spawn positions."; }
				else { spawnButtonMessage = "Failed generating the spawn position."; }
				GenerateRenderStartpointData();
			}

			for (size_t i = 0; i < NUM_DRIVERS; i++)
			{
				if (ImGui::TreeNode(("Driver " + std::to_string(i)).c_str()))
				{
					if (ImGui::Button(("Set from selection##" + std::to_string(i)).c_str()))
					{
						m_spawn[i].pos = m_rendererQueryPoint;
						GenerateRenderStartpointData();
					}
					ImGui::Text("Pos:"); ImGui::SameLine();
					bool changed = ImGui::InputFloat3("##pos", m_spawn[i].pos.Data());

					ImGui::Text("Rot:"); ImGui::SameLine();
					if (ImGui::InputFloat3("##rot", m_spawn[i].rot.Data()))
					{
						changed = true;
						m_spawn[i].rot.x = Clamp(m_spawn[i].rot.x, -360.0f, 360.0f);
						m_spawn[i].rot.y = Clamp(m_spawn[i].rot.y, -360.0f, 360.0f);
						m_spawn[i].rot.z = Clamp(m_spawn[i].rot.z, -360.0f, 360.0f);
					};
					if (changed) { GenerateRenderStartpointData(); }
					ImGui::TreePop();
				}
			}
		}
		ImGui::End();
	}

	if (Settings::w_level)
	{
		if (ImGui::Begin("Level", &Settings::w_level))
		{
			if (ImGui::TreeNode("Flags"))
			{
				UIFlagCheckbox(m_configFlags, LevConfigFlags::ENABLE_SKYBOX_GRADIENT, "Enable Skybox Gradient");
				UIFlagCheckbox(m_configFlags, LevConfigFlags::MASK_GRAB_UNDERWATER, "Mask Grab Underwater");
				UIFlagCheckbox(m_configFlags, LevConfigFlags::ANIMATE_WATER_VERTEX, "Animated VColors");
				ImGui::SetItemTooltip("This flag decide if you can use Water, or Animated VColors (Roo Tubes effect). They are mutually exclusive");
				ImGui::TreePop();
			}
			if (ImGui::TreeNode("Sky Gradient"))
			{
				for (size_t i = 0; i < NUM_GRADIENT; i++)
				{
					if (ImGui::TreeNode(("Gradient " + std::to_string(i)).c_str()))
					{
						ImGui::Text("From:"); ImGui::SameLine(); ImGui::InputFloat("##pos_from", &m_skyGradient[i].posFrom);
						ImGui::Text("To:  "); ImGui::SameLine(); ImGui::InputFloat("##pos_to", &m_skyGradient[i].posTo);
						ImGui::Text("From:"); ImGui::SameLine();
						float colorFrom[3] = {m_skyGradient[i].colorFrom.Red(), m_skyGradient[i].colorFrom.Green(), m_skyGradient[i].colorFrom.Blue()};
						if (ImGui::ColorEdit3("##color_from", colorFrom))
						{
							m_skyGradient[i].colorFrom = Color(static_cast<float>(colorFrom[0]), colorFrom[1], colorFrom[2]);
						}
						ImGui::Text("To:  "); ImGui::SameLine();
						float colorTo[3] = {m_skyGradient[i].colorTo.Red(), m_skyGradient[i].colorTo.Green(), m_skyGradient[i].colorTo.Blue()};
						if (ImGui::ColorEdit3("##color_to", colorTo))
						{
							m_skyGradient[i].colorTo = Color(static_cast<float>(colorTo[0]), colorTo[1], colorTo[2]);
						}
						ImGui::TreePop();
					}
				}
				ImGui::TreePop();
			}
			if (ImGui::TreeNode("Clear Color"))
			{
				float clearColor[3] = {m_clearColor.Red(), m_clearColor.Green(), m_clearColor.Blue()};
				if (ImGui::ColorEdit3("##color", clearColor))
				{
					m_clearColor = Color(static_cast<float>(clearColor[0]), clearColor[1], clearColor[2]);
				}
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("Weather"))
			{
				static WeatherPreset weatherPreset = WeatherPreset::CUSTOM;
				const char* presetNames[] = { "Custom", "Rain", "Snow" };
				int selectedIndex = static_cast<int>(weatherPreset);

				if (ImGui::Combo("Preset", &selectedIndex, presetNames, IM_ARRAYSIZE(presetNames)))
				{
					weatherPreset = static_cast<WeatherPreset>(selectedIndex);

					if (weatherPreset == WeatherPreset::RAIN)
					{
						m_weather.velocity = Vec3(0.3125f, -1.875f, 0.0f);
						m_weather.colorTop = Color(static_cast<uint8_t>(64), 64, 64);
						m_weather.colorBottom = Color(static_cast<uint8_t>(255), 255, 255);
						m_weather.fillMode = 0xE1000A60;
						m_weather.OTindex = 0x1;
					}
					else if (weatherPreset == WeatherPreset::SNOW)
					{
						m_weather.velocity = Vec3(0.0f, -0.125f, 0.0f);
						m_weather.colorTop = Color(static_cast<uint8_t>(64), 64, 64);
						m_weather.colorBottom = Color(static_cast<uint8_t>(255), 255, 255);
						m_weather.fillMode = 0xE1000A20;
						m_weather.OTindex = 0x1;
					}
				}

				ImGui::BeginDisabled(weatherPreset != WeatherPreset::CUSTOM);

				ImGui::InputFloat3("Velocity##weatherlev", m_weather.velocity.Data());
				ImGui::SetItemTooltip("Speed the weather falls at. Control both direction and magnitude.");
				float topColor[3] = { m_weather.colorTop.Red(), m_weather.colorTop.Green(), m_weather.colorTop.Blue() };
				if (ImGui::ColorEdit3("Top Color", topColor))
				{
					m_weather.colorTop = Color(topColor[0], topColor[1], topColor[2]);
				}
				float bottomColor[3] = { m_weather.colorBottom.Red(), m_weather.colorBottom.Green(), m_weather.colorBottom.Blue() };
				if (ImGui::ColorEdit3("Bottom Color", bottomColor))
				{
					m_weather.colorBottom = Color(bottomColor[0], bottomColor[1], bottomColor[2]);
				}
				ImGui::InputScalar("Fill Mode (Hex)", ImGuiDataType_U32, &m_weather.fillMode, NULL, NULL, "%08X", ImGuiInputTextFlags_CharsHexadecimal);
				ImGui::SetItemTooltip("PS1 primCode");
				if (ImGui::InputInt("OTindex", &m_weather.OTindex)) { m_weather.OTindex = Clamp(m_weather.OTindex, 0, 0x3FF); }
				ImGui::SetItemTooltip("Z buffer packet");

				ImGui::EndDisabled();
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("Stars"))
			{
				ImGui::InputScalar("Number of Stars", ImGuiDataType_U16, &m_stars.numStars);

				ImGui::InputScalar("Seed", ImGuiDataType_U16, &m_stars.seed);
				ImGui::SetItemTooltip("Controls the random spread distance from the horizon.\n");

				ImGui::InputScalar("Z Depth", ImGuiDataType_U16, &m_stars.zDepth);
				ImGui::SetItemTooltip("Distance from screen (OT). Default is 1022.\nSkybox is drawn at 1023.");

				ImGui::Checkbox("Spread Stars Below Horizon", &m_stars.spread);
				ImGui::SetItemTooltip("When disabled, stars appear only above the horizon.\nWhen enabled, stars will also appear below the horizon.");

				ImGui::TreePop();
			}

			if (ImGui::TreeNode("Jump vertical Speed Cap"))
			{
				ImGui::Text("Jump Vertical Speed Cap");
				ImGui::SameLine();
				if (ImGui::InputInt("##jysc", &m_jumpYSpeedCap)) 
					m_jumpYSpeedCap = Clamp(m_jumpYSpeedCap, 0, 80);
				ImGui::SetItemTooltip("Set the maximum vertical speed you can have from jumping\n");
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("Moving Instances Path"))
			{
				ImGui::SeparatorText("Loading settings");
				ImGui::Checkbox("Normalize Distances##mip", &m_instPathSettings.normalize); 
				ImGui::BeginDisabled(!m_instPathSettings.normalize);
				ImGui::SameLine();
				ImGui::SetNextItemWidth(200.0f);
				ImGui::DragFloat("Node Distance", &m_instPathSettings.normalizeDist, 0.1f, 0.1f, 100.0f, "%.1f");
				ImGui::EndDisabled();
				ImGui::Checkbox("Snap to Ground##mip", &m_instPathSettings.groundSnap);
				ImGui::BeginDisabled(!m_instPathSettings.groundSnap);
				ImGui::SameLine();
				ImGui::SetNextItemWidth(150.0f);
				ImGui::DragFloat("Below ground threshold##mip", &m_instPathSettings.negSnapDist, 0.1f, -50.0f, -0.1f, "%.1f");
				ImGui::SameLine();
				ImGui::SetNextItemWidth(150.0f);
				ImGui::DragFloat("Above ground threshold##mip", &m_instPathSettings.posSnapDist, 0.1f, 0.1f, 50.0f, "%.1f");
				ImGui::EndDisabled();
				ImGui::Checkbox("Rolling##mip", &m_instPathSettings.rolling);
				ImGui::SetItemTooltip("Only used for pos + rot");
				ImGui::BeginDisabled(!m_instPathSettings.rolling);
				ImGui::SameLine();
				ImGui::SetNextItemWidth(200.0f);
				ImGui::DragFloat("Object Radius##mip", &m_instPathSettings.radius, 0.1f, 0.1f, 100.0f, "%.1f");
				ImGui::SetItemTooltip("Only used for pos + rot");
				ImGui::EndDisabled();
				ImGui::Checkbox("Loop##mip", &m_instPathSettings.loop);
				ImGui::SetItemTooltip("Loop or Point to Point");

				ImGui::SeparatorText("Path positions only");
				for (size_t i = 0; i < m_spawntypes.size(); i++)
				{
					ImGui::PushID(static_cast<int>(i));
					
					if (ImGui::TreeNode(("Path " + std::to_string(i) + "##st2pos").c_str()))
					{
						
						if (ImGui::Button("Load Path##st2pos"))
						{
							auto selection = pfd::open_file("Select Path OBJ", m_parentPath.string(),
								{ "OBJ Files", "*.obj", "All Files", "*" }).result();

							if (!selection.empty())
							{
								std::vector<Vec3> vec = LoadPath(selection[0]);
								if (m_instPathSettings.normalize)
									vec = NormalizePos(vec, m_instPathSettings.normalizeDist, m_instPathSettings.loop);
								if (m_instPathSettings.groundSnap)
								{
									std::vector<size_t> quadindexes;
									for (size_t j = 0; j < m_quadblocks.size(); j++)
									{
										if (m_quadblocks[j].GetFlags() & QuadFlags::GROUND)
											quadindexes.push_back(j);
									}
									Vec3 dummyRot;
									for (Vec3& pos : vec)
									{
										SnapToClosestQuad(m_quadblocks, quadindexes, pos, dummyRot, Vec3(0.0f, 1.0f, 0.0f),
											m_instPathSettings.negSnapDist, m_instPathSettings.posSnapDist);
									}
										
								}
								m_spawntypes[i].clear();
								m_spawntypes[i] = std::move(vec);
							}		
						}
						ImGui::SameLine();
						if (ImGui::Button("Delete Path##st2pos"))
						{
							m_spawntypes.erase(m_spawntypes.begin() + i);
						}
						ImGui::SeparatorText("");

						for (size_t j = 0; j < m_spawntypes[i].size(); j++)
						{
							ImGui::PushID(static_cast<int>(j));
							ImGui::Text(("Pos " + std::to_string(j) + " : ").c_str()); ImGui::SameLine();
							ImGui::InputFloat3("##pos", m_spawntypes[i][j].Data());
							ImGui::Separator();
							ImGui::PopID();
						}
						if (ImGui::Button("Add Node##st2pos"))
						{
							m_spawntypes[i].emplace_back();
						}
						ImGui::SameLine();
						if (ImGui::Button("Delete Node##st2pos"))
						{
							m_spawntypes[i].pop_back();
						}
						ImGui::TreePop();
					}
					
					ImGui::PopID();
				}
				if (ImGui::Button("Add Path##st2pos"))
				{
					m_spawntypes.push_back({});
				}
				ImGui::SeparatorText("Path positions+rotations");
				for (size_t i = 0; i < m_spawntypesPosRot.size(); i++)
				{
					ImGui::PushID(static_cast<int>(i));
					
					if (ImGui::TreeNode(("Path " + std::to_string(i) + "##st2posRot").c_str()))
					{
						if (ImGui::Button("Load##st2posrot"))
						{
							auto selection = pfd::open_file("Select Path OBJ", m_parentPath.string(),
								{ "OBJ Files", "*.obj", "All Files", "*" }).result();

							if (!selection.empty())
							{
								std::vector<Vec3> posvec = LoadPath(selection[0]);
								if (m_instPathSettings.normalize)
									posvec = NormalizePos(posvec, m_instPathSettings.normalizeDist, m_instPathSettings.loop);
								std::vector<Vec3> rotvec = ComputeYaw(posvec, m_instPathSettings.loop);
								std::vector<Vec3> upvec(rotvec.size(), Vec3(0.0f, 1.0f, 0.0f));
								if (m_instPathSettings.groundSnap)
								{
									std::vector<size_t> quadindexes;
									for (size_t j = 0; j < m_quadblocks.size(); j++)
									{
										if (m_quadblocks[j].GetFlags() & QuadFlags::GROUND)
											quadindexes.push_back(j);
									}
									for (size_t j = 0; j < posvec.size() ; j++)
									{

										int quadId = SnapToClosestQuad(m_quadblocks, quadindexes, posvec[j], rotvec[j], Vec3(0.0f, 1.0f, 0.0f),
											m_instPathSettings.negSnapDist, m_instPathSettings.posSnapDist);
										if (quadId != -1)
											upvec[j] = m_quadblocks[quadId].GetNormal();
									}
								}
								if (m_instPathSettings.rolling)
								{
									float totalDist = 0.0f;
									for (size_t j = 0; j < posvec.size() - 1; j++)
									{
										Vec3& pos = posvec[j + 1];
										Vec3& prevpos = posvec[j];
										totalDist += (pos - prevpos).Length();
										float rotAngleRad = totalDist / m_instPathSettings.radius;
										Quaternion preRoll(rotvec[j + 1]);
										Quaternion rolling(Vec3(1.0f, 0.0f, 0.0f), rotAngleRad);
										rotvec[j + 1] = (preRoll * rolling).ToEulerYXZ();
									}
									for (size_t j = 0; j < posvec.size(); j++)
									{
										posvec[j] += upvec[j] * m_instPathSettings.radius;
									}
								}
								m_spawntypesPosRot[i].clear();
								for (size_t j = 0; j < posvec.size(); j++)
								{
									m_spawntypesPosRot[i].emplace_back(posvec[j], rotvec[j]);
								}
								
							}
						}
						ImGui::SameLine();
						if (ImGui::Button("Delete##st2posrot"))
						{
							m_spawntypesPosRot.erase(m_spawntypesPosRot.begin() + i);
						}
						ImGui::SeparatorText("");

						for (size_t j = 0; j < m_spawntypesPosRot[i].size(); j++)
						{
							ImGui::PushID(static_cast<int>(j));
							ImGui::Text(("Pos " + std::to_string(j) + " : ").c_str()); ImGui::SameLine();
							ImGui::InputFloat3("##pos", m_spawntypesPosRot[i][j].pos.Data());
							ImGui::Text(("Rot " + std::to_string(j) + " : ").c_str()); ImGui::SameLine();
							ImGui::InputFloat3("##rot", m_spawntypesPosRot[i][j].rot.Data());
							ImGui::Separator();
							ImGui::PopID();
						}
						if (ImGui::Button("Add Node##st2posRot"))
						{
							m_spawntypesPosRot[i].emplace_back();
						}
						ImGui::SameLine();
						if (ImGui::Button("Delete Node##st2posRot"))
						{
							m_spawntypesPosRot[i].pop_back();
						}
						ImGui::TreePop();
					}
				
					ImGui::PopID();
				}
				if (ImGui::Button("Add Path##st2posrot"))
				{
					m_spawntypesPosRot.push_back({});
				}
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("SplitLines"))
			{
				ImGui::Text("SplitLine 1:"); 
				ImGui::SameLine(); 
				ImGui::InputFloat("##sl1", &m_splitLines[0]);
				ImGui::SetItemTooltip("Reflection 1 flag\n");
				ImGui::SameLine();
				if (ImGui::Button(("Set from selection##1")))
				{
					m_splitLines[0] = m_rendererQueryPoint.y;
				}

				ImGui::Text("SplitLine 2:");
				ImGui::SameLine();
				ImGui::InputFloat("##sl2", &m_splitLines[1]);
				ImGui::SetItemTooltip("Reflection 2 flag\n");
				ImGui::SameLine();
				if (ImGui::Button(("Set from selection##2")))
				{
					m_splitLines[1] = m_rendererQueryPoint.y;
				}
			}

			if (ImGui::TreeNode("Minimap"))
			{
				if (m_minimapConfig.RenderUI(m_quadblocks, [&]() { this->UpdateAnimationRenderData(); }) && GuiRenderSettings::showMinimapBounds)
				{
					GenerateRenderMinimapBoundsData();
				}
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("Skybox"))
			{
				if (m_skybox.RenderUI())
				{
					GenerateRenderSkyboxData();
				}
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("Water"))
			{
				if (ImGui::TreeNode("Settings##Water"))
				{	
					ImGui::SeparatorText("Base UV");
					ImGui::DragFloat("World Tex size", &m_waterAnimSettings.sizeTex, 0.5f, 0.0f, 100.0f, "%.1f");
					ImGui::SetItemTooltip("Size of the full texture in world units");

					ImGui::SeparatorText("Scrolling UV");
					ImGui::InputInt("U Cycle count##scroll", &m_waterAnimSettings.ScrollULoops);
					ImGui::InputInt("V Cycle count##scroll", &m_waterAnimSettings.ScrollVLoops);

					ImGui::SeparatorText("Waving UV");
					ImGui::DragFloat("Wave Length##uv", &m_waterAnimSettings.waveLength, 0.1f, 0.0f, 1000.0f, "%.1f");
					ImGui::InputInt("U Cycle count##wave", &m_waterAnimSettings.waveCyclesTimeU);
					ImGui::InputInt("V Cycle count##wave", &m_waterAnimSettings.waveCyclesTimeV);
					ImGui::DragFloat("Wave Amplitude", &m_waterAnimSettings.waveAmplitude, 0.1f, 0.0f, 64.0f, "%.1f pixels");

					ImGui::SeparatorText("Brightness");
					// --- Brightness & Shimmer ---
					ImGui::DragFloat("Base Brightness", &m_waterAnimSettings.baseBrightness, 0.1f, 0.0f, 15.0f, "%.1f");
					ImGui::SetItemTooltip("Base brightness (range 0 to 15).");

					if (ImGui::DragFloat("Brightness amplitude", &m_waterAnimSettings.brightAmp, 0.1f, 0.0f, 15.0f, "%.1f"))
					{
						m_waterAnimSettings.brightAmp = Clamp(m_waterAnimSettings.brightAmp, 0.0f, 15.0f);
					}
					ImGui::SetItemTooltip("Base lighting brightness (range 0 to 15).");						
					ImGui::InputInt("Brightness Cycles Time", &m_waterAnimSettings.brightWaveCycle);
					ImGui::SetItemTooltip("Temporal cycles over loop (different from ripple cycles so waves and shimmer don't lock-step).");

				//	ImGui::Separator();

				//	// --- Variation ---
				//	ImGui::InputFloat("Seed", &m_waterAnimSettings.seed);
				//	ImGui::SetItemTooltip("Vary between separate, unconnected water bodies.");

					ImGui::TreePop();
				}
				m_materialToTexture[m_envMapMatName].RenderUI({}, m_quadblocks, [&]() { this->UpdateAnimationRenderData(); });
				static std::string buttonMessage;
				static ButtonUI generateWaterButton = ButtonUI();
				if (generateWaterButton.Show("Generate Water Animations", buttonMessage, false))
				{
					if (GenerateOceanVertices()) { buttonMessage = "Successfully generated the ocean animations."; }
					else { buttonMessage = "Failed to create water (this shouldn't be possible)."; }
				}
				ImGui::TreePop();
			}
		}
		ImGui::End();
	}

	if (Settings::w_material)
	{
		if (ImGui::Begin("Material", &Settings::w_material))
		{
			for (const auto& [material, quadblockIndexes] : m_materialToQuadblocks)
			{
				if (ImGui::TreeNode(material.c_str()))
				{
					if (ImGui::TreeNode("Quadblocks"))
					{
						constexpr size_t QUADS_PER_LINE = 10;
						for (size_t i = 0; i < quadblockIndexes.size(); i++)
						{
							ImGui::Text((m_quadblocks[quadblockIndexes[i]].GetName() + ", ").c_str());
							if (((i + 1) % QUADS_PER_LINE) == 0 || i == quadblockIndexes.size() - 1) { continue; }
							ImGui::SameLine();
						}
						ImGui::TreePop();
					}

					m_propTerrain.RenderUI(material, quadblockIndexes, m_quadblocks);
					m_propQuadFlags.RenderUI(material, quadblockIndexes, m_quadblocks);
					if (ImGui::TreeNode("Draw Flags"))
					{
						m_propDoubleSided.RenderUI(material, quadblockIndexes, m_quadblocks);
						m_propDrawOrderHigh.RenderUI(material, quadblockIndexes, m_quadblocks);
						ImGui::TreePop();
					}
					
					m_propCheckpoints.RenderUI(material, quadblockIndexes, m_quadblocks);
					m_propCheckpointPathable.RenderUI(material, quadblockIndexes, m_quadblocks);
					m_propWater.RenderUI(material, quadblockIndexes, m_quadblocks);
					m_propVisTreeTransparent.RenderUI(material, quadblockIndexes, m_quadblocks);
					if (m_propTurboPads.RenderUI(material, quadblockIndexes, m_quadblocks))
					{
						for (size_t index : quadblockIndexes) { ManageTurbopad(m_quadblocks[index]); }
						if (m_bsp.IsValid())
						{
							m_bsp.Clear();
							GenerateRenderBspData();
						}
					}
					m_propSpeedImpact.RenderUI(material, quadblockIndexes, m_quadblocks);
					m_propWeatherIntensity.RenderUI(material, quadblockIndexes, m_quadblocks);
					m_propWeatherVanishRate.RenderUI(material, quadblockIndexes, m_quadblocks);

					if (m_materialToTexture.contains(material))
					{
						m_materialToTexture[material].RenderUI(quadblockIndexes, m_quadblocks, [&]() { this->UpdateAnimationRenderData(); });
					}

					ImGui::TreePop();
				}
			}
		}
		ImGui::End();
	}

	if (!Settings::w_material) { RestoreMaterials(this); }

	if (Settings::w_animtex)
	{
		if (ImGui::Begin("Animated Textures", &Settings::w_animtex))
		{
			static std::string animTexQuerry;
			std::vector<std::string> animTexNames;
			for (const AnimTexture& currAnimTex : m_animTextures)
			{
				animTexNames.push_back(currAnimTex.GetName());
			}
			ImGui::InputTextWithHint("Search##", "Search Query...", &animTexQuerry);
			static std::string errorLoadingAnim;
			if (ImGui::Button("Load"))
			{
				auto selection = pfd::open_file("Animated Texture", m_parentPath.string(), {"Animated Texture Files", "*.obj"}, pfd::opt::force_path).result();
				if (!selection.empty())
				{
					const std::filesystem::path& animTexPath = selection.front();
					AnimTexture animTex = AnimTexture(animTexPath, animTexNames);
					if (!animTex.IsEmpty()) { m_animTextures.push_back(animTex); errorLoadingAnim.clear(); }
					else { errorLoadingAnim = "Error loading " + animTexPath.string(); }
				}
			}
			if (!errorLoadingAnim.empty()) { ImGui::Text(errorLoadingAnim.c_str()); }
			size_t remIndex = 0;
			std::vector<size_t> remAnimTexIndex;
			std::vector<AnimTexture> newTextures;
			for (AnimTexture& tex : m_animTextures)
			{
				if (!tex.RenderUI(animTexNames, m_quadblocks, m_materialToQuadblocks, animTexQuerry, newTextures))
				{
					remAnimTexIndex.push_back(remIndex);
				}
				remIndex++;
			}
			for (int i = static_cast<int>(remAnimTexIndex.size()) - 1; i >= 0; i--)
			{
				m_animTextures.erase(m_animTextures.begin() + remAnimTexIndex[i]);
			}
			for (const AnimTexture& newTex : newTextures)
			{
				bool foundEquivalent = false;
				for (AnimTexture& tex : m_animTextures)
				{
					if (newTex.IsEquivalent(tex))
					{
						const std::vector<size_t>& newIndexes = newTex.GetQuadblockIndexes();
						for (size_t index : newIndexes) { tex.AddQuadblockIndex(index); }
						foundEquivalent = true;
						break;
					}
				}
				if (!foundEquivalent) { m_animTextures.push_back(newTex); }
			}
		}
		ImGui::End();
	}

	static std::string quadblockQuery;
	if (Settings::w_quadblocks)
	{
		bool resetBsp = false;
		if (ImGui::Begin("Quadblocks", &Settings::w_quadblocks))
		{
			ImGui::InputTextWithHint("Search", "Search Quadblocks...", &quadblockQuery);
			for (Quadblock& quadblock : m_quadblocks)
			{
				if (!quadblock.GetHide() && Matches(quadblock.GetName(), quadblockQuery))
				{
					if (quadblock.RenderUI(m_checkpoints.size() - 1, resetBsp))
					{
						ManageTurbopad(quadblock);
					}
				}
			}
		}
		ImGui::End();
		if (resetBsp && m_bsp.IsValid())
		{
			m_bsp.Clear();
			GenerateRenderBspData();
		}
	}

	if (!quadblockQuery.empty() && !Settings::w_quadblocks) { quadblockQuery.clear(); }

	static std::string checkpointQuery;
	if (Settings::w_checkpoints)
	{
		if (ImGui::Begin("Checkpoints", &Settings::w_checkpoints))
		{
			ImGui::InputTextWithHint("Search##", "Search Quadblocks...", &checkpointQuery);
			if (ImGui::TreeNode("Checkpoints"))
			{
				std::vector<int> checkpointsDelete;
				for (int i = 0; i < m_checkpoints.size(); i++)
				{
					m_checkpoints[i].RenderUI(m_checkpoints.size(), m_quadblocks);
					if (m_checkpoints[i].GetDelete()) { checkpointsDelete.push_back(i); }
				}
				if (!checkpointsDelete.empty())
				{
					for (int i = static_cast<int>(checkpointsDelete.size()) - 1; i >= 0; i--)
					{
						m_checkpoints.erase(m_checkpoints.begin() + checkpointsDelete[i]);
					}
					for (int i = 0; i < m_checkpoints.size(); i++)
					{
						m_checkpoints[i].RemoveInvalidCheckpoints(checkpointsDelete);
						m_checkpoints[i].UpdateInvalidCheckpoints(checkpointsDelete);
						m_checkpoints[i].SetIndex(i);
					}
				}
				if (ImGui::Button("Add Checkpoint"))
				{
					m_checkpoints.emplace_back(static_cast<int>(m_checkpoints.size()));
				}
				ImGui::TreePop();
			}
		}
		if (ImGui::TreeNode("Generate"))
		{
			for (size_t i = 0; i < m_checkpointPaths.size(); i++)
			{
				bool insertAbove = false;
				bool removePath = false;
				Path& path = m_checkpointPaths[i];
				const std::string pathTitle = "Path " + std::to_string(path.GetIndex());
				path.RenderUI(pathTitle, m_quadblocks, checkpointQuery, insertAbove, removePath, m_rendererSelectedQuadblockIndexes, true);
				if (insertAbove)
				{
					m_checkpointPaths.insert(m_checkpointPaths.begin() + path.GetIndex(), Path());
					for (size_t j = 0; j < m_checkpointPaths.size(); j++) { m_checkpointPaths[j].SetIndex(j); }
				}
				if (removePath)
				{
					m_checkpointPaths.erase(m_checkpointPaths.begin() + path.GetIndex());
					for (size_t j = 0; j < m_checkpointPaths.size(); j++) { m_checkpointPaths[j].SetIndex(j); }
				}
			}

			if (ImGui::Button("Create Path"))
			{
				m_checkpointPaths.push_back(Path(m_checkpointPaths.size()));
			}
			ImGui::SameLine();
			ImGui::BeginDisabled(m_checkpointPaths.empty());
			if (ImGui::Button("Delete Path"))
			{
				m_checkpointPaths.pop_back();
			}
			ImGui::EndDisabled();

			bool ready = !m_checkpointPaths.empty();
			for (const Path& path : m_checkpointPaths)
			{
				if (!path.IsReady()) { ready = false; break; }
			}
			ImGui::BeginDisabled(!ready);
			static ButtonUI generateButton;
			static bool showWarning = false;
			static std::chrono::time_point<std::chrono::steady_clock> warningStart;
			if (generateButton.Show("Generate", "Checkpoints successfully generated.", false))
			{
				if (!GenerateCheckpoints())
				{
					showWarning = true;
					warningStart = std::chrono::steady_clock::now();
				}
			}
			if (showWarning)
			{
				auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - warningStart).count();
				if (elapsed < 5) 
					ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Warning: some paths are overlapping.\nCheck console for details.");
				else
					showWarning = false;
			}
			ImGui::EndDisabled();
			ImGui::TreePop();
		}
		ImGui::End();
	}

	if (!checkpointQuery.empty() && !Settings::w_checkpoints) { checkpointQuery.clear(); }

	if (Settings::w_bsp)
	{
		if (ImGui::Begin("BSP Tree", &Settings::w_bsp))
		{
			if (!m_bsp.IsEmpty()) { m_bsp.RenderUI(m_quadblocks); }

			static std::string buttonMessage;
			static ButtonUI generateBSPButton = ButtonUI();
			static ButtonUI generateVisTreeButton = ButtonUI();
			if (ImGui::TreeNode("Advanced"))
			{
				if (ImGui::TreeNodeEx("BSP Settings", ImGuiTreeNodeFlags_DefaultOpen))
				{
					if (ImGui::InputInt("Max Quad Per Leaf", &m_bspSettings.maxQuadPerLeaf)) { m_bspSettings.maxQuadPerLeaf = std::max(m_bspSettings.maxQuadPerLeaf, 1); }
					ImGui::SetItemTooltip("Lower values improve rendering performance, but increases file size and slows down vis tree generation.");
					if (ImGui::InputFloat("Max Leaf Axis Length", &m_bspSettings.maxAxisDistance)) { m_bspSettings.maxAxisDistance = std::max(m_bspSettings.maxAxisDistance, 0.0f); }
					ImGui::SetItemTooltip("Lower values improve rendering performance, but increases file size and slows down vis tree generation.");
					ImGui::Checkbox("Separate Material", &m_bspSettings.separateMaterial);
					ImGui::TreePop();
				}
				if (ImGui::TreeNodeEx("Vis Tree Settings", ImGuiTreeNodeFlags_DefaultOpen))
				{
					if (ImGui::InputFloat("Near Clip Distance", &m_visTreeSettings.nearClipDistance)) { m_visTreeSettings.nearClipDistance = std::max(m_visTreeSettings.nearClipDistance, -1.0f); }
					ImGui::SetItemTooltip("Minimum drawing distance. Higher values decrease performance and speed up the vis tree generation.");
					if (ImGui::InputFloat("Far Clip Distance", &m_visTreeSettings.farClipDistance)) { m_visTreeSettings.farClipDistance = std::max(m_visTreeSettings.farClipDistance, 0.0f); }
					ImGui::SetItemTooltip("Maximum drawing distance. Lower values improve performance and speed up the vis tree generation.");
					ImGui::Checkbox("Self target Near Clip Distance", &m_visTreeSettings.selfTargetNearClip);
					ImGui::SetItemTooltip("Spread visibility depending on the distance to ray emmitor instead of ray target");
					ImGui::Checkbox("Assume Commutative Rays", &m_visTreeSettings.commutativeRays);
					ImGui::SetItemTooltip("Speeds up VisTree generation by a factor of 2x to 3x with minimal loss of precision.");
					ImGui::Checkbox("Center-Only Samples", &m_visTreeSettings.centerOnlySamples);
					ImGui::SetItemTooltip("Only casts rays from each quad center (skips corner samples). Much faster, but may miss narrow visibility paths.");
					ImGui::TreePop();
				}
				ImGui::TreePop();
			}
			if (generateBSPButton.Show("Generate BSP", buttonMessage, false))
			{
				if (GenerateBSP()) { buttonMessage = "Successfully generated the BSP tree."; }
				else { buttonMessage = "Failed generating the BSP tree."; }
			}
			if (generateVisTreeButton.Show("Generate VisTree", buttonMessage, false))
			{
				if (GenerateVisTreeOnly()) { buttonMessage = "Successfully generated the VisTree."; }
				else { buttonMessage = "Failed generating the VisTree."; }
			}
			ImGui::SetItemTooltip("Generating the vis tree may take several minutes, but the gameplay will be more performant.");
		}
		ImGui::End();
	}

	if (Settings::w_ghost)
	{
		if (ImGui::Begin("Ghost", &Settings::w_ghost))
		{
			static ButtonUI saveGhostButton(20);
			static std::string saveGhostFeedback;
			if (saveGhostButton.Show("Save Ghost", saveGhostFeedback, false))
			{
				saveGhostFeedback = "Failed retrieving ghost data from the emulator.\nMake sure that you have saved your ghost in-game\nbefore clicking this button.";

				std::string filename = "ghost";
				auto selection = pfd::save_file("CTR Ghost File", filename.c_str(), {"Ghost File (*.ctrghost)", "*.ctrghost"}).result();
				if (!selection.empty())
				{
					const std::filesystem::path path = selection + ".ctrghost";
					if (SaveGhostData("duckstation", path)) { saveGhostFeedback = "Ghost file successfully saved."; }
				}
			}

			auto ConvertTime = [](uint32_t time)
				{
					constexpr uint32_t ms = 32;
					constexpr uint32_t fps = 30;
					constexpr uint32_t second = ms * fps;
					constexpr uint32_t minute = 60 * second;

					uint32_t minutes = time / minute;
					time -= minutes * minute;

					uint32_t seconds = time / second;
					time -= seconds * second;

					uint32_t milis = (time * 1000) / second;
					return std::to_string(minutes) + ":" + std::to_string(seconds) + "." + std::to_string(milis);
				};

			static std::string tropyPath;
			static ButtonUI tropyPathButton(10);
			static std::string tropyImportFeedback;
			if (ImGui::TreeNode("Slot 1"))
			{
				ImGui::Text("Filename:"); ImGui::SameLine();
				ImGui::InputText("##tropyghost", &tropyPath, ImGuiInputTextFlags_ReadOnly); ImGui::SameLine();
				if (tropyPathButton.Show("...##tropypath", tropyImportFeedback, false))
				{
					tropyImportFeedback = "Error: invalid ghost file format.";
					auto selection = pfd::open_file("CTR Ghost File", m_parentPath.string(), {"CTR Ghost Files (*.ctrghost)", "*.ctrghost"}, pfd::opt::force_path).result();
					if (!selection.empty())
					{
						tropyPath = selection.front();
						if (SetGhostData(tropyPath, true)) { tropyImportFeedback = "Slot 1 ghost successfully set."; }
					}
				}

				if (!m_tropyGhost.empty())
				{
					uint16_t character = 0;
					uint32_t time = 0;
					memcpy(&character, &m_tropyGhost[6], sizeof(uint16_t));
					memcpy(&time, &m_tropyGhost[16], sizeof(uint32_t));

					std::string characterText = "Character: " + CTR_CHARACTERS[character];
					std::string timeText = "Time: " + ConvertTime(time);
					ImGui::Text(characterText.c_str());
					ImGui::Text(timeText.c_str());
				}
				ImGui::TreePop();
			}

			static std::string oxidePath;
			static ButtonUI oxidePathButton(10);
			static std::string oxideImportFeedback;
			if (ImGui::TreeNode("Slot 2"))
			{
				ImGui::Text("Filename:"); ImGui::SameLine();
				ImGui::InputText("##oxideghost", &oxidePath, ImGuiInputTextFlags_ReadOnly); ImGui::SameLine();
				if (oxidePathButton.Show("...##oxidepath", oxideImportFeedback, false))
				{
					oxideImportFeedback = "Error: invalid ghost file format.";
					auto selection = pfd::open_file("CTR Ghost File", m_parentPath.string(), {"CTR Ghost Files", "*.ctrghost"}, pfd::opt::force_path).result();
					if (!selection.empty())
					{
						oxidePath = selection.front();
						if (SetGhostData(oxidePath, false)) { oxideImportFeedback = "Slot 2 ghost successfully set"; }
					}
				}

				if (!m_oxideGhost.empty())
				{
					uint16_t character = 0;
					uint32_t time = 0;
					memcpy(&character, &m_oxideGhost[6], sizeof(uint16_t));
					memcpy(&time, &m_oxideGhost[16], sizeof(uint32_t));

					std::string characterText = "Character: " + CTR_CHARACTERS[character];
					std::string timeText = "Time: " + ConvertTime(time);
					ImGui::Text(characterText.c_str());
					ImGui::Text(timeText.c_str());
				}
				ImGui::TreePop();
			}
		}
		ImGui::End();
	}

	if (Settings::w_renderer)
	{
		if (ImGui::Begin("Renderer", &Settings::w_renderer))
		{
			static std::unordered_map<ImGuiKey, std::string> keyOptions;
			if (keyOptions.empty())
			{
				keyOptions.reserve(ImGuiKey_NamedKey_END - ImGuiKey_NamedKey_BEGIN);
				for (int key = ImGuiKey_NamedKey_BEGIN; key < ImGuiKey_NamedKey_END; key++)
				{
					std::string label = ImGui::GetKeyName(static_cast<ImGuiKey>(key));
					keyOptions.insert({static_cast<ImGuiKey>(key), label});
				}
				keyOptions.insert({static_cast<ImGuiKey>(ImGuiKey_ModShift), "Shift"});
				keyOptions.insert({static_cast<ImGuiKey>(ImGuiKey_ModCtrl), "Ctrl"});
				keyOptions.insert({static_cast<ImGuiKey>(ImGuiKey_ModAlt), "Alt"});
				keyOptions.insert({static_cast<ImGuiKey>(ImGuiKey_ModSuper), "Super"});
			}

			if (ImGui::TreeNodeEx("Settings", ImGuiTreeNodeFlags_DefaultOpen))
			{
				ImGui::Text("Shader:");
				ImGui::SameLine();
				ImGui::Combo("##Shader", &GuiRenderSettings::renderType, GuiRenderSettings::renderTypeLabels.data(), static_cast<int>(GuiRenderSettings::renderTypeLabels.size()));
				ImGui::Text("Filter:");
				ImGui::SameLine();
				ImGui::Checkbox("##Filter", &GuiRenderSettings::filterActive);
				ImGui::SameLine();
				ImGui::Text("Default Color:");
				ImGui::SameLine();
				float filterColor[3] = {GuiRenderSettings::defaultFilterColor.Red(), GuiRenderSettings::defaultFilterColor.Green(), GuiRenderSettings::defaultFilterColor.Blue()};
				if (ImGui::ColorEdit3("##FilterColor", filterColor, ImGuiColorEditFlags_NoInputs))
				{
					Color newFilterColor = Color(filterColor[0], filterColor[1], filterColor[2]);
					for (Quadblock& quadblock : m_quadblocks)
					{
						if (quadblock.GetFilterColor() != GuiRenderSettings::defaultFilterColor) { continue; }
						quadblock.SetFilterColor(newFilterColor);
						UpdateFilterRenderData(quadblock);
					}
					GuiRenderSettings::defaultFilterColor = newFilterColor;
				}
				ImGui::SameLine();
				if (ImGui::Button("Reset Filter")) { ResetFilter(); }
				ImGui::Text("Flags:");
				if (ImGui::BeginTable("Renderer Flags", 2, ImGuiTableFlags_SizingStretchSame))
				{
					constexpr unsigned REND_FLAGS_NONE = 0;
					constexpr unsigned REND_FLAGS_COLUMN_0 = 1;
					constexpr unsigned REND_FLAGS_COLUMN_1 = 2;
					auto checkboxPair = [](const char* leftLabel, bool* leftValue, const char* rightLabel, bool* rightValue) -> unsigned
						{
							unsigned ret = REND_FLAGS_NONE;
							ImGui::TableNextRow();
							ImGui::TableSetColumnIndex(0);
							if (leftValue && ImGui::Checkbox(leftLabel, leftValue)) { ret |= REND_FLAGS_COLUMN_0; }
							ImGui::TableSetColumnIndex(1);
							if (rightValue && ImGui::Checkbox(rightLabel, rightValue)) { ret |= REND_FLAGS_COLUMN_1; }
							return ret;
						};

					checkboxPair("Show Low LOD", &GuiRenderSettings::showLowLOD, "Show Wireframe", &GuiRenderSettings::showWireframe);
					checkboxPair("Show Backfaces", &GuiRenderSettings::showBackfaces, "Show Vertices", &GuiRenderSettings::showVerts);
					unsigned cpStartPoints = checkboxPair("Show Checkpoints", &GuiRenderSettings::showCheckpoints, "Show Starting Positions", &GuiRenderSettings::showStartpoints);
					if (cpStartPoints & REND_FLAGS_COLUMN_1) { GenerateRenderStartpointData(); }
					checkboxPair("Show BSP", &GuiRenderSettings::showBspRectTree, "Show Vis Tree", &GuiRenderSettings::showVisTree);
					unsigned skyboxRenderChanged = checkboxPair("Show Skybox", &GuiRenderSettings::showSkybox, "Show BotNodes", &GuiRenderSettings::showBots);
					if (skyboxRenderChanged & REND_FLAGS_COLUMN_0) { GenerateRenderSkyboxData(); }
					ImGui::TableNextRow();
					ImGui::TableSetColumnIndex(0);
					unsigned minimapBoundsChanged = checkboxPair("Show Intances", &GuiRenderSettings::showInstances, "Show Minimap Bounds", &GuiRenderSettings::showMinimapBounds);

					if (minimapBoundsChanged & REND_FLAGS_COLUMN_0) { GenerateRenderSkyboxData(); }
					if (minimapBoundsChanged & REND_FLAGS_COLUMN_1) { GenerateRenderMinimapBoundsData(); }

					ImGui::EndTable();
				}

				ImGui::Text("BSP Depth:");
				ImGui::BeginDisabled(!GuiRenderSettings::showBspRectTree);
				if (ImGui::BeginTable("BSP Depth", 2, ImGuiTableFlags_SizingStretchSame))
				{
					ImGui::TableNextRow();
					ImGui::TableSetColumnIndex(0);
					if (ImGui::SliderInt("Top", &GuiRenderSettings::bspTreeTopDepth, 0, GuiRenderSettings::bspTreeMaxDepth)) //top changed
					{
						GuiRenderSettings::bspTreeBottomDepth = std::max(GuiRenderSettings::bspTreeBottomDepth, GuiRenderSettings::bspTreeTopDepth);
						GenerateRenderBspData();
					}
					ImGui::TableSetColumnIndex(1);
					if (ImGui::SliderInt("Bottom", &GuiRenderSettings::bspTreeBottomDepth, 0, GuiRenderSettings::bspTreeMaxDepth)) //bottom changed
					{
						GuiRenderSettings::bspTreeTopDepth = std::min(GuiRenderSettings::bspTreeTopDepth, GuiRenderSettings::bspTreeBottomDepth);
						GenerateRenderBspData();
					}
					ImGui::EndTable();
				}
				ImGui::EndDisabled();

				ImGui::Text("Camera:");
				if (ImGui::BeginTable("Renderer Inputs", 2, ImGuiTableFlags_SizingStretchSame))
				{
					auto inputPair = [](const char* leftLabel, float& leftValue, float leftMin, float leftMax,
						const char* rightLabel, float& rightValue, float rightMin, float rightMax)
						{
							ImGui::TableNextRow();
							ImGui::TableSetColumnIndex(0);
							if (leftLabel) { if (ImGui::InputFloat(leftLabel, &leftValue)) { leftValue = Clamp(leftValue, leftMin, leftMax); } }
							else { ImGui::Dummy(ImVec2(0.0f, 0.0f)); }
							ImGui::TableSetColumnIndex(1);
							if (rightLabel) { if (ImGui::InputFloat(rightLabel, &rightValue)) { rightValue = Clamp(rightValue, rightMin, rightMax); } }
							else { ImGui::Dummy(ImVec2(0.0f, 0.0f)); }
						};

					inputPair("Move Mult", GuiRenderSettings::camMoveMult, 0.0f, std::numeric_limits<float>::max(),
						"Rotate Mult", GuiRenderSettings::camRotateMult, 0.0f, std::numeric_limits<float>::max());
					inputPair("Zoom Mult", GuiRenderSettings::camZoomMult, 0.0f, std::numeric_limits<float>::max(),
						"Sprint Mult", GuiRenderSettings::camSprintMult, 0.0f, std::numeric_limits<float>::max());
					float dummy;
					inputPair("FOV", GuiRenderSettings::camFovDeg, 5.0f, 150.0f, nullptr, dummy, 0.0f, 0.0f);
					ImGui::EndTable();
				}

				ImGui::Text("Camera Bindings:");
				auto DrawKeyRow = [](const char* label, int& keyValue)
					{
						ImGui::TableNextRow();
						ImGui::TableSetColumnIndex(0);
						ImGui::TextUnformatted(label);
						ImGui::TableSetColumnIndex(1);
						ImGuiKey currentKey = static_cast<ImGuiKey>(keyValue);
						std::string preview = keyOptions[currentKey];
						std::string comboId = std::string("##") + label;
						if (ImGui::BeginCombo(comboId.c_str(), preview.c_str()))
						{
							for (const auto& entry : keyOptions)
							{
								bool selected = (entry.first == currentKey);
								if (ImGui::Selectable(entry.second.c_str(), selected))
								{
									keyValue = entry.first;
								}
								if (selected) { ImGui::SetItemDefaultFocus(); }
							}
							ImGui::EndCombo();
						}
					};

				auto DrawMouseRow = [](const char* label, int& buttonValue)
					{
						constexpr std::pair<int, const char*> mouseOptions[] = {
							{ImGuiMouseButton_Left, "Left"},
							{ImGuiMouseButton_Right, "Right"},
							{ImGuiMouseButton_Middle, "Middle"},
						};

						ImGui::TableNextRow();
						ImGui::TableSetColumnIndex(0);
						ImGui::TextUnformatted(label);
						ImGui::TableSetColumnIndex(1);
						std::string comboId = std::string("##") + label;
						if (ImGui::BeginCombo(comboId.c_str(), mouseOptions[buttonValue].second))
						{
							for (const auto& entry : mouseOptions)
							{
								bool selected = (entry.first == buttonValue);
								if (ImGui::Selectable(entry.second, selected))
								{
									buttonValue = entry.first;
								}
								if (selected) { ImGui::SetItemDefaultFocus(); }
							}
							ImGui::EndCombo();
						}
					};

				if (ImGui::BeginTable("Camera Bindings Table", 2, ImGuiTableFlags_SizingStretchSame))
				{
					DrawMouseRow("Orbit/Drag Mouse Button", GuiRenderSettings::camOrbitMouseButton);
					DrawKeyRow("Forward", GuiRenderSettings::camKeyForward);
					DrawKeyRow("Back", GuiRenderSettings::camKeyBack);
					DrawKeyRow("Left", GuiRenderSettings::camKeyLeft);
					DrawKeyRow("Right", GuiRenderSettings::camKeyRight);
					DrawKeyRow("Up", GuiRenderSettings::camKeyUp);
					DrawKeyRow("Down", GuiRenderSettings::camKeyDown);
					DrawKeyRow("Sprint", GuiRenderSettings::camKeySprint);
					ImGui::EndTable();
				}
				ImGui::TreePop();
			}

			ImGui::Separator();
			ImGui::Checkbox("Show Selected Quadblock Info", &GuiRenderSettings::showSelectedQuadblockInfo);
			ImGui::Separator();
			ImGui::NewLine();

			static size_t prevSelectedQuadblock = REND_NO_SELECTED_QUADBLOCK;
			if (!m_rendererSelectedQuadblockIndexes.empty() && GuiRenderSettings::showSelectedQuadblockInfo)
			{
				size_t currentIndex = m_rendererSelectedQuadblockIndexes.back();
				if (currentIndex < m_quadblocks.size())
				{
					Quadblock& quadblock = m_quadblocks[currentIndex];
					bool resetBsp = false;
					if (prevSelectedQuadblock != currentIndex)
					{
						prevSelectedQuadblock = currentIndex;
						ImGui::SetNextItemOpen(true);
					}
					if (quadblock.RenderUI(m_checkpoints.size() - 1, resetBsp))
					{
						ManageTurbopad(quadblock);
					}
					if (resetBsp && m_bsp.IsValid())
					{
						m_bsp.Clear();
						GenerateRenderBspData();
					}
				}
			}
		}
		ImGui::End();
	}

	if (Settings::w_python)
	{
		ImGui::SetNextWindowSize(ImVec2(600.0f, 320.0f), ImGuiCond_FirstUseEver);
		if (ImGui::Begin("Python", &Settings::w_python))
		{
			ImGui::Text("Script Editor");
			ImGui::SameLine();
			if (ImGui::Button("Open .py"))
			{
				auto selection = pfd::open_file("Open Python Script", Settings::m_lastOpenedScriptFolder, {"Python Files", "*.py"}, pfd::opt::force_path).result();
				if (!selection.empty())
				{
					Settings::m_lastOpenedScriptFolder = std::filesystem::path(selection.front()).parent_path().string();
					std::string pathError;
					if (!Script::AppendPythonPath(std::filesystem::path(selection.front()).parent_path(), pathError))
					{
						printf(pathError.c_str());
					}
					std::ifstream input(selection.front(), std::ios::binary);
					if (input)
					{
						std::ostringstream buffer;
						buffer << input.rdbuf();
						m_pythonScript = buffer.str();
					}
				}
			}
			ImGui::Separator();
			const ImVec2 avail = ImGui::GetContentRegionAvail();
			const float editorHeight = avail.y * 0.7f;
			const float consoleHeight = std::max(0.0f, avail.y - editorHeight - ImGui::GetFrameHeightWithSpacing() * 2.0f);
			ImVec2 editorSize = ImVec2(avail.x, editorHeight);
			ImGui::InputTextMultiline("##python_editor", &m_pythonScript, editorSize, ImGuiInputTextFlags_AllowTabInput);

			if (ImGui::Button("Run"))
			{
				m_saveScript = true;
				m_pythonConsole.clear();
				std::string result = Script::ExecutePythonScript(*this, renderer, m_pythonScript);
				if (result.empty()) { result = "[No output]"; }
				if (!m_pythonConsole.empty() && m_pythonConsole.back() != '\n')
				{
					m_pythonConsole += '\n';
				}
				m_pythonConsole += result;
				if (m_pythonConsole.back() != '\n') { m_pythonConsole += '\n'; }
			}
			ImGui::SameLine();
			static bool displayHelper = true;
			if (ImGui::Button("Clear Console")) { m_pythonConsole.clear(); displayHelper = false; }
			ImGui::SameLine();
			if (ImGui::Button("Copy to Clipboard"))
			{
				if (!m_pythonConsole.empty())
				{
					ImGui::SetClipboardText(m_pythonConsole.c_str());
				}
			}

			ImGui::Separator();
			ImGui::Text("Console Output:");
			ImGui::BeginChild("##python_console", ImVec2(0.0f, consoleHeight), true, ImGuiWindowFlags_HorizontalScrollbar);
			if (displayHelper && m_pythonConsole.empty())
			{
				ImGui::TextUnformatted("Console output will appear here.");
			}
			else
			{
				ImGui::TextUnformatted(m_pythonConsole.c_str());
			}
			ImGui::EndChild();
		}
		ImGui::End();
	}


	if (Settings::w_modelImporter)
	{
		if (ImGui::Begin("Model Importer", &Settings::w_modelImporter))
		{
			static std::string modelPathString = "";
			static std::filesystem::path modelPath;
			ImGui::Text("Model Path"); ImGui::SameLine();
			ImGui::InputText("##modelpath_importer", &modelPathString, ImGuiInputTextFlags_ReadOnly);
			ImGui::SetItemTooltip(modelPathString.c_str()); ImGui::SameLine();
			if (ImGui::Button("...##modelimporter"))
			{
				auto selection = pfd::open_file("CTR Model File", Settings::m_lastOpenedModelFolder, { "CTR Model Files", "*.json" }, pfd::opt::force_path).result();
				if (!selection.empty())
				{ 
					Settings::m_lastOpenedModelFolder = std::filesystem::path(selection.front()).parent_path().string();
					modelPath = selection.front();
					modelPathString = modelPath.string();
				}
			}

			bool disabled = modelPath.empty();
			ImGui::BeginDisabled(disabled);

			static ButtonUI importModelButton = ButtonUI();
			static std::string importModelButtonMessage;
			if (importModelButton.Show("Import Model", importModelButtonMessage, false))
			{
				InstanceModel model(modelPath, m_materialToTexture);
				if (model.IsValid())
				{
					importModelButtonMessage = "Successfully imported" + model.GetName();
					size_t modelKey = GenerateUniqueModelKey();
					m_instanceModels[modelKey] = model;
				}
				else
					importModelButtonMessage = "Failed to import the model";
			}
			ImGui::EndDisabled();
			if (disabled) { ImGui::SetItemTooltip("You must select a .json file before importing."); }
			ImGui::SameLine();
			if (ImGui::Button("New Model"))
			{
				InstanceModel model;
				size_t modelKey = GenerateUniqueModelKey();
				m_instanceModels[modelKey] = model;
			}

			// Show list of currently loaded models
			ImGui::Separator();
			if (ImGui::TreeNodeEx((void*)this, ImGuiTreeNodeFlags_None, "Loaded models (%zu)", m_instanceModels.size()))
			{
				ImGui::Separator();

				if (!m_instanceModels.empty())
				{
					std::vector<size_t> modelToDelete;
					for (auto& [modelKey, instModel] : m_instanceModels)
					{
						ImGui::PushID(static_cast<int>(modelKey));

						if (instModel.RenderUI(m_materialToTexture, [&]() { this->UpdateAnimationRenderData(); }))
							modelToDelete.push_back(modelKey);
						ImGui::PopID();
						
					}

					// Delete the model after iteration to avoid iterator invalidation
					for (size_t key : modelToDelete)
						m_instanceModels.erase(key);
					modelToDelete.clear();
				}
				else
				{
					ImGui::TextDisabled("No models loaded");
				}

				ImGui::TreePop();
			}

			// Model Instances section
			ImGui::Separator();
			ImGui::Text("Model Instances (%zu)", m_instances.size());
			ImGui::Separator();

			// Add Instance button
			bool hasModels = !m_instanceModels.empty();
			if (!hasModels)
			{
				ImGui::BeginDisabled();
			}

			if (ImGui::Button("+ Add Instance"))
			{
				m_instances.emplace_back(m_instanceModels.begin()->first);
				GenerateRenderInstanceData();
			}

			if (!hasModels)
			{
				ImGui::EndDisabled();
				if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
				{
					ImGui::SetTooltip("Import a model first");
				}
			}

			// Create Instance Row section
			bool instanceRowCreated = false;
			if (!m_instances.empty())
			{
				ImGui::Separator();
				if (ImGui::TreeNodeEx("Create Instance Row", 0))
				{
					static int createRowInstanceIndex = 0;
					static int createRowNumInstances = 4;
					static float createRowSpacing = 4.5f;
					static bool createRowDeleteAfter = false;
					static ButtonUI createRowButton = ButtonUI();
					static std::string createRowMessage;

					if (createRowInstanceIndex >= static_cast<int>(m_instances.size()))
						createRowInstanceIndex = static_cast<int>(m_instances.size()) - 1;

					std::string preview = m_instances[createRowInstanceIndex].GetName();
					if (preview.empty())
						preview = "Instance " + std::to_string(createRowInstanceIndex + 1);

					if (ImGui::BeginCombo("Source Instance", preview.c_str()))
					{
						for (size_t i = 0; i < m_instances.size(); i++)
						{
							bool isSelected = (createRowInstanceIndex == static_cast<int>(i));
							std::string label = m_instances[i].GetName();
							if (label.empty())
								label = "Instance " + std::to_string(i + 1);
							if (ImGui::Selectable(label.c_str(), isSelected))
								createRowInstanceIndex = static_cast<int>(i);
							if (isSelected)
								ImGui::SetItemDefaultFocus();
						}
						ImGui::EndCombo();
					}

					ImGui::SetNextItemWidth(100.0f);
					ImGui::InputInt("Count", &createRowNumInstances);
					if (createRowNumInstances < 1) createRowNumInstances = 1;

					ImGui::SetNextItemWidth(200.0f);
					ImGui::DragFloat("Spacing", &createRowSpacing, 0.1f, 0.1f, 30.0f, "%.1f");

					ImGui::Checkbox("Delete instance after", &createRowDeleteAfter);

					int checkpointIndex = -1;
					if (!m_rendererSelectedQuadblockIndexes.empty())
					{
						size_t qbIdx = m_rendererSelectedQuadblockIndexes.back();
						if (qbIdx < m_quadblocks.size())
							checkpointIndex = m_quadblocks[qbIdx].GetCheckpoint();
					}

					if (checkpointIndex < 0)
					{
						ImGui::BeginDisabled();
					}

					if (createRowButton.Show("Create Instance Row", createRowMessage, false))
					{
						if (checkpointIndex < 0 || checkpointIndex >= static_cast<int>(m_checkpoints.size()))
						{
							createRowMessage = "No valid checkpoint selected.";
						}
						else if (GenerateInstanceRow(checkpointIndex, createRowInstanceIndex, createRowNumInstances, createRowSpacing, createRowDeleteAfter))
						{
							GenerateRenderInstanceData();
							createRowMessage = "Successfully created the instance row.";
							if (createRowDeleteAfter)
							{
								m_closeInstanceIndex = -1;
								m_openInstanceIndex = createRowInstanceIndex;
							}
							else
							{
								m_closeInstanceIndex = createRowInstanceIndex;
								m_openInstanceIndex = createRowInstanceIndex + 1;
							}
							instanceRowCreated = true;
						}
						else
						{
							createRowMessage = "Failed creating the instance row.";
						}
					}

					if (checkpointIndex < 0)
					{
						ImGui::EndDisabled();
						if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
							ImGui::SetTooltip("Select a quadblock with a checkpoint assigned");
					}

					ImGui::TreePop();
				}
			}

			// Show instances
			int instanceToDelete = -1;
			int instanceToDuplicate = -1;
			bool renderInstanceNeedsUpdate = false;
			for (size_t i = 0; i < m_instances.size(); i++)
			{
				// Auto-open/close for duplication
				if (static_cast<int>(i) == m_closeInstanceIndex)
					ImGui::SetNextItemOpen(false);
				if (static_cast<int>(i) == m_openInstanceIndex)
					ImGui::SetNextItemOpen(true);

				ImGui::PushID(static_cast<int>(i));
				bool shouldDelete = false;
				bool shouldDuplicate = false;
				if (m_instances[i].RenderUI(shouldDelete, shouldDuplicate, static_cast<int>(i), m_instanceModels, m_rendererQueryPoint, m_quadblocks))
					renderInstanceNeedsUpdate = true;
				if (shouldDelete)
					instanceToDelete = static_cast<int>(i);
				if (shouldDuplicate)
					instanceToDuplicate = static_cast<int>(i);
				ImGui::PopID();
			}

		// Delete instance after iteration
		if (instanceToDelete >= 0)
		{
			m_instances.erase(m_instances.begin() + instanceToDelete);
			GenerateRenderInstanceData();
		}

			// Duplicate instance after iteration
			if (instanceRowCreated)
			{
				m_openInstanceIndex = -1;
				m_closeInstanceIndex = -1;
			}
			else if (instanceToDuplicate >= 0)
			{
				m_closeInstanceIndex = instanceToDuplicate;
				m_instances.push_back(m_instances[instanceToDuplicate]);
				GenerateRenderInstanceData();
				Instance& dup = m_instances.back();

				dup.SetName(GenerateUniqueInstanceName(dup.GetName()));

				// Track for auto-open
				m_openInstanceIndex = static_cast<int>(m_instances.size() - 1);
			}
			else
			{
				m_openInstanceIndex = -1;
				m_closeInstanceIndex = -1;
			}

			if (renderInstanceNeedsUpdate)
				GenerateRenderInstanceData();

			if (m_instances.empty())
			{
				ImGui::TextDisabled("No instances created");
			}
		}
		ImGui::End();

		// Live update instance transforms without full regeneration
		if (!m_instances.empty())
		{
			Model* instanceModel = GetInstancesModel();
			if (instanceModel && instanceModel->GetModelCount() >= m_instances.size() * 2)
			{
				for (size_t i = 0; i < m_instances.size(); i++)
				{
					const Instance& inst = m_instances[i];
					Vec3 pos = inst.GetPos();

					Model* geom = instanceModel->GetModel(i * 2);
					if (geom) {
						geom->SetPosition(pos);
						geom->SetRotationYXZ(inst.GetRot());
						geom->SetScale(inst.GetScale());
					}

					Model* label = instanceModel->GetModel(i * 2 + 1);
					if (label) {
						Vec3 labelPos = pos;
						labelPos.y += 3.0f;
						label->SetPosition(labelPos);
					}
				}
			}
		}
	}

	if (Settings::w_bot)
	{
		if (ImGui::Begin("Bot Paths", &Settings::w_bot))
		{
			static std::filesystem::path s_objPaths[3];
			static std::string s_objNames[3] = { "No file selected", "No file selected", "No file selected" };
			static std::string generatePathButtonMessage[3];
			static ButtonUI generatePathButton[3] = { ButtonUI(), ButtonUI(), ButtonUI() };

			ImGui::SeparatorText("Settings");
			ImGui::Checkbox("Use Manual Path", &m_botPathSettings.useManualPath);
			ImGui::SameLine();
			ImGui::BeginDisabled(m_botPathSettings.useManualPath);
			ImGui::SetNextItemWidth(200.0f);
			ImGui::DragFloat("Sideway Path Offset", &m_botPathSettings.sidewayOffset, 0.1f, 0.1f, 30.0f, "%.1f");
			ImGui::EndDisabled();
			ImGui::Checkbox("Normalize Node Distance", &m_botPathSettings.normalizeNodeDist);
			if (m_botPathSettings.normalizeNodeDist)
			{
				ImGui::SameLine();
				ImGui::SetNextItemWidth(200.0f);
				ImGui::DragFloat("Node Distance", &m_botPathSettings.nodeDistance, 0.1f, 0.1f, 30.0f, "%.1f");
			}

			for (int i = 0; i < 3; i++)
			{
				ImGui::PushID(i);
				const std::string pathLabel = "Bot Path " + std::to_string(i);
				ImGui::SeparatorText(pathLabel.c_str());
				// File selection
				ImGui::SetNextItemWidth(200.0f);
				ImGui::BeginDisabled();
				ImGui::InputText("##botpathobj", &s_objNames[i], ImGuiInputTextFlags_ReadOnly);
				ImGui::EndDisabled();
				ImGui::SameLine();
				ImGui::BeginDisabled(!m_botPathSettings.useManualPath);
				if (ImGui::Button(("Browse##selectbotpath" + std::to_string(i)).c_str()))
				{
					auto selection = pfd::open_file("Select Path OBJ", ".",
						{ "OBJ Files", "*.obj", "All Files", "*" }).result();

					if (!selection.empty())
					{
						s_objPaths[i] = selection[0];
						s_objNames[i] = s_objPaths[i].filename().string();
					}
				}

				ImGui::SameLine();

				if (ImGui::Button(("Clear##selectbotpath" + std::to_string(i)).c_str()))
				{
					s_objPaths[i].clear();
					s_objNames[i] = "No file selected";
				}
				ImGui::EndDisabled();

				// Generate button

				if (generatePathButton[i].Show(("Generate BotPath " + std::to_string(i)).c_str(), generatePathButtonMessage[i], false))
				{
					bool success = false;
					if (m_botPathSettings.useManualPath && !s_objPaths[i].empty())
					{
						std::vector<Vec3> vec = LoadPath(s_objPaths[i]);
						if (m_botPathSettings.normalizeNodeDist)
							vec = NormalizePos(vec, m_botPathSettings.nodeDistance, true); 
						success = m_botPaths[i].GeneratePath(vec, m_quadblocks);
					}
					else
					{
						// use checkpoint node for the bot node.
						std::vector<Vec3> vec;
						int ckpt_id = 0;
						while (m_checkpoints[ckpt_id].GetUp() > 0)
						{
							vec.push_back(m_checkpoints[ckpt_id].GetPos());
							ckpt_id = m_checkpoints[ckpt_id].GetUp();
						}

						if (m_botPathSettings.normalizeNodeDist) 
							vec = NormalizePos(vec, m_botPathSettings.nodeDistance, true); 
						if (i == 1) // Middle Path
						{
							success = m_botPaths[i].GeneratePath(vec, m_quadblocks);
						}
						else
						{
							BotPath middlePath{};
							middlePath.GeneratePath(vec, m_quadblocks);
							vec = GenerateLateralPath(middlePath.GetNodes(), i == 0 ? -m_botPathSettings.sidewayOffset : +m_botPathSettings.sidewayOffset, m_quadblocks);
							if (m_botPathSettings.normalizeNodeDist) 
								vec = NormalizePos(vec, m_botPathSettings.nodeDistance, true);
							success = m_botPaths[i].GeneratePath(vec, m_quadblocks);
						}
					}

					if (!success)
						generatePathButtonMessage[i] = "Failed to generate BotPath";
					else
						generatePathButtonMessage[i] = "Successfully generated BotPath";
					UpdateRenderBotData();
					GenerateBotPathChangeCode();
				}
				m_botPaths[i].RenderUI(i);
				if (i<2){ ImGui::Separator(); }
				ImGui::PopID();
			}
		}
		ImGui::End();
	}
}

void Path::RenderUI(const std::string& title, const std::vector<Quadblock>& quadblocks, const std::string& searchQuery, bool& insertAbove, bool& removePath, const std::vector<size_t>& selectedIndexes, bool mainPath)
{
	auto QuadListUI = [this, &selectedIndexes](std::vector<size_t>& indexes, size_t& value, std::string& label, const std::string& title, const std::vector<Quadblock>& quadblocks, const std::string& searchQuery, ButtonUI& button)
		{
			if (ImGui::BeginChild(title.c_str(), {0, 0}, ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AutoResizeX))
			{
				ImGui::Text(title.substr(0, title.find("##")).c_str());
				if (ImGui::TreeNode("Quad list:"))
				{
					std::vector<size_t> deleteList;
					for (size_t i = 0; i < indexes.size(); i++)
					{
						ImGui::Text(quadblocks[indexes[i]].GetName().c_str()); ImGui::SameLine();
						if (ImGui::Button(("Remove##" + title + std::to_string(i)).c_str()))
						{
							deleteList.push_back(i);
						}
					}
					if (!deleteList.empty())
					{
						for (int i = static_cast<int>(deleteList.size()) - 1; i >= 0; i--)
						{
							indexes.erase(indexes.begin() + deleteList[i]);
						}
					}
					ImGui::TreePop();
				}

				if (ImGui::BeginCombo(("##" + title).c_str(), label.c_str()))
				{
					for (size_t i = 0; i < quadblocks.size(); i++)
					{
						if (Matches(quadblocks[i].GetName(), searchQuery))
						{
							if (ImGui::Selectable(quadblocks[i].GetName().c_str()))
							{
								label = quadblocks[i].GetName();
								value = i;
							}
						}
					}
					ImGui::EndCombo();
				}

				auto AppendIndex = [&indexes](size_t value)
					{
						bool found = false;
						for (const size_t index : indexes)
						{
							if (index == value) { found = true; break; }
						}
						if (!found) { indexes.push_back(value); }
					};

				if (button.Show(("Add##" + title).c_str(), "Quadblock successfully\nadded to path.", false))
				{
					AppendIndex(value);
				}

				ImGui::SameLine();
				if (ImGui::Button(("Add Selected##" + title).c_str()))
				{
					for (size_t index : selectedIndexes) { AppendIndex(index); }
				}
			}
			ImGui::EndChild();
		};

	bool popColor = false;
	if (mainPath)
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(m_color.Red(), m_color.Green(), m_color.Blue(), 1.0f));
		popColor = true;
	}
	if (ImGui::TreeNode(title.c_str()))
	{
		if (mainPath)
		{
			ImGui::Text("Color:"); ImGui::SameLine();
			float color[3] = {m_color.Red(), m_color.Green(), m_color.Blue()};
			if (ImGui::ColorEdit3(("##color" + title).c_str(), color))
			{
				m_color = Color(color[0], color[1], color[2]);
				if (m_left) { m_left->SetColor(m_color); }
				if (m_right) { m_right->SetColor(m_color); }
			}
			ImGui::PopStyleColor();
			popColor = false;
		}
		if (ImGui::BeginChild(("##" + title).c_str(), {0, 0}, ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AutoResizeX))
		{
			bool dummyInsert, dummyRemove = false;
			if (m_left) { m_left->RenderUI("Left Path", quadblocks, searchQuery, dummyInsert, dummyRemove, selectedIndexes, false); }
			if (m_right) { m_right->RenderUI("Right Path", quadblocks, searchQuery, dummyInsert, dummyRemove, selectedIndexes, false); }

			static ButtonUI startButton = ButtonUI();
			static ButtonUI endButton = ButtonUI();
			static ButtonUI ignoreButton = ButtonUI();
			QuadListUI(m_quadIndexesStart, m_previewValueStart, m_previewLabelStart, "Start##" + title, quadblocks, searchQuery, startButton);
			ImGui::SameLine();
			QuadListUI(m_quadIndexesEnd, m_previewValueEnd, m_previewLabelEnd, "End##" + title, quadblocks, searchQuery, endButton);
			ImGui::SameLine();
			QuadListUI(m_quadIndexesIgnore, m_previewValueIgnore, m_previewLabelIgnore, "Ignore##" + title, quadblocks, searchQuery, ignoreButton);

			if (ImGui::Button("Add Left Path"))
			{
				if (!m_left)
				{
					m_left = new Path(m_index + 1);
					m_left->SetColor(m_color);
				}
			} ImGui::SameLine();
			ImGui::BeginDisabled(m_left == nullptr);
			if (ImGui::Button("Delete Left Path"))
			{
				if (m_left)
				{
					delete m_left;
					m_left = nullptr;
				}
			}
			ImGui::EndDisabled();

			if (ImGui::Button("Add Right Path"))
			{
				if (!m_right)
				{
					m_right = new Path(m_index + 2);
					m_right->SetColor(m_color);
				}
			}
			ImGui::SameLine();
			ImGui::BeginDisabled(m_right == nullptr);
			if (ImGui::Button("Delete Right Path"))
			{
				if (m_right)
				{
					delete m_right;
					m_right = nullptr;
				}
			}
			ImGui::EndDisabled();
		}
		ImGui::EndChild();

		if (mainPath)
		{
			static ButtonUI insertAboveButton;
			static ButtonUI removePathButton;
			if (insertAboveButton.Show(("Insert Path Above##" + std::to_string(m_index)).c_str(), "You're editing the new path.", false)) { insertAbove = true; }
			if (removePathButton.Show(("Remove Current Path##" + std::to_string(m_index)).c_str(), "Path successfully deleted.", false)) { removePath = true; }
		}

		ImGui::TreePop();
	}
	if (popColor) { ImGui::PopStyleColor(); }
}

bool Quadblock::RenderUI(size_t checkpointCount, bool& resetBsp)
{
	bool ret = false;
	if (ImGui::TreeNode(m_name.c_str()))
	{
		if (ImGui::TreeNode("Vertices"))
		{
			for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
			{
				bool editedPos = false;
				m_p[i].RenderUI(i, editedPos);
				if (editedPos)
				{
					resetBsp = true;
					ComputeBoundingBox();
				}
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Bounding Box"))
		{
			m_bbox.RenderUI();
			ImGui::TreePop();
		}
		if (!m_texPath.empty() && ImGui::TreeNode("Texture"))
		{
			std::string texPath = m_texPath.string();
			ImGui::Text("Path:"); ImGui::SameLine();
			ImGui::BeginDisabled();
			ImGui::InputText("##texpath", &texPath, ImGuiInputTextFlags_ReadOnly);
			ImGui::EndDisabled();
			ImGui::Text("UVs:");
			for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
			{
				std::string title = i == NUM_FACES_QUADBLOCK ? "Low Quad" : "Quad " + std::to_string(i);
				if (ImGui::TreeNode(title.c_str()))
				{
					ImGui::InputFloat2("Top left:", &m_uvs[i][0].x, "%.2f");
					ImGui::InputFloat2("Top right:", &m_uvs[i][1].x, "%.2f");
					ImGui::InputFloat2("Bottom left:", &m_uvs[i][2].x, "%.2f");
					ImGui::InputFloat2("Bottom right:", &m_uvs[i][3].x, "%.2f");
					ImGui::TreePop();
				}
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Terrain"))
		{
			std::string terrainLabel;
			for (const auto& [label, terrain] : TerrainType::LABELS)
			{
				if (terrain == m_terrain) { terrainLabel = label; break; }
			}
			if (ImGui::BeginCombo("##terrain", terrainLabel.c_str()))
			{
				for (const auto& [label, terrain] : TerrainType::LABELS)
				{
					if (ImGui::Selectable(label.c_str()))
					{
						m_terrain = terrain;
					}
				}
				ImGui::EndCombo();
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Quad Flags"))
		{
			for (const auto& [label, flag] : QuadFlags::LABELS)
			{
				UIFlagCheckbox(m_flags, flag, label);
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Draw Flags"))
		{
			ImGui::Checkbox("Double Sided", &m_doubleSided);
			ImGui::Text("Z depth bias:");
			ImGui::SameLine();
			if (ImGui::InputInt("##draworderHigh", &m_drawOrderHigh)) { m_drawOrderHigh = Clamp(m_drawOrderHigh, static_cast<int>(INT8_MIN), static_cast<int>(INT8_MAX)); }
			const std::vector<std::string> s_rotateFlip = {"None", "Rotate 90", "Rotate 180", "Rotate -90", "Flip + Rotate 90", "Flip + Rotate 180", "Flip + Rotate -90", "Flip"};
			const std::vector<std::string> s_faceDrawMode = {"Both", "Left", "Right", "None"};

			auto UISelectable = [](size_t quadIndex, const std::string& label, const std::vector<std::string>& options, uint32_t* data)
				{
					if (ImGui::BeginCombo((label + "##" + std::to_string(quadIndex)).c_str(), options[data[quadIndex]].c_str()))
					{
						for (size_t i = 0; i < options.size(); i++)
						{
							if (ImGui::Selectable(options[i].c_str()))
							{
								data[quadIndex] = static_cast<int>(i);
							}
						}
						ImGui::EndCombo();
					}
				};
			for (size_t i = 0; i < NUM_FACES_QUADBLOCK; i++)
			{
				ImGui::Text(("Face " + std::to_string(i)).c_str());
				UISelectable(i, "Rotate Flip", s_rotateFlip, m_faceRotateFlip);
				UISelectable(i, "Draw Mode", s_faceDrawMode, m_faceDrawMode);
			}

			ImGui::TreePop();
		}
		ImGui::Text("Downforce:");
		ImGui::SameLine();
		if (ImGui::InputInt("##downforceQuad", &m_downforce)) { m_downforce = Clamp(m_downforce, static_cast<int>(INT8_MIN), static_cast<int>(INT8_MAX)); }
		ImGui::Text("Weather Intensity:");
		ImGui::SameLine();
		if (ImGui::InputInt("##Weather IntensityQuad", &m_weatherIntensity)) { m_weatherIntensity = Clamp(m_weatherIntensity, static_cast<int>(0), static_cast<int>(UINT8_MAX)); }
		ImGui::Text("Weather Vanish Rate:");
		ImGui::SameLine();
		if (ImGui::InputInt("##Weather Vanish RateQuad", &m_weatherVanishRate)) { m_weatherVanishRate = Clamp(m_weatherVanishRate, static_cast<int>(0), static_cast<int>(UINT8_MAX)); }
		ImGui::Checkbox("Checkpoint", &m_checkpointStatus);
		ImGui::SameLine();
		ImGui::Checkbox("Checkpoint Pathable", &m_checkpointPathable);
		ImGui::Text("Checkpoint Index: ");
		ImGui::SameLine();
		if (ImGui::InputInt("##cp", &m_checkpointIndex)) { m_checkpointIndex = Clamp(m_checkpointIndex, -1, static_cast<int>(checkpointCount)); }
		ImGui::Checkbox("Water", &m_water);
		ImGui::Checkbox("VisTree Transparency", &m_visTreeTransparent);
		ImGui::Text("Trigger:");
		if (ImGui::RadioButton("None", m_trigger == QuadblockTrigger::NONE))
		{
			m_trigger = QuadblockTrigger::NONE;
			m_flags = QuadFlags::DEFAULT;
			resetBsp = true;
			ret = true;
		} ImGui::SameLine();
		if (ImGui::RadioButton("Turbo Pad", m_trigger == QuadblockTrigger::TURBO_PAD))
		{
			m_trigger = QuadblockTrigger::TURBO_PAD;
			resetBsp = true;
			ret = true;
		} ImGui::SameLine();
		if (ImGui::RadioButton("Super Turbo Pad", m_trigger == QuadblockTrigger::SUPER_TURBO_PAD))
		{
			m_trigger = QuadblockTrigger::SUPER_TURBO_PAD;
			resetBsp = true;
			ret = true;
		}
		ImGui::TreePop();
	}
	return ret;
}

void Vertex::RenderUI(size_t index, bool& editedPos)
{
	if (ImGui::TreeNode(("Vertex " + std::to_string(index)).c_str()))
	{
		ImGui::Text("Pos: "); ImGui::SameLine();
		if (ImGui::InputFloat3("##pos", m_pos.Data())) { editedPos = true; }
		ImGui::Text("High:"); ImGui::SameLine();
		float colorHighData[3] = {m_colorHigh.Red(), m_colorHigh.Green(), m_colorHigh.Blue()};
		if (ImGui::ColorEdit3("##high", colorHighData))
		{
			m_colorHigh = Color(static_cast<float>(colorHighData[0]), colorHighData[1], colorHighData[2]);
		}
		ImGui::Text("Low: "); ImGui::SameLine();
		float colorLowData[3] = {m_colorLow.Red(), m_colorLow.Green(), m_colorLow.Blue()};
		if (ImGui::ColorEdit3("##low", colorLowData))
		{
			m_colorLow = Color(static_cast<float>(colorLowData[0]), colorLowData[1], colorLowData[2]);
		}
		ImGui::TreePop();
	}
}

void Texture::RenderUI(const std::vector<size_t>& quadblockIndexes, std::vector<Quadblock>& quadblocks, std::function<void(void)> refreshTextureStores)
{
	std::string texPath = GetPath().string();
	if (ImGui::TreeNode(("Texture##" + texPath).c_str()))
	{
		ImGui::Text("Path:"); ImGui::SameLine();
		ImGui::BeginDisabled();
		ImGui::InputText("##texpath", &texPath, ImGuiInputTextFlags_ReadOnly);
		ImGui::EndDisabled();
		ImGui::SetItemTooltip(texPath.c_str());
		ImGui::SameLine();
		if (ImGui::Button("..."))
		{
			std::filesystem::path currentPath = GetPath();
			std::string defaultDir = currentPath.has_parent_path() ? currentPath.parent_path().string() : ".";
			std::string defaultFile = currentPath.filename().string();
			auto selection = pfd::open_file("Texture File", defaultDir, {"Texture Files", "*.bmp, *.jpeg, *.jpg, *.png"}).result();
			if (!selection.empty())
			{
				const std::filesystem::path& newTexPath = selection.front();
				UpdateTexture(newTexPath);
				refreshTextureStores();
				for (const size_t index : quadblockIndexes) { quadblocks[index].SetTexPath(newTexPath); }
			}
		}
		if (IsEmpty()) { ImGui::TreePop(); return; }
		constexpr size_t NUM_BLEND_MODES = 4;
		const std::array<std::string, NUM_BLEND_MODES> BLEND_MODES = {"Half Transparent", "Additive", "Subtractive", "Additive Translucent"};
		uint16_t blendMode = GetBlendMode();
		ImGui::Text("Blend Mode:"); ImGui::SameLine();
		if (ImGui::BeginCombo("##blendmode", BLEND_MODES[blendMode].c_str()))
		{
			for (size_t i = 0; i < NUM_BLEND_MODES; i++)
			{
				if (ImGui::Selectable(BLEND_MODES[i].c_str()))
				{
					SetBlendMode(static_cast<uint16_t>(i));
				}
			}
			ImGui::EndCombo();
		}
		ImGui::TreePop();
	}
}

void Texture::RenderUI()
{
	std::vector<size_t> dummyIndexes;
	std::vector<Quadblock> dummyQuadblocks;
	RenderUI(dummyIndexes, dummyQuadblocks, []() {});
}

bool AnimTexture::RenderUI(std::vector<std::string>& animTexNames, std::vector<Quadblock>& quadblocks, const std::map<std::string, std::vector<size_t>>& materialMap, const std::string& query, std::vector<AnimTexture>& newTextures)
{
	bool ret = true;
	if (ImGui::TreeNode(m_name.c_str()))
	{
		if (ImGui::TreeNode("Quadblocks"))
		{
			constexpr size_t QUADS_PER_LINE = 10;
			for (size_t i = 0; i < m_quadblockIndexes.size(); i++)
			{
				ImGui::Text((quadblocks[m_quadblockIndexes[i]].GetName() + ", ").c_str());
				if (((i + 1) % QUADS_PER_LINE) == 0 || i == m_quadblockIndexes.size() - 1) { continue; }
				ImGui::SameLine();
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Settings"))
		{
			size_t frames = m_frames.size();
			ImGui::Text(("Frames: " + std::to_string(frames)).c_str());
			ImGui::Text("Start at Frame:"); ImGui::SameLine();
			ImGui::SliderInt("##startat", &m_startAtFrame, 0, static_cast<int>(frames));
			float durationMS = (1.0f + static_cast<float>(m_duration)) / 30.0f;
			ImGui::Text("Duration per Frame:"); ImGui::SameLine();
			ImGui::InputInt("##duration", &m_duration);
			m_duration = std::max(m_duration, 0);
			std::stringstream ss;
			ss << std::fixed << std::setprecision(3) << durationMS;
			ImGui::Text(("Duration per Frame: " + ss.str() + "s").c_str());

			ImGui::Text("Enable manual rotation:"); ImGui::SameLine();
			ImGui::Checkbox("##manualrot", &m_manualOrientation);
			ImGui::SetItemTooltip("When manual rotation is disabled,\nthe editor will try to find the matching quadblock direction\nbased on the UV coordinates of the original quadblock.");
			ImGui::BeginDisabled(!m_manualOrientation);
			ImGui::Text("Rotation:"); ImGui::SameLine();
			if (ImGui::RadioButton("0 deg", m_rotation == 0))
			{
				RotateFrames(0 - m_rotation);
				m_rotation = 0;
			} ImGui::SameLine();
			if (ImGui::RadioButton("90 deg", m_rotation == 90))
			{
				RotateFrames(90 - m_rotation);
				m_rotation = 90;
			} ImGui::SameLine();
			if (ImGui::RadioButton("180 deg", m_rotation == 180))
			{
				RotateFrames(180 - m_rotation);
				m_rotation = 180;
			} ImGui::SameLine();
			if (ImGui::RadioButton("270 deg", m_rotation == 270))
			{
				RotateFrames(270 - m_rotation);
				m_rotation = 270;
			}
			ImGui::Text("Mirror:"); ImGui::SameLine();
			ImGui::Checkbox("Horizontal", &m_horMirror); ImGui::SameLine();
			ImGui::Checkbox("Vertical", &m_verMirror);
			ImGui::EndDisabled();
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Textures"))
		{
			for (Texture& tex : m_textures) { tex.RenderUI(); }
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Manage"))
		{
			ImGui::Text("Select Quadblock:");
			if (ImGui::BeginCombo("##quadcombo", m_previewQuadName.c_str()))
			{
				for (size_t i = 0; i < quadblocks.size(); i++)
				{
					const Quadblock& quadblock = quadblocks[i];
					if (!quadblock.GetHide() && Matches(quadblock.GetName(), query) && ImGui::Selectable(quadblock.GetName().c_str()))
					{
						m_previewQuadName = quadblock.GetName();
						m_previewQuadIndex = i;
					}
				}
				ImGui::EndCombo();
			}

			auto FindBestOrientation = [this](std::array<QuadUV, 5>& animUVs, const std::array<QuadUV, 5>& quadUVs) -> uint32_t
				{
					auto FindBestRotation = [this](std::array<QuadUV, 5>& animUVs, const std::array<QuadUV, 5>& quadUVs) -> std::tuple<uint32_t, float>
						{
							auto MeanSquareErrorUVs = [](const std::array<QuadUV, 5>& src, const std::array<QuadUV, 5>& tgt) -> float
								{
									float mse = 0.0f;
									for (size_t i = 0; i < 4; i++)
									{
										const QuadUV& srcUV = src[i];
										const QuadUV& tgtUV = tgt[i];
										for (size_t j = 0; j < 4; j++)
										{
											mse += ((srcUV[j].x - tgtUV[j].x) * (srcUV[j].x - tgtUV[j].x)) + ((srcUV[j].y - tgtUV[j].y) * (srcUV[j].y - tgtUV[j].y));
										}
									}
									return mse;
								};

							float bestMSE = std::numeric_limits<float>::max();
							uint32_t bestRotation = 0;
							for (size_t i = 0; i < 4; i++)
							{
								float mse = MeanSquareErrorUVs(quadUVs, animUVs);
								if (mse < bestMSE)
								{
									bestMSE = mse;
									bestRotation = static_cast<uint32_t>(i);
								}
								RotateQuadUV(animUVs);
							}

							return {bestRotation, bestMSE};
						};

					std::tuple<uint32_t, float> bestOrientation = FindBestRotation(animUVs, quadUVs);

					MirrorQuadUV(true, animUVs);
					std::tuple<uint32_t, float> currOrientation = FindBestRotation(animUVs, quadUVs);
					if (std::get<float>(currOrientation) < std::get<float>(bestOrientation))
					{
						std::get<float>(bestOrientation) = std::get<float>(currOrientation);
						std::get<uint32_t>(bestOrientation) = std::get<uint32_t>(currOrientation) | 4u;
					}
					MirrorQuadUV(true, animUVs);

					MirrorQuadUV(false, animUVs);
					currOrientation = FindBestRotation(animUVs, quadUVs);
					if (std::get<float>(currOrientation) < std::get<float>(bestOrientation))
					{
						std::get<float>(bestOrientation) = std::get<float>(currOrientation);
						std::get<uint32_t>(bestOrientation) = std::get<uint32_t>(currOrientation) | 8u;
					}
					MirrorQuadUV(false, animUVs);

					return std::get<uint32_t>(bestOrientation);
				};

			static ButtonUI applyQuadBtn;
			bool found = m_previewQuadName.empty();
			if (!found)
			{
				for (size_t index : m_quadblockIndexes) { if (index == m_previewQuadIndex) { found = true; break; } }
			}
			if (applyQuadBtn.Show("Add", "Animation successfully added to quadblock.", !found) && !found)
			{
				if (!m_manualOrientation)
				{
					const std::array<QuadUV, 5>& quadUVs = quadblocks[m_previewQuadIndex].GetUVs();
					std::array<QuadUV, 5>& animUVs = m_frames[m_startAtFrame].uvs;
					uint32_t bestOrientation = FindBestOrientation(animUVs, quadUVs);
					if (bestOrientation == 0) { m_quadblockIndexes.push_back(m_previewQuadIndex); }
					else
					{
						AnimTexture newTex = AnimTexture(m_path, animTexNames);
						newTex.CopyParameters(*this);

						bool horMirror = (bestOrientation & 4) == 4;
						if (horMirror)
						{
							newTex.MirrorFrames(true);
							newTex.m_horMirror = !m_horMirror;
						}

						bool verMirror = (bestOrientation & 8) == 8;
						if (verMirror)
						{
							newTex.MirrorFrames(false);
							newTex.m_verMirror = !m_verMirror;
						}

						uint32_t rotation = bestOrientation & 0b11;
						int totalRotation = static_cast<int>(rotation) * 90;
						newTex.RotateFrames(totalRotation);
						newTex.m_rotation = (m_rotation + totalRotation) % 360;

						newTex.m_quadblockIndexes.push_back(m_previewQuadIndex);
						newTex.m_previewQuadName = m_previewQuadName;
						newTex.m_previewQuadIndex = m_previewQuadIndex;
						newTextures.push_back(newTex);
					}
				}
				else { m_quadblockIndexes.push_back(m_previewQuadIndex); }
				quadblocks[m_previewQuadIndex].SetAnimated(true);
			}

			static ButtonUI remQuadBtn;
			if (remQuadBtn.Show("Remove##quadblock", "Animation successfully removed from quadblock.", false))
			{
				auto it = m_quadblockIndexes.begin();
				for (; it != m_quadblockIndexes.end(); it++)
				{
					if (*it == m_previewQuadIndex)
					{
						quadblocks[m_previewQuadIndex].SetAnimated(false);
						m_quadblockIndexes.erase(it);
						break;
					}
				}
			}

			ImGui::Text("Apply by Material:");
			if (ImGui::BeginCombo("##matcombo", m_previewMaterialName.c_str()))
			{
				for (const auto& [material, indexes] : materialMap)
				{
					if (Matches(material, query) && ImGui::Selectable(material.c_str()))
					{
						m_previewMaterialName = material;
					}
				}
				ImGui::EndCombo();
			}

			static ButtonUI applyMatBtn;
			found = m_previewMaterialName.empty();
			if (!found) { found = m_previewMaterialName == m_lastAppliedMaterialName; }
			if (applyMatBtn.Show("Apply", "Animation successfully applied to all material quadblocks", !found) && !found)
			{
				std::unordered_map<uint32_t, AnimTexture> newAnims = {};
				std::array<QuadUV, 5>& animUVs = m_frames[m_startAtFrame].uvs;
				for (const auto& [material, indexes] : materialMap)
				{
					if (m_previewMaterialName == material)
					{
						for (const size_t index : indexes)
						{
							if (!m_manualOrientation)
							{
								const std::array<QuadUV, 5>& quadUVs = quadblocks[index].GetUVs();
								uint32_t bestOrientation = FindBestOrientation(animUVs, quadUVs);
								if (bestOrientation == 0) { m_quadblockIndexes.push_back(index); }
								else
								{
									if (newAnims.contains(bestOrientation)) { newAnims[bestOrientation].m_quadblockIndexes.push_back(index); }
									else
									{
										AnimTexture newTex = AnimTexture(m_path, animTexNames);
										animTexNames.push_back(newTex.GetName());
										newTex.CopyParameters(*this);

										if (bestOrientation & 4)
										{
											newTex.MirrorFrames(true);
											newTex.m_horMirror = !m_horMirror;
										}

										if (bestOrientation & 8)
										{
											newTex.MirrorFrames(false);
											newTex.m_verMirror = !m_verMirror;
										}

										uint32_t rotation = bestOrientation & 0b11;
										int totalRotation = static_cast<int>(rotation) * 90;
										newTex.RotateFrames(totalRotation);
										newTex.m_rotation = (m_rotation + totalRotation) % 360;

										newTex.m_lastAppliedMaterialName = m_lastAppliedMaterialName;
										newTex.m_quadblockIndexes.push_back(index);
										newAnims[bestOrientation] = newTex;
									}
								}
							}
							else { m_quadblockIndexes.push_back(index); }
							quadblocks[index].SetAnimated(true);
						}
						if (!newAnims.empty())
						{
							for (const auto& [rot, newAnim] : newAnims)
							{
								newTextures.push_back(newAnim);
							}
						}
						m_lastAppliedMaterialName = m_previewMaterialName;
						break;
					}
				}
			}

			static ButtonUI remMatBtn;
			if (remMatBtn.Show("Remove##material", "Animation successfully removed from material.", false))
			{
				std::vector<std::vector<size_t>::iterator> remList;
				for (const auto& [material, indexes] : materialMap)
				{
					if (m_previewMaterialName == material)
					{
						auto it = m_quadblockIndexes.begin();
						for (; it != m_quadblockIndexes.end(); it++)
						{
							for (const size_t index : indexes)
							{
								if (*it == index)
								{
									quadblocks[index].SetAnimated(false);
									remList.push_back(it);
								}
							}
						}
						break;
					}
				}
				for (int i = static_cast<int>(remList.size()) - 1; i >= 0; i--)
				{
					m_quadblockIndexes.erase(remList[i]);
				}
			}

			ImGui::TreePop();
		}
		if (ImGui::Button("Delete Animated Texture"))
		{
			ret = false;
		}
		ImGui::TreePop();
	}
	return ret;
}
