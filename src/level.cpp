#include "level.h"
#include "ui.h"
#include "psx_types.h"
#include "io.h"
#include "utils.h"
#include "geo.h"
#include "process.h"
#include "gui_render_settings.h"
#include "renderer.h"
#include "vistree.h"
#include "text3d.h"
#include "minimap.h"


#include <filesystem>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <array>
#include <system_error>
#include <set>
#include <map>
#include <algorithm>
#include <cstring>
#include <stb_image_write.h>

bool Level::Load(const std::filesystem::path& filename, bool isLevel)
{
	Clear(true);
	if (!filename.has_filename() || !filename.has_extension()) { return false; }
	std::filesystem::path ext = filename.extension();
	if (ext == ".lev") { return LoadLEV(filename); }
	if (ext == ".obj") { return LoadOBJ(filename, isLevel); }
	return false;
}

bool Level::Save(const std::filesystem::path& path)
{
	return false;// SaveLEV(path);
}

bool Level::IsLoaded() const
{
	return m_loaded;
}

void Level::OpenHotReloadWindow()
{
	m_showHotReloadWindow = true;
}


void Level::Clear(bool clearErrors)
{
	m_loaded = false;
	m_showHotReloadWindow = false;
	for (size_t i = 0; i < NUM_DRIVERS; i++) { m_spawn[i] = Spawn(); }
	for (size_t i = 0; i < NUM_GRADIENT; i++) { m_skyGradient[i] = ColorGradient(); }
	if (clearErrors)
	{
		m_showLogWindow = false;
		m_logMessage.clear();
		m_invalidQuadblocks.clear();
	}
	m_configFlags = LevConfigFlags::NONE;
	m_clearColor = Color();
	m_stars = {};
	m_stars.zDepth = static_cast<uint16_t>(OT_SIZE) - 2;
	m_weather = {};
	m_name.clear();
	m_hotReloadLevPath.clear();
	m_hotReloadVRMPath.clear();
	m_quadblocks.clear();
	m_checkpoints.clear();
	m_bsp.Clear();
	m_materialToQuadblocks.clear();
	m_materialToTexture.clear();
	m_checkpointPaths.clear();
	m_tropyGhost.clear();
	m_oxideGhost.clear();
	m_animTextures.clear();
	m_rendererQueryPoint = Vec3();
	m_rendererSelectedQuadblockIndexes.clear();
	//m_genVisTree = false;
	m_bspVis.Clear();
	m_bspSettings = BSPTreeSettings();
	m_visTreeSettings = VisTreeSettings();
	m_pythonConsole.clear();
	m_saveScript = false;
	m_vrm.clear();
	m_lastAnimTextureCount = 0;
	m_minimapConfig = {};
	DeleteMaterials(this);
	m_skybox.Clear();
	m_splitLines[0] = 0.0;
	m_splitLines[1] = 0.0;
	m_jumpYSpeedCap = 0;
	m_instances.clear();
	m_instanceModels.clear();
	m_spawntypes.clear();
	m_spawntypesPosRot.clear();
	m_vramData.clear();
	std::error_code ec;
	std::filesystem::remove_all(std::filesystem::temp_directory_path() / "CTE_tex_cache", ec);

	m_openInstanceIndex = -1;
	m_closeInstanceIndex = -1;
	for (int i = 0; i < 3; i++)
	{
		m_botPaths[i].Clear();
	}

	for (Model* model : m_models)
	{
		if (model) { model->Clear(model != m_models[LevelModels::LEVEL]); }
	}
	m_envMapMatName.clear();
}



const std::string& Level::GetName() const
{
	return m_name;
}

std::vector<Quadblock>& Level::GetQuadblocks()
{
	return m_quadblocks;
}

BSP& Level::GetBSP()
{
	return m_bsp;
}

BitMatrix& Level::GetVisTree()
{
	return m_bspVis;
}

std::vector<Checkpoint>& Level::GetCheckpoints()
{
	return m_checkpoints;
}

std::vector<Path>& Level::GetCheckpointPaths()
{
	return m_checkpointPaths;
}

std::vector<BotNode>& Level::GetBotPath(int i)
{
	return m_botPaths[i].GetNodes();
}

const std::filesystem::path& Level::GetParentPath() const
{
	return m_parentPath;
}

std::vector<std::string> Level::GetMaterialNames() const
{
	std::vector<std::string> names;
	names.reserve(m_materialToQuadblocks.size());
	for (const auto& [key, value] : m_materialToQuadblocks) { names.push_back(key); }
	return names;
}

std::vector<size_t> Level::GetMaterialQuadblockIndexes(const std::string& material) const
{
	if (!m_materialToQuadblocks.contains(material)) { return std::vector<size_t>(); }
	return m_materialToQuadblocks.at(material);
}

std::tuple<std::vector<Quadblock*>, Vec3> Level::GetRendererSelectedData()
{
	std::vector<Quadblock*> quadblocks;
	quadblocks.reserve(m_rendererSelectedQuadblockIndexes.size());
	for (size_t index : m_rendererSelectedQuadblockIndexes)
	{
		if (index < m_quadblocks.size()) { quadblocks.push_back(&m_quadblocks[index]); }
	}
	return std::make_tuple(std::move(quadblocks), m_rendererQueryPoint);
}

Model* Level::GetLevelModel()
{
	return m_models[LevelModels::LEVEL];
}

Model* Level::GetBspModel()
{
	return m_models[LevelModels::BSP];
}

Model* Level::GetSpawnModel()
{
	return m_models[LevelModels::SPAWN];
}

Model* Level::GetCheckpointModel()
{
	return m_models[LevelModels::CHECKPOINT];
}

Model* Level::GetBotModel()
{
	return m_models[LevelModels::BOT];
}

Model* Level::GetSelectedModel()
{
	return m_models[LevelModels::SELECTED];
}

Model* Level::GetMultiSelectedModel()
{
	return m_models[LevelModels::MULTI_SELECTED];
}

Model* Level::GetFilterModel()
{
	return m_models[LevelModels::FILTER];
}

Model* Level::GetInstancesModel()
{
	return m_models[LevelModels::INSTANCES];
}


bool Level::GenerateSpawn(float colSpacing, float rowSpacing, float centerOffset)
{
	if (m_checkpoints.size() < 2)
		return false;

	Vec3 up = { 0.0f, 1.0f, 0.0f };
	Vec3 cp0 = m_checkpoints[0].GetPos();
	Vec3 cp1 = m_checkpoints[1].GetPos();
	Vec3 center = m_checkpoints[m_checkpoints[0].GetDown()].GetPos();
	Vec3 forward = cp1 - cp0;
	forward.y = 0;
	float yaw = - std::atan2(forward.z, forward.x) * (180.0f / MATH_PI);
	yaw = std::fmod(yaw, 360.0f);
	forward.Normalize();
	Vec3 right = forward.Cross(up);

	int lastCkpt = m_checkpoints[0].GetDown();
	int prevCkpt = m_checkpoints[lastCkpt].GetDown();
	std::vector<size_t> quadindexes;
	for (size_t j = 0; j < m_quadblocks.size(); j++)
	{
		Quadblock& quad = m_quadblocks[j];
		if (quad.GetCheckpoint() != lastCkpt && quad.GetCheckpoint() != prevCkpt)
			continue;
		quadindexes.push_back(j);
	}

	for (int row = 0; row < 2; row++)
	{
		for (int col = 0; col < 4; col++)
		{
			int index = row * 4 + col;
			float lateralOffset = (col - 1.5f) * colSpacing;
			float forwardOffset = (row - 0.5f) * rowSpacing;
			Vec3 pos = center + right * lateralOffset + forward * forwardOffset + forward * centerOffset;
			Vec3 rot(0.0f, yaw, 0.0f);

			if (-1 == SnapToClosestQuad(m_quadblocks, quadindexes, pos, rot, Vec3(0.0f, 1.0f, 0.0f), -10.0f, 10.0f))
				return false;

			m_spawn[index].pos = pos;
			m_spawn[index].rot = rot;
		}
	}
	return true;
}

std::string Level::GenerateUniqueInstanceName(const std::string& name) const
{
	std::string stripped = name;
	size_t hashPos = stripped.rfind('#');
	if (hashPos != std::string::npos && hashPos + 1 < stripped.size())
	{
		std::string suffix = stripped.substr(hashPos + 1);
		if (!suffix.empty() && std::all_of(suffix.begin(), suffix.end(), ::isdigit))
			stripped = stripped.substr(0, hashPos);
	}

	int maxN = 0;
	for (const auto& inst : m_instances)
	{
		const std::string& n = inst.GetName();
		if (n.size() > stripped.size() + 1 &&
			n.compare(0, stripped.size(), stripped) == 0 &&
			n[stripped.size()] == '#')
		{
			std::string suffix = n.substr(stripped.size() + 1);
			if (!suffix.empty() && std::all_of(suffix.begin(), suffix.end(), ::isdigit))
			{
				int val = std::stoi(suffix);
				if (val > maxN) maxN = val;
			}
		}
	}

	return stripped + "#" + std::to_string(maxN + 1);
}

bool Level::QueryGround(const Vec3& pos, float& dist, Vec3& normal) const
{
	constexpr float GROUND_THRESHOLD = 8.0f;
	float best_dist = GROUND_THRESHOLD;
	Vec3 best_normal(0.0f, 0.0f, 0.0f);
	Vec3 up(0.0f, 1.0f, 0.0f);
	bool found = false;
	for (const Quadblock& quad : m_quadblocks)
	{
		if (!(quad.GetFlags() & QuadFlags::GROUND))
			continue;

		if (quad.IntersectRay(pos, up, dist, normal))
		{
			if (std::abs(dist) < std::abs(best_dist))
			{
				best_dist = dist;
				best_normal = normal;
				found = true;
			}
		}
	}
	normal = best_normal;
	dist = best_dist;
	return found;
}

bool Level::GenerateInstanceRow(int checkpointIndex, size_t instanceIndex, int numInstances, float spacing, bool deleteAfter)
{
	if (m_checkpoints.empty())
		return false;
	if (instanceIndex >= m_instances.size())
		return false;
	if (checkpointIndex < 0 || checkpointIndex >= static_cast<int>(m_checkpoints.size()))
		return false;
	if (numInstances < 1)
		return false;

	const Checkpoint& cp = m_checkpoints[checkpointIndex];
	Vec3 center = cp.GetPos();

	int down = cp.GetDown();
	int up = cp.GetUp();
	Vec3 forward;
	if (down != NONE_CHECKPOINT_INDEX && down >= 0 && down < static_cast<int>(m_checkpoints.size()))
		forward = m_checkpoints[down].GetPos() - center;
	else if (up != NONE_CHECKPOINT_INDEX && up >= 0 && up < static_cast<int>(m_checkpoints.size()))
		forward = center - m_checkpoints[up].GetPos();
	else
		forward = Vec3(0.0f, 0.0f, 1.0f);

	float yaw = -std::atan2(forward.z, forward.x) * (180.0f / MATH_PI);
	yaw = std::fmod(yaw, 360.0f);
	forward.Normalize();
	Vec3 right = forward.Cross(Vec3(0.0f, 1.0f, 0.0f));
	right.Normalize();

	Vec3 centerRot(0.0f, yaw, 0.0f);

	std::vector<size_t> quadindexes;
	for (size_t j = 0; j < m_quadblocks.size(); j++)
	{
		if (m_quadblocks[j].GetFlags() & QuadFlags::GROUND)
			quadindexes.push_back(j);
	}
	SnapToClosestQuad(m_quadblocks, quadindexes, center, centerRot, Vec3(0.0f, 1.0f, 0.0f), -10.0f, 10.0f);
	Instance original = m_instances[instanceIndex];
	size_t insertPos = instanceIndex + 1;
	for (int col = 0; col < numInstances; col++)
	{
		float lateralOffset = (col - (numInstances - 1) * 0.5f) * spacing;
		Vec3 pos = center + right * lateralOffset;
		Vec3 rot(0.0f, yaw, 0.0f);

		Instance newInstance = original;
		newInstance.SetName(GenerateUniqueInstanceName(original.GetName()));

		SnapToClosestQuad(m_quadblocks, quadindexes, pos, rot, Vec3(0.0f, 1.0f, 0.0f), -10.0f, 10.0f);

		newInstance.SetPos(pos);
		newInstance.SetRot(rot);
		
		m_instances.insert(m_instances.begin() + insertPos + col, newInstance);
	}

	if (deleteAfter)
	{
		m_instances.erase(m_instances.begin() + instanceIndex);
	}

	return true;
}

bool Level::GenerateBSP()
{
	std::vector<size_t> quadIndexes;
	for (size_t i = 0; i < m_quadblocks.size(); i++) { quadIndexes.push_back(i); }
	m_bsp.Clear();
	m_bspVis.Clear();
	ResetAllBSPID();
	m_bsp.SetId(0);
	m_bsp.SetQuadblockIndexes(quadIndexes, m_quadblocks);
	m_bsp.ComputeBoundingBox(m_quadblocks);
	m_bsp.Generate(m_quadblocks, m_bspSettings);
	if (m_bsp.IsValid())
	{
		GenerateRenderBspData();
		return true;
	}
	m_bsp.Clear();
	return false;
}

bool Level::ReOrderBSP()
{
	ResetAllBSPID();
	std::vector<BSP*> bspNodes = m_bsp.GetTree();
	std::sort(bspNodes.begin(), bspNodes.end(), 
		[](const BSP* a, const BSP* b) 
		{
			if (a->GetId() == b->GetId())
				printf("ERROR : 2 BSP NODES SHARE THE SAME ID : %zu\n", b->GetId());
			return a->GetId() < b->GetId(); 
		});
	std::unordered_map<size_t, size_t> bspIDOverride; // Map old ID -> New ID
	for (const BSP* bsp : bspNodes)
	{
		size_t oldID = bsp->GetId();
		size_t newID = bspIDOverride.size();
		if (oldID != newID)
		{
			printf("INFO : BSP ID WAS CHANGED %zu -> %zu\n", oldID, newID);
		}
		bspIDOverride[oldID] = newID;
	}
	for (BSP* bsp : bspNodes)
	{
		bsp->SetId(bspIDOverride[bsp->GetId()]);
	}
	for (Quadblock& quad : m_quadblocks)
	{
		quad.SetBSPID(bspIDOverride[quad.GetBSPID()]);
	}
	return true;
}

bool Level::EmplaceInstanceBSP() //Update BSP BBox and InstancesIndexes. One Leaf for each Instance with collision
{
	std::vector<BSP*> nodes = m_bsp.GetTree();

	for (size_t i = 0; i < m_instances.size(); i++)
	{
		const Instance& inst = m_instances[i];
		if (!m_instanceModels[inst.GetModelKey()].IsValid())
			continue;
		const InstanceHitbox& settings = inst.GetHitbox();
		if (!settings.enabled) 
			continue; 

		float bestDist = std::numeric_limits<float>::max();
		BSP* closestLeaf = nullptr;
		const Vec3 instCenter = inst.Center();
		for (BSP* node : nodes)
		{
			if (node->IsBranch())
				continue;
			float dist = node->GetBoundingBox().Distance(instCenter);
			if (dist < bestDist)
			{
				bestDist = dist;
				closestLeaf = node;
			}
		}
		if (closestLeaf != nullptr)
		{
			closestLeaf->UpdateBoundingBox(inst.ComputeBBox());
			closestLeaf->GetInstanceIndexes().push_back(i);
		}
	}
	return true;
}

bool Level::GenerateVisTreeOnly(bool simpleVisTree, float distanceNearClip, float distanceFarClip)
{
	if (m_bsp.IsValid())
	{
		VisTreeSettings settings;
		settings.farClipDistance = distanceFarClip;
		settings.nearClipDistance = distanceNearClip;
		settings.centerOnlySamples = simpleVisTree; 
		settings.commutativeRays = false;
		m_bspVis = GenerateVisTree(m_quadblocks, &m_bsp, settings);
		return true;
	}
	return false;
}


bool Level::GenerateVisTreeOnly()
{
	if (m_bsp.IsValid())
	{
		m_bspVis = GenerateVisTree(m_quadblocks, &m_bsp, m_visTreeSettings);
		return true;
	}
	return false;
}


void Level::GenerateBotPathChangeCode()
{
	// For each node on a path, find the closest node on the target path
	// and set the PathChange and PathChangeIndex accordingly.
	auto findClosestNode = [](const std::vector<BotNode>& targetNodes, const Vec3& pos) -> int
		{
			int   bestIndex = 0;
			float bestDist = FLT_MAX;
			for (int i = 0; i < static_cast<int>(targetNodes.size()); i++)
			{
				const Vec3& targetPos = targetNodes[i].GetPos();
				const float dx = pos.x - targetPos.x;
				const float dy = pos.y - targetPos.y;
				const float dz = pos.z - targetPos.z;
				const float dist = dx * dx + dy * dy + dz * dz; // squared, no need for sqrt
				if (dist < bestDist)
				{
					bestDist = dist;
					bestIndex = i;
				}
			}
			return bestIndex;
		};

	// Validate that all 3 paths are valid before proceeding
	for (int i = 0; i < 3; i++)
	{
		if (!m_botPaths[i].IsValid())
		{
			// Can't generate path change codes without all 3 paths
			return;
		}
	}

	const std::vector<BotNode>& leftNodes = m_botPaths[0].GetNodes();
	const std::vector<BotNode>& middleNodes = m_botPaths[1].GetNodes();
	const std::vector<BotNode>& rightNodes = m_botPaths[2].GetNodes();

	// --- Path 0 (Left): can only switch to Middle (1) ---
	for (int i = 0; i < static_cast<int>(leftNodes.size()); i++)
	{
		BotNode& node = m_botPaths[0].GetNode(i);
		const int closestMid = findClosestNode(middleNodes, node.GetPos());
		node.SetPathChange(1);
		node.SetPathChangeIndex(closestMid);
	}

	// --- Path 2 (Right): can only switch to Middle (1) ---
	for (int i = 0; i < static_cast<int>(rightNodes.size()); i++)
	{
		BotNode& node = m_botPaths[2].GetNode(i);
		const int closestMid = findClosestNode(middleNodes, node.GetPos());
		node.SetPathChange(1);
		node.SetPathChangeIndex(closestMid);
	}

	// --- Path 1 (Middle): can switch to Left (0) or Right (2) ---
	// Alternate between left and right to distribute switches evenly,
	// so the AI doesn't always prefer one side.
	for (int i = 0; i < static_cast<int>(middleNodes.size()); i++)
	{
		BotNode& node = m_botPaths[1].GetNode(i);
		if (i % 2 == 0)
		{
			// Switch to Left
			const int closestLeft = findClosestNode(leftNodes, node.GetPos());
			node.SetPathChange(0);
			node.SetPathChangeIndex(closestLeft);
		}
		else
		{
			// Switch to Right
			const int closestRight = findClosestNode(rightNodes, node.GetPos());
			node.SetPathChange(2);
			node.SetPathChangeIndex(closestRight);
		}
	}
}


void Level::GenerateBotPathLeft()
{
	std::vector<Vec3> pos;
	for (BotNode& node : m_botPaths[0].GetNodes())
	{
		pos.push_back(node.GetPos());
	}
	m_botPaths[0].GeneratePath(pos, m_quadblocks);
}

bool Level::GenerateCheckpoints()
{
	if (m_checkpointPaths.empty()) { return false; }

	for (const Path& path : m_checkpointPaths) { if (!path.IsReady()) { return false; } }

	ResetFilter();
	for (size_t i = 0; i < m_quadblocks.size(); i++)
	{
		m_quadblocks[i].SetCheckpoint(-1);
	}
	size_t checkpointIndex = 0;
	std::vector<size_t> linkNodeIndexes;
	std::vector<std::vector<Checkpoint>> pathCheckpoints;
	bool overlap = false;
	for (Path& path : m_checkpointPaths)
	{
		pathCheckpoints.push_back(path.GeneratePath(checkpointIndex, m_quadblocks, overlap));
		checkpointIndex += pathCheckpoints.back().size();
		linkNodeIndexes.push_back(path.GetStart());
		linkNodeIndexes.push_back(path.GetEnd());
	}
	m_checkpoints.clear();
	for (const std::vector<Checkpoint>& checkpoints : pathCheckpoints)
	{
		for (const Checkpoint& checkpoint : checkpoints)
		{
			m_checkpoints.push_back(checkpoint);
		}
	}

	int lastPathIndex = static_cast<int>(m_checkpointPaths.size()) - 1;
	const Checkpoint* currStartCheckpoint = &m_checkpoints[0];
	float distFinish = 0.0f;
	for (int i = lastPathIndex; i >= 0; i--)
	{
		m_checkpointPaths[i].UpdateDist(distFinish, currStartCheckpoint->GetPos(), m_checkpoints);
		currStartCheckpoint = &m_checkpoints[m_checkpointPaths[i].GetStart()];
		distFinish = currStartCheckpoint->GetDistFinish();
	}

	for (size_t i = 0; i < linkNodeIndexes.size(); i++)
	{
		Checkpoint& node = m_checkpoints[linkNodeIndexes[i]];
		if (i % 2 == 0)
		{
			size_t linkDown = (i == 0) ? linkNodeIndexes.size() - 1 : i - 1;
			node.UpdateDown(static_cast<int>(linkNodeIndexes[linkDown]));
		}
		else
		{
			size_t linkUp = (i + 1) % linkNodeIndexes.size();
			node.UpdateUp(static_cast<int>(linkNodeIndexes[linkUp]));
		}
	}

	for (Path& path : m_checkpointPaths)
	{
		const Checkpoint& middleStart = m_checkpoints[path.GetStart()];
		const Checkpoint& middleEnd = m_checkpoints[path.GetEnd()];

		Path* sides[2] = { path.GetLeft(), path.GetRight() };
		for (Path* side : sides)
		{
			if (!side) { continue; }
			m_checkpoints[side->GetStart()].UpdateDown(middleStart.GetDown());
			m_checkpoints[side->GetEnd()].UpdateUp(middleEnd.GetUp());
		}
	}

	// Cap the number of checkpoints to 255
	const size_t MAX_CHECKPOINTS = 255;
	if (m_checkpoints.size() > MAX_CHECKPOINTS)
	{
		std::unordered_set<size_t> protectedIndices(linkNodeIndexes.begin(), linkNodeIndexes.end());
		for (size_t i = 0; i < m_checkpoints.size(); ++i)
		{
			const Checkpoint& cp = m_checkpoints[i];
			if (cp.GetRight() != NONE_CHECKPOINT_INDEX || cp.GetLeft() != NONE_CHECKPOINT_INDEX)
			{
				protectedIndices.insert(i);
			}
		}

		// Build dist->index map for candidates
		std::multimap<float, size_t> distToNextMap;
		std::unordered_map<size_t, float> currentDistances;

		for (size_t i = 0; i < m_checkpoints.size(); ++i)
		{
			if (protectedIndices.find(i) != protectedIndices.end()) continue;
			const Checkpoint& cp = m_checkpoints[i];
			int downIndex = cp.GetDown();
			if (downIndex != NONE_CHECKPOINT_INDEX)
			{
				float distToNext = (m_checkpoints[downIndex].GetPos() - cp.GetPos()).Length();
				currentDistances[i] = distToNext;
				distToNextMap.insert({distToNext, i});
			}
		}

		size_t total = m_checkpoints.size();
		size_t numToRemove = total - MAX_CHECKPOINTS;
		size_t numRemovable = distToNextMap.size();
		if (numToRemove > numRemovable)
		{
			numToRemove = numRemovable;
		}

		// Heuristic: Pick smallest-dist-to-next checkpoint to remove
		std::unordered_set<size_t> indexesToRemove;
		auto it = distToNextMap.begin();
		for (size_t i = 0; i < numToRemove && it != distToNextMap.end(); )
		{
			size_t candidateIndex = it->second;
			indexesToRemove.insert(candidateIndex);

			// Update distance for previous checkpoint
			const Checkpoint& removedCP = m_checkpoints[candidateIndex];
			int upIndex = removedCP.GetUp();
			int downIndex = removedCP.GetDown();

			if (upIndex != NONE_CHECKPOINT_INDEX &&
				downIndex != NONE_CHECKPOINT_INDEX &&
				protectedIndices.find(upIndex) == protectedIndices.end())
			{
				// Update the distance for the checkpoint before this one
				float oldDist = currentDistances[upIndex];
				float removedDist = currentDistances[candidateIndex];
				float newDist = oldDist + removedDist;

				auto range = distToNextMap.equal_range(oldDist);
				for (auto mapIt = range.first; mapIt != range.second; ++mapIt)
				{
					if (mapIt->second == upIndex)
					{
						distToNextMap.erase(mapIt);
						break;
					}
				}

				currentDistances[upIndex] = newDist;
				distToNextMap.insert({newDist, upIndex});
			}

			++it;
			++i;
		}

		// Build mapping oldIndex -> newIndex
		std::vector<int> oldToNew(total, -1);
		std::vector<Checkpoint> newCheckpoints;
		newCheckpoints.reserve(MAX_CHECKPOINTS);

		for (size_t old = 0; old < total; ++old)
		{
			if (indexesToRemove.find(old) == indexesToRemove.end())
			{
				int newIdx = static_cast<int>(newCheckpoints.size());
				oldToNew[old] = newIdx;
				// copy original checkpoint
				newCheckpoints.push_back(m_checkpoints[old]);
			}
		}

		// Update links
		const int N = static_cast<int>(newCheckpoints.size());

		for (int i = 0; i < N; ++i)
		{
			newCheckpoints[i].SetIndex(i);

			int newUp = (i + 1) % N;
			int newDown = (i == 0) ? (N - 1) : (i - 1);
			newCheckpoints[i].UpdateUp(newUp);
			newCheckpoints[i].UpdateDown(newDown);
		}

		// Update quadblock checkpoint references
		for (Quadblock& qb : m_quadblocks)
		{
			int oldCheckpoint = qb.GetCheckpoint();
			if (oldCheckpoint >= 0 && oldCheckpoint < static_cast<int>(oldToNew.size()))
			{
				int newCheckpoint = oldToNew[oldCheckpoint];
				if (newCheckpoint == -1)
				{
					// This checkpoint was removed, find nearest valid checkpoint
					float minDist = std::numeric_limits<float>::max();
					int nearestCheckpoint = 0;
					Vec3 qbCenter = qb.GetBoundingBox().Midpoint();

					for (int i = 0; i < N; ++i)
					{
						float dist = (newCheckpoints[i].GetPos() - qbCenter).Length();
						if (dist < minDist)
						{
							minDist = dist;
							nearestCheckpoint = i;
						}
					}
					qb.SetCheckpoint(nearestCheckpoint);
				}
				else
				{
					qb.SetCheckpoint(newCheckpoint);
				}
			}
		}

		m_checkpoints = std::move(newCheckpoints);
	}

	UpdateRenderCheckpointData();
	return !overlap;
}


bool Level::GenerateOceanVertices()
{
	WaterAnimSettings& p = m_waterAnimSettings;
	
	const int brightCyclesTime = p.brightWaveCycle;
	const float baseBright = p.baseBrightness;
	const float waveLength = std::max(p.waveLength, 1.0f);
	const float waveK = 2.0f * MATH_PI / waveLength;

	for (Quadblock& quad : m_quadblocks)
	{
		if (!quad.GetWater())
			continue;
		const std::vector<Vertex>& vertices = quad.GetVertices();
		for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
		{
			Vec3 vPos = vertices[i].m_pos;
			const float baseU = vPos.x * p.sizeTex;
			const float baseV = vPos.z * p.sizeTex;
			const float spaceWave = (std::cos(waveK * vPos.x) + std::cos(waveK * vPos.z)) / 2;

			PSX::OceanVertex ov{};
			for (int f = 0; f < NUM_FRAME_OVERT; f++)
			{
				const float frac = static_cast<float>(f) / NUM_FRAME_OVERT;

				const float scrollU = p.ScrollULoops * 64.0f * frac;
				const float scrollV = p.ScrollVLoops * 64.0f * frac;
				const float waveU = p.waveAmplitude  * spaceWave * std::sin(2.0f * MATH_PI * p.waveCyclesTimeU * frac);
				const float waveV = p.waveAmplitude  * spaceWave * std::sin(2.0f * MATH_PI * p.waveCyclesTimeV * frac);
				const int u = static_cast<int>(std::round(baseU + scrollU + waveU));
				const int v = static_cast<int>(std::round(baseV + scrollV + waveV));

				const float brightTemporalPhase = 2.0f * MATH_PI * brightCyclesTime * frac;
				const float waveBright = p.brightAmp * std::sin(brightTemporalPhase) * spaceWave;
				const int b = static_cast<int>(std::round(baseBright + waveBright));

				PSX::OceanVertexFrame frame{};
				frame.u = static_cast<uint16_t>(((u % 64) + 64) % 64);
				frame.v = static_cast<uint16_t>(((v % 64) + 64) % 64);
				frame.brightness = static_cast<uint16_t>(Clamp(b, 0, 15));
				ov.frames[f] = frame;
			}
			quad.SetOceanVertex(ov, i);
		}
	}
	return true;
}


namespace
{
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


bool Level::GenerateMinimap()
{
	int targetHeight = m_minimapSettings.textureHeight;
	if (targetHeight < 3)
	{
		printf("WARNING: MinimapConfig targetHeight (%d) too small once padding is reserved, using 3 instead\n", targetHeight);
		targetHeight = 3;
	}
	const int contentHeight = targetHeight - 1;

	// Build quad list to use for the minimap 
	std::vector<size_t> usedQuadIds;
	for (size_t i = 0; i < m_quadblocks.size(); i++)
	{
		if (m_minimapSettings.checkpointQuads && m_quadblocks[i].GetCheckpoint() != -1)
			usedQuadIds.push_back(i);
		else if (m_minimapSettings.checkpointPathableQuads && m_quadblocks[i].GetCheckpointPathable() && m_quadblocks[i].GetCheckpointStatus())
			usedQuadIds.push_back(i);
		else if (m_minimapSettings.materials.contains(m_quadblocks[i].GetMaterial()))
			usedQuadIds.push_back(i);
	}
	if (usedQuadIds.empty()) return false;

	// Build Triangle list
	std::vector<Tri> tris;
	for (size_t i : usedQuadIds)
	{
		for (const std::array<size_t, 3>&face : m_quadblocks[i].GetTriFacesIndexes())
		{
			const std::array<Vec3, 3> f = m_quadblocks[i].GetTriFace(face[0], face[1], face[2]);
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
	m_minimapConfig.worldStartX = worldBox.min.x;
	m_minimapConfig.worldEndX = worldBox.max.x;
	m_minimapConfig.worldStartZ = worldBox.min.z;
	m_minimapConfig.worldEndZ = worldBox.max.z;

	const float spanX = m_minimapConfig.worldEndX - m_minimapConfig.worldStartX;
	const float spanZ = m_minimapConfig.worldEndZ - m_minimapConfig.worldStartZ;

	// World -> pixel mapping
	if (m_minimapSettings.orientation == MinimapOrientation::AUTO)
		if (spanX > spanZ)
			m_minimapConfig.orientationMode = MinimapOrientation::DOWN;
		else
			m_minimapConfig.orientationMode = MinimapOrientation::RIGHT;
	else
		m_minimapConfig.orientationMode = m_minimapSettings.orientation;

	const bool swapped = (m_minimapConfig.orientationMode == MinimapOrientation::DOWN || m_minimapConfig.orientationMode == MinimapOrientation::UP);
	const float colSpanWorld = swapped ? spanZ : spanX;
	const float rowSpanWorld = swapped ? spanX : spanZ;
	constexpr float minimapStretchX = 1.6f;
	const int contentWidth = std::max(1, static_cast<int>(std::lround(contentHeight * (colSpanWorld * minimapStretchX) / rowSpanWorld)));
	const int targetWidth = contentWidth + 1; // One extra column reserved the same way as the padding row (see below).

	auto toPixelSpace = [&](const Vec3& worldPos) // Convert World Pos to Pixel coordinate on the image
		{
			float x = worldPos.x, z = worldPos.z;
			float colFrac = 0.0, rowFrac = 0.0;
			switch (m_minimapConfig.orientationMode)
			{
			case MinimapOrientation::RIGHT: colFrac = (x - m_minimapConfig.worldStartX) / spanX; rowFrac = (z - m_minimapConfig.worldStartZ) / spanZ; break;
			case MinimapOrientation::DOWN:  colFrac = (m_minimapConfig.worldEndZ - z) / spanZ;   rowFrac = (x - m_minimapConfig.worldStartX) / spanX; break;
			case MinimapOrientation::LEFT:  colFrac = (m_minimapConfig.worldEndX - x) / spanX;   rowFrac = (m_minimapConfig.worldEndZ - z) / spanZ;   break;
			case MinimapOrientation::UP:    colFrac = (z - m_minimapConfig.worldStartZ) / spanZ; rowFrac = (m_minimapConfig.worldEndX - x) / spanX;   break;
			}
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
	switch (m_minimapConfig.orientationMode)
	{
	case MinimapOrientation::RIGHT: m_minimapConfig.worldEndX += extCol; m_minimapConfig.worldEndZ += extRow; break;
	case MinimapOrientation::DOWN:  m_minimapConfig.worldStartZ -= extCol; m_minimapConfig.worldEndX += extRow; break;
	case MinimapOrientation::LEFT:  m_minimapConfig.worldStartX -= extCol; m_minimapConfig.worldStartZ -= extRow; break;
	case MinimapOrientation::UP:    m_minimapConfig.worldEndZ += extCol; m_minimapConfig.worldStartX -= extRow; break;
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

	const std::filesystem::path pngPath = GetParentPath() / ("auto-minimap.png");
	if (!stbi_write_png(pngPath.string().c_str(), targetWidth, targetHeight, 4, rgba.data(), targetWidth * 4))
	{
		printf("ERROR: Failed to write minimap PNG\n");
		return false;
	}

	m_minimapConfig.texture = Texture(pngPath);
	if (m_minimapConfig.texture.IsEmpty())
	{
		printf("ERROR: Failed to load generated minimap texture %s\n", pngPath.string().c_str());
		return false;
	}
	m_minimapConfig.texture.SetBlendMode(static_cast<uint16_t>(PSX::BlendMode::ADDITIVE));
	return true;
}

enum class PresetHeader : unsigned
{
	SPAWN, LEVEL, PATH, MATERIAL, TURBO_PAD, ANIM_TEXTURES, SCRIPT, MINIMAP
};

bool Level::LoadPreset(const std::filesystem::path& filename)
{
	m_showLogWindow = true;
	nlohmann::json json = nlohmann::json::parse(std::ifstream(filename));
	if (!json.contains("header"))
	{
		m_logMessage += "\nFailed loaded preset: " + filename.string();
		return false;
	}

	const PresetHeader header = json["header"];
	if (header == PresetHeader::SPAWN)
	{
		if (json.contains("spawn")) { m_spawn = json["spawn"]; }
	}
	else if (header == PresetHeader::LEVEL)
	{
		if (json.contains("configFlags")) { m_configFlags = json["configFlags"]; }
		if (json.contains("jumpYSpeedCap")) { m_jumpYSpeedCap = json["jumpYSpeedCap"]; }
		if (json.contains("skyGradient")) { m_skyGradient = json["skyGradient"]; }
		if (json.contains("clearColor")) { m_clearColor = json["clearColor"]; }
		if (json.contains("stars")) { json["stars"].get_to(m_stars); }
		if (json.contains("splitLines"))
		{
			m_splitLines[0] = json["splitLines"][0];
			m_splitLines[1] = json["splitLines"][1];
		}
		if (json.contains("weather")) { json["weather"].get_to(m_weather); }
		if (json.contains("skyboxObjPath"))
		{
			std::string skyboxPath = json["skyboxObjPath"];
			if (!skyboxPath.empty())
			{
				if (m_skybox.LoadOBJ(skyboxPath))
				{
					GenerateRenderSkyboxData();
				}
			}
		}
	}
	else if (header == PresetHeader::PATH)
	{
		if (json.contains("pathCount"))
		{
			const size_t pathCount = json["pathCount"];
			m_checkpointPaths.resize(pathCount);
			for (size_t i = 0; i < pathCount; i++)
			{
				if (!json.contains("path" + std::to_string(i))) { continue; }

				nlohmann::json& pathJson = json["path" + std::to_string(i)];
				if (!pathJson.contains("index")) { continue; }

				size_t index = pathJson["index"];
				Path& path = m_checkpointPaths[index];
				path.FromJson(pathJson, m_quadblocks);
			}
			GenerateCheckpoints();
		}
	}
	else if (header == PresetHeader::MATERIAL)
	{
		if (json.contains("materials"))
		{
			std::vector<std::string> materials = json["materials"];
			for (const std::string& material : materials)
			{
				if (m_materialToQuadblocks.contains(material))
				{
					if (json.contains(material + "_terrain"))
					{
						m_propTerrain.SetPreview(material, json[material + "_terrain"]);
						m_propTerrain.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_quadflags"))
					{
						m_propQuadFlags.SetPreview(material, json[material + "_quadflags"]);
						m_propQuadFlags.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_drawflags"))
					{
						m_propDoubleSided.SetPreview(material, json[material + "_drawflags"]);
						m_propDoubleSided.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_checkpoint"))
					{
						m_propCheckpoints.SetPreview(material, json[material + "_checkpoint"]);
						m_propCheckpoints.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_trigger"))
					{
						QuadblockTrigger trigger = json[material + "_trigger"];
						m_propTurboPads.GetBackup(material) = trigger;
						m_propTurboPads.GetPreview(material) = trigger;
					}
					if (json.contains(material + "_speedImpact"))
					{
						m_propSpeedImpact.SetPreview(material, json[material + "_speedImpact"]);
						m_propSpeedImpact.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_weatherIntensity"))
					{
						m_propWeatherIntensity.SetPreview(material, json[material + "_weatherIntensity"]);
						m_propWeatherIntensity.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_weatherVanishRate"))
					{
						m_propWeatherVanishRate.SetPreview(material, json[material + "_weatherVanishRate"]);
						m_propWeatherVanishRate.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_checkpointPathable"))
					{
						m_propCheckpointPathable.SetPreview(material, json[material + "_checkpointPathable"]);
						m_propCheckpointPathable.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_visTreeTransparent"))
					{
						m_propVisTreeTransparent.SetPreview(material, json[material + "_visTreeTransparent"]);
						m_propVisTreeTransparent.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_drawOrderHigh"))
					{
						m_propDrawOrderHigh.SetPreview(material, json[material + "_drawOrderHigh"]);
						m_propDrawOrderHigh.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_water"))
					{
						m_propWater.SetPreview(material, json[material + "_water"]);
						m_propWater.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
				}
			}
		}
	}
	else if (header == PresetHeader::ANIM_TEXTURES)
	{
		if (json.contains("animCount"))
		{
			const size_t animCount = json["animCount"];
			for (size_t i = 0; i < animCount; i++)
			{
				if (!json.contains("anim" + std::to_string(i))) { continue; }
				AnimTexture animTexture;
				animTexture.FromJson(json["anim" + std::to_string(i)], m_quadblocks, m_parentPath);
				if (animTexture.IsPopulated()) { m_animTextures.push_back(animTexture); }
			}
		}
	}
	else if (header == PresetHeader::TURBO_PAD)
	{
		if (json.contains("turbopads"))
		{
			std::unordered_set<std::string> turboPads = json["turbopads"];
			for (Quadblock& quadblock : m_quadblocks)
			{
				const std::string& quadName = quadblock.GetName();
				if (turboPads.contains(quadName))
				{
					if (!json.contains(quadName + "_trigger")) { continue; }
					quadblock.SetTrigger(json[quadName + "_trigger"]);
					ManageTurbopad(quadblock);
					if (m_bsp.IsValid())
					{
						m_bsp.Clear();
						GenerateRenderBspData();
					}
				}
			}
		}
	}
	else if (header == PresetHeader::SCRIPT)
	{
		m_pythonScript = json["script"];
	}
	else if (header == PresetHeader::MINIMAP)
	{
		if (json.contains("minimap"))
		{
			m_minimapConfig = json["minimap"];
		}
	}
	else
	{
		m_logMessage += "\nFailed loaded preset: " + filename.string();
		return false;
	}
	m_logMessage += "\nSuccessfully loaded preset: " + filename.string();
	return true;
}

bool Level::SavePreset(const std::filesystem::path& path)
{
	std::filesystem::path dirPath = path / (m_name + "_presets");
	if (!std::filesystem::exists(dirPath)) { std::filesystem::create_directory(dirPath); }

	auto SaveJSON = [](const std::filesystem::path& path, const nlohmann::json& json)
		{
			std::ofstream pathFile(path);
			pathFile << std::setw(4) << json << std::endl;
		};

	nlohmann::json spawnJson = {};
	spawnJson["header"] = PresetHeader::SPAWN;
	spawnJson["spawn"] = m_spawn;
	SaveJSON(dirPath / "spawn.json", spawnJson);

	nlohmann::json levelJson = {};
	levelJson["header"] = PresetHeader::LEVEL;
	levelJson["configFlags"] = m_configFlags;
	levelJson["skyGradient"] = m_skyGradient;
	levelJson["clearColor"] = m_clearColor;
	levelJson["stars"] = m_stars;
	levelJson["jumpYSpeedCap"] = m_jumpYSpeedCap;
	levelJson["splitLines"] = { m_splitLines[0], m_splitLines[1] };
	levelJson["weather"] = m_weather;
	if (!m_skybox.m_objPath.empty()) { levelJson["skyboxObjPath"] = m_skybox.m_objPath.string(); }
	SaveJSON(dirPath / "level.json", levelJson);

	nlohmann::json pathJson = {};
	pathJson["header"] = PresetHeader::PATH;
	pathJson["pathCount"] = m_checkpointPaths.size();
	for (size_t i = 0; i < m_checkpointPaths.size(); i++)
	{
		pathJson["path" + std::to_string(i)] = nlohmann::json();
		m_checkpointPaths[i].ToJson(pathJson["path" + std::to_string(i)], m_quadblocks);
	}
	SaveJSON(dirPath / "path.json", pathJson);

	if (!m_materialToQuadblocks.empty())
	{
		nlohmann::json materialJson = {};
		materialJson["header"] = PresetHeader::MATERIAL;
		std::vector<std::string> materials; materials.reserve(m_materialToQuadblocks.size());
		for (const auto& [key, value] : m_materialToQuadblocks)
		{
			materials.push_back(key);
			materialJson[key + "_terrain"] = m_propTerrain.GetBackup(key);
			materialJson[key + "_quadflags"] = m_propQuadFlags.GetBackup(key);
			materialJson[key + "_drawflags"] = m_propDoubleSided.GetBackup(key);
			materialJson[key + "_checkpoint"] = m_propCheckpoints.GetBackup(key);
			materialJson[key + "_checkpointPathable"] = m_propCheckpointPathable.GetBackup(key);
			materialJson[key + "_visTreeTransparent"] = m_propVisTreeTransparent.GetBackup(key);
			materialJson[key + "_trigger"] = m_propTurboPads.GetBackup(key);
			materialJson[key + "_speedImpact"] = m_propSpeedImpact.GetBackup(key);
			materialJson[key + "_drawOrderHigh"] = m_propDrawOrderHigh.GetBackup(key);
			materialJson[key + "_water"] = m_propWater.GetBackup(key);
			materialJson[key + "_weatherIntensity"] = m_propWeatherIntensity.GetBackup(key);
			materialJson[key + "_weatherVanishRate"] = m_propWeatherVanishRate.GetBackup(key);
		}
		materialJson["materials"] = materials;
		SaveJSON(dirPath / "material.json", materialJson);
	}

	if (!m_animTextures.empty())
	{
		nlohmann::json animJson = {};
		animJson["header"] = PresetHeader::ANIM_TEXTURES;
		animJson["animCount"] = m_animTextures.size();
		size_t i = 0;
		for (const AnimTexture& animTexture : m_animTextures)
		{
			animTexture.ToJson(animJson["anim" + std::to_string(i++)], m_quadblocks);
		}
		SaveJSON(dirPath / "animtex.json", animJson);
	}

	std::unordered_set<std::string> turboPads;
	nlohmann::json turboPadJson = {};
	for (const Quadblock& quadblock : m_quadblocks)
	{
		if (quadblock.GetTurboPadIndex() == TURBO_PAD_INDEX_NONE) { continue; }
		const std::string& quadName = quadblock.GetName();
		turboPads.insert(quadName);
		turboPadJson[quadName + "_trigger"] = quadblock.GetTrigger();
	}
	if (!turboPads.empty())
	{
		turboPadJson["header"] = PresetHeader::TURBO_PAD;
		turboPadJson["turbopads"] = turboPads;
		SaveJSON(dirPath / "turbopad.json", turboPadJson);
	}

	if (m_saveScript)
	{
		nlohmann::json scriptJson = {};
		scriptJson["header"] = PresetHeader::SCRIPT;
		scriptJson["script"] = m_pythonScript;
		SaveJSON(dirPath / "script.json", scriptJson);
	}

	nlohmann::json minimapJson = {};
	minimapJson["header"] = PresetHeader::MINIMAP;
	minimapJson["minimap"] = m_minimapConfig;
	SaveJSON(dirPath / "minimap.json", minimapJson);
	
	return true;
}

void Level::ResetFilter()
{
	for (Quadblock& qb : m_quadblocks)
	{
		qb.SetFilter(false);
		qb.SetFilterColor(GuiRenderSettings::defaultFilterColor);
	}
}

void Level::ResetRendererSelection()
{
	m_rendererQueryPoint = Vec3();
	m_rendererSelectedQuadblockIndexes.clear();
	m_models[LevelModels::SELECTED]->GetMesh().Clear();
}

void Level::ManageTurbopad(Quadblock& quadblock)
{
	bool stp = true;
	size_t turboPadIndex = TURBO_PAD_INDEX_NONE;
	switch (quadblock.GetTrigger())
	{
	case QuadblockTrigger::TURBO_PAD:
		stp = false;
	case QuadblockTrigger::SUPER_TURBO_PAD:
	{
		Quadblock turboPad = quadblock;
		const Vec3 up(0.0f, 1.0f, 0.0f);
		turboPad.Translate(TURBO_PAD_QUADBLOCK_TRANSLATION, up);
		turboPad.SetCheckpoint(-1);
		turboPad.SetCheckpointStatus(false);
		turboPad.SetVisTreeTransparent(false);
		turboPad.SetName(quadblock.GetName() + (stp ? "_stp" : "_tp"));
		turboPad.SetFlag(QuadFlags::TRIGGER_SCRIPT | QuadFlags::INVISIBLE_TRIGGER | QuadFlags::WALL);
		turboPad.SetTerrain(stp ? TerrainType::SUPER_TURBO_PAD : TerrainType::TURBO_PAD);
		turboPad.SetTurboPadIndex(TURBO_PAD_INDEX_NONE);
		turboPad.SetHide(true);
		turboPad.SetAnimated(false);
		turboPad.SetDrawOrderHigh(0);

		size_t index = m_quadblocks.size();
		turboPadIndex = quadblock.GetTurboPadIndex();
		quadblock.SetTurboPadIndex(index);
		m_quadblocks.push_back(turboPad);
		if (turboPadIndex == TURBO_PAD_INDEX_NONE) { break; }
	}
	case QuadblockTrigger::NONE:
	{
		bool clearTurboPadIndex = false;
		if (turboPadIndex == TURBO_PAD_INDEX_NONE)
		{
			clearTurboPadIndex = true;
			turboPadIndex = quadblock.GetTurboPadIndex();
		}
		if (turboPadIndex == TURBO_PAD_INDEX_NONE) { break; }

		for (Quadblock& quad : m_quadblocks)
		{
			size_t index = quad.GetTurboPadIndex();
			if (index > turboPadIndex) { quad.SetTurboPadIndex(index - 1); }
		}

		if (clearTurboPadIndex) { quadblock.SetTurboPadIndex(TURBO_PAD_INDEX_NONE); }
		m_quadblocks.erase(m_quadblocks.begin() + turboPadIndex);
		break;
	}
	}
}


bool Level::LoadLEV(const std::filesystem::path& levFile)
{
	std::ifstream file(levFile, std::ios::binary);
	if (!file.is_open()) return false;

	// Read VRAM for model texture extraction
	{
		std::filesystem::path vrmPath = levFile;
		vrmPath.replace_extension(".vrm");
		m_vramData = ReadRawVRAM(vrmPath);
	}

	m_hasRawTexture = true;

	m_parentPath = levFile.parent_path();
	m_name = levFile.filename().replace_extension().string() + "_edit";
	std::filesystem::path modelCacheDir = levFile.parent_path() / (levFile.stem().string() + "_models");

	uint32_t offPointerMap;
	Read(file, offPointerMap);

	std::streampos offLev = file.tellg();

	std::set<uint32_t> pointerMap;
	file.seekg(offLev + std::streampos(offPointerMap));
	uint32_t pointerMapSize;
	Read(file, pointerMapSize);
	for (size_t i = 0; i < pointerMapSize / sizeof(uint32_t); i++)
	{
		uint32_t pointer;
		Read(file, pointer);
		pointerMap.insert(pointer);
	}

	file.seekg(offLev);
	PSX::LevHeader header = {};
	Read(file, header);


	printf("NumSpwanType2 : %d at offset 0x%x\n", header.numSpawnType_2, header.offSpawnType_2);
	if (header.offSpawnType_2 != 0)
	{	
		for (uint32_t i = 0; i < header.numSpawnType_2; i++)
		{
			std::vector<Vec3> spawntype;
			file.seekg(offLev + std::streampos(header.offSpawnType_2 + i * sizeof(PSX::SpawnType2)));
			PSX::SpawnType2 st2{};
			Read(file, st2);
			printf("SpawnType2 ID %d, numCoord : %d, offCoord : 0x%x\n", i, st2.numCoord, st2.offPos);
			if (st2.offPos != 0)
			{
				for (uint32_t j = 0; j < st2.numCoord; j++)
				{
					file.seekg(offLev + std::streampos(st2.offPos + j * sizeof(PSX::Vec3)));
					PSX::Vec3 pos{};
					Read(file, pos);
					Vec3 realPos = ConvertPSXVec3(pos, FP_ONE_GEO);
					//printf("Coord ID %d, x : %.2f, y : %.2f, z : %.2f\n", j, realPos.x, realPos.y, realPos.z);
					spawntype.push_back(realPos);
				}
			}
			m_spawntypes.push_back(spawntype);
		}
	}

	printf("NumSpwanType2 PosRot: %d at offset 0x%x\n", header.numSpawnType_2_posRot, header.offSpawnType_2_posRot);
	if (header.offSpawnType_2_posRot != 0)
	{
		for (uint32_t i = 0; i < header.numSpawnType_2_posRot; i++)
		{
			std::vector<Spawn> spawntypePosRot;
			file.seekg(offLev + std::streampos(header.offSpawnType_2_posRot + i * sizeof(PSX::SpawnType2)));
			PSX::SpawnType2 st2{};
			Read(file, st2);
			printf("SpawnType2 ID %d, numCoord : %d, offCoord : 0x%x\n", i, st2.numCoord, st2.offPos);
			if (st2.offPos != 0)
			{
				for (uint32_t j = 0; j < st2.numCoord; j++)
				{
					file.seekg(offLev + std::streampos(st2.offPos + j * sizeof(PSX::Spawn)));
					PSX::Spawn spwn{};
					Read(file, spwn);
					Spawn realSpwn{};
					realSpwn.pos = ConvertPSXVec3(spwn.pos, FP_ONE_GEO);
					realSpwn.rot = ConvertPSXAngle(spwn.rot);
					//printf("Coord ID %d, x : %.2f, y : %.2f, z : %.2f\n", j, realSpwn.pos.x, realSpwn.pos.y, realSpwn.pos.z);
					spawntypePosRot.push_back(realSpwn);
				}
			}
			m_spawntypesPosRot.push_back(spawntypePosRot);
		}
	}














	m_configFlags = header.config;
	m_clearColor = ConvertColor(header.clear);
	m_stars = ConvertStars(header.stars);
	m_jumpYSpeedCap = static_cast<int>(header.jumpYSpeedCap);
	m_splitLines[0] = ConvertFP(header.splitLines[0], FP_ONE_GEO);
	m_splitLines[1] = ConvertFP(header.splitLines[1], FP_ONE_GEO);
	m_weather = ConvertWeather(header.weather);
	for (size_t i = 0; i < m_spawn.size(); i++)
	{
		m_spawn[i].pos = ConvertPSXVec3(header.driverSpawn[i].pos, FP_ONE_GEO);
		m_spawn[i].rot = ConvertPSXAngle(header.driverSpawn[i].rot);
	}
	for (size_t i = 0; i < NUM_GRADIENT; i++)
	{
		m_skyGradient[i].posFrom = ConvertFP(header.skyGradient[i].posFrom, 1u);
		m_skyGradient[i].posTo = ConvertFP(header.skyGradient[i].posTo, 1u);
		m_skyGradient[i].colorFrom = ConvertColor(header.skyGradient[i].colorFrom);
		m_skyGradient[i].colorTo = ConvertColor(header.skyGradient[i].colorTo);
	}

	PSX::MeshInfo meshInfo = {};
	file.seekg(offLev + std::streampos(header.offMeshInfo));
	Read(file, meshInfo);

	std::vector<PSX::Vertex> vertices;
	vertices.reserve(meshInfo.numVertices);
	file.seekg(offLev + std::streampos(meshInfo.offVertices));
	for (uint32_t i = 0; i < meshInfo.numVertices; i++)
	{
		PSX::Vertex vertex = {};
		Read(file, vertex);
		vertices.push_back(vertex);
	}





	//Load preset models
	std::filesystem::path folderPath(Settings::m_lastOpenedModelFolder);
	if (std::filesystem::exists(folderPath) && std::filesystem::is_directory(folderPath))
	{
		for (const auto& entry : std::filesystem::directory_iterator(folderPath))
		{
			if (!entry.is_regular_file())
				continue;

			//todo
		}
	}




	// Loading textures and animated textures and quadblocks
	std::filesystem::path vrmPath = levFile;
	vrmPath.replace_extension(".vrm");
	std::vector<uint16_t> vram =  ReadRawVRAM(vrmPath);
	int texCounter = 0;
	std::vector<uint32_t> quadblocksVisibleSetOff; // List of VisibleSetOffset for quadblock. Needed for vistree loading, parsed with quadblocks.
	std::unordered_map<LayoutKey, PixelBounds> textureToPixelBounds; // Map Layout key -> Pixels bounds of the texture.
	std::unordered_map<LayoutKey, std::string> materialCache; // Layout Key -> matName
	std::map<size_t, std::map<size_t, uint32_t>> quadblockFaceToAnimOffset; // Map: quadblock index -> face index -> AnimTex offset
	std::unordered_map<uint32_t, std::string> textureGroupToMaterial; // Map : texture group offset -> material name
	m_rawAnimTex.clear(); // Map : Absolute Offset -> PSX::AnimTex
	m_rawTextureGroup.clear(); // Map : Absolute Offset ->  PSX::TextureGroup
	m_rawAnimTexFrames.clear(); // Map : Absolute Offset -> List of Absolute Offset for PSX::TextureGroup

	// WATER
	if (header.offEnvironmentMap != 0)
	{
		file.seekg(offLev + std::streampos(header.offEnvironmentMap));
		Read(file, m_rawWaterLayout);
	}

	//ICONS
	std::vector<PSX::Icon> levelIcons;
	if (header.offIconsLookup != 0)
	{
		PSX::LevelIconHeader levelIconHeader{};
		file.seekg(offLev + std::streampos(header.offIconsLookup));
		Read(file, levelIconHeader);
		if (levelIconHeader.offFirstIcon != 0)
		{
			for (int32_t iconId = 0; iconId < levelIconHeader.numIcon; iconId++)
			{
				file.seekg(offLev + std::streampos(levelIconHeader.offFirstIcon + iconId * sizeof(PSX::Icon)));
				PSX::Icon icon{};
				Read(file, icon);
				levelIcons.push_back(icon);
				/*PSX::TextureLayout& layout = icon.texLayout;
				LayoutKey key(layout);

				if (!materialCache.contains(key))
				{
					std::string newMatName = "icon_" + std::to_string(texCounter++);
					materialCache[key] = newMatName;
				}
				RawUV rawUV(layout);
				textureToPixelBounds[key].Update(rawUV);*/
				//printf("Icon %I32u, name :%s, tex:%s, globalArrayId %I32u\n", iconId, icon.name, materialCache[key].c_str(), icon.globalIconArrayIndex);
			}
		}
	}

	std::filesystem::path tempDir = levFile.parent_path() / (levFile.stem().string() + "_textures");
	std::filesystem::create_directories(tempDir);

	bool hasAnimData = header.offAnimTex > 0;
	size_t offAnimStart = header.offAnimTex;


	//Extract Environment map and minimap (todo)
	LayoutKey waterkey(m_rawWaterLayout);
	PixelBounds waterBound{ 0, 0, 63, 63 }; // Always 64x64. Actual UVs in the TextureLayout are irrelevant.
	m_envMapMatName = "envMap";
	Texture envMapTex(waterkey, waterBound, vram, m_envMapMatName, tempDir, true);
	m_materialToTexture[m_envMapMatName] = envMapTex;


	Texture minimapTop; Texture minimapBottom;
	for (PSX::Icon& icon : levelIcons)
	{
		if (icon.globalIconArrayIndex == PSX::ICON_INDEX_MAP_TOP)
		{
			LayoutKey mapKey(icon.texLayout);
			PixelBounds bounds{};
			bounds.Update(RawUV(icon.texLayout));
			minimapTop = Texture(mapKey, bounds, vram, "minimap_top", tempDir, true);

		} 
		if (icon.globalIconArrayIndex == PSX::ICON_INDEX_MAP_BOTTOM)
		{
			LayoutKey mapKey(icon.texLayout);
			PixelBounds bounds{};
			bounds.Update(RawUV(icon.texLayout));
			minimapBottom = Texture(mapKey, bounds, vram, "minimap_bottom", tempDir, true);
		}
	}
	if (!minimapTop.IsEmpty() && !minimapBottom.IsEmpty())
	{
		m_minimapConfig.texture = Texture(minimapTop, minimapBottom, "minimap", tempDir);
	}



	
	// 1st pass : Parse Quadblock, find TextureGroups, and caclulate UV bounds
	// Take care of all texture group for static quad and animated quads
	file.seekg(offLev + std::streampos(meshInfo.offQuadblocks));
	for (uint32_t i = 0; i < meshInfo.numQuadblocks; i++)
	{
		PSX::Quadblock psxQuad = {};
		Read(file, psxQuad);
		std::streampos currentPosQuad = file.tellg();
		for (int f = 0; f < NUM_FACES_QUADBLOCK + 1; f++)
		{
			uint32_t texOffset = f == NUM_FACES_QUADBLOCK ? psxQuad.offLowTexture : psxQuad.offMidTextures[f];

			// How to know if a texture is animated or not : POINTERFLAG. ODD = ANIMTEX. EVEN = STATICTEX
			if (hasAnimData && texOffset >= offAnimStart && pointerMap.contains(texOffset - 1)) // Anim Textures
			{
				if (!m_rawAnimTex.contains(texOffset-1))
				{
					file.seekg(offLev + std::streampos(texOffset-1));
					PSX::AnimTex animTex;
					Read(file, animTex);
					m_rawAnimTex[texOffset - 1] = animTex;

					std::vector<uint32_t> frameTextureGroupOffset;
					for (uint16_t frame = 0; frame < animTex.frameCount; frame++)
					{
						uint32_t frameTexOffset;
						Read(file, frameTexOffset);
						frameTextureGroupOffset.push_back(frameTexOffset);

						std::streampos currentPos = file.tellg();
						file.seekg(offLev + static_cast<std::streamoff>(frameTexOffset));
						PSX::TextureGroup group = {};
						Read(file, group);
						file.seekg(currentPos);
						// Tempfix for vanilla : group.mosaic is broken for a lot of texture, need research
						PSX::TextureGroup tempTexGroup = {};
						tempTexGroup.far = group.far;
						tempTexGroup.middle = group.middle;
						tempTexGroup.near = group.near;
						tempTexGroup.mosaic = group.near;
						m_rawTextureGroup[frameTexOffset] = tempTexGroup;
						const PSX::TextureLayout& layout = group.middle;
						LayoutKey key(layout);

						if (!materialCache.contains(key))
						{
							std::string newMatName = "tex_" + std::to_string(texCounter++);
							materialCache[key] = newMatName;
						}
						textureGroupToMaterial[frameTexOffset] = materialCache[key];

						RawUV rawUV(layout);
						textureToPixelBounds[key].Update(rawUV);

					}
					m_rawAnimTexFrames[texOffset - 1] = frameTextureGroupOffset;
				}

				quadblockFaceToAnimOffset[i][f] = texOffset - 1;

			}
			else // Regular Textures
			{
				file.seekg(offLev + static_cast<std::streamoff>(texOffset));
				PSX::TextureGroup group = {};
				Read(file, group);
				// Tempfix for vanilla : group.mosaic is broken for a lot of texture, need research
				PSX::TextureGroup tempTexGroup = {};
				tempTexGroup.far = group.far;
				tempTexGroup.middle = group.middle;
				tempTexGroup.near = group.near;
				tempTexGroup.mosaic = group.near;
				m_rawTextureGroup[texOffset] = tempTexGroup;
				const PSX::TextureLayout& layout = group.middle;
				LayoutKey key(layout);

				if (!materialCache.contains(key))
				{
					std::string newMatName = "tex_" + std::to_string(texCounter++);
					materialCache[key] = newMatName;
				}
				textureGroupToMaterial[texOffset] = materialCache[key];

				RawUV rawUV(layout, psxQuad.drawOrderLow, f);
				textureToPixelBounds[key].Update(rawUV);
			}

		}
		file.seekg(currentPosQuad);
	}

	std::set<uint32_t> parsedModelOffsets;
	// 2nd pass : Find TextureLayouts from Instances, fill Layout Keys
	if (header.offInstances != 0)
	{
		file.seekg(offLev + std::streampos(header.offInstances));
		for (uint32_t i = 0; i < header.numInstances; i++)
		{
			PSX::InstDef inst{};
			Read(file, inst);
			if (inst.offModel != 0)
			{
				std::streampos currentPosInst = file.tellg();
				file.seekg(offLev + std::streampos(inst.offModel));
				PSX::Model model{};
				Read(file, model);
				if (parsedModelOffsets.contains(inst.offModel))
				{
					file.seekg(currentPosInst);
					continue;
				}
				else
				{
					parsedModelOffsets.insert(inst.offModel);
				}
				if (model.offHeaders != 0 && model.numHeaders > 0)
				{
					for (uint32_t j = 0; j < model.numHeaders; j++)
					{
						file.seekg(offLev + std::streampos(model.offHeaders + j * sizeof(PSX::ModelHeader)));
						PSX::ModelHeader modelHeader{};
						Read(file, modelHeader);
						if (modelHeader.offCommandList != 0 && modelHeader.offTexLayout != 0)
						{
							// --- Pass 1: scan command list to find how many texture layouts are actually used ---
							file.seekg(offLev + std::streampos(modelHeader.offCommandList));
							uint32_t colorCount = 0;
							Read(file, colorCount);
							uint32_t maxTexCoordIndex = 0;
							while (true)
							{
								PSX::InstDrawCommand cmd{};
								Read(file, cmd);
								if (cmd.command == 0xFFFFFFFF) break;
								if (cmd.texCoordIndex > maxTexCoordIndex)
									maxTexCoordIndex = cmd.texCoordIndex;
							}
							// --- Pass 2: read each TextureLayout via the pointer array ---
							for (uint32_t ti = 0; ti < maxTexCoordIndex; ti++)
							{
								file.seekg(offLev + std::streampos(modelHeader.offTexLayout + ti * sizeof(uint32_t)));
								uint32_t offLayout = 0;
								Read(file, offLayout);
								if (offLayout == 0) continue;

								file.seekg(offLev + std::streampos(offLayout));
								PSX::TextureLayout layout{};
								Read(file, layout);
								//
								LayoutKey key(layout);
								if (!materialCache.contains(key))
								{
									std::string newMatName = "tex_model_" + std::to_string(texCounter++);
									materialCache[key] = newMatName;
								}
								/*printf("Instance %s, Model %s, ModelHeader %s, texturename %s\n",
									std::string(inst.name, strnlen(inst.name, sizeof(inst.name))).c_str(),
									std::string(model.name, strnlen(model.name, sizeof(model.name))).c_str(),
									std::string(modelHeader.name, strnlen(modelHeader.name, sizeof(modelHeader.name))).c_str(),
									materialCache[key].c_str());*/
								//
								RawUV rawUV(layout);
								textureToPixelBounds[key].Update(rawUV);
							}
						}
					}
				}
				file.seekg(currentPosInst);
			}
		}
	}


	// 3rd pass : Create PNGs and Materials
	for (const auto& [key, bounds] : textureToPixelBounds)
	{
		std::string newMatName = materialCache[key];
		Texture newTexture(key, bounds, vram, newMatName, tempDir, true);
		m_materialToTexture[newMatName] = newTexture;
	}
	
	// 4.1th pass : Create Models/Header with UVs and textures Assign QuadUVs to Models/Headers
	std::unordered_map<uint32_t, size_t> offsetToModelKey;
	if (header.offInstances != 0)
	{
		for (uint32_t i = 0; i < header.numInstances; i++)
		{
			file.seekg(offLev + std::streampos(header.offInstances + i * sizeof(PSX::InstDef)));
			PSX::InstDef inst{};
			Read(file, inst);
			if (inst.offModel != 0)
			{
				file.seekg(offLev + std::streampos(inst.offModel));
				PSX::Model model{};
				Read(file, model);
				std::string modelName(model.name, strnlen(model.name, sizeof(model.name)));
				if (offsetToModelKey.contains(inst.offModel))
					continue;
					
				size_t modelKey = GenerateUniqueModelKey();
				offsetToModelKey[inst.offModel] = modelKey;
				m_instanceModels[modelKey] = InstanceModel(model);
				if (model.offHeaders != 0 && model.numHeaders > 0)
				{
					for (uint32_t j = 0; j < model.numHeaders; j++)
					{
						file.seekg(offLev + std::streampos(model.offHeaders + j * sizeof(PSX::ModelHeader)));
						PSX::ModelHeader modelHeader{};
						Read(file, modelHeader);
						bool isAnimated = modelHeader.offAnimations != 0;

						if ((modelHeader.numAnimations != 0) != isAnimated ||
							modelHeader.offAnimtex != 0 ||							// still unsupported
							modelHeader.offCommandList == 0 ||
							modelHeader.offColors == 0 ||
							(!isAnimated && modelHeader.offFrameData == 0) ||
							(!isAnimated && modelHeader.offStaticDeltaArray != 0) ||// compressed static: still unsupported
							(isAnimated && modelHeader.offFrameData != 0))			// ambiguous per RenderBucket_GetFrame
						{
							printf("Couldn't import model %s, offAnim 0x%x, numAnim %d, offCommand 0x%x, offAnimTex 0x%x, offColors 0x%x, offSDT 0x%x, offFrameData 0x%x\n",
								modelName.c_str(), modelHeader.offAnimations, modelHeader.numAnimations, modelHeader.offCommandList, modelHeader.offAnimtex, modelHeader.offColors, modelHeader.offStaticDeltaArray, modelHeader.offFrameData);
							m_instanceModels[modelKey].SetValid(false);
							continue;
						}
							
						Vec3 modelScale = ConvertPSXVec3(modelHeader.scale, FP_ONE_MODEL_SCALE);

						// Step 1 : Decode all commands
						file.seekg(offLev + std::streampos(modelHeader.offCommandList));
						uint32_t colorCount = 0;
						Read(file, colorCount);
						std::vector<PSX::InstDrawCommand> commandList;
						while (true)
						{
							PSX::InstDrawCommand cmd{};
							Read(file, cmd);
							if (cmd.command == 0xFFFFFFFF)
								break;
							else
								commandList.push_back(cmd);			
						}
						int numVerts = 0;
						for (PSX::InstDrawCommand& command : commandList)
						{
							if ((command.command & 0xFFFF0000) == 0) continue; // color-only, doesn't push
							if (!command.readNextVertFromStackIndexFlag) numVerts++;
						}

						// Step 2: locate the source of the base / rest vertex data.
						PSX::ModelFrame baseFrame{};
						size_t baseFrameFileOffset = 0;
						std::vector<PSX::ModelAnim> animHeaders;
						std::vector<uint32_t> animOffsets;

						if (!isAnimated)
						{
							baseFrameFileOffset = modelHeader.offFrameData;
							file.seekg(offLev + std::streampos(baseFrameFileOffset));
							Read(file, baseFrame);
						}
						else
						{
							if (modelHeader.numAnimations == 0) { m_instanceModels[modelKey].SetValid(false); continue; }
							animOffsets.resize(modelHeader.numAnimations);
							animHeaders.resize(modelHeader.numAnimations);
							bool valid = true;
							for (uint32_t a = 0; a < modelHeader.numAnimations; a++)
							{
								file.seekg(offLev + std::streampos(modelHeader.offAnimations + a * sizeof(uint32_t)));
								Read(file, animOffsets[a]);
								if (animOffsets[a] == 0) { valid = false; break; }
								file.seekg(offLev + std::streampos(animOffsets[a]));
								Read(file, animHeaders[a]);
							}
							if (!valid || animHeaders[0].offDeltaArray != 0)
							{
								m_instanceModels[modelKey].SetValid(false);
								continue;
							}
							baseFrameFileOffset = animOffsets[0] + sizeof(PSX::ModelAnim);
							file.seekg(offLev + std::streampos(baseFrameFileOffset));
							Read(file, baseFrame);
						}


						// Decodes one frame's raw per-vertex positions from its own file offset.
						auto DecodeFrameVertices = [&](size_t frameFileOffset, const PSX::ModelFrame& frame) -> std::vector<Vec3>
							{
								Vec3 origin = ConvertPSXVec3(frame.pos, FP_ONE_MODEL_ORIGIN);
								std::vector<Vec3> raw(numVerts);
								for (int vi = 0; vi < numVerts; vi++)
								{
									file.seekg(offLev + std::streampos(frameFileOffset + frame.vertexOffset + vi * sizeof(PSX::Vec3b)));
									PSX::Vec3b vert;
									Read(file, vert);
									raw[vi] = (ConvertPSXVec3b(vert, 255) + origin) * modelScale;
								}
								return raw;
							};


						// Decode Vertices from base				
						std::vector<Point> headerVertices;
						for (Vec3 pos : DecodeFrameVertices(baseFrameFileOffset, baseFrame))
						{
							Point p{}; p.pos = pos;
							headerVertices.push_back(p);
						}

						// Decode topology
						std::vector<Point> stack(256);
						std::vector<int> stackVertexIndex(256, -1);
						int vertexIndex = 0;
						int stripLength = 0;
						Point temp[4] = {};
						int tempVertexIndex[4] = { -1, -1, -1, -1 };
						std::vector<Tri> triList;
						std::vector<bool> triDoubleSided;
						std::vector<std::array<int, 3>> triSourceVertexIndices;

						for (PSX::InstDrawCommand& command : commandList)
						{
							if ((command.command & 0xFFFF0000) == 0) { continue; }
							if (!command.readNextVertFromStackIndexFlag)
							{
								stack[command.stackWriteLocationIndex] = headerVertices[vertexIndex];
								stackVertexIndex[command.stackWriteLocationIndex] = vertexIndex;
								vertexIndex++;
							}
							temp[0] = temp[1]; temp[1] = temp[2]; temp[2] = temp[3];
							temp[3] = stack[command.stackWriteLocationIndex];
							tempVertexIndex[0] = tempVertexIndex[1];
							tempVertexIndex[1] = tempVertexIndex[2];
							tempVertexIndex[2] = tempVertexIndex[3];
							tempVertexIndex[3] = stackVertexIndex[command.stackWriteLocationIndex];

							int colorIdx = command.colorCoordIndex;
							file.seekg(offLev + std::streampos(modelHeader.offColors + colorIdx * sizeof(uint32_t)));
							PSX::Color psxCol;
							Read(file, psxCol);
							temp[3].color = ConvertColor(psxCol);

							if (command.swapFlag) { temp[1] = temp[0]; tempVertexIndex[1] = tempVertexIndex[0]; }
							if (command.resetFlag) { stripLength = 0; }

							if (stripLength >= 2)
							{
								Tri tri{};
								std::string texName = "default";
								QuadUV uvs{};
								int texIdx = command.texCoordIndex;
								if (texIdx > 0)
								{
									file.seekg(offLev + std::streampos(modelHeader.offTexLayout + (texIdx - 1) * sizeof(uint32_t)));
									uint32_t offLayout = 0;
									Read(file, offLayout);
									if (offLayout == 0) { stripLength++; continue; }
									file.seekg(offLev + std::streampos(offLayout));
									PSX::TextureLayout layout{};
									Read(file, layout);
									LayoutKey key(layout);
									PixelBounds& bounds = textureToPixelBounds[key];
									RawUV rawUV(layout);
									uvs = MakeUV(bounds, rawUV);
									texName = materialCache[key];
								}

								tri.p[0].pos = temp[3].pos; tri.p[1].pos = temp[2].pos; tri.p[2].pos = temp[1].pos;
								tri.p[0].color = temp[3].color; tri.p[1].color = temp[2].color; tri.p[2].color = temp[1].color;
								tri.p[0].uv = uvs[2]; tri.p[1].uv = uvs[1]; tri.p[2].uv = uvs[0];
								tri.texture = texName;
								tri.doubleSided = command.noBackfaceFlag != 1;
								triList.push_back(tri);

								std::array<int, 3> src = { tempVertexIndex[3], tempVertexIndex[2], tempVertexIndex[1] };
								if (command.normalFlipFlag)
								{
									Tri& last = triList.back();
									std::swap(last.p[1].pos, last.p[2].pos);
									std::swap(last.p[1].color, last.p[2].color);
									std::swap(last.p[1].uv, last.p[2].uv);
									std::swap(src[1], src[2]);
								}
								triSourceVertexIndices.push_back(src);
							}
							stripLength++;
						}

						// Decode all frames of animations
						std::vector<ModelAnimation> animations;
						if (!isAnimated)
						{
							ModelAnimation staticAnim{};
							staticAnim.name = ""; 
							staticAnim.interpolated = false;
							staticAnim.frames.push_back(triList);
							animations.push_back(std::move(staticAnim));
						}
						else
						{
							for (uint32_t a = 0; a < modelHeader.numAnimations; a++)
							{
								const PSX::ModelAnim& anim = animHeaders[a];
								if (anim.offDeltaArray != 0)
								{
									printf("SKIPPING compressed animation '%.16s' on model %s\n", anim.name, modelName.c_str());
									continue;
								}

								ModelAnimation animation{};
								animation.name = std::string(anim.name, strnlen(anim.name, sizeof(anim.name)));
								animation.interpolated = (anim.numFrames & PSX::ANIM_INTERPOLATED_BIT) != 0;
								size_t numStoredFrames = PSX::StoredFrameCount(anim.numFrames);

								for (size_t f = 0; f < numStoredFrames; f++)
								{
									size_t offFrame = animOffsets[a] + sizeof(PSX::ModelAnim) + f * anim.frameSize;
									file.seekg(offLev + std::streampos(offFrame));
									PSX::ModelFrame animFrame{};
									Read(file, animFrame);
									std::vector<Vec3> rawVerts = DecodeFrameVertices(offFrame, animFrame);
									std::vector<Tri> frame = triList;
									for (size_t t = 0; t < triSourceVertexIndices.size(); t++)
										for (int c = 0; c < 3; c++)
											frame[t].p[c].pos = rawVerts[triSourceVertexIndices[t][c]];
									animation.frames.push_back(frame);
								}
								animations.push_back(std::move(animation));
							}
							if (animations.empty())
							{
								m_instanceModels[modelKey].SetValid(false);
								continue;
							}
						}
						m_instanceModels[modelKey].m_headers.emplace_back(modelHeader, baseFrame, colorCount, std::move(animations), isAnimated);
					}
				}
			}
		}
	}
	// Delete invalid models
	std::vector<uint32_t> modelToDel;
	for (auto& [offset, key] : offsetToModelKey)
	{
		if (!m_instanceModels[key].IsValid())
			modelToDel.push_back(offset);
	}
	for (uint32_t& offset : modelToDel)
	{
		m_instanceModels.erase(offsetToModelKey[offset]);
		offsetToModelKey.erase(offset);
	}
		

	//Export model to modifiable state 
	std::filesystem::create_directories(modelCacheDir);
	for (auto& [key, model] : m_instanceModels)
	{
		model.Export(modelCacheDir, m_materialToTexture);
	}
	

	std::map<uint32_t, std::vector<std::tuple<size_t, size_t>>> vertToQuad; // Map vertex absolute offset -> List of (quad, vert id) containing the vert.

	// 4th pass : create quadblocks with material, UVs and texture	
	file.seekg(offLev + std::streampos(meshInfo.offQuadblocks));
	for (uint32_t i = 0; i < meshInfo.numQuadblocks; i++)
	{
		PSX::Quadblock psxQuad = {};
		Read(file, psxQuad);
		quadblocksVisibleSetOff.push_back(psxQuad.offVisibleSet);
		Quadblock& qb = m_quadblocks.emplace_back(psxQuad, vertices, [this](const Quadblock& qb) { UpdateFilterRenderData(qb); });
		bool materialAssigned = false;
		std::string qbMatName = "default";
		for (int f = 0; f < 4; f++) 
		{
			uint32_t texOffset = psxQuad.offMidTextures[f];
			if (hasAnimData && texOffset >= offAnimStart && pointerMap.contains(texOffset - 1)) // Anim Texture
			{
				qb.SetAnimated(true);
			}
			
			else 
			{
				std::streampos currentPos = file.tellg();
				file.seekg(offLev + static_cast<std::streamoff>(texOffset));
				PSX::TextureGroup group = {};
				Read(file, group);
				file.seekg(currentPos);

				const PSX::TextureLayout& layout = group.middle;
				LayoutKey key(layout);

				if (!materialAssigned)
				{
					qbMatName = materialCache[key];
					qb.SetMaterial(qbMatName);
					qb.SetTexPath(m_materialToTexture[qbMatName].GetPath());
					m_materialToQuadblocks[qbMatName].push_back(i);
					materialAssigned = true;
				}

				RawUV rawUV(layout, psxQuad.drawOrderLow, f);
				const PixelBounds& bounds = textureToPixelBounds[key];
				qb.SetFaceUVs(f, MakeUV(bounds, rawUV));
			}
		}
		if (!materialAssigned) 
		{
			qb.SetMaterial("default");
			m_materialToQuadblocks["default"].push_back(i);
		}
		for (size_t j = 0; j < NUM_VERTICES_QUADBLOCK; j++)
		{
			vertToQuad[static_cast<uint32_t>(meshInfo.offVertices + psxQuad.index[j] * sizeof(PSX::Vertex))].push_back(std::make_tuple(i, j));
		}
	}

	//5th pass : Create .obj for AnimText, and assign to quads
	if (hasAnimData)
	{
		std::map<std::map<size_t, uint32_t>, std::set<size_t>> facePatternToQuadblocks;
		for (const auto& [quadIdx, faceMap] : quadblockFaceToAnimOffset)
		{
			facePatternToQuadblocks[faceMap].insert(quadIdx);
		}

		std::set<std::map<size_t, uint32_t>> processedPatterns;
		for (const auto& [faceMap, quadSet] : facePatternToQuadblocks)
		{
			if (processedPatterns.contains(faceMap)) continue;
			if (faceMap.empty()) continue;

			std::vector<size_t> quadIndices(quadSet.begin(), quadSet.end());

			uint32_t firstAnimOffset = faceMap.begin()->second;
			if (!m_rawAnimTex.contains(firstAnimOffset)) continue;

			const PSX::AnimTex& firstAnimData = m_rawAnimTex[firstAnimOffset];
			size_t frameCount = firstAnimData.frameCount;

			// Verify all AnimTex in this pattern have the same frame count
			bool validAnimation = true;
			for (const auto& [faceIdx, animOffset] : faceMap)
			{
				if (!m_rawAnimTex.contains(animOffset) || !m_rawAnimTexFrames.contains(animOffset) || m_rawAnimTex[animOffset].frameCount != frameCount)
				{
					validAnimation = false;
					break;
				}
			}
			if (!validAnimation) continue;


			std::array<std::vector<PSX::TextureLayout>, 4> faceFrameLayouts;
			std::array<std::vector<std::string>, 4> faceFrameMaterials;

			bool allMaterialsFound = true;

			for (const auto& [faceIdx, animOffset] : faceMap)
			{
				for (uint32_t textureGroupOffset : m_rawAnimTexFrames.at(animOffset))
				{
					if (!textureGroupToMaterial.contains(textureGroupOffset)) 
					{
						allMaterialsFound = false;
						break;
					}
					faceFrameMaterials[faceIdx].push_back(textureGroupToMaterial[textureGroupOffset]);
					std::streampos savedPos = file.tellg();
					file.seekg(offLev + std::streampos(textureGroupOffset));
					PSX::TextureGroup group = {};
					Read(file, group);
					file.seekg(savedPos);
					faceFrameLayouts[faceIdx].push_back(group.middle);
				}
				if (!allMaterialsFound) break;
			}

			if (!allMaterialsFound) continue;

			// Create temporary OBJ file
			std::string animName = "";
			for (size_t faceIdx = 0; faceIdx < 4; faceIdx++)
			{
				if (faceFrameMaterials[faceIdx].size() != 0)
				{
					animName = faceFrameMaterials[faceIdx][0];
					break;
				}
			}

			std::filesystem::path animDir = tempDir / animName;
			std::filesystem::create_directories(animDir);

			AnimTexture animTexture(animName, tempDir, faceFrameLayouts, faceFrameMaterials, quadIndices, m_quadblocks, textureToPixelBounds, m_materialToTexture, firstAnimData, m_animTextures);

			if (!animTexture.IsEmpty())
			{
				animTexture.SetStartFrame(firstAnimData.startAtFrame);
				animTexture.SetDuration(firstAnimData.frameDuration);

				for (size_t quadIdx : quadIndices)
				{
					animTexture.AddQuadblockIndex(quadIdx);
					std::string oldMat = m_quadblocks[quadIdx].GetMaterial();
					auto& v = m_materialToQuadblocks[oldMat];
					v.erase(std::remove(v.begin(), v.end(), quadIdx), v.end());
					m_quadblocks[quadIdx].SetMaterial(animName);
					m_materialToQuadblocks[animName].push_back(quadIdx);
				}
				m_animTextures.push_back(animTexture);
				processedPatterns.insert(faceMap);
			}
			else
			{
				printf("WARNING : Empty animtex\n");
			}
		}
	}



	if (header.offWaterVertices != 0)
	{
		file.seekg(offLev + std::streampos(header.offWaterVertices));
		for (uint32_t i = 0; i < header.numWaterVertices; i++)
		{
			PSX::WaterVertex wv;
			Read(file, wv);
			std::streampos currentPos = file.tellg();

			if (wv.offVertex == 0 || wv.offOceanVertex == 0)
			{
				printf("ERROR : WaterVertex with nullptr at vertex number %d\n", i);
				continue;
			}
			file.seekg(offLev + std::streampos(wv.offVertex));
			PSX::Vertex v;
			Read(file, v);
			
			file.seekg(offLev + std::streampos(wv.offOceanVertex));
			PSX::OceanVertex ov;
			Read(file, ov);

			for (std::tuple<size_t, size_t>& tuple : vertToQuad[wv.offVertex])
			{
				size_t quadID = std::get<0>(tuple);
				size_t vertID = std::get<1>(tuple);
				m_quadblocks[quadID].SetOceanVertex(ov, vertID);
			}
			file.seekg(currentPos);
		}
	}
	

	m_bsp.Clear();
	//Must reset the BSP global IDs
	file.seekg(offLev + std::streampos(meshInfo.offBSPNodes));
	std::vector<BSP*> bspArray;
	for (uint32_t i = 0; i < meshInfo.numBSPNodes; i++)
	{
		bspArray.push_back(new BSP());
	}

	for (uint32_t i = 0; i < meshInfo.numBSPNodes; i++)
	{
		uint16_t flag;
		std::streampos nodeStart = file.tellg();
		Read(file, flag);
		file.seekg(nodeStart);

		if (flag & BSPFlags::LEAF)
		{
			PSX::BSPLeaf leaf = {};
			Read(file, leaf);
			bspArray[leaf.id]->PopulateLeaf(leaf, bspArray, m_quadblocks, meshInfo.offQuadblocks, meshInfo.numBSPNodes);
		}
		else
		{
			PSX::BSPBranch branch = {};
			Read(file, branch);
			if (branch.unk2 != 0 || branch.unk3 != 0) { printf("Branch ID%d has child 3\n", branch.id); }
			bspArray[branch.id]->PopulateBranch(branch, bspArray, meshInfo.numBSPNodes);
		}
	}

	if (!bspArray.empty())
	{
		m_bsp = *(bspArray[0]);
		m_bsp.PopulateBranchQuadIndexes();
		if (m_bsp.IsValid()) { GenerateRenderBspData(); }
		else { m_bsp.Clear(); }
	}
	else { m_bsp.Clear(); }
	std::set<size_t> validID;
	
	printf("BSP ARRAY SIZE : %zu\n", bspArray.size());
	std::vector<const BSP*> tree = static_cast<const BSP&>(m_bsp).GetTree();
	printf("BSP TREE SIZE : %zu\n", tree.size());
	for (const BSP* bsp : tree) { validID.insert(bsp->GetId()); }
	for (BSP* bsp : bspArray) { if (!validID.contains(bsp->GetId())) { printf("ID %zu isn't in tree\n", bsp->GetId()); } }
	


	// Load VisTree
	if (header.offVisMem != 0)
	{
		file.seekg(offLev + static_cast<std::streamoff>(header.offVisMem));
		PSX::VisualMem visMem = {};
		Read(file, visMem);

		if (visMem.offNodes[0] != 0)
		{
			std::vector<const BSP*> bspLeaves = m_bsp.GetLeaves();
			std::vector<const BSP*> bspNodes = static_cast<const BSP&>(m_bsp).GetTree();

			m_bspVis = BitMatrix(bspLeaves.size(), bspLeaves.size());

			std::map<size_t, size_t> leafIdToMatrix;
			for (size_t i = 0; i < bspLeaves.size(); i++)
			{
				leafIdToMatrix[bspLeaves[i]->GetId()] = i;
			}

			const size_t visNodeSize = (bspNodes.size() + 31) / 32;

			auto decompressVisNodes = [&](std::streampos srcPos) -> std::vector<uint32_t>
				{
					std::vector<uint8_t> dst(visNodeSize * sizeof(uint32_t), 0);
					file.seekg(srcPos);
					size_t dstIdx = 0;
					while (dstIdx < dst.size())
					{
						int8_t c;
						Read(file, c);
						if (c == 0) { break; }
						if (c < 0)
						{
							int count = (-c) + 1;
							uint8_t val;
							Read(file, val);
							for (int i = 0; i < count && dstIdx < dst.size(); i++)
								dst[dstIdx++] = val;
						}
						else
						{
							int count = c;
							for (int i = 0; i < count && dstIdx < dst.size(); i++)
							{
								uint8_t val;
								Read(file, val);
								dst[dstIdx++] = val;
							}
						}
					}
					std::vector<uint32_t> result(visNodeSize);
					std::memcpy(result.data(), dst.data(), dst.size());
					return result;
				};

			for (size_t q = 0; q < m_quadblocks.size(); q++)
			{
				uint32_t offVisibleSet = quadblocksVisibleSetOff[q];
				if (offVisibleSet == 0) { continue; }
				PSX::VisibleSet visSet = {};
				file.seekg(offLev + static_cast<std::streamoff>(offVisibleSet));
				Read(file, visSet);
				if (visSet.offVisibleBSPNodes == 0) { continue; }
				size_t leafID = m_quadblocks[q].GetBSPID() & ~BSPID::LEAF;
				if (!leafIdToMatrix.contains(leafID)) { continue; }
				size_t visTreeID = leafIdToMatrix[leafID];

				bool compressed = visSet.offVisibleBSPNodes & 1;
				uint32_t actualOff = visSet.offVisibleBSPNodes & ~3u;
				std::streampos srcPos = offLev + static_cast<std::streamoff>(actualOff);

				std::vector<uint32_t> visNodes;
				if (compressed)
				{
					visNodes = decompressVisNodes(srcPos);
				}
				else
				{
					file.seekg(srcPos);
					visNodes.resize(visNodeSize);
					for (size_t i = 0; i < visNodeSize; i++) { Read(file, visNodes[i]); }
				}

				for (size_t i = 0; i < bspLeaves.size(); i++)
				{
					size_t destBspId = bspLeaves[i]->GetId();
					if (destBspId / 32 >= visNodes.size()) { continue; }
					uint32_t word = visNodes[destBspId / 32];
					uint32_t bit = 1u << (31 - (destBspId % 32));
					if (word & bit) { m_bspVis.Set(true, visTreeID, i); }
				}
			}
		}
		int count = 0;
		for (size_t x = 0; x < m_bspVis.GetHeight(); x++)
		{
			for (size_t y = 0; y < m_bspVis.GetWidth(); y++)
			{
				if (m_bspVis.Get(x, y))
					count++;
			}
		}
		int max = static_cast<int>(m_bspVis.GetHeight() * m_bspVis.GetWidth());
		float ratio = 100.0f * static_cast<float>(count) / static_cast<float>(max);
		printf("Visibility: %d/%d,  %f%%\n", count, max, ratio);
	}

	file.seekg(offLev + std::streampos(header.offCheckpointNodes));
	for (uint32_t i = 0; i < header.numCheckpointNodes; i++)
	{
		PSX::Checkpoint checkpoint = {};
		Read(file, checkpoint);
		m_checkpoints.emplace_back(checkpoint, static_cast<int>(i));
	}
	UpdateRenderCheckpointData();

	// Ghost checkpoint reading
	file.seekg(offLev + std::streampos(header.offCheckpointNodes) + static_cast<std::streamoff>(255 * sizeof(PSX::Checkpoint)));
	PSX::Checkpoint checkpoint255 = {};
	Read(file, checkpoint255);
	file.seekg(offLev + std::streampos(header.offCheckpointNodes) + static_cast<std::streamoff>(checkpoint255.linkUp * sizeof(PSX::Checkpoint)));
	PSX::Checkpoint checkpoint255Next = {};
	Read(file, checkpoint255Next);
	file.seekg(offLev + std::streampos(header.offCheckpointNodes) + static_cast<std::streamoff>(checkpoint255Next.linkUp * sizeof(PSX::Checkpoint)));
	PSX::Checkpoint checkpoint255NextNext = {};
	Read(file, checkpoint255NextNext);
	printf("Checkpoint 255's next : %d\n Checkpoint 255's next 's next : %d\n", checkpoint255.linkUp, checkpoint255Next.linkUp);






	m_tropyGhost.clear();
	m_oxideGhost.clear();
	if (header.offExtra > 0)
	{
		file.seekg(offLev + std::streampos(header.offExtra));
		PSX::LevelExtraHeader extraHeader = {};
		Read(file, extraHeader);
		// Read N. Tropy Ghost
		if (extraHeader.count >= PSX::LevelExtra::N_TROPY_GHOST + 1 &&
			extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST] > 0)
		{
			file.seekg(offLev + std::streampos(extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST]));
			size_t ghostSize = 0;
			if (extraHeader.count > PSX::LevelExtra::N_OXIDE_GHOST && extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] > 0)
			{
				ghostSize = extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] - extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST];
			}
			else
			{
				ghostSize = header.offLevNavTable - extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST];
			}
			m_tropyGhost.resize(ghostSize);
			file.read(reinterpret_cast<char*>(m_tropyGhost.data()), ghostSize);
		}
		// Read N. Oxide Ghost
		if (extraHeader.count >= PSX::LevelExtra::N_OXIDE_GHOST + 1 && extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] > 0)
		{
			file.seekg(offLev + std::streampos(extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST]));
			size_t ghostSize = header.offLevNavTable - extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST];
			m_oxideGhost.resize(ghostSize);
			file.read(reinterpret_cast<char*>(m_oxideGhost.data()), ghostSize);
		}

		// Read minimap
		if (extraHeader.count > PSX::LevelExtra::MINIMAP && extraHeader.offsets[PSX::LevelExtra::MINIMAP] != 0)
		{
			file.seekg(offLev + std::streampos(extraHeader.offsets[PSX::LevelExtra::MINIMAP]));
			PSX::Map minimap{};
			Read(file, minimap);
			m_minimapConfig.LoadFromPSX(minimap);
		}
	}

	//Load Skybox
	if (header.offSkybox != 0)
	{
		PSX::Skybox psxSkybox = {};
		file.seekg(offLev + std::streampos(header.offSkybox));
		Read(file, psxSkybox);

		std::vector<PSX::SkyboxVertex> psxVerts(psxSkybox.numVertex);
		file.seekg(offLev + std::streampos(psxSkybox.offVertex));
		for (uint32_t i = 0; i < psxSkybox.numVertex; i++)
		{
			Read(file, psxVerts[i]);
		}

		std::vector<std::vector<uint16_t>> segmentIndices(PSX::NUM_SKYBOX_SEGMENTS);
		for (size_t seg = 0; seg < PSX::NUM_SKYBOX_SEGMENTS; seg++)
		{
			const int16_t faceCount = psxSkybox.numFaces[seg];
			if (faceCount <= 0 || psxSkybox.offFaces[seg] == 0) { continue; }

			const size_t indexCount = static_cast<size_t>(faceCount) * PSX::SKYBOX_FACE_STRIDE;
			segmentIndices[seg].resize(indexCount);
			file.seekg(offLev + std::streampos(psxSkybox.offFaces[seg]));
			for (size_t i = 0; i < indexCount; i++)
			{
				Read(file, segmentIndices[seg][i]);
			}
		}

		std::filesystem::path objPath = m_parentPath / (levFile.filename().replace_extension().string() + "_skybox.obj");
		if (m_skybox.LoadFromPSX(psxSkybox, psxVerts, segmentIndices, objPath)) 
		{
			GenerateRenderSkyboxData();
		}
	}

	if (header.offLevNavTable != 0)
	{
		//printf("off Lev Nav Table : 0x%x\n", header.offLevNavTable);
		file.seekg(offLev + std::streampos(header.offLevNavTable));
		PSX::levAINavTable navTable{};
		Read(file, navTable);
		for (int i = 0; i < 3; i++)
		{
			if (navTable.offAIPathArray[i] != 0)
			{
				//printf("off AI Path Array %d : 0x%x\n", i, navTable.offAIPathArray[i]);
				file.seekg(offLev + std::streampos(navTable.offAIPathArray[i]));
				PSX::NavHeader navHeader{};
				Read(file, navHeader);

				std::vector<PSX::NavFrame> nodes;
				PSX::NavFrame startLine{};
				Read(file, startLine);
				nodes.push_back(startLine);
				for (int j = 0; j < navHeader.numPoints; j++)
				{
					PSX::NavFrame navFrame{};
					Read(file, navFrame);
					nodes.push_back(navFrame);
					//printf("Pos %d : x=%d, y=%d, z=%d\n", j, navFrame.pos.x, navFrame.pos.y, navFrame.pos.z);
					//printf("Rot %d : %d, %d, %d, %d\n", j, navFrame.rot[0], navFrame.rot[1], navFrame.rot[2], navFrame.rot[3]);
				}
				m_botPaths[i] = BotPath(navHeader, nodes);
			}
		}
		UpdateRenderBotData();
	}

	// Collect instance ptr
	std::vector<uint32_t> instPtrs;
	if (header.numInstances > 0 && header.offInstancePtrArray != 0)
	{
		file.seekg(offLev + std::streampos(header.offInstancePtrArray));
		instPtrs.resize(header.numInstances);
		for (uint32_t i = 0; i < header.numInstances; i++)
		{
			Read(file, instPtrs[i]);
		}
	}

	// Load instances from .lev
	std::unordered_map<uint32_t, size_t> offsetToInstancesID;
	file.clear();
	if (header.numInstances > 0 && header.offInstancePtrArray != 0)
	{
		for (uint32_t i = 0; i < header.numInstances; i++)
		{
			if (instPtrs[i] == 0) { break; } // shouldn't be continue ?

			file.seekg(offLev + std::streampos(instPtrs[i]));
			PSX::InstDef psxInst = {};
			Read(file, psxInst);

			if (!offsetToModelKey.contains(psxInst.offModel))
				continue; // invalid model
			
			offsetToInstancesID[instPtrs[i]] = m_instances.size();
			m_instances.emplace_back(psxInst, offsetToModelKey[psxInst.offModel]);
		}
	}

	// Load instance hitbox data from BSP leaf offHitbox lists using file stream
	if (!m_instances.empty())
	{
		std::vector<const BSP*> bspLeaves = m_bsp.GetLeaves();
		for (const BSP* leaf : bspLeaves)
		{
			uint32_t offHitbox = leaf->GetOffHitbox();
			if (offHitbox == 0) continue;

			file.clear();
			// Read hitbox entries from LEV using file stream
			for (size_t hi = 0; ; hi++)
			{
				PSX::InstHitbox hitbox = {};
				file.clear();
				file.seekg(offLev + std::streampos(offHitbox) + static_cast<std::streamoff>(hi * sizeof(PSX::InstHitbox)));
				if (!file.read(reinterpret_cast<char*>(&hitbox), sizeof(PSX::InstHitbox))) break;
				if (hitbox.flags == 0 || hitbox.offInstDef == 0) break;

				if (!offsetToInstancesID.contains(hitbox.offInstDef))
					continue;
				m_instances[offsetToInstancesID[hitbox.offInstDef]].SetHitbox(hitbox);
			}
		}
	}


	m_loaded = true;
	file.close();
	GenerateRenderLevData();
	GenerateRenderInstanceData();
	return true;
}



bool Level::SaveLEV(const std::filesystem::path& path, bool useRawTextures)
{
	/*
	*	Serialization order:
	*		- offMap
	*		- LevHeader
	*		- MeshInfo
	*		- Textures
	*		- Animated Textures
	*		- Array of quadblocks
	*		- Array of VisibleSets
	*		- Array of PVS
	*		- Array of vertices
	*		- Array of BSP
	*		- Array of checkpoints
	*		- N. Tropy Ghost
	*		- N. Oxide Ghost
	*		- LevelExtraHeader
	*		- NavHeaders
	*		- VisMem
	*		- Skybox
	*		- PointerMap
	*/
	m_hotReloadLevPath = path / (m_name + ".lev");
	std::ofstream file(m_hotReloadLevPath, std::ios::binary);

	if (m_bsp.IsEmpty()) { GenerateBSP(); }
	ReOrderBSP();
	EmplaceInstanceBSP();

	std::vector<const BSP*> bspNodes = static_cast<const BSP&>(m_bsp).GetTree();
	std::set<size_t> bspIds;
	for (const BSP* bsp : bspNodes) { bspIds.insert(bsp->GetId()); }
	size_t bspcounter = 0;
	for (size_t bspid : bspIds)
	{
		if (bspcounter != bspid)
		{
			printf("BSP ID MISMATCH AT ID %zu\n", bspcounter);
		}
		bspcounter++;
	}
	std::vector<const BSP*> orderedBSPNodes(bspNodes.size());
	for (const BSP* bsp : bspNodes) { orderedBSPNodes[bsp->GetId()] = bsp; }

	PSX::LevHeader header = {};
	const size_t offHeader = 0;
	//printf(nameof(offHeader) " = %zx\n", offHeader);
	size_t currOffset = sizeof(header);

	PSX::MeshInfo meshInfo = {};
	const size_t offMeshInfo = currOffset;
	//printf(nameof(offMeshInfo) " = %zx\n", offMeshInfo);
	currOffset += sizeof(meshInfo);

	const size_t offTexture = currOffset;
	//printf(nameof(offTexture) " = %zx\n", offTexture);
	size_t offAnimData = 0;

	PSX::TextureLayout defaultTex = {};
	defaultTex.clut.self = 32 | (20 << 6);
	defaultTex.texPage.self = (512 >> 6) | ((0 >> 8) << 4) | (0 << 5) | (0 << 7);
	defaultTex.u0 = 0;		defaultTex.v0 = 0;
	defaultTex.u1 = 15;		defaultTex.v1 = 0;
	defaultTex.u2 = 0;		defaultTex.v2 = 15;
	defaultTex.u3 = 15;		defaultTex.v3 = 15;

	PSX::TextureGroup defaultTexGroup = {};
	defaultTexGroup.far = defaultTex;
	defaultTexGroup.middle = defaultTex;
	defaultTexGroup.near = defaultTex;
	defaultTexGroup.mosaic = defaultTex;

	std::vector<uint8_t> animData;
	std::vector<size_t> animPtrMapOffsets;
	std::vector<PSX::TextureGroup> texGroups;
	std::vector<PSX::AnimTex> animTexGroups;
	std::unordered_map<PSX::TextureLayout, size_t> savedLayouts;
	std::vector<PSX::TextureLayout> modelLayouts;
	std::unordered_map<PSX::TextureLayout, size_t> modelLayoutsIndexes;

	if (useRawTextures)
	{
		std::map<uint32_t, size_t> rawOffsetRemap;
		std::map<uint32_t, size_t> rawAnimOffsetRemap;
		std::vector<std::pair<size_t, size_t>> animatedQuadFaceOffsets; // <quadIndex, faceIndex> -> animTexOffset
		std::map<std::pair<size_t, size_t>, size_t> quadFaceToAnimOffset;

		for (size_t qi = 0; qi < m_quadblocks.size(); qi++)
		{
			Quadblock& currQuad = m_quadblocks[qi];
			if (currQuad.GetAnimated())
			{
				for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
				{
					uint32_t rawTexOffset = currQuad.GetRawTexOffset(i);
					uint32_t animTexKey = rawTexOffset - 1;

					if (!m_rawAnimTex.contains(animTexKey))
					{
						// This face is not animated, treat it as a static texture
						if (!rawOffsetRemap.contains(rawTexOffset))
						{
							rawOffsetRemap[rawTexOffset] = texGroups.size();
							if (!m_rawTextureGroup.contains(rawTexOffset))
							{
								printf("MISSING TEXTURE FOR %s FACE %zu\n", currQuad.GetName().c_str(), i);
							}
							texGroups.push_back(m_rawTextureGroup[rawTexOffset]);
						}
						currQuad.SetTextureID(rawOffsetRemap[rawTexOffset], i);
						continue;
					}

					const PSX::AnimTex& animTex = m_rawAnimTex[animTexKey];
					const std::vector<uint32_t>& frameOffsets = m_rawAnimTexFrames[animTexKey];

					std::vector<size_t> remappedFrameIndexes;
					for (uint32_t frameRawOffset : frameOffsets)
					{
						if (!rawOffsetRemap.contains(frameRawOffset))
						{
							rawOffsetRemap[frameRawOffset] = texGroups.size();
							if (!m_rawTextureGroup.contains(frameRawOffset))
							{
								printf("MISSING FRAME TEXTURE FOR %s FACE %zu FRAME OFFSET %u\n",
									currQuad.GetName().c_str(), i, frameRawOffset);
							}
							texGroups.push_back(m_rawTextureGroup[frameRawOffset]);
						}
						remappedFrameIndexes.push_back(rawOffsetRemap[frameRawOffset]);
					}

					if (i == NUM_FACES_QUADBLOCK)
					{
						currQuad.SetTextureID(remappedFrameIndexes[0], i);
						continue;
					}

					if (!rawAnimOffsetRemap.contains(animTexKey))
					{
						size_t animTexOffset = animData.size();
						rawAnimOffsetRemap[animTexKey] = animTexOffset;
						animPtrMapOffsets.push_back(animTexOffset);

						PSX::AnimTex rawAnimTex = animTex;
						rawAnimTex.offActiveFrame = static_cast<uint32_t>(
							offTexture + (remappedFrameIndexes[0] * sizeof(PSX::TextureGroup)));
						animData.resize(animData.size() + sizeof(PSX::AnimTex));
						memcpy(&animData[animTexOffset], &rawAnimTex, sizeof(PSX::AnimTex));

						for (size_t j = 0; j < remappedFrameIndexes.size(); j++)
						{
							uint32_t offset = static_cast<uint32_t>(
								(remappedFrameIndexes[j] * sizeof(PSX::TextureGroup)) + offTexture);
							size_t offAnimTexArr = animData.size();
							animPtrMapOffsets.push_back(offAnimTexArr);
							for (size_t k = 0; k < sizeof(uint32_t); k++) { animData.push_back(0); }
							memcpy(&animData[offAnimTexArr], &offset, sizeof(uint32_t));
						}
					}

					quadFaceToAnimOffset[{qi, i}] = rawAnimOffsetRemap[animTexKey];
				}
				continue;
			}
			for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
			{
				uint32_t rawTexOffset = currQuad.GetRawTexOffset(i);
				if (!rawOffsetRemap.contains(rawTexOffset))
				{
					rawOffsetRemap[rawTexOffset] = texGroups.size();
					if (!m_rawTextureGroup.contains(rawTexOffset)) { printf("MISSING TEXTURE FOR %s FACE %zu\n", currQuad.GetName().c_str(), i); }
					texGroups.push_back(m_rawTextureGroup[rawTexOffset]);
				}
				currQuad.SetTextureID(rawOffsetRemap[rawTexOffset], i);
			}
		}

		//texGroups.push_back(defaultTexGroup);
		offAnimData = currOffset + (sizeof(PSX::TextureGroup) * texGroups.size());

		// Second pass: now offAnimData is known
		for (auto& [quadFace, animTexOffset] : quadFaceToAnimOffset)
		{
			m_quadblocks[quadFace.first].SetAnimTextureOffset(animTexOffset, offAnimData, quadFace.second);
		}

		animPtrMapOffsets.push_back(animData.size());
		size_t offEndAnimData = animData.size();
		for (size_t i = 0; i < sizeof(uint32_t); i++) { animData.push_back(0); }
		memcpy(&animData[offEndAnimData], &offAnimData, sizeof(uint32_t));
	}
	else
	{
		if (UpdateVRM())
		{
			for (auto& [material, texture] : m_materialToTexture)
			{
				std::vector<size_t>& quadIndexes = m_materialToQuadblocks[material];
				for (size_t index : quadIndexes)
				{
					Quadblock& currQuad = m_quadblocks[index];
					if (currQuad.GetAnimated()) { continue; }
					for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
					{
						size_t textureID = 0;
						const QuadUV& uvs = currQuad.GetQuadUV(i);
						PSX::TextureLayout layout = texture.Serialize(uvs);
						if (savedLayouts.contains(layout)) { textureID = savedLayouts[layout]; }
						else
						{
							textureID = texGroups.size();
							savedLayouts[layout] = textureID;

							PSX::TextureGroup texGroup = {};
							texGroup.far = layout;
							texGroup.middle = layout;
							texGroup.near = layout;
							texGroup.mosaic = layout;
							texGroups.push_back(texGroup);
						}
						currQuad.SetTextureID(textureID, i);
					}
				}


			}

			if (!m_animTextures.empty())
			{
				std::vector<std::array<size_t, NUM_FACES_QUADBLOCK>> animOffsetPerQuadblock;
				for (AnimTexture& animTex : m_animTextures)
				{
					const std::vector<AnimTextureFrame>& animFrames = animTex.GetFrames();
					const std::vector<Texture>& animTextures = animTex.GetTextures();
					std::vector<std::vector<size_t>> texgroupIndexesPerFrame(NUM_FACES_QUADBLOCK);
					bool firstFrame = true;
					for (const AnimTextureFrame& frame : animFrames)
					{
						Texture& texture = const_cast<Texture&>(animTextures[frame.textureIndex]);
						for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
						{
							if (i == NUM_FACES_QUADBLOCK && !firstFrame) { continue; }
							size_t textureID = 0;
							const QuadUV& uvs = frame.uvs[i];
							PSX::TextureLayout layout = texture.Serialize(uvs);
							if (savedLayouts.contains(layout)) { textureID = savedLayouts[layout]; }
							else
							{
								textureID = texGroups.size();
								savedLayouts[layout] = textureID;

								PSX::TextureGroup texGroup = {};
								texGroup.far = layout;
								texGroup.middle = layout;
								texGroup.near = layout;
								texGroup.mosaic = layout;
								texGroups.push_back(texGroup);
							}
							if (firstFrame && i == NUM_FACES_QUADBLOCK)
							{
								const std::vector<size_t>& quadblockIndexes = animTex.GetQuadblockIndexes();
								for (size_t index : quadblockIndexes)
								{
									m_quadblocks[index].SetTextureID(textureID, i);
								}
							}
							else { texgroupIndexesPerFrame[i].push_back(textureID); }
						}
						firstFrame = false;
					}
					std::array<size_t, NUM_FACES_QUADBLOCK> offsetPerQuadblock = {};
					for (size_t i = 0; i < NUM_FACES_QUADBLOCK; i++)
					{
						bool foundEquivalent = false;
						for (size_t j = 0; j < i; j++)
						{
							if (texgroupIndexesPerFrame[i] == texgroupIndexesPerFrame[j])
							{
								offsetPerQuadblock[i] = offsetPerQuadblock[j];
								foundEquivalent = true;
								break;
							}
						}
						if (foundEquivalent) { continue; }
						std::vector<uint8_t> buffer = animTex.Serialize(texgroupIndexesPerFrame[i][0], offTexture);
						size_t animTexOffset = animData.size();
						offsetPerQuadblock[i] = animTexOffset;
						animPtrMapOffsets.push_back(animTexOffset);
						for (uint8_t byte : buffer) { animData.push_back(byte); }
						for (size_t j = 0; j < animFrames.size(); j++)
						{
							uint32_t offset = static_cast<uint32_t>((texgroupIndexesPerFrame[i][j] * sizeof(PSX::TextureGroup)) + offTexture);
							size_t offAnimTexArr = animData.size();
							animPtrMapOffsets.push_back(offAnimTexArr);
							for (size_t k = 0; k < sizeof(uint32_t); k++) { animData.push_back(0); }
							memcpy(&animData[offAnimTexArr], &offset, sizeof(uint32_t));
						}
					}
					animOffsetPerQuadblock.push_back(offsetPerQuadblock);
				}

  			offAnimData = currOffset + (sizeof(PSX::TextureGroup) * texGroups.size());
	  		//printf(nameof(offAnimData) " = %zx\n", offAnimData);

				animPtrMapOffsets.push_back(animData.size());
				size_t offEndAnimData = animData.size();
				for (size_t i = 0; i < sizeof(uint32_t); i++) { animData.push_back(0); }
				memcpy(&animData[offEndAnimData], &offAnimData, sizeof(uint32_t));

				for (size_t i = 0; i < m_animTextures.size(); i++)
				{
					const std::vector<size_t>& quadblockIndexes = m_animTextures[i].GetQuadblockIndexes();
					for (size_t index : quadblockIndexes)
					{
						Quadblock& quadblock = m_quadblocks[index];
						for (size_t j = 0; j < NUM_FACES_QUADBLOCK; j++)
						{
							quadblock.SetAnimTextureOffset(animOffsetPerQuadblock[i][j], offAnimData, j);
						}
					}
				}
			}
			else
			{
				offAnimData = currOffset + (sizeof(PSX::TextureGroup) * texGroups.size());
				for (size_t i = 0; i < sizeof(uint32_t); i++) { animData.push_back(0); }
				memcpy(&animData[0], &offAnimData, sizeof(uint32_t));
				animPtrMapOffsets.push_back(0);
			}

			m_hotReloadVRMPath = path / (m_name + ".vrm");
			std::ofstream vrmFile(m_hotReloadVRMPath, std::ios::binary);
			Write(vrmFile, m_vrm.data(), m_vrm.size());
			vrmFile.close();
		}
		else
		{
			texGroups.push_back(defaultTexGroup);
			offAnimData = currOffset + (sizeof(PSX::TextureGroup) * texGroups.size());
			//printf(nameof(offAnimData) " = %zx\n", offAnimData);
			for (size_t i = 0; i < sizeof(uint32_t); i++) { animData.push_back(0); }
			memcpy(&animData[0], &offAnimData, sizeof(uint32_t));
			animPtrMapOffsets.push_back(0);
		}
	}
	

	currOffset += (sizeof(PSX::TextureGroup) * texGroups.size()) + animData.size();


	PSX::TextureLayout envMapLayout{}; // must be 64x64, uvs don't matter
	if (useRawTextures)
		envMapLayout = m_rawWaterLayout;
	else
	{
		Texture& tex = m_materialToTexture[m_envMapMatName];
		if (!tex.IsEmpty())
		{
			envMapLayout = tex.Serialize(QuadUV{});
		}
	}
	const size_t offEnvMapLayout = currOffset;
	currOffset += sizeof(PSX::TextureLayout);

	const size_t offQuadblocks = currOffset;
	//printf(nameof(offQuadblocks) " = %zx\n", offQuadblocks);
	std::vector<std::vector<uint8_t>> serializedBSPs;
	std::vector<std::vector<uint8_t>> serializedQuads;
	std::vector<const Quadblock*> orderedQuads;
	std::unordered_map<Vertex, size_t> vertexMap;
	std::unordered_map<PSX::OceanVertex, size_t> oVertexMap;
	std::vector<Vertex> orderedVertices;
	std::vector<PSX::OceanVertex> orderedOVert;
	std::set<std::tuple<size_t, size_t>> waterVerticesIndexes; // list of tuple (Vertex id, OVert id)
	size_t bspSize = 0;
	for (const BSP* bsp : orderedBSPNodes)
	{
		serializedBSPs.push_back(bsp->Serialize(currOffset, m_quadblocks));
		bspSize += serializedBSPs.back().size();
		if (bsp->IsBranch()) { continue; }
		const std::vector<size_t>& quadIndexes = bsp->GetQuadblockIndexes();
		for (const size_t index : quadIndexes)
		{
			const Quadblock& quadblock = m_quadblocks[index];
			std::vector<Vertex> quadVertices = quadblock.GetVertices();
			std::vector<size_t> verticesIndexes;
			for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK ;i++)
			{
				const Vertex& vertex = quadVertices[i];
				if (!vertexMap.contains(vertex))
				{
					size_t vertexIndex = orderedVertices.size();
					orderedVertices.push_back(vertex);
					vertexMap[vertex] = vertexIndex;
				}
				verticesIndexes.push_back(vertexMap[vertex]);
				if (quadblock.GetWater())
				{
					PSX::OceanVertex oVert = quadblock.GetOceanVertex(i);
					if (!oVertexMap.contains(oVert))
					{
						size_t vertexIndex = orderedOVert.size();
						orderedOVert.push_back(oVert);
						oVertexMap[oVert] = vertexIndex;
					}
					waterVerticesIndexes.insert(std::make_tuple(vertexMap[vertex], oVertexMap[oVert]));
				}
			}
			size_t quadIndex = serializedQuads.size();
			serializedQuads.push_back(quadblock.Serialize(quadIndex, offTexture, verticesIndexes));
			orderedQuads.push_back(&quadblock);
			currOffset += serializedQuads.back().size();
		}
	}

	constexpr size_t BITS_PER_SLOT = sizeof(uint32_t) * 8;
	std::vector<std::tuple<std::vector<uint32_t>, size_t>> visibleNodes;
	std::vector<std::vector<uint32_t>> uniqueVisNodes;
	std::map<std::vector<uint32_t>, size_t> visNodesOffsetMap;
	std::vector<std::tuple<std::vector<uint32_t>, size_t>> visibleQuads;
	std::vector<std::vector<uint32_t>> uniqueVisQuads;
	std::map<std::vector<uint32_t>, size_t> visQuadsOffsetMap;
	std::vector<std::tuple<std::vector<uint32_t>, size_t>> visibleInstances;
	std::vector<std::tuple<std::vector<uint32_t>, size_t>> visibleExtra;
	size_t visNodeSize = static_cast<size_t>(std::ceil(static_cast<float>(bspNodes.size()) / static_cast<float>(BITS_PER_SLOT)));
	size_t visQuadSize = static_cast<size_t>(std::ceil(static_cast<float>(m_quadblocks.size()) / static_cast<float>(BITS_PER_SLOT)));
	size_t visExtraSize = static_cast<size_t>(std::ceil(static_cast<float>(waterVerticesIndexes.size()) / static_cast<float>(BITS_PER_SLOT)));
	std::vector<uint32_t> visibleExtraAll(visExtraSize, 0xFFFFFFFF);
	std::vector<uint32_t> visibleNodeAll(visNodeSize, 0xFFFFFFFF);
	for (const BSP* bsp : orderedBSPNodes)
	{
		if (bsp->GetFlags() & BSPFlags::INVISIBLE) { visibleNodeAll[bsp->GetId() / BITS_PER_SLOT] &= ~(1 << (bsp->GetId() % BITS_PER_SLOT)); }
	}

	std::vector<uint32_t> visibleQuadsAll(visQuadSize, 0xFFFFFFFF);
	size_t quadIndex = 0;
	for (const Quadblock* quad : orderedQuads)
	{
		if (quad->GetFlags() & QuadFlags::INVISIBLE_TRIGGER)
		{
			visibleQuadsAll[quadIndex / BITS_PER_SLOT] &= ~(1 << (quadIndex % BITS_PER_SLOT));
		}
		quadIndex++;
	}
	
	const bool validVisTree = !m_bspVis.IsEmpty();
	const std::vector<const BSP*> bspLeaves = m_bsp.GetLeaves();
	std::unordered_map<size_t, const BSP*> idToLeaf;
	std::unordered_map<const BSP*, size_t> leafToMatrix;
	for (const BSP* leaf : bspLeaves) { idToLeaf[leaf->GetId()] = leaf; }
	for (size_t i = 0; i < bspLeaves.size(); i++) { leafToMatrix[bspLeaves[i]] = i; }
	if (validVisTree)
	{
		for (const Quadblock* quad : orderedQuads)
		{
			std::vector<uint32_t> visNodes(visNodeSize, 0x0);
			const BSP* bspLeaf = idToLeaf[quad->GetBSPID()];
			const size_t matrixId = leafToMatrix[bspLeaf];
			for (size_t i = 0; i < bspLeaves.size(); i++)
			{
				if (m_bspVis.Get(matrixId, i))
				{
					const BSP* curr = bspLeaves[i];
					while (curr != nullptr)
					{
						visNodes[curr->GetId() / BITS_PER_SLOT] |= (1 << (31 - (curr->GetId() % BITS_PER_SLOT)));
						curr = curr->GetParent();
					}
				}
			}
			if (visNodesOffsetMap.contains(visNodes))
			{
				visibleNodes.push_back({ visNodes, visNodesOffsetMap.at(visNodes) });
			}
			else
			{
				visNodesOffsetMap[visNodes] = currOffset;
				visibleNodes.push_back({ visNodes, currOffset });
				uniqueVisNodes.push_back(visNodes);
				currOffset += visNodes.size() * sizeof(uint32_t);
			}
		}
		for (const Quadblock* quad : orderedQuads)
		{
			std::vector<uint32_t> visQuads(visQuadSize, 0x0);
			const BSP* bspLeaf = idToLeaf[quad->GetBSPID()];
			const size_t matrixId = leafToMatrix[bspLeaf];
			/*for (size_t i = 0; i < bspLeaves.size(); i++)
			{
				if (m_bspVis.Get(matrixId, i))
				{
					quadIndex = 0;
					for (const Quadblock* quad : orderedQuads)
					{
						if (leafToMatrix[idToLeaf[quad->GetBSPID()]] == i)
						{
							visQuads[quadIndex / BITS_PER_SLOT] |= (1 << (quadIndex % BITS_PER_SLOT));
						}
						quadIndex++;
					}
				}
			}*/
			visQuads = visibleQuadsAll; //Saves space, doesn't seem to cost performances.
			if (visQuadsOffsetMap.contains(visQuads))
			{
				visibleQuads.push_back({ visQuads, visQuadsOffsetMap.at(visQuads) });
			}
			else
			{
				visQuadsOffsetMap[visQuads] = currOffset;
				visibleQuads.push_back({ visQuads, currOffset });
				uniqueVisQuads.push_back(visQuads);
				currOffset += visQuads.size() * sizeof(uint32_t);
			}
		}
	}
	else // not valid vistree
	{
		visibleNodes.push_back({visibleNodeAll, currOffset});
		uniqueVisNodes.push_back(visibleNodeAll);
		currOffset += visibleNodeAll.size() * sizeof(uint32_t);

		visibleQuads.push_back({ visibleQuadsAll, currOffset });
		uniqueVisQuads.push_back(visibleQuadsAll);
		currOffset += visibleQuadsAll.size() * sizeof(uint32_t);
	}
	printf("visibleNodesOffsetMapSize %zu\n", visNodesOffsetMap.size());
	printf("visibleQuadsOffsetMapSize %zu\n", visQuadsOffsetMap.size());

	std::vector<uint32_t> visibleInstancesDummy;
	visibleInstancesDummy.push_back(0xFFFFFFFF);
	visibleInstances.push_back({visibleInstancesDummy, currOffset});
	currOffset += visibleInstancesDummy.size() * sizeof(uint32_t);


	visibleExtra.push_back({ visibleExtraAll, currOffset });
	currOffset += visibleExtraAll.size() * sizeof(uint32_t);


	std::unordered_map<PSX::VisibleSet, size_t> visibleSetMap;
	std::vector<PSX::VisibleSet> visibleSets;
	const size_t offVisibleSet = currOffset;
	//printf(nameof(offVisibleSet) " = %zx\n", offVisibleSet);

	for (size_t quadCount = 0; quadCount < orderedQuads.size(); quadCount++)
	{
		PSX::VisibleSet set = {};
		if (validVisTree) 
		{ 
			set.offVisibleBSPNodes = static_cast<uint32_t>(std::get<size_t>(visibleNodes[quadCount]));
			set.offVisibleQuadblocks = static_cast<uint32_t>(std::get<size_t>(visibleQuads[quadCount]));
		}
		else 
		{ 
			set.offVisibleBSPNodes = static_cast<uint32_t>(std::get<size_t>(visibleNodes[0])); 
			set.offVisibleQuadblocks = static_cast<uint32_t>(std::get<size_t>(visibleQuads[0]));
		}
		if (orderedQuads.size() % 2 == 0 && quadCount == 0)
			set.offVisibleInstances = static_cast<uint32_t>(0);
		else
			set.offVisibleInstances = static_cast<uint32_t>(1);
		set.offVisibleExtra = static_cast<uint32_t>(std::get<size_t>(visibleExtra[0]));

		size_t visibleSetIndex = 0;
		if (visibleSetMap.contains(set)) { visibleSetIndex = visibleSetMap.at(set); }
		else
		{
			visibleSetIndex = visibleSets.size();
			visibleSets.push_back(set);
			visibleSetMap[set] = visibleSetIndex;
		}

		PSX::Quadblock* serializedQuad = reinterpret_cast<PSX::Quadblock*>(serializedQuads[quadCount].data());
		serializedQuad->offVisibleSet = static_cast<uint32_t>(offVisibleSet + sizeof(PSX::VisibleSet) * visibleSetIndex);
	}
	//printf("visibleSetsSize %d\n", visibleSets.size());
	currOffset += visibleSets.size() * sizeof(PSX::VisibleSet);

	const size_t offWaterVertices = currOffset;
	std::vector<PSX::WaterVertex> waterVertices;
	std::vector<std::tuple<size_t, size_t >> orderedWaterVerticesIndexes;

	for (auto& tuple : waterVerticesIndexes)
	{
		PSX::WaterVertex waterVert{};
		waterVertices.push_back(waterVert);
		currOffset += sizeof(waterVert);
		orderedWaterVerticesIndexes.push_back(tuple);
	}
	
	const size_t offVertices = currOffset;
	//printf(nameof(offVertices) " = %zx\n", offVertices);
	std::vector<std::vector<uint8_t>> serializedVertices;
	for (const Vertex& vertex : orderedVertices)
	{
		serializedVertices.push_back(vertex.Serialize());
		currOffset += serializedVertices.back().size();
	}
	

	for (size_t i = 0; i < waterVertices.size(); i++)
	{

		std::tuple<size_t, size_t >& tuple = orderedWaterVerticesIndexes[i];
		size_t vertID = std::get<0>(tuple);
		waterVertices[i].offVertex = static_cast<uint32_t>(offVertices + vertID * sizeof(PSX::Vertex));
	}
	

	const size_t offOverts = currOffset;
	std::vector<std::vector<uint8_t>> serializedOVertices;
	std::vector<uint32_t> oVertOffsets(orderedOVert.size());

	for (size_t i = 0; i < orderedOVert.size(); i++)
	{
		PSX::OceanVertex& overt = orderedOVert[i];
		std::vector<uint8_t> buffer(sizeof(overt));
		std::memcpy(buffer.data(), &overt, sizeof(overt));
		oVertOffsets[i] = static_cast<uint32_t>(currOffset);
		serializedOVertices.push_back(buffer);
		currOffset += serializedOVertices.back().size();
	}

	for (size_t i = 0; i < waterVertices.size(); i++)
	{
		std::tuple<size_t, size_t>& tuple = orderedWaterVerticesIndexes[i];
		size_t oVertID = std::get<1>(tuple);
		waterVertices[i].offOceanVertex = oVertOffsets[oVertID];
	}

	const size_t offBSP = currOffset;
	//printf(nameof(offBSP) " = %zx\n", offBSP);
	currOffset += bspSize;

	meshInfo.numQuadblocks = static_cast<uint32_t>(serializedQuads.size());
	meshInfo.numVertices = static_cast<uint32_t>(serializedVertices.size());
	meshInfo.offQuadblocks = static_cast<uint32_t>(offQuadblocks);
	meshInfo.offVertices = static_cast<uint32_t>(offVertices);
	meshInfo.unk1 = 0;
	meshInfo.unk2 = 0;
	meshInfo.offBSPNodes = static_cast<uint32_t>(offBSP);
	meshInfo.numBSPNodes = static_cast<uint32_t>(serializedBSPs.size());

	const size_t offCheckpoints = currOffset;
	//printf(nameof(offCheckpoints) " = %zx\n", offCheckpoints);
	std::vector<std::vector<uint8_t>> serializedCheckpoints;
	for (const Checkpoint& checkpoint : m_checkpoints)
	{
		serializedCheckpoints.push_back(checkpoint.Serialize());
		currOffset += serializedCheckpoints.back().size();
	}

	const size_t offTropyGhost = m_tropyGhost.empty() ? 0 : currOffset;
	//printf(nameof(offTropyGhost) " = %zx\n", offTropyGhost);
	currOffset += m_tropyGhost.size();

	const size_t offOxideGhost = m_oxideGhost.empty() ? 0 : currOffset;
	//printf(nameof(offOxideGhost) " = %zx\n", offOxideGhost);
	currOffset += m_oxideGhost.size();



	// INSTANCE THAT NEED IT : flamejet (i assume the fire totem), orca, armadillo and plant

	constexpr size_t SPAWN_META_ENTRY_COUNT = 20; // covers plant's metaArray[digit * 2 + 1] for digits 0-9
	const std::vector<int16_t> spawnMeta(SPAWN_META_ENTRY_COUNT, 0);
	const size_t offSpawnMeta = currOffset;
	printf(nameof(offSpawnMeta) " = %zx\n", offSpawnMeta);
	currOffset += spawnMeta.size() * sizeof(int16_t);


	// TODO : VERIFY THIS PART AND MERGE CORRECTLY
	// Note: extraHeader.offsets[MINIMAP] will be updated later after minimap data is serialized
	PSX::LevelExtraHeader extraHeader = {};
	extraHeader.count = 0;
	extraHeader.offsets[PSX::LevelExtra::MINIMAP] = 0;
	extraHeader.offsets[PSX::LevelExtra::SPAWN] = static_cast<uint32_t>(offSpawnMeta);
	extraHeader.offsets[PSX::LevelExtra::CAMERA_END_OF_RACE] = 0;
	extraHeader.offsets[PSX::LevelExtra::CAMERA_DEMO] = 0;
	extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST] = static_cast<uint32_t>(offTropyGhost);
	extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] = static_cast<uint32_t>(offOxideGhost);
	extraHeader.offsets[PSX::LevelExtra::CREDITS] = 0;
	// count = number of valid entries in offsets[]

	if (extraHeader.offsets[PSX::LevelExtra::SPAWN] && extraHeader.count < PSX::LevelExtra::SPAWN + 1)
		extraHeader.count = PSX::LevelExtra::SPAWN + 1;
	if (extraHeader.offsets[PSX::LevelExtra::CAMERA_END_OF_RACE] && extraHeader.count < PSX::LevelExtra::CAMERA_END_OF_RACE + 1)
		extraHeader.count = PSX::LevelExtra::CAMERA_END_OF_RACE + 1;
	if (extraHeader.offsets[PSX::LevelExtra::CAMERA_DEMO] && extraHeader.count < PSX::LevelExtra::CAMERA_DEMO + 1)
		extraHeader.count = PSX::LevelExtra::CAMERA_DEMO + 1;
	if (extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST] && extraHeader.count < PSX::LevelExtra::N_TROPY_GHOST + 1)
		extraHeader.count = PSX::LevelExtra::N_TROPY_GHOST + 1;
	if (extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] && extraHeader.count < PSX::LevelExtra::N_OXIDE_GHOST + 1)
		extraHeader.count = PSX::LevelExtra::N_OXIDE_GHOST + 1;
	if (extraHeader.offsets[PSX::LevelExtra::CREDITS] && extraHeader.count < PSX::LevelExtra::CREDITS + 1)
		extraHeader.count = PSX::LevelExtra::CREDITS + 1;
	const size_t offExtraHeader = currOffset;
	//printf(nameof(offExtraHeader) " = %zx\n", offExtraHeader);
	currOffset += sizeof(extraHeader);


	size_t offSpawnType = currOffset;
	std::vector<PSX::SpawnType2> serializedSpawnTypeHeader;
	std::vector<std::vector<PSX::Vec3>> serializedSpawnTypes;
	serializedSpawnTypes.resize(m_spawntypes.size());
	for (std::vector<Vec3>& spawnType : m_spawntypes)
	{
		PSX::SpawnType2 psxSpawnType{};
		psxSpawnType.numCoord = static_cast<uint32_t>(spawnType.size());
		serializedSpawnTypeHeader.push_back(psxSpawnType);
		currOffset += sizeof(psxSpawnType);
	}
	int spawnTypePosCount = 0;
	for (size_t i = 0; i < serializedSpawnTypeHeader.size(); i++)
	{
		serializedSpawnTypeHeader[i].offPos = static_cast<uint32_t>(currOffset);
		std::vector<Vec3>& spawnType = m_spawntypes[i];
		for (Vec3& pos : spawnType)
		{
			serializedSpawnTypes[i].push_back(ConvertVec3(pos, FP_ONE_GEO));
			currOffset += sizeof(PSX::Vec3);
			spawnTypePosCount++;
		}
	}
	std::vector<uint16_t> spawnTypePosPadding;
	if (spawnTypePosCount % 2)
	{
		spawnTypePosPadding.push_back(0);
		currOffset += sizeof(uint16_t);
	}

	


	size_t offSpawnTypePosRot = currOffset;
	std::vector<PSX::SpawnType2> serializedSpawnTypePosRotHeader;
	std::vector<std::vector<PSX::Spawn>> serializedSpawnPosRotTypes;
	serializedSpawnPosRotTypes.resize(m_spawntypesPosRot.size());
	for (std::vector<Spawn>& spawnType : m_spawntypesPosRot)
	{
		PSX::SpawnType2 psxSpawnType{};
		psxSpawnType.numCoord = static_cast<uint32_t>(spawnType.size());
		serializedSpawnTypePosRotHeader.push_back(psxSpawnType);
		currOffset += sizeof(psxSpawnType);
	}
	for (size_t i = 0; i < serializedSpawnTypePosRotHeader.size(); i++)
	{
		serializedSpawnTypePosRotHeader[i].offPos = static_cast<uint32_t>(currOffset);
		std::vector<Spawn>& spawnType = m_spawntypesPosRot[i];
		for (Spawn& spawn : spawnType)
		{
			PSX::Spawn psxSpawn{};
			psxSpawn.pos = ConvertVec3(spawn.pos, FP_ONE_GEO);
			psxSpawn.rot = ConvertAngle(spawn.rot);
			serializedSpawnPosRotTypes[i].push_back(psxSpawn);
			currOffset += sizeof(PSX::Spawn);
		}
	}


	constexpr size_t BOT_PATH_COUNT = 3;
	PSX::levAINavTable navTable{};
	std::vector<std::vector<uint8_t>> serializedBotPaths;

	const size_t offNavTable = currOffset;
	currOffset += sizeof(navTable);

	for (int i = 0; i < BOT_PATH_COUNT; i++)
	{
		if (m_botPaths[i].IsValid())
		{
			navTable.offAIPathArray[i] = static_cast<uint32_t>(currOffset);
			serializedBotPaths.push_back(m_botPaths[i].Serialize(m_instances));
			currOffset += serializedBotPaths.back().size();
		}
		else
		{
			navTable.offAIPathArray[i] = 0;
		}
	}


	std::vector<uint32_t> visMemNodesP1(visNodeSize);
	const size_t offVisMemNodesP1 = currOffset;
	//printf(nameof(offVisMemNodesP1) " = %zx\n", offVisMemNodesP1);
	currOffset += visMemNodesP1.size() * sizeof(uint32_t);

	std::vector<uint32_t> visMemQuadsP1(visQuadSize);
	const size_t offVisMemQuadsP1 = currOffset;
	//printf(nameof(offVisMemQuadsP1) " = %zx\n", offVisMemQuadsP1);
	currOffset += visMemQuadsP1.size() * sizeof(uint32_t);

	std::vector<uint32_t> visMemBSPP1(bspNodes.size() * 2);
	const size_t offVisMemBSPP1 = currOffset;
	//printf(nameof(offVisMemBSPP1) " = %zx\n", offVisMemBSPP1);
	currOffset += visMemBSPP1.size() * sizeof(uint32_t);

	std::vector<uint32_t> visMemOceanP1(visQuadSize);
	const size_t offvisMemOceanP1 = currOffset;
	currOffset += visMemOceanP1.size() * sizeof(uint32_t);

	PSX::VisualMem visMem = {};
	visMem.offNodes[0] = static_cast<uint32_t>(offVisMemNodesP1);
	visMem.offQuads[0] = static_cast<uint32_t>(offVisMemQuadsP1);
	visMem.offBSP[0] = static_cast<uint32_t>(offVisMemBSPP1);
	visMem.offOcean[0] = static_cast<uint32_t>(offVisMemBSPP1);
	const size_t offVisMem = currOffset;
  //printf(nameof(offVisMem) " = %zx\n", offVisMem);
	currOffset += sizeof(visMem);

	// Minimap data serialization
	size_t offMinimapStruct = 0;
	size_t offLevelIconHeader = 0;
	size_t offMinimapIcons = 0;
	std::vector<uint8_t> minimapData;
	std::vector<size_t> minimapPtrMapOffsets;

	if (!m_minimapConfig.texture.IsEmpty())
	{
		// Map struct - this is what extraHeader.offsets[MINIMAP] will point to
		offMinimapStruct = currOffset;
		PSX::Map mapStruct = m_minimapConfig.Serialize();
		size_t mapStructOffset = minimapData.size();
		minimapData.resize(minimapData.size() + sizeof(PSX::Map));
		memcpy(&minimapData[mapStructOffset], &mapStruct, sizeof(PSX::Map));
		currOffset += sizeof(PSX::Map);

		// Icon structs (top and bottom minimap textures)
		offMinimapIcons = currOffset;

		// IMPORTANT NOTE : WE NEED TOP AND BOTTOM TEXTURE TO BE THE SAME SIZE, BUT MINIMAP HAVE ODD HEIGHT : 
		// WHAT ND DID IS ADD 1 ROW OF PIXEL AT THE BOTTOM OF THE TOP TEXTURE 
		QuadUV topUV = {{Vec2(0.0f, 0.0f), Vec2(1.0f, 0.0f), Vec2(0.0f, 0.5001f), Vec2(1.0f, 0.5001f)}};
		QuadUV bottomUV = {{Vec2(0.0f, 0.4999f), Vec2(1.0f, 0.4999f), Vec2(0.0f, 1.0f), Vec2(1.0f, 1.0f)}};

		// Top icon
		PSX::Icon topIcon = {};
		strncpy_s(topIcon.name, sizeof(topIcon.name), "minimap-top", _TRUNCATE);
		topIcon.globalIconArrayIndex = PSX::ICON_INDEX_MAP_TOP;
		topIcon.texLayout = m_minimapConfig.texture.Serialize(topUV);
		size_t topIconOffset = minimapData.size();
		minimapData.resize(minimapData.size() + sizeof(PSX::Icon));
		memcpy(&minimapData[topIconOffset], &topIcon, sizeof(PSX::Icon));
		currOffset += sizeof(PSX::Icon);

		// Bottom icon
		PSX::Icon bottomIcon = {};
		strncpy_s(bottomIcon.name, sizeof(bottomIcon.name), "minimap-bot", _TRUNCATE);
		bottomIcon.globalIconArrayIndex = PSX::ICON_INDEX_MAP_BOTTOM;
		bottomIcon.texLayout = m_minimapConfig.texture.Serialize(bottomUV);
		size_t bottomIconOffset = minimapData.size();
		minimapData.resize(minimapData.size() + sizeof(PSX::Icon));
		memcpy(&minimapData[bottomIconOffset], &bottomIcon, sizeof(PSX::Icon));
		currOffset += sizeof(PSX::Icon);

		// LevelIconHeader struct (pointed to by header.offIconsLookup)
		offLevelIconHeader = currOffset;
		PSX::LevelIconHeader levelIconHeader = {};
		levelIconHeader.numIcon = 2;
		levelIconHeader.offFirstIcon = static_cast<uint32_t>(offMinimapIcons);
		levelIconHeader.numIconGroup = 0;
		levelIconHeader.offFirstIconGroupPtr = 0;
		size_t levelIconHeaderOffset = minimapData.size();
		minimapData.resize(minimapData.size() + sizeof(PSX::LevelIconHeader));
		memcpy(&minimapData[levelIconHeaderOffset], &levelIconHeader, sizeof(PSX::LevelIconHeader));
		minimapPtrMapOffsets.push_back(currOffset + offsetof(PSX::LevelIconHeader, offFirstIcon)); // Pointer to first icon
		currOffset += sizeof(PSX::LevelIconHeader);

		// Update extraHeader to point to the minimap Map struct directly
		extraHeader.offsets[PSX::LevelExtra::MINIMAP] = static_cast<uint32_t>(offMinimapStruct);
		if (extraHeader.offsets[PSX::LevelExtra::MINIMAP] && extraHeader.count < PSX::LevelExtra::MINIMAP + 1)
			extraHeader.count = PSX::LevelExtra::MINIMAP + 1;
	}
	
	// Skybox data serialization
	size_t offSkyboxData = 0;
	std::vector<uint8_t> skyboxData;
	std::vector<size_t> skyboxPtrMapOffsets;

	if (m_skybox.IsReady())
	{
		offSkyboxData = currOffset;
		//printf(nameof(offSkyboxData) " = %zx\n", offSkyboxData);
		skyboxData = m_skybox.Serialize(offSkyboxData, skyboxPtrMapOffsets);
		currOffset += skyboxData.size();
	}

	header.offMeshInfo = static_cast<uint32_t>(offMeshInfo);
	header.offAnimTex = static_cast<uint32_t>(offAnimData);
	for (size_t i = 0; i < NUM_DRIVERS; i++)
	{
		header.driverSpawn[i].pos = ConvertVec3(m_spawn[i].pos, FP_ONE_GEO);
		header.driverSpawn[i].rot = ConvertAngle(m_spawn[i].rot);
	}
	header.config = m_configFlags;
	for (size_t i = 0; i < NUM_GRADIENT; i++)
	{
		header.skyGradient[i].posFrom = ConvertFloat(m_skyGradient[i].posFrom, 1u);
		header.skyGradient[i].posTo = ConvertFloat(m_skyGradient[i].posTo, 1u);
		header.skyGradient[i].colorFrom = ConvertColor(m_skyGradient[i].colorFrom);
		header.skyGradient[i].colorTo = ConvertColor(m_skyGradient[i].colorTo);
	}
	header.stars = ConvertStars(m_stars);
	header.jumpYSpeedCap = static_cast<uint32_t>(m_jumpYSpeedCap);
	header.splitLines[0] = ConvertFloat(m_splitLines[0], FP_ONE_GEO);
	header.splitLines[1] = ConvertFloat(m_splitLines[1], FP_ONE_GEO);
	header.weather = ConvertWeather(m_weather);
	header.offExtra = static_cast<uint32_t>(offExtraHeader);
	header.numCheckpointNodes = static_cast<uint32_t>(m_checkpoints.size());
	header.offCheckpointNodes = static_cast<uint32_t>(offCheckpoints);
	header.offVisMem = static_cast<uint32_t>(offVisMem);
	header.offLevNavTable = static_cast<uint32_t>(offNavTable);
	header.offSpawnType_2 = static_cast<uint32_t>(offSpawnType);
	header.numSpawnType_2 = static_cast<uint32_t>(serializedSpawnTypeHeader.size());
	header.offSpawnType_2_posRot = static_cast<uint32_t>(offSpawnTypePosRot);
	header.numSpawnType_2_posRot = static_cast<uint32_t>(serializedSpawnTypePosRotHeader.size());
	header.offWaterVertices = static_cast<uint32_t>(offWaterVertices);
	header.numWaterVertices = static_cast<uint32_t>(waterVertices.size());
	header.offEnvironmentMap = static_cast<uint32_t>(offEnvMapLayout);

	// Set minimap pointers in header if enabled
	if (!m_minimapConfig.texture.IsEmpty())
	{
		header.offIconsLookup = static_cast<uint32_t>(offLevelIconHeader);
		header.offIcons = static_cast<uint32_t>(offMinimapIcons);
	}
	
	// Set skybox pointer in header if enabled
	if (m_skybox.IsReady())
	{
		header.offSkybox = static_cast<uint32_t>(offSkyboxData);
	}

	// Count unique models referenced by instances
	std::unordered_set<size_t> uniqueModelKeys;
	for (const Instance& inst : m_instances)
	{
		if (m_instanceModels[inst.GetModelKey()].IsValid())
			uniqueModelKeys.insert(inst.GetModelKey());
		else
		{
			printf("WARNING : Model %s, valid : %d, header size : %zu\n", 
				m_instanceModels[inst.GetModelKey()].GetName().c_str(), m_instanceModels[inst.GetModelKey()].IsValid(), m_instanceModels[inst.GetModelKey()].m_headers.size());
		}
	}
	header.numModels = static_cast<uint32_t>(uniqueModelKeys.size());


	// Write Model data for each unique model
	std::unordered_map<size_t, size_t> modelOffsets; // ModelKey -> Serialized ModelOffset
	std::vector<size_t> modelOrder(uniqueModelKeys.begin(), uniqueModelKeys.end());
	std::vector<std::vector<uint8_t>> serializedModels(modelOrder.size());
	std::vector<std::vector<uint32_t>> modelPointerLocations(modelOrder.size());

	for (size_t i = 0; i < modelOrder.size(); i++)
	{
		const size_t modelKey = modelOrder[i];

		InstanceModel& model = m_instanceModels[modelKey];
		const uint32_t offModel = static_cast<uint32_t>(currOffset);
		modelOffsets[modelKey] = offModel;

		serializedModels[i] = model.Serialize(offModel, m_materialToTexture, modelPointerLocations[i]);

		//printf("offModel[%s] = %zx (%zu bytes)\n", modelName.c_str(), (size_t)offModel, serializedModels[i].size());
		currOffset += serializedModels[i].size();
	}

	// Write Model pointer array (NULL-terminated)
	const size_t offModelList_ptrArray = currOffset;
	//printf(nameof(offModelList_ptrArray) " = %zx\n", offModelList_ptrArray);
	currOffset += (uniqueModelKeys.size() + 1) * sizeof(uint32_t);
	header.offModels = static_cast<uint32_t>(offModelList_ptrArray);

	
	

	// Write InstDefs
	size_t offInstDefArray = currOffset;
	std::vector<size_t> instDefOffsets;
	std::vector<std::vector<uint8_t>> serializedInstDef;
	for (size_t i = 0; i < m_instances.size(); i++)
	{
		if (!m_instanceModels[m_instances[i].GetModelKey()].IsValid())
			continue;

		uint32_t offModel = static_cast<uint32_t>(modelOffsets[m_instances[i].GetModelKey()]);
		serializedInstDef.push_back(m_instances[i].Serialize(offModel));
		const size_t offInstDef = currOffset;
		instDefOffsets.push_back(offInstDef);
		//printf("offInstDef[%zu] = %zx\n", i, offInstDef);
		currOffset += sizeof(PSX::InstDef);
	}
	header.numInstances = static_cast<uint32_t>(serializedInstDef.size());

	printf("uniqueModelNames: %zu, serializedModels non-empty: %zu, serializedInstDef: %zu (m_instances total: %zu)\n",
		uniqueModelKeys.size(), modelOrder.size(), serializedInstDef.size(), m_instances.size());

	// Write InstDef pointer array (NULL-terminated)
	const size_t offInstDefList_ptrArray = currOffset;
	//printf(nameof(offInstDefList_ptrArray) " = %zx\n", offInstDefList_ptrArray);
	currOffset += (instDefOffsets.size() + 1) * sizeof(uint32_t);

	// Write second InstDef pointer array for visibility (NULL-terminated)
	const size_t offInstDefList2_ptrArray = currOffset;
	//printf(nameof(offInstDefList2_ptrArray) " = %zx\n", offInstDefList2_ptrArray);
	currOffset += (instDefOffsets.size() + 1) * sizeof(uint32_t);

	// Write third InstDef pointer array for visibility even quadcount (NULL-terminated)
	const size_t offInstDefList3_ptrArray = currOffset;
	//printf(nameof(offInstDefList3_ptrArray) " = %zx\n", offInstDefList3_ptrArray);
	currOffset += (instDefOffsets.size() + 1) * sizeof(uint32_t);

	// Update visible sets to point to InstDef list
	bool first = true;
	for (auto& set : visibleSets)
	{
		size_t index = visibleSetMap[set];
		visibleSetMap.erase(set);
		if (first && orderedQuads.size() % 2 == 0)
			set.offVisibleInstances = static_cast<uint32_t>(offInstDefList3_ptrArray);
		else 
			set.offVisibleInstances = static_cast<uint32_t>(offInstDefList2_ptrArray);
		visibleSetMap[set] = index;
		first = false;
	}

	header.offInstances = (serializedInstDef.size() > 0) ? static_cast<uint32_t>(offInstDefArray) : 0;
	header.offInstancePtrArray = static_cast<uint32_t>(offInstDefList_ptrArray);

	struct LeafHitboxList
	{
		size_t leafFileOffset; // file offset of the BSP leaf node
		size_t listFileOffset; // file offset of this hitbox list
		std::vector<PSX::InstHitbox> entries;
	};
	std::vector<LeafHitboxList> leafHitboxLists;

	size_t nodeFileOffset = offBSP;
	for (size_t node = 0; node < serializedBSPs.size(); node++)
	{
		const size_t currNodeOffset = nodeFileOffset;
		nodeFileOffset += serializedBSPs[node].size();
		if (orderedBSPNodes[node]->IsBranch()) { continue; }

		PSX::BSPLeaf* leaf = reinterpret_cast<PSX::BSPLeaf*>(serializedBSPs[node].data());
		std::vector<PSX::InstHitbox> overlapping;
		for (size_t instIndex : orderedBSPNodes[node]->GetInstanceIndexes())
		{
			overlapping.push_back(m_instances[instIndex].SerializeHitbox(static_cast<uint32_t>(instDefOffsets[instIndex])));
		}
		if (overlapping.empty()) { continue; }

		leaf->offHitbox = static_cast<uint32_t>(currOffset);
		//printf("offLeafHitboxList[node %zu] = %zx (%zu entries)\n", node, currOffset, overlapping.size());
		leafHitboxLists.push_back({currNodeOffset, currOffset, std::move(overlapping)});
		currOffset += leafHitboxLists.back().entries.size() * sizeof(PSX::InstHitbox) + sizeof(uint32_t); // entries + terminator
	}
	
	
	

	size_t paddingSizeForMultOfFour = (4 - (currOffset % 4)) % 4;
	//printf(nameof(paddingSizeForMultOfFour) " = %zx\n", paddingSizeForMultOfFour);
	currOffset += paddingSizeForMultOfFour;

	const size_t offPointerMap = currOffset;
	//printf(nameof(offPointerMap) " = %zx\n", offPointerMap);

	std::vector<uint32_t> pointerMap =
	{
		CALCULATE_OFFSET(PSX::LevHeader, offMeshInfo, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offInstances, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offModels, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offInstancePtrArray, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offExtra, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offCheckpointNodes, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offVisMem, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offAnimTex, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offLevNavTable, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offWaterVertices, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offEnvironmentMap, offHeader),
		CALCULATE_OFFSET(PSX::MeshInfo, offQuadblocks, offMeshInfo),
		CALCULATE_OFFSET(PSX::MeshInfo, offVertices, offMeshInfo),
		CALCULATE_OFFSET(PSX::MeshInfo, offBSPNodes, offMeshInfo),
		CALCULATE_OFFSET(PSX::VisualMem, offNodes[0], offVisMem),
		CALCULATE_OFFSET(PSX::VisualMem, offQuads[0], offVisMem),
		CALCULATE_OFFSET(PSX::VisualMem, offBSP[0], offVisMem),
		CALCULATE_OFFSET(PSX::VisualMem, offOcean[0], offVisMem),
	};

	// Add InstDef.offModel pointers
	for (size_t i = 0; i < instDefOffsets.size(); i++)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::InstDef, offModel, instDefOffsets[i]));
	}

	// Add pointer array entries
	for (size_t i = 0; i < instDefOffsets.size(); i++)
	{
		pointerMap.push_back(static_cast<uint32_t>(offInstDefList_ptrArray + (i * sizeof(uint32_t))));
		pointerMap.push_back(static_cast<uint32_t>(offInstDefList2_ptrArray + (i * sizeof(uint32_t))));
		pointerMap.push_back(static_cast<uint32_t>(offInstDefList3_ptrArray + (i * sizeof(uint32_t))));
	}

	for (size_t i = 0; i < modelOrder.size(); i++)
	{
		pointerMap.push_back(static_cast<uint32_t>(offModelList_ptrArray + (i * sizeof(uint32_t))));
	}

	// Add model internal pointers to .lev patch table
	for (size_t i = 0; i < modelOrder.size(); i++)
	{
		if (!m_instanceModels.contains(modelOrder[i]))
			continue;
		for (uint32_t loc : modelPointerLocations[i])
		{
			pointerMap.push_back(loc);
		}
	}
	
	// Add minimap header pointers to pointer map
	if (!m_minimapConfig.texture.IsEmpty())
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::LevHeader, offIconsLookup, offHeader));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::LevHeader, offIcons, offHeader));
	}
	
	// Add skybox header pointer to pointer map
	if (m_skybox.IsReady())
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::LevHeader, offSkybox, offHeader));
	}


	// Add BSP-leaf hitbox pointers: each leaf's offHitbox field, plus the
	// InstDef pointer inside every hitbox entry
	for (const LeafHitboxList& list : leafHitboxLists)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::BSPLeaf, offHitbox, list.leafFileOffset));
		for (size_t i = 0; i < list.entries.size(); i++)
		{
			pointerMap.push_back(CALCULATE_OFFSET(PSX::InstHitbox, offInstDef, list.listFileOffset + (i * sizeof(PSX::InstHitbox))));
		}
	}

	// Every non-zero entry in the extra header is a pointer and must be relocated.
	// Previously only the two ghost entries were registered, so any other offset the
	// editor started writing would reach the game as a raw file offset.
	for (size_t i = 0; i < PSX::LevelExtra::COUNT; i++)
	{
		if (extraHeader.offsets[i] == 0) { continue; }
		pointerMap.push_back(static_cast<uint32_t>(offExtraHeader + offsetof(PSX::LevelExtraHeader, offsets) + (i * sizeof(uint32_t))));
	}

	for (size_t i = 0; i < animPtrMapOffsets.size(); i++)
	{
		pointerMap.push_back(static_cast<uint32_t>(animPtrMapOffsets[i] + offAnimData));
	}

	size_t offCurrQuad = offQuadblocks;
	for (size_t i = 0; i < serializedQuads.size(); i++)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offMidTextures[0], offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offMidTextures[1], offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offMidTextures[2], offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offMidTextures[3], offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offLowTexture, offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offVisibleSet, offCurrQuad));
		offCurrQuad += serializedQuads[i].size();
	}

	size_t offCurrNode = offBSP;
	for (size_t i = 0; i < serializedBSPs.size(); i++)
	{
		if (orderedBSPNodes[i]->IsBranch()) { offCurrNode += serializedBSPs[i].size(); continue; }
		size_t visMemListIndex = 2 * i + 1;
		visMemBSPP1[visMemListIndex] = static_cast<uint32_t>(offCurrNode);
		visMemBSPP1[visMemListIndex - 1] = static_cast<uint32_t>(offInstDefList2_ptrArray);
		pointerMap.push_back(static_cast<uint32_t>(offVisMemBSPP1 + visMemListIndex * sizeof(uint32_t)));
		pointerMap.push_back(static_cast<uint32_t>(offVisMemBSPP1 + (visMemListIndex - 1) * sizeof(uint32_t)));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::BSPLeaf, offQuads, offCurrNode));
		offCurrNode += serializedBSPs[i].size();
	}

	size_t offCurrVisibleSet = offVisibleSet;
	for (const PSX::VisibleSet& visibleSet : visibleSets)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::VisibleSet, offVisibleBSPNodes, offCurrVisibleSet));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::VisibleSet, offVisibleQuadblocks, offCurrVisibleSet));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::VisibleSet, offVisibleInstances, offCurrVisibleSet));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::VisibleSet, offVisibleExtra, offCurrVisibleSet));
		offCurrVisibleSet += sizeof(PSX::VisibleSet);
	}

	// Add minimap internal pointers to pointer map
	for (size_t offset : minimapPtrMapOffsets)
	{
		pointerMap.push_back(static_cast<uint32_t>(offset));
	}
	
	// Add skybox internal pointers to pointer map
	for (size_t offset : skyboxPtrMapOffsets)
	{
		pointerMap.push_back(static_cast<uint32_t>(offset));
	}

	if (header.offSpawnType_2 != 0)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::LevHeader, offSpawnType_2, offHeader));
		uint32_t i = 0;
		for (PSX::SpawnType2 st2 : serializedSpawnTypeHeader)
		{
			pointerMap.push_back(CALCULATE_OFFSET(PSX::SpawnType2, offPos, offSpawnType + i * sizeof(PSX::SpawnType2) ));
			i++;
		}
	}

	if (header.offSpawnType_2_posRot != 0)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::LevHeader, offSpawnType_2_posRot, offHeader));
		uint32_t i = 0;
		for (PSX::SpawnType2 st2 : serializedSpawnTypePosRotHeader)
		{
			pointerMap.push_back(CALCULATE_OFFSET(PSX::SpawnType2, offPos, offSpawnTypePosRot + i * sizeof(PSX::SpawnType2)));
			i++;
		}
	}

	for (size_t i = 0; i < 3; i++)
	{
		if (m_botPaths[i].IsValid())
			pointerMap.push_back(CALCULATE_OFFSET(PSX::levAINavTable, offAIPathArray[i], offNavTable));
	}

	for (size_t i = 0; i < waterVertices.size(); i++)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::WaterVertex, offVertex, offWaterVertices + i * sizeof(PSX::WaterVertex)));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::WaterVertex, offOceanVertex, offWaterVertices + i * sizeof(PSX::WaterVertex)));
	}
  
  #undef CALCULATE_OFFSET



	const size_t pointerMapBytes = pointerMap.size() * sizeof(uint32_t);

	Write(file, &offPointerMap, sizeof(uint32_t));
	Write(file, &header, sizeof(header));
	Write(file, &meshInfo, sizeof(meshInfo));
	Write(file, texGroups.data(), texGroups.size() * sizeof(PSX::TextureGroup));
	if (!animData.empty()) { Write(file, animData.data(), animData.size()); }
	Write(file, &envMapLayout, sizeof(envMapLayout));
	for (const std::vector<uint8_t>& serializedQuad : serializedQuads) { Write(file, serializedQuad.data(), serializedQuad.size()); }
	for (const auto& visNode : uniqueVisNodes) { Write(file, visNode.data(), visNode.size() * sizeof(uint32_t)); }
	for (const auto& visQuad : uniqueVisQuads) { Write(file, visQuad.data(), visQuad.size() * sizeof(uint32_t)); }
	for (const auto& tuple : visibleInstances)
	{
		const std::vector<uint32_t>& visibleInst = std::get<0>(tuple);
		Write(file, visibleInst.data(), visibleInst.size() * sizeof(uint32_t));
	}
	for (const auto& tuple : visibleExtra)
	{
		const std::vector<uint32_t>& v = std::get<0>(tuple);
		Write(file, v.data(), v.size() * sizeof(uint32_t));
	}
	Write(file, visibleSets.data(), visibleSets.size() * sizeof(PSX::VisibleSet));
	for (const PSX::WaterVertex& waterVert : waterVertices) { Write(file, &waterVert, sizeof(waterVert)); }
	for (const std::vector<uint8_t>& serializedVertex : serializedVertices) { Write(file, serializedVertex.data(), serializedVertex.size()); }
	for (const std::vector<uint8_t>& serializedOVertex : serializedOVertices) { Write(file, serializedOVertex.data(), serializedOVertex.size()); }
	for (const std::vector<uint8_t>& serializedBSP : serializedBSPs) { Write(file, serializedBSP.data(), serializedBSP.size()); }
	for (const std::vector<uint8_t>& serializedCheckpoint : serializedCheckpoints) { Write(file, serializedCheckpoint.data(), serializedCheckpoint.size()); }
	if (!m_tropyGhost.empty()) { Write(file, m_tropyGhost.data(), m_tropyGhost.size()); }
	if (!m_oxideGhost.empty()) { Write(file, m_oxideGhost.data(), m_oxideGhost.size()); }
	Write(file, spawnMeta.data(), spawnMeta.size() * sizeof(int16_t));
	Write(file, &extraHeader, sizeof(extraHeader));
	for (PSX::SpawnType2 st2 : serializedSpawnTypeHeader) { Write(file, &st2, sizeof(st2)); }
	for (auto& spawntypes : serializedSpawnTypes) { for (PSX::Vec3 pos : spawntypes) { Write(file, &pos, sizeof(pos)); } }
	for (uint16_t pad : spawnTypePosPadding) { Write(file, &pad, sizeof(pad)); }
	for (PSX::SpawnType2 st2 : serializedSpawnTypePosRotHeader) { Write(file, &st2, sizeof(st2)); }
	for (auto& spawntypes : serializedSpawnPosRotTypes) { for (PSX::Spawn pos : spawntypes) { Write(file, &pos, sizeof(pos)); } }
	Write(file, &navTable, sizeof(navTable));
	for (const std::vector<uint8_t>& serializedBotPath : serializedBotPaths) { Write(file, serializedBotPath.data(), serializedBotPath.size()); }
	Write(file, visMemNodesP1.data(), visMemNodesP1.size() * sizeof(uint32_t));
	Write(file, visMemQuadsP1.data(), visMemQuadsP1.size() * sizeof(uint32_t));
	Write(file, visMemBSPP1.data(), visMemBSPP1.size() * sizeof(uint32_t));
	Write(file, visMemOceanP1.data(), visMemOceanP1.size() * sizeof(uint32_t));
	Write(file, &visMem, sizeof(visMem));
	// Write minimap data if present
	if (!minimapData.empty()) { Write(file, minimapData.data(), minimapData.size()); }
	// Write skybox data if present
	if (!skyboxData.empty()) { Write(file, skyboxData.data(), skyboxData.size()); }

	uint32_t nullTerm = 0;
	// Write Model data
	for (size_t i = 0; i < modelOrder.size(); i++)
	{
		if (!m_instanceModels.contains(modelOrder[i]))
			continue;
		Write(file, serializedModels[i].data(), serializedModels[i].size());
	}
	// Write Model pointer array (NULL-terminated, stored offsets - game adds 4 to get actual position)
	for (const size_t modelKey : modelOrder)
	{
		uint32_t ptr = static_cast<uint32_t>(modelOffsets[modelKey]);
		Write(file, &ptr, sizeof(ptr));
	}
	Write(file, &nullTerm, sizeof(nullTerm));

	for (const std::vector<uint8_t>& inst : serializedInstDef) { Write(file, inst.data(), inst.size()); }
	// Write InstDef pointer arrays (NULL-terminated, stored offsets - game adds 4 to get actual position)
	for (size_t offset : instDefOffsets)
	{
		uint32_t ptr = static_cast<uint32_t>(offset);
		Write(file, &ptr, sizeof(ptr));
	}
	
	Write(file, &nullTerm, sizeof(nullTerm));

	// Write second InstDef pointer array
	for (size_t offset : instDefOffsets)
	{
		uint32_t ptr = static_cast<uint32_t>(offset);
		Write(file, &ptr, sizeof(ptr));
	}
	Write(file, &nullTerm, sizeof(nullTerm));

	// Write third InstDef pointer array
	for (size_t offset : instDefOffsets)
	{
		uint32_t ptr = static_cast<uint32_t>(offset);
		Write(file, &ptr, sizeof(ptr));
	}
	Write(file, &nullTerm, sizeof(nullTerm));

	// Write BSP-leaf instance hitbox lists (each NULL-terminated)
	for (const LeafHitboxList& list : leafHitboxLists)
	{
		Write(file, list.entries.data(), list.entries.size() * sizeof(PSX::InstHitbox));
		Write(file, &nullTerm, sizeof(nullTerm));
	}

	uint32_t fourBytesOfZero = 0;
	if (paddingSizeForMultOfFour > 0)
	{
		printf("WARNING: HAD TO PAD %zu BYTES\n", paddingSizeForMultOfFour);
		Write(file, &fourBytesOfZero, paddingSizeForMultOfFour);
	}
	Write(file, &pointerMapBytes, sizeof(uint32_t));
	Write(file, pointerMap.data(), pointerMapBytes);
	file.close();
	return true;
}

bool Level::LoadOBJ(const std::filesystem::path& objFile, bool isLevel)
{
	std::string line;
	std::ifstream file(objFile);
	m_name = objFile.filename().replace_extension().string();
	m_parentPath = objFile.parent_path();

	m_hasRawTexture = false;

	bool ret = true;
	std::unordered_map<std::string, std::vector<Tri>> triMap;
	std::unordered_map<std::string, std::vector<Quad>> quadMap;
	std::unordered_map<std::string, std::vector<Vec3>> normalMap;
	std::unordered_map<std::string, std::string> materialMap;
	std::unordered_map<std::string, bool> meshMap;
	std::unordered_set<std::string> materials;
	std::vector<Point> vertices;
	std::vector<Vec3> normals;
	std::vector<Vec2> uvs;
	std::string currQuadblockName;
	bool currQuadblockGoodUV = true;
	size_t quadblockCount = 0;
	while (std::getline(file, line))
	{
		std::vector<std::string> tokens = Split(line);
		if (tokens.empty()) { continue; }
		const std::string& command = tokens[0];
		if (command == "v")
		{
			if (tokens.size() < 4) { continue; }
			vertices.emplace_back(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3]));
			if (tokens.size() < 7) { continue; }
			vertices.back().color = Color(std::stof(tokens[4]), std::stof(tokens[5]), std::stof(tokens[6]));
		}
		else if (command == "vn")
		{
			if (tokens.size() < 4) { continue; }
			normals.emplace_back(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3]));
		}
		else if (command == "vt")
		{
			if (tokens.size() < 3) { continue; }
			Vec2 uv = {std::stof(tokens[1]), std::stof(tokens[2])};
			if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f)
			{
				m_invalidQuadblocks.emplace_back(currQuadblockName, "WARNING: UV outside of expect range [0.0f, 1.0f].");
			}
			auto Wrap = [](float x)
				{
					if (x >= 0.0f && x <= 1.0f) { return x; }
					float r = fmodf(x, 1.0f);
					if (r < 0.0f) { r += 1.0f; }
					if (r == 0.0f && x > 0.0f) { r = 1.0f; }
					return r;
				};
			uv.x = Wrap(uv.x);
			uv.y = 1.0f - Wrap(uv.y);
			uvs.emplace_back(uv);
		}
		else if (command == "o")
		{
			if (tokens.size() < 2 || meshMap.contains(tokens[1]))
			{
				ret = false;
				m_invalidQuadblocks.emplace_back(tokens[1], "Duplicated mesh name.");
				continue;
			}
			currQuadblockName = tokens[1];
			currQuadblockGoodUV = true;
			meshMap[currQuadblockName] = false;
			quadblockCount++;
		}
		else if (command == "usemtl")
		{
			if (tokens.size() < 2) { continue; }
			if (currQuadblockName.empty() || materialMap.contains(currQuadblockName)) { continue; } /* TODO: return false, generate error message */
			materialMap[currQuadblockName] = tokens[1];
		}
		else if (command == "f")
		{
			if (currQuadblockName.empty()) { return false; }
			if (tokens.size() < 4) { continue; }

			if (meshMap.contains(currQuadblockName) && meshMap.at(currQuadblockName))
			{
				ret = false;
				m_invalidQuadblocks.emplace_back(currQuadblockName, "Triblock and Quadblock merged in the same mesh.");
				continue;
			}

			bool isQuadblock = tokens.size() == 5;

			std::vector<std::string> token0 = Split(tokens[1], '/');
			std::vector<std::string> token1 = Split(tokens[2], '/');
			std::vector<std::string> token2 = Split(tokens[3], '/');

			const size_t EXPECTED_INFORMATION_PER_TOKEN = 3; /* pos, opt uvs, normals */
			if (token0.size() < EXPECTED_INFORMATION_PER_TOKEN ||
				token1.size() < EXPECTED_INFORMATION_PER_TOKEN ||
				token2.size() < EXPECTED_INFORMATION_PER_TOKEN)
			{
				ret = false;
				m_invalidQuadblocks.emplace_back(currQuadblockName, "Missing vertex normals.");
				continue;
			}

			int i0 = std::stoi(token0[0]) - 1;
			int i1 = std::stoi(token1[0]) - 1;
			int i2 = std::stoi(token2[0]) - 1;
			int ni0 = std::stoi(token0[2]) - 1;
			int ni1 = std::stoi(token1[2]) - 1;
			int ni2 = std::stoi(token2[2]) - 1;
			normalMap[currQuadblockName].push_back(normals[ni0]);
			normalMap[currQuadblockName].push_back(normals[ni1]);
			normalMap[currQuadblockName].push_back(normals[ni2]);

			vertices[i0].normal = normals[ni0];
			vertices[i1].normal = normals[ni1];
			vertices[i2].normal = normals[ni2];

			if (currQuadblockGoodUV)
			{
				int uv0 = 0;
				int uv1 = 0;
				int uv2 = 0;
				try
				{
					uv0 = std::stoi(token0[1]) - 1;
					uv1 = std::stoi(token1[1]) - 1;
					uv2 = std::stoi(token2[1]) - 1;
				}
				catch (...) { currQuadblockGoodUV = false; }

				if (currQuadblockGoodUV)
				{
					vertices[i0].uv = uvs[uv0];
					vertices[i1].uv = uvs[uv1];
					vertices[i2].uv = uvs[uv2];
				}
			}

			if (!currQuadblockGoodUV)
			{
				m_invalidQuadblocks.emplace_back(currQuadblockName, "Missing UVs.");
			}

			bool blockFetched = false;
			if (isQuadblock)
			{
				std::vector<std::string> token3 = Split(tokens[4], '/');
				int i3 = std::stoi(token3[0]) - 1;
				int ni3 = std::stoi(token3[2]) - 1;
				normalMap[currQuadblockName].push_back(normals[ni3]);
				vertices[i3].normal = normals[ni3];
				if (currQuadblockGoodUV)
				{
					int uv3 = std::stoi(token3[1]) - 1;
					vertices[i3].uv = uvs[uv3];
				}

				if (!quadMap.contains(currQuadblockName)) { quadMap[currQuadblockName] = std::vector<Quad>(); }
				quadMap[currQuadblockName].emplace_back(vertices[i0], vertices[i1], vertices[i2], vertices[i3]);
				blockFetched = quadMap[currQuadblockName].size() == 4;
			}
			else
			{
				if (!triMap.contains(currQuadblockName)) { triMap[currQuadblockName] = std::vector<Tri>(); }
				triMap[currQuadblockName].emplace_back(vertices[i0], vertices[i1], vertices[i2]);
				blockFetched = triMap[currQuadblockName].size() == 4;
			}

			if (blockFetched)
			{
				Vec3 averageNormal = Vec3();
				for (const Vec3& normal : normalMap[currQuadblockName])
				{
					averageNormal = averageNormal + normal;
				}
				averageNormal = averageNormal / averageNormal.Length();
				std::string material;
				if (materialMap.contains(currQuadblockName))
				{
					material = materialMap[currQuadblockName];
					m_materialToQuadblocks[material].push_back(m_quadblocks.size());
					if (!materials.contains(material))
					{
						materials.insert(material);
						m_materialToTexture[material] = Texture();
						m_propTerrain.SetDefaultValue(material, TerrainType::DEFAULT);
						m_propQuadFlags.SetDefaultValue(material, QuadFlags::DEFAULT);
						m_propDoubleSided.SetDefaultValue(material, false);
						m_propCheckpoints.SetDefaultValue(material, false);
						m_propTurboPads.SetDefaultValue(material, QuadblockTrigger::NONE);
						m_propWeatherIntensity.SetDefaultValue(material, 0);
						m_propWeatherVanishRate.SetDefaultValue(material, 0);
						m_propCheckpointPathable.SetDefaultValue(material, true);
						m_propVisTreeTransparent.SetDefaultValue(material, false);
						m_propDrawOrderHigh.SetDefaultValue(material, static_cast<int>(0));
						m_propWater.SetDefaultValue(material, false);
						m_propTerrain.RegisterMaterial(this);
						m_propQuadFlags.RegisterMaterial(this);
						m_propDoubleSided.RegisterMaterial(this);
						m_propCheckpoints.RegisterMaterial(this);
						m_propTurboPads.RegisterMaterial(this);
						m_propSpeedImpact.RegisterMaterial(this);
						m_propWeatherIntensity.RegisterMaterial(this);
						m_propWeatherVanishRate.RegisterMaterial(this);
						m_propCheckpointPathable.RegisterMaterial(this);
						m_propVisTreeTransparent.RegisterMaterial(this);
						m_propDrawOrderHigh.RegisterMaterial(this);
						m_propWater.RegisterMaterial(this);
					}
				}
				bool sameUVs = true;
				if (isQuadblock)
				{
					Quad& q0 = quadMap[currQuadblockName][0];
					Quad& q1 = quadMap[currQuadblockName][1];
					Quad& q2 = quadMap[currQuadblockName][2];
					Quad& q3 = quadMap[currQuadblockName][3];
					const Vec2& targetUV = q0.p[0].uv;
					for (size_t i = 0; i < 4; i++)
					{
						const Quad& q = quadMap[currQuadblockName][i];
						for (size_t j = 0; j < 4; j++)
						{
							if (q.p[j].uv != targetUV) { sameUVs = false; break; }
						}
						if (!sameUVs) { break; }
					}
					try
					{
						m_quadblocks.emplace_back(currQuadblockName, q0, q1, q2, q3, averageNormal, material, currQuadblockGoodUV, [this](const Quadblock& qb) { UpdateFilterRenderData(qb); });
						meshMap[currQuadblockName] = true;
					}
					catch (const QuadException& e)
					{
						ret = false;
						m_invalidQuadblocks.emplace_back(currQuadblockName, e.what());
					}
				}
				else
				{
					Tri& t0 = triMap[currQuadblockName][0];
					Tri& t1 = triMap[currQuadblockName][1];
					Tri& t2 = triMap[currQuadblockName][2];
					Tri& t3 = triMap[currQuadblockName][3];
					const Vec2& targetUV = t0.p[0].uv;
					for (size_t i = 0; i < 4; i++)
					{
						const Tri& t = triMap[currQuadblockName][i];
						for (size_t j = 0; j < 3; j++)
						{
							if (t.p[j].uv != targetUV) { sameUVs = false; break; }
						}
						if (!sameUVs) { break; }
					}
					try
					{
						m_quadblocks.emplace_back(currQuadblockName, t0, t1, t2, t3, averageNormal, material, currQuadblockGoodUV, [this](const Quadblock& qb) { UpdateFilterRenderData(qb); });
						meshMap[currQuadblockName] = true;
					}
					catch (const QuadException& e)
					{
						ret = false;
						m_invalidQuadblocks.emplace_back(currQuadblockName, e.what());
					}
				}
				if (sameUVs)
				{
					m_invalidQuadblocks.emplace_back(currQuadblockName, "Degenerated UV data.");
				}
			}
		}
	}
	file.close();

	m_showLogWindow = !m_invalidQuadblocks.empty();

	if (!materials.empty())
	{
		std::filesystem::path mtlPath = m_parentPath / (objFile.stem().string() + ".mtl");
		if (std::filesystem::exists(mtlPath))
		{
			std::ifstream mtl(mtlPath);
			std::string currMaterial;
			while (std::getline(mtl, line))
			{
				std::vector<std::string> tokens = Split(line);
				if (tokens.empty()) { continue; }

				const std::string& command = tokens[0];
				if (command == "newmtl") { currMaterial = tokens[1]; }
				else if (command == "map_Kd")
				{
					std::string imagePath = tokens[1];
					for (size_t i = 2; i < tokens.size(); i++) { imagePath += " " + tokens[i]; }
					std::filesystem::path materialPath = imagePath;
					if (!std::filesystem::exists(materialPath)) { materialPath = m_parentPath / materialPath.filename(); }
					if (std::filesystem::exists(materialPath))
					{
						m_materialToTexture[currMaterial] = Texture(materialPath);
					}
				}
			}
		}
	}

	if (ret)
	{
		for (const auto& [material, texture] : m_materialToTexture)
		{
			const bool semiTransparent = texture.IsSemiTransparent();
			m_propVisTreeTransparent.SetDefaultValue(material, semiTransparent);

			const std::filesystem::path& texPath = texture.GetPath();
			const std::vector<size_t>& quadblockIndexes = m_materialToQuadblocks[material];
			for (const size_t index : quadblockIndexes)
			{
				m_quadblocks[index].SetTexPath(texPath);
				m_quadblocks[index].SetVisTreeTransparent(semiTransparent);
			}
		}
	}

	if (quadblockCount != m_quadblocks.size())
	{
		m_showLogWindow = true;
		m_logMessage = "Error: number of meshes does not equal number of quadblocks.\n\nNumber of meshes found: " + std::to_string(quadblockCount) + "\nNumber of quadblocks: " + std::to_string(m_quadblocks.size());;
		m_logMessage += "\n\nThe following meshes are not a quadblock:\n\n";
		constexpr size_t QUADS_PER_LINE = 10;
		size_t invalidQuadblocks = 0;
		for (auto& [name, status] : meshMap)
		{
			if (status) { continue; }
			m_logMessage += name + ", ";
			if (((invalidQuadblocks + 1) % QUADS_PER_LINE) == 0) { m_logMessage += "\n"; }
			invalidQuadblocks++;
		}
		ret = false;
	}
	m_loaded = ret;

	if (m_loaded && isLevel)
	{
		std::filesystem::path presetFolder = m_parentPath / (m_name + "_presets");
		if (std::filesystem::is_directory(presetFolder))
		{
			for (const auto& entry : std::filesystem::directory_iterator(presetFolder))
			{
				const std::filesystem::path json = entry.path();
				if (json.has_extension() && json.extension() == ".json") { LoadPreset(json); }
			}
		}

		//Load preset models
		std::filesystem::path folderPath(Settings::m_lastOpenedModelFolder);
		if (std::filesystem::exists(folderPath) && std::filesystem::is_directory(folderPath))
		{
			for (const auto& entry : std::filesystem::directory_iterator(folderPath))
			{
				if (!entry.is_regular_file())
					continue;
				//todo
			}
		}
	}
	GenerateRenderLevData();
	GenerateBSP();
	return ret;
}





bool Level::SaveOBJ(const std::filesystem::path& objFile) 
{
	std::ofstream file(objFile);
	if (!file.is_open()) { return false; }

	// --- Collect all unique vertices, normals, UVs across all quadblocks ---
	std::vector<std::pair<Vec3, Color>> allVertices;
	std::vector<Vec3> allNormals;
	std::vector<Vec2> allUVs;

	// Maps for deduplication (using string keys for float precision safety)
	auto Vec3Key = [](const Vec3& v) {
		return std::to_string(v.x) + "," + std::to_string(v.y) + "," + std::to_string(v.z);
		};
	auto Vec2Key = [](const Vec2& v) {
		return std::to_string(v.x) + "," + std::to_string(v.y);
		};
	auto PointKey = [](const Vec3& pos) {
		return std::to_string(pos.x) + "," + std::to_string(pos.y) + "," + std::to_string(pos.z);
		};

	std::unordered_map<std::string, int> vertexIndexMap;  // 1-based
	std::unordered_map<std::string, int> normalIndexMap;  // 1-based
	std::unordered_map<std::string, int> uvIndexMap;      // 1-based

	// Per-quadblock face data: each face = list of (vi, uvi, ni) tuples
	struct FaceVertex { int vi, uvi, ni; };
	struct FaceData { std::vector<std::vector<FaceVertex>> faces; }; // each face is 3 or 4 verts
	std::vector<FaceData> quadblockFaces(m_quadblocks.size());

	auto GetOrAddVertex = [&](size_t quadblockIndex, int vertexSlot, const Vec3& pos, const Color& color) -> int {
		std::string key = std::to_string(quadblockIndex) + ":" + std::to_string(vertexSlot);
		auto it = vertexIndexMap.find(key);
		if (it != vertexIndexMap.end()) { return it->second; }
		int idx = (int)allVertices.size() + 1;
		allVertices.push_back({ pos, color });
		vertexIndexMap[key] = idx;
		return idx;
		};
	auto GetOrAddNormal = [&](const Vec3& n) -> int {
		std::string key = Vec3Key(n);
		auto it = normalIndexMap.find(key);
		if (it != normalIndexMap.end()) { return it->second; }
		int idx = (int)allNormals.size() + 1;
		allNormals.push_back(n);
		normalIndexMap[key] = idx;
		return idx;
		};
	auto GetOrAddUV = [&](const Vec2& uv) -> int {
		// Invert Y back to Blender convention before storing
		Vec2 blenderUV = { uv.x, 1.0f - uv.y };
		std::string key = Vec2Key(blenderUV);
		auto it = uvIndexMap.find(key);
		if (it != uvIndexMap.end()) { return it->second; }
		int idx = (int)allUVs.size() + 1;
		allUVs.push_back(blenderUV);
		uvIndexMap[key] = idx;
		return idx;
		};

	// --- Pass 1: gather all geometry into index buffers ---
	for (size_t qi = 0; qi < m_quadblocks.size(); qi++)
	{
		const Quadblock& qb = m_quadblocks[qi];
		FaceData& fd = quadblockFaces[qi];
		const Vertex* verts = qb.GetUnswizzledVertices();

		if (qb.IsQuadblock())
		{
			static constexpr int QUAD_FACES[4][4] = {
	{0, 3, 4, 1}, 
	{1, 4, 5, 2},
	{3, 6, 7, 4},
	{4, 7, 8, 5}
			};
			static constexpr int QUAD_UV_REMAP[4] = { 0, 2, 3, 1 }; 

			for (int f = 0; f < 4; f++)
			{
				const QuadUV& faceUVs = qb.GetQuadUV(f);
				std::vector<FaceVertex> face;
				for (int v = 0; v < 4; v++)
				{
					int slot = QUAD_FACES[f][v]; // p0..p8 index
					const Vertex& vert = verts[slot];
					face.push_back({
						GetOrAddVertex(qi, slot, vert.m_pos, vert.GetColor(true)),
						GetOrAddUV(faceUVs[QUAD_UV_REMAP[v]]),
						GetOrAddNormal(vert.m_normal)
						});
				}
				fd.faces.push_back(face);
			}
		}
		else // triblock
		{
			static constexpr int TRI_FACES[4][3] = {
	{3, 1, 0},  // tri {0,1,3} reversed
	{3, 4, 1},  // tri {1,4,3} reversed
	{4, 2, 1},  // tri {1,2,4} reversed
	{6, 4, 3}   // tri {3,4,6} reversed
			};

			// Which quadface UV to source from (same face index as parent quad)
			static constexpr int TRI_UV_FACE[4] = { 0, 0, 1, 2 };

			static constexpr int TRI_UV_REMAP[4][3] = {
				{2, 1, 0},  // face 0: correct
				{2, 3, 1},  // face 1: correct
				{2, 1, 0},  
				{2, 1, 0}   
			};

			for (int f = 0; f < 4; f++)
			{
				const QuadUV& faceUVs = qb.GetQuadUV(TRI_UV_FACE[f]);
				std::vector<FaceVertex> face;
				for (int v = 0; v < 3; v++)
				{
					int slot = TRI_FACES[f][v];
					const Vertex& vert = verts[slot];
					face.push_back({
						GetOrAddVertex(qi, slot, vert.m_pos, vert.GetColor(true)),
						GetOrAddUV(faceUVs[TRI_UV_REMAP[f][v]]),
						GetOrAddNormal(vert.m_normal)
						});
				}
				fd.faces.push_back(face);
			}
		}
	}

	// --- Write vertex positions ---
	file << std::fixed << std::setprecision(6);
	for (const auto& [pos, c] : allVertices)
	{
		file << "v " << pos.x << " " << pos.y << " " << pos.z
			<< " " << c.Red() << " " << c.Green() << " " << c.Blue() << "\n";
	}
	file << "\n";

	// --- Write UVs ---
	for (const Vec2& uv : allUVs)
	{
		file << "vt " << uv.x << " " << uv.y << "\n";
	}
	file << "\n";

	// --- Write normals ---
	for (const Vec3& n : allNormals)
	{
		file << "vn " << n.x << " " << n.y << " " << n.z << "\n";
	}
	file << "\n";

	// --- Write MTL reference ---
	std::string stem = objFile.stem().string();
	bool hasMaterials = !m_materialToTexture.empty();
	if (hasMaterials)
	{
		file << "mtllib " << stem << ".mtl\n\n";
	}

	// --- Write meshes (one per quadblock) ---
	for (size_t qi = 0; qi < m_quadblocks.size(); qi++)
	{
		const Quadblock& qb = m_quadblocks[qi];
		const FaceData& fd = quadblockFaces[qi];

		file << "o " << qb.GetName() << "\n";

		// Find this quadblock's material
		std::string material;
		for (const auto& [mat, indexes] : m_materialToQuadblocks)
		{
			for (size_t idx : indexes)
			{
				if (idx == qi) { material = mat; break; }
			}
			if (!material.empty()) { break; }
		}

		if (!material.empty())
		{
			file << "usemtl " << material << "\n";
		}

		file << "s off\n"; // smoothing group, standard Blender export

		for (const std::vector<FaceVertex>& face : fd.faces)
		{
			file << "f";
			for (const FaceVertex& fv : face)
			{
				file << " " << fv.vi << "/" << fv.uvi << "/" << fv.ni;
			}
			file << "\n";
		}
		file << "\n";
	}

	file.close();

	// --- Write MTL file ---
	if (hasMaterials)
	{
		std::filesystem::path mtlPath = objFile.parent_path() / (stem + ".mtl");
		std::ofstream mtl(mtlPath);
		if (mtl.is_open())
		{
			for (const auto& [material, texture] : m_materialToTexture)
			{
				mtl << "newmtl " << material << "\n";
				mtl << "Ka 1.000 1.000 1.000\n";
				mtl << "Kd 1.000 1.000 1.000\n";
				mtl << "Ks 0.000 0.000 0.000\n";
				mtl << "illum 1\n";

				const std::filesystem::path& texPath = texture.GetPath();
				if (!texPath.empty() && std::filesystem::exists(texPath))
				{
					mtl << "map_Kd " << texPath.filename().string() << "\n";
				}
				mtl << "\n";
			}
			mtl.close();
		}
	}

	return true;
}





bool Level::StartEmuIPC(const std::string& emulator)
{
	constexpr size_t PSX_RAM_SIZE = 0x800000;
	int pid = Process::GetPID(emulator);
	if (pid == Process::INVALID_PID || !Process::OpenMemoryMap(emulator + "_" + std::to_string(pid), PSX_RAM_SIZE)) { return false; }
	return true;
}

bool Level::HotReload(const std::string& levPath, const std::string& vrmPath, const std::string& emulator)
{
	bool vrmOnly = false;
	if (levPath.empty())
	{
		if (vrmPath.empty()) { return false; }
		vrmOnly = true;
	}

	if (!StartEmuIPC(emulator)) { return false; }

	constexpr size_t GAMEMODE_ADDR = 0x80096b20;
	constexpr uint32_t GAME_PAUSED = 0xF;
	if (Process::At<uint32_t>(GAMEMODE_ADDR) & GAME_PAUSED) { return false; }

	constexpr size_t HOST_SETTINGS_LOCATION = 0x8000C080;
	constexpr size_t HOST_SETTINGS_MAGIC = 0x53544553;
	constexpr size_t VRAM_ADDR = 0x80200000;
	constexpr size_t RAM_ADDR = 0x80300000;
	constexpr size_t SIGNAL_ADDR = 0x8000C000;
	constexpr size_t SIGNAL_ADDR_VRAM_ONLY = 0x8000C004;
	constexpr int HOT_RELOAD_START = 1;
	constexpr int HOT_RELOAD_READY = 3;
	constexpr int HOT_RELOAD_EXEC = 4;

	if (!vrmOnly)
	{
		Process::At<int32_t>(SIGNAL_ADDR) = HOT_RELOAD_START;
		while (Process::At<volatile int32_t>(SIGNAL_ADDR) != HOT_RELOAD_READY) {}
	}
	if (!vrmPath.empty())
	{
		std::vector<uint8_t> vrm;
		ReadBinaryFile(vrm, vrmPath);
		for (size_t i = 0; i < vrm.size(); i++) { Process::At<uint8_t>(VRAM_ADDR + i) = vrm[i]; }
	}

	if (!levPath.empty())
	{
		std::vector<uint8_t> lev;
		ReadBinaryFile(lev, levPath);
		for (size_t i = 0; i < lev.size(); i++) { Process::At<uint8_t>(RAM_ADDR + i) = lev[i]; }
	}

	{
		static int32_t hotReloadGlobalSequence = 0;
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, sequence)) = hotReloadGlobalSequence++;
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, relicSapphire)) = static_cast<int32_t>(m_hotReloadSettings.relicSapphire * 1000.0f);
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, relicGold)) = static_cast<int32_t>(m_hotReloadSettings.relicGold * 1000.0f);
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, relicPlatinum)) = static_cast<int32_t>(m_hotReloadSettings.relicPlatinum * 1000.0f);
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, crystalTime)) = static_cast<int32_t>(m_hotReloadSettings.crystalTime * 1000.0f);
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, introCutscene)) = m_hotReloadSettings.introCutscene ? 1 : 0;
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, ghost)) = m_hotReloadSettings.ghost ? 1 : 0;
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, magic)) = static_cast<int32_t>(HOST_SETTINGS_MAGIC);
	}

	if (vrmOnly) { Process::At<int32_t>(SIGNAL_ADDR_VRAM_ONLY) = 1; }
	else { Process::At<int32_t>(SIGNAL_ADDR) = HOT_RELOAD_EXEC; }

	return true;
}

bool Level::SaveGhostData(const std::string& emulator, const std::filesystem::path& path)
{
	constexpr size_t SIGNAL_ADDR = 0x8000C008;
	if (!StartEmuIPC(emulator) || Process::At<int32_t>(SIGNAL_ADDR) == 0) { return false; }

	std::vector<uint8_t> data;
	constexpr size_t GHOST_SIZE_ADDR = 0x80270038;
	constexpr size_t GHOST_DATA_ADDR = 0x8027003C;

	size_t fileSize = static_cast<size_t>(Process::At<uint32_t>(GHOST_SIZE_ADDR));
	if (fileSize != GHOST_DATA_FILESIZE) { return false; }

	data.resize(fileSize);
	for (size_t i = 0; i < data.size(); i++) { data[i] = Process::At<uint8_t>(GHOST_DATA_ADDR + i); }
	Process::At<int32_t>(SIGNAL_ADDR) = 0;

	std::ofstream file(path, std::ios::binary);
	Write(file, data.data(), data.size() * sizeof(uint8_t));
	file.close();
	return true;
}

bool Level::SetGhostData(const std::filesystem::path& path, bool tropy)
{
	std::vector<uint8_t> data;
	ReadBinaryFile(data, path);
	if (data.size() != GHOST_DATA_FILESIZE) { return false; }

	if (tropy) { m_tropyGhost.resize(GHOST_DATA_FILESIZE); }
	else { m_oxideGhost.resize(GHOST_DATA_FILESIZE); }
	memcpy(tropy ? m_tropyGhost.data() : m_oxideGhost.data(), data.data(), data.size());
	return true;
}

bool Level::UpdateVRM()
{
	std::vector<Texture*> textures;
	std::vector<std::tuple<Texture*, Texture*>> copyTextureAttributes;
	std::set<std::string> usedMaterials;

	for (const Quadblock& quad : m_quadblocks) // Quad textures
	{
		if (quad.GetFlags() & QuadFlags::INVISIBLE_TRIGGER)
			continue;
		usedMaterials.insert(quad.GetMaterial());
	}
	for (const Instance& inst : m_instances) // Model textures
	{
		InstanceModel& model = m_instanceModels[inst.GetModelKey()];
		for (InstanceModelHeader& head : model.m_headers)
		{
			for (Tri& tri : head.GetGeometry())
			{
				usedMaterials.insert(tri.texture);
			}
		}
	}
	usedMaterials.insert(m_envMapMatName); // Water texture

	for (std::string material : usedMaterials)
	{
		if (!m_materialToTexture.contains(material))
			continue;
		Texture& texture = m_materialToTexture[material];
		bool foundEqual = false;
		for (Texture* addedTexture : textures)
		{
			if (texture == *addedTexture)
			{
				copyTextureAttributes.push_back({addedTexture, &texture});
				foundEqual = true;
				break;
			}
		}
		if (foundEqual) { continue; }
		textures.push_back(&texture);
	}

	for (const AnimTexture& animTex : m_animTextures)
	{
		const std::vector<AnimTextureFrame>& animFrames = animTex.GetFrames();
		const std::vector<Texture>& animTextures = animTex.GetTextures();
		for (const AnimTextureFrame& frame : animFrames)
		{
			bool foundEqual = false;
			Texture* texture = const_cast<Texture*>(&animTextures[frame.textureIndex]);
			for (Texture* addedTexture : textures)
			{
				if (*texture == *addedTexture)
				{
					copyTextureAttributes.push_back({addedTexture, texture});
					foundEqual = true;
					break;
				}
			}
			if (foundEqual) { continue; }
			textures.push_back(texture);
		}
	}
	
	// Add minimap textures if enabled
	if (!m_minimapConfig.texture.IsEmpty())
	{
		Texture* tex = &m_minimapConfig.texture;
		bool foundEqual = false;
		for (Texture* addedTexture : textures)
		{
			if (*tex == *addedTexture)
			{
				copyTextureAttributes.push_back({ addedTexture, tex });
				foundEqual = true;
				break;
			}
		}
		if (!foundEqual)
			textures.push_back(tex);
	}

	m_vrm = PackVRM(textures);




	if (m_vrm.empty()) { return false; }

	for (auto& [from, to] : copyTextureAttributes)
	{
		to->CopyVRAMAttributes(*from);
	}

	return true;
}

std::vector<uint16_t> Level::ReadRawVRAM(std::filesystem::path vrmPath)
{
	std::vector<uint16_t> vram(1024 * 512, 0); 

	if (std::filesystem::exists(vrmPath))
	{
		std::ifstream vrmFile(vrmPath, std::ios::binary);

		// Read the raw file into temporary memory
		vrmFile.seekg(0, std::ios::end);
		size_t vrmSize = vrmFile.tellg();
		vrmFile.seekg(0, std::ios::beg);

		std::vector<uint8_t> rawVrmData(vrmSize);
		vrmFile.read(reinterpret_cast<char*>(rawVrmData.data()), vrmSize);
		vrmFile.close();

		const uint8_t* pVrm = rawVrmData.data();
		uint32_t vrmMagic;
		memcpy(&vrmMagic, pVrm, sizeof(uint32_t));
		pVrm += sizeof(uint32_t);

		// If magic is 0x20, we have a multi-block VRM (Standard for this level format)
		if (vrmMagic == 0x20) {
			for (int block = 0; block < 2; block++) {
				PSX::VRMHeader blockHead;
				memcpy(&blockHead, pVrm, sizeof(PSX::VRMHeader));
				pVrm += sizeof(PSX::VRMHeader);

				for (size_t y = 0; y < blockHead.height; y++) {
					// Use the absolute coordinates provided in the VRM header
					size_t vramIdx = (blockHead.y + y) * 1024 + blockHead.x;
					size_t rowByteSize = blockHead.width * sizeof(uint16_t);

					if (vramIdx + blockHead.width <= vram.size()) {
						memcpy(&vram[vramIdx], pVrm, rowByteSize);
					}
					pVrm += rowByteSize;
				}
			}
		}
	}
	return vram;
}

bool Level::UpdateAnimTextures(float deltaTime)
{
	bool changed = false;
	if (m_animTextures.size() != m_lastAnimTextureCount)
	{
		m_lastAnimTextureCount = m_animTextures.size();
		changed = true;
	}

	for (AnimTexture& animTex : m_animTextures)
	{
		if (animTex.AdvanceRender(deltaTime)) { changed = true; }
	}

	return changed;
}

void Level::InitModels(Renderer& renderer)
{
	m_models[LevelModels::LEVEL] = renderer.CreateModel();
	m_models[LevelModels::LEVEL]->SetRenderCondition([]() { return GuiRenderSettings::showLevel; });

	m_models[LevelModels::BSP] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::BSP]->SetRenderCondition([]() { return GuiRenderSettings::showBspRectTree; });

	m_models[LevelModels::SPAWN] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::SPAWN]->SetRenderCondition([]() { return GuiRenderSettings::showStartpoints; });

	m_models[LevelModels::CHECKPOINT] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::CHECKPOINT]->SetRenderCondition([]() { return GuiRenderSettings::showCheckpoints; });

	m_models[LevelModels::SELECTED] = m_models[LevelModels::LEVEL]->AddModel();

	m_models[LevelModels::MULTI_SELECTED] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::MULTI_SELECTED]->SetRenderCondition([]() { return GuiRenderSettings::showVisTree; });

	m_models[LevelModels::FILTER] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::FILTER]->SetRenderCondition([]() { return GuiRenderSettings::filterActive; });

	m_models[LevelModels::MINIMAP_BOUNDS] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::MINIMAP_BOUNDS]->SetRenderCondition([]() { return GuiRenderSettings::showMinimapBounds; });
	m_models[LevelModels::SKYBOX] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::SKYBOX]->SetRenderCondition([]() { return GuiRenderSettings::showSkybox; });

	m_models[LevelModels::BOT] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::BOT]->SetRenderCondition([]() { return GuiRenderSettings::showBots; });

	m_models[LevelModels::INSTANCES] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::INSTANCES]->SetRenderCondition([]() { return GuiRenderSettings::showInstances; });
}

void Level::GenerateRenderLevData()
{
	if (!m_models[LevelModels::LEVEL] || !m_models[LevelModels::FILTER]) { return; }

	std::vector<Primitive> levTriangles;
	std::vector<Primitive> filterTriangles;
	levTriangles.reserve(m_quadblocks.size() * 8);
	filterTriangles.reserve(m_quadblocks.size() * 8);

	auto CountPrimitiveTriangles = [](const std::vector<Primitive>& primitives)
		{
			size_t count = 0;
			for (const Primitive& primitive : primitives) { count += (primitive.type == PrimitiveType::QUAD) ? 2 : 1; }
			return count;
		};

	size_t triangleOffset = 0;
	for (Quadblock& qb : m_quadblocks)
	{
		std::vector<Primitive> qbTriangles = qb.ToGeometry(false);
		if (qbTriangles.empty()) { continue; }

		qb.SetRenderPrimitiveIndex(triangleOffset);
		std::vector<Primitive> qbFilterTriangles = qb.ToGeometry(true);
		levTriangles.insert(levTriangles.end(), qbTriangles.begin(), qbTriangles.end());
		filterTriangles.insert(filterTriangles.end(), qbFilterTriangles.begin(), qbFilterTriangles.end());
		const size_t qbTriCount = CountPrimitiveTriangles(qbTriangles);
		triangleOffset += qbTriCount;
	}

	m_models[LevelModels::LEVEL]->GetMesh().SetGeometry(levTriangles, Mesh::RenderFlags::AllowPointRender | Mesh::RenderFlags::QuadblockLod, Mesh::ShaderFlags::None);
	m_models[LevelModels::FILTER]->GetMesh().SetGeometry(filterTriangles,
		Mesh::RenderFlags::DrawWireframe | Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::ForceDrawOnTop | Mesh::RenderFlags::DrawLinesAA | Mesh::RenderFlags::DontOverrideRenderFlags | Mesh::RenderFlags::ThickLines | Mesh::RenderFlags::QuadblockLod,
		Mesh::ShaderFlags::DiscardZeroColor);
}

void Level::UpdateAnimationRenderData()
{
	if (!m_models[LevelModels::LEVEL]) { return; }

	for (const AnimTexture& animTex : m_animTextures)
	{
		if (!animTex.IsPopulated()) { continue; }
		const std::vector<Texture>& textures = animTex.GetTextures();
		const AnimTextureFrame& frame = animTex.GetRenderFrame();
		for (size_t qbIndex : animTex.GetQuadblockIndexes())
		{
			Quadblock& qb = m_quadblocks[qbIndex];
			const std::array<QuadUV, NUM_FACES_QUADBLOCK + 1>& uvs = frame.uvs;
			const size_t basePrimitiveIndex = qb.GetRenderPrimitiveIndex();
			if (basePrimitiveIndex == RENDER_INDEX_NONE) { continue; }

			const std::filesystem::path texturePath = textures[frame.textureIndex].GetPath();
			std::vector<Primitive> qbTriangles = qb.ToGeometry(false, &uvs, &texturePath);
			size_t primitiveIndex = basePrimitiveIndex;
			for (const Primitive& primitive : qbTriangles)
			{
				primitiveIndex = m_models[LevelModels::LEVEL]->GetMesh().UpdatePrimitive(primitive, primitiveIndex);
			}
		}
	}
}

void Level::UpdateFilterRenderData(const Quadblock& qb)
{
	if (!m_models[LevelModels::FILTER]) { return; }

	const size_t basePrimitiveIndex = qb.GetRenderPrimitiveIndex();
	if (basePrimitiveIndex == RENDER_INDEX_NONE) { return; }

	std::vector<Primitive> qbFilterTriangles = qb.ToGeometry(true);
	size_t primitiveIndex = basePrimitiveIndex;
	for (const Primitive& primitive : qbFilterTriangles)
	{
		primitiveIndex = m_models[LevelModels::FILTER]->GetMesh().UpdatePrimitive(primitive, primitiveIndex);
	}
}

void Level::GenerateRenderBspData()
{
	if (!m_models[LevelModels::BSP]) { return; }

	struct NodeDepth
	{
		const BSP* node;
		int depth;
	};
	std::vector<Primitive> triangles;
	std::vector<NodeDepth> stack;
	stack.push_back({&m_bsp, 0});
	GuiRenderSettings::bspTreeMaxDepth = 0;
	while (!stack.empty())
	{
		const NodeDepth entry = stack.back();
		stack.pop_back();
		if (!entry.node) { continue; }

		if (GuiRenderSettings::bspTreeMaxDepth < entry.depth)
		{
			GuiRenderSettings::bspTreeMaxDepth = entry.depth;
		}

		const bool drawDepth = (GuiRenderSettings::bspTreeTopDepth <= entry.depth && GuiRenderSettings::bspTreeBottomDepth >= entry.depth);
		if (drawDepth)
		{
			const Color c = Color(entry.depth * 30.0, 1.0, 1.0);
			std::vector<Primitive> nodeTriangles = entry.node->GetBoundingBox().ToGeometry();
			for (Primitive& primitive : nodeTriangles)
			{
				for (unsigned i = 0; i < primitive.pointCount; i++) { primitive.p[i].color = c; }
				triangles.push_back(primitive);
			}
		}

		if (entry.node->GetLeftChildren() != nullptr)
		{
			stack.push_back({entry.node->GetLeftChildren(), entry.depth + 1});
		}
		if (entry.node->GetRightChildren() != nullptr)
		{
			stack.push_back({entry.node->GetRightChildren(), entry.depth + 1});
		}
	}

	m_models[LevelModels::BSP]->GetMesh().SetGeometry(triangles, Mesh::RenderFlags::DrawWireframe | Mesh::RenderFlags::DontOverrideRenderFlags);
}

void Level::UpdateRenderCheckpointData()
{
	Model* checkpointModel = m_models[LevelModels::CHECKPOINT];
	if (!checkpointModel) { return; }

	checkpointModel->ClearModels();
	if (m_checkpoints.empty())
	{
		checkpointModel->GetMesh().Clear();
		return;
	}

	std::vector<Primitive> checkTriangles;
	checkTriangles.reserve(m_checkpoints.size() * 8);
	std::unordered_set<int> selectedCheckpointIndexes;
	for (size_t index : m_rendererSelectedQuadblockIndexes)
	{
		int checkpointIndex = m_quadblocks[index].GetCheckpoint();
		selectedCheckpointIndexes.insert(checkpointIndex);
	}

	constexpr float labelHeightOffset = 1.5f;
	for (const Checkpoint& e : m_checkpoints)
	{
		bool selected = selectedCheckpointIndexes.contains(e.GetIndex());
		const Color& c = selected ? GuiRenderSettings::selectedCheckpointColor : e.GetColor();
		Vertex v = Vertex(Point(e.GetPos().x, e.GetPos().y, e.GetPos().z, c.r, c.g, c.b));
		const std::vector<Primitive> tris = v.ToGeometry();
		checkTriangles.insert(checkTriangles.end(), tris.begin(), tris.end());

		Model* label = checkpointModel->AddModel();
		label->GetMesh().SetGeometry("CP " + std::to_string(e.GetIndex()), Text3D::Align::CENTER, Color(c.r, c.g, c.b, static_cast<unsigned char>(255u)));
		Vec3 labelPos = e.GetPos();
		labelPos.y += labelHeightOffset;
		label->SetPosition(labelPos);
	}

	checkpointModel->GetMesh().SetGeometry(checkTriangles, Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags);
}


void Level::UpdateRenderBotData()
{
	Model* botModel = m_models[LevelModels::BOT];
	if (!botModel) { return; }
	botModel->ClearModels();

	// Check if any path has nodes at all
	bool anyNodes = false;
	for (const BotPath& path : m_botPaths)
		if (path.GetNodeCount() > 0) { anyNodes = true; break; }

	if (!anyNodes)
	{
		botModel->GetMesh().Clear();
		return;
	}

	// One fixed color per path (left, middle, right)
	static const Color pathColors[3] =
	{
		Color(0.86f, 0.31f, 0.31f), // left   red
		Color(0.31f, 0.78f, 0.31f), // mid    green
		Color(0.31f, 0.51f, 0.86f), // right  blue
	};

	constexpr float labelHeightOffset = 1.5f;
	std::vector<Primitive> botTriangles;

	for (int pathIndex = 0; pathIndex < 3; pathIndex++)
	{
		const BotPath& path = m_botPaths[pathIndex];
		const Color& c = pathColors[pathIndex % 3];

		for (size_t nodeIndex = 0; nodeIndex < path.GetNodeCount(); nodeIndex++)
		{
			const BotNode& node = path.GetNode(nodeIndex);
			const Vec3& pos = node.GetPos();

			Vertex v = Vertex(Point(pos.x, pos.y, pos.z, c.r, c.g, c.b));
			const std::vector<Primitive> tris = v.ToGeometry();
			botTriangles.insert(botTriangles.end(), tris.begin(), tris.end());

			Model* label = botModel->AddModel();
			label->GetMesh().SetGeometry(
				std::to_string(nodeIndex),
				Text3D::Align::CENTER,
				Color(c.r, c.g, c.b, 255u)
			);
			Vec3 labelPos = pos;
			labelPos.y += labelHeightOffset;
			label->SetPosition(labelPos);
		}
	}

	botModel->GetMesh().SetGeometry(
		botTriangles,
		Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags
	);
}



void Level::GenerateRenderInstanceData()
{
	Model* instanceModel = m_models[LevelModels::INSTANCES];
	if (!instanceModel) { return; }

	instanceModel->ClearModels();

	if (m_instances.empty())
	{
		instanceModel->GetMesh().Clear();
		return;
	}

	constexpr float labelHeightOffset = 3.0f;

	for (size_t i = 0; i < m_instances.size(); i++)
	{
		const Instance& inst = m_instances[i];
		const Vec3& pos = inst.GetPos();
		const size_t modelKey = inst.GetModelKey();

		// Geometry child (always created, ensures stride = 2 per instance)
		Model* childModel = instanceModel->AddModel();
		if (true)
		{
			auto it = m_instanceModels.find(modelKey);
			if (it != m_instanceModels.end())
			{
				InstanceModel& instModel = it->second;
				std::vector<Primitive> primitives = instModel.GetGeometry();
				childModel->GetMesh().SetGeometry(primitives,
					Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags);			
			}
		}
		childModel->SetPosition(inst.GetPos());
		childModel->SetRotationYXZ(inst.GetRot());
		childModel->SetScale(inst.GetScale());

		// Label with instance name
		Model* label = instanceModel->AddModel();
		std::string labelText = inst.GetName();
		if (labelText.empty())
			labelText = "Instance " + std::to_string(i + 1);
		label->GetMesh().SetGeometry(labelText, Text3D::Align::CENTER, Color(static_cast<unsigned char>(0), static_cast<unsigned char>(200), static_cast<unsigned char>(200), static_cast<unsigned char>(255)));
		Vec3 labelPos = pos;
		labelPos.y += labelHeightOffset;
		label->SetPosition(labelPos);
	}

	// Placeholder mesh so parent Model::IsReady() returns true
	std::vector<Primitive> dummy;
	dummy.push_back(Tri(Point(0,0,0,0,0,0), Point(0,0,0,0,0,0), Point(0,0,0,0,0,0)));
	instanceModel->GetMesh().SetGeometry(
		dummy,
		Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags
	);
}


void Level::GenerateRenderStartpointData()
{
	if (!m_models[LevelModels::SPAWN]) { return; }

	std::vector<Primitive> spawnsTriangles;
	spawnsTriangles.reserve(m_spawn.size() * 8);

	for (const Spawn& e : m_spawn)
	{
		Vertex v = Vertex(Point(e.pos.x, e.pos.y, e.pos.z, 0, 128, 255));
		const std::vector<Primitive> tris = v.ToGeometry();
		spawnsTriangles.insert(spawnsTriangles.end(), tris.begin(), tris.end());
	}

	m_models[LevelModels::SPAWN]->GetMesh().SetGeometry(spawnsTriangles, Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags);
}

void Level::GenerateRenderMinimapBoundsData()
{
	if (!m_models[LevelModels::MINIMAP_BOUNDS]) { return; }

	if (m_minimapConfig.texture.IsEmpty())
	{
		m_models[LevelModels::MINIMAP_BOUNDS]->GetMesh().Clear();
		return;
	}

	// Add some height to the minimap bounds for better visibility
	float minY = -10.0f;
	float maxY = 10.0f;

	// Create bounding box for minimap bounds
	BoundingBox bbox;
	bbox.min = Vec3(m_minimapConfig.worldStartX, minY, m_minimapConfig.worldStartZ);
	bbox.max = Vec3(m_minimapConfig.worldEndX, maxY, m_minimapConfig.worldEndZ);

	// Magenta color for minimap bounds
	Color c = Color(static_cast<unsigned char>(255), static_cast<unsigned char>(0), static_cast<unsigned char>(255));

	std::vector<Primitive> triangles = bbox.ToGeometry();
	for (Primitive& primitive : triangles)
	{
		for (unsigned i = 0; i < primitive.pointCount; i++) { primitive.p[i].color = c; }
	}

	m_models[LevelModels::MINIMAP_BOUNDS]->GetMesh().SetGeometry(triangles, Mesh::RenderFlags::DrawWireframe | Mesh::RenderFlags::DontOverrideRenderFlags);
}

void Level::GenerateRenderSkyboxData()
{
	if (!m_models[LevelModels::SKYBOX]) { return; }

	std::vector<Primitive> triangles = m_skybox.ToGeometry(m_bsp.GetBoundingBox());
	m_models[LevelModels::SKYBOX]->GetMesh().SetGeometry(triangles, Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags);
}

void Level::GenerateRenderSelectedBlockData(const Quadblock& quadblock, const Vec3& queryPoint)
{
	if (!m_models[LevelModels::SELECTED]) { return; }

	m_rendererQueryPoint = queryPoint;

	std::vector<Primitive> triangles;
	triangles.reserve(m_rendererSelectedQuadblockIndexes.size() * 8 + 8);

	const std::filesystem::path emptyTexturePath;
	const std::array<QuadUV, NUM_FACES_QUADBLOCK + 1> emptyUvs = {};
	for (size_t index : m_rendererSelectedQuadblockIndexes)
	{
		const Quadblock& qb = m_quadblocks[index];
		std::vector<Primitive> qbTriangles = qb.ToGeometry(false, &emptyUvs, &emptyTexturePath);
		for (Primitive& primitive : qbTriangles)
		{
			for (unsigned i = 0; i < primitive.pointCount; i++) { primitive.p[i].color = primitive.p[i].color.Negated(); }
			triangles.push_back(primitive);
		}
	}

	Vertex v = Vertex(Point(queryPoint.x, queryPoint.y, queryPoint.z, 255, 0, 0));
	const std::vector<Primitive> queryTriangles = v.ToGeometry();
	triangles.insert(triangles.end(), queryTriangles.begin(), queryTriangles.end());

	m_models[LevelModels::SELECTED]->GetMesh().SetGeometry(triangles,
		Mesh::RenderFlags::DrawWireframe | Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::ForceDrawOnTop | Mesh::RenderFlags::DrawLinesAA | Mesh::RenderFlags::DontOverrideRenderFlags | Mesh::RenderFlags::QuadblockLod,
		Mesh::ShaderFlags::Blinky);

	if (GuiRenderSettings::showVisTree)
	{
		std::vector<const BSP*> bspLeaves = m_bsp.GetLeaves();
		size_t myBSPIndex = 0;
		for (size_t bsp_index = 0; bsp_index < bspLeaves.size(); bsp_index++)
		{
			const BSP& bsp = *bspLeaves[bsp_index];
			if (bsp.GetId() == quadblock.GetBSPID()) { myBSPIndex = bsp_index; }
		}

		std::vector<Primitive> multiTriangles;
		for (size_t bsp_index = 0; bsp_index < bspLeaves.size(); bsp_index++)
		{
			const BSP& bsp = *bspLeaves[bsp_index];
			if (m_bspVis.Get(myBSPIndex, bsp_index))
			{
				const std::vector<size_t> qbIndeces = bsp.GetQuadblockIndexes();
				for (size_t qbInd : qbIndeces)
				{
					Quadblock& qb = m_quadblocks[qbInd];
					std::vector<Primitive> qbTriangles = qb.ToGeometry(false, &emptyUvs, &emptyTexturePath);
					for (Primitive& primitive : qbTriangles)
					{
						for (unsigned i = 0; i < primitive.pointCount; i++) { primitive.p[i].color = primitive.p[i].color.Negated(); }
						multiTriangles.push_back(primitive);
					}
				}
			}
		}

		m_models[LevelModels::MULTI_SELECTED]->GetMesh().SetGeometry(multiTriangles,
			Mesh::RenderFlags::DrawWireframe | Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::ForceDrawOnTop | Mesh::RenderFlags::DrawLinesAA | Mesh::RenderFlags::DontOverrideRenderFlags | Mesh::RenderFlags::QuadblockLod,
			Mesh::ShaderFlags::Blinky);
	}
}

void Level::ViewportClickHandleBlockSelection(int pixelX, int pixelY, bool appendSelection, const Renderer& rend)
{
	std::function<std::optional<std::tuple<const Quadblock*, const glm::vec3>>(int, int, std::vector<Quadblock>&, unsigned)> check = [&rend](int pixelCoordX, int pixelCoordY, const std::vector<Quadblock>& qbs, unsigned index)
		{
			std::vector<std::tuple<const Quadblock*, glm::vec3, float>> passed;

			for (const Quadblock& qb : qbs)
			{
				bool collided = false;
				const Vertex* verts = qb.GetUnswizzledVertices();
				glm::vec3 tri[3];
				bool isQuadblock = qb.IsQuadblock();

				std::tuple<glm::vec3, float> queryResult;
				glm::vec3 worldSpaceRay = rend.ScreenspaceToWorldRay(pixelCoordX, pixelCoordY);

				tri[0] = glm::vec3(verts[0].m_pos.x, verts[0].m_pos.y, verts[0].m_pos.z);
				tri[1] = glm::vec3(verts[2].m_pos.x, verts[2].m_pos.y, verts[2].m_pos.z);
				tri[2] = glm::vec3(verts[6].m_pos.x, verts[6].m_pos.y, verts[6].m_pos.z);

				queryResult = rend.WorldspaceRayTriIntersection(worldSpaceRay, tri);
				collided |= (std::get<1>(queryResult) != -1.0f);

				if (collided) { passed.push_back(std::tuple<const Quadblock*, glm::vec3, float>(&qb, std::get<0>(queryResult), std::get<1>(queryResult))); continue; }

				if (!isQuadblock) { continue; }

				tri[0] = glm::vec3(verts[2].m_pos.x, verts[2].m_pos.y, verts[2].m_pos.z);
				tri[1] = glm::vec3(verts[6].m_pos.x, verts[6].m_pos.y, verts[6].m_pos.z);
				tri[2] = glm::vec3(verts[8].m_pos.x, verts[8].m_pos.y, verts[8].m_pos.z);

				queryResult = rend.WorldspaceRayTriIntersection(worldSpaceRay, tri);
				collided |= (std::get<1>(queryResult) != -1.0f);

				if (collided) { passed.push_back(std::tuple<const Quadblock*, glm::vec3, float>(&qb, std::get<0>(queryResult), std::get<1>(queryResult))); continue; }
			}

			// sort collided blocks by time value (distance from camera).
			std::sort(passed.begin(), passed.end(),
				[](const std::tuple<const Quadblock*, glm::vec3, float>& a, const std::tuple<const Quadblock*, glm::vec3, float>& b) {
					return std::get<2>(a) < std::get<2>(b);
				});

			std::optional<std::tuple<const Quadblock*, glm::vec3>> result;
			if (passed.size() > 0)
			{
				const auto& tuple = passed[index % passed.size()];
				const Quadblock* qb = std::get<0>(tuple);
				result = std::make_optional(std::tuple<const Quadblock*, glm::vec3>(qb, std::get<1>(tuple)));
			}
			else { result.reset(); }
			return result;
		};

	static int lastClickedX = pixelX;
	static int lastClickedY = pixelY;
	static int indenticalClickTimes = -1;

	if (!appendSelection && pixelX == lastClickedX && pixelY == lastClickedY)
	{
		indenticalClickTimes++;
	}
	else
	{
		lastClickedX = pixelX;
		lastClickedY = pixelY;
		indenticalClickTimes = 0;
	}

	std::optional<std::tuple<const Quadblock*, const glm::vec3>> collidedQB = check(pixelX, pixelY, m_quadblocks, indenticalClickTimes);

	if (collidedQB.has_value())
	{
		const Quadblock* clickedQuadblock = std::get<0>(collidedQB.value());
		glm::vec3 p = std::get<1>(collidedQB.value());
		Vec3 point = Vec3(p.x, p.y, p.z);
		size_t clickedIndex = REND_NO_SELECTED_QUADBLOCK;
		for (size_t i = 0; i < m_quadblocks.size(); i++)
		{
			if (&m_quadblocks[i] == clickedQuadblock)
			{
				clickedIndex = i;
				break;
			}
		}

		if (clickedIndex != REND_NO_SELECTED_QUADBLOCK)
		{
			if (!appendSelection) { m_rendererSelectedQuadblockIndexes.clear(); }

			auto selectedIt = std::find(m_rendererSelectedQuadblockIndexes.begin(), m_rendererSelectedQuadblockIndexes.end(), clickedIndex);
			bool alreadySelected = selectedIt != m_rendererSelectedQuadblockIndexes.end();
			if (appendSelection && alreadySelected)
			{
				m_rendererSelectedQuadblockIndexes.erase(selectedIt);
			}
			else if (!alreadySelected)
			{
				m_rendererSelectedQuadblockIndexes.push_back(clickedIndex);
			}
		}

		GenerateRenderSelectedBlockData(*clickedQuadblock, point);
	}
	else
	{
		m_models[LevelModels::SELECTED]->GetMesh().Clear();
	}
	UpdateRenderCheckpointData();
}
