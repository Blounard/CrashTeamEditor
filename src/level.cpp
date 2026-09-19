#include "level.h"
#include "psx_types.h"
#include "io.h"
#include "utils.h"
#include "geo.h"
#include "process.h"
#include "settings.h"
#include "renderer.h"
#include "vistree.h"
#include "text3d.h"
#include "minimap.h"

#include <fstream>
#include <unordered_set>
#include <set>
#include <map>
#include <algorithm>
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

bool Level::IsLoaded() const
{
	return m_loaded;
}

bool Level::HasRawTexture() const
{
	return m_hasRawTexture;
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
	m_materialToQuadFaces.clear();
	m_materialToTexture.clear();
	m_checkpointPaths.clear();
	m_tropyGhost.clear();
	m_oxideGhost.clear();
	m_animTextures.clear();
	m_rendererQueryPoint = Vec3();
	m_rendererSelectedQuadblockIndexes.clear();
	m_bspVis.Clear();
	m_pythonConsole.clear();
	m_saveScript = false;
	m_vrm.clear();
	m_lastAnimTextureCount = 0;
	m_minimap = {};
	DeleteMaterials(this);
	m_skybox.Clear();
	m_splitLines[0] = 0.0;
	m_splitLines[1] = 0.0;
	m_jumpYSpeedCap = 0;
	for (int i = 0; i < 3; i++)
	{
		m_botPaths[i].Clear();
	}
	for (Model* model : m_models)
	{
		if (model) { model->Clear(model != m_models[LevelModels::LEVEL]); }
	}
	m_hasRawTexture = false;
	m_envMapTex.ClearTexture();
	m_rawWaterLayout = {};
	m_materialCache.clear();
	m_textureToPixelBounds.clear();
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

const std::filesystem::path& Level::GetParentPath() const
{
	return m_parentPath;
}

std::vector<std::string> Level::GetMaterialNames() const
{
	std::vector<std::string> names;
	names.reserve(m_materialToTexture.size());
	for (const auto& [key, value] : m_materialToTexture) { names.push_back(key); }
	return names;
}

std::vector<size_t> Level::GetMaterialQuadblockIndexes(const std::string& material) const
{
	if (!m_materialToQuadFaces.contains(material)) { return std::vector<size_t>(); }
	std::vector<size_t> res;
	for (auto& quadFace : m_materialToQuadFaces.at(material))
	{
		res.push_back(quadFace.first);
	}
	return res;
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
	float yaw = -std::atan2(forward.z, forward.x) * (180.0f / MATH_PI);
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
			float forwardOffset = (0.5f - row) * rowSpacing;
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
	m_bsp.Generate(m_quadblocks);
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

bool Level::GenerateVisTreeLev()
{
	if (m_bsp.IsValid())
	{
		m_bspVis = GenerateVisTree(m_quadblocks, &m_bsp);
		return true;
	}
	return false;
}

void Level::GenerateBotPathChangeCode()
{
	constexpr float DIST_NEXT_NODE = 10.0f;
	auto findTargetNode = [](const std::vector<BotNode>& targetNodes, const Vec3& pos) -> int
		{
			size_t nodeCount = targetNodes.size();
			// Find closest first
			int   targetIndex = 0;
			float bestDist = std::numeric_limits<float>::max();
			for (int i = 0; i < static_cast<int>(targetNodes.size()); i++)
			{
				const float dist = (targetNodes[i].GetPos() - pos).LengthSquared();
				if (dist < bestDist)
				{
					bestDist = dist;
					targetIndex = i;
				}
			}
			// Take a node some distance after
			float dist = 0.0f;
			int k = 0;
			while (dist < DIST_NEXT_NODE && k < 15)
			{
				dist += (targetNodes[(targetIndex + 1) % nodeCount].GetPos() - targetNodes[targetIndex].GetPos()).Length();
				targetIndex = (targetIndex + 1) % nodeCount;
				k++;
			}
			return targetIndex;
		};

	for (int i = 0; i < 3; i++)
	{
		if (!m_botPaths[i].IsValid()) return;
	}
	const std::vector<BotNode>& leftNodes = m_botPaths[0].GetNodes();
	const std::vector<BotNode>& middleNodes = m_botPaths[1].GetNodes();
	const std::vector<BotNode>& rightNodes = m_botPaths[2].GetNodes();

	for (int i = 0; i < static_cast<int>(leftNodes.size()); i++)
	{
		BotNode& node = m_botPaths[0].GetNode(i);
		const int closestMid = findTargetNode(middleNodes, node.GetPos());
		node.SetPathChange(1);
		node.SetPathChangeIndex(closestMid);
	}

	for (int i = 0; i < static_cast<int>(rightNodes.size()); i++)
	{
		BotNode& node = m_botPaths[2].GetNode(i);
		const int closestMid = findTargetNode(middleNodes, node.GetPos());
		node.SetPathChange(1);
		node.SetPathChangeIndex(closestMid);
	}

	for (int i = 0; i < static_cast<int>(middleNodes.size()); i++)
	{
		BotNode& node = m_botPaths[1].GetNode(i);
		if (i % 2 == 0)
		{
			node.SetPathChange(0);
			node.SetPathChangeIndex(findTargetNode(leftNodes, node.GetPos()));
		}
		else
		{
			node.SetPathChange(2);
			node.SetPathChangeIndex(findTargetNode(rightNodes, node.GetPos()));
		}
	}
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
	bool anyOverlap = false;
	int pathId = 0;
	for (Path& path : m_checkpointPaths)
	{
		bool overlap = false;
		pathCheckpoints.push_back(path.GeneratePath(checkpointIndex, m_quadblocks, overlap));
		checkpointIndex += pathCheckpoints.back().size();
		linkNodeIndexes.push_back(path.GetStart());
		linkNodeIndexes.push_back(path.GetEnd());
		anyOverlap = anyOverlap || overlap;
		if (overlap)
		{
			m_showLogWindow = true;
			m_logMessage += "\n\nWarning : Path " + std::to_string(pathId) + " is touching a previous path.";
			m_logMessage += "\nMake sure that all your path are not sharing any quadblocks.";
			m_logMessage += "\nIf 2 paths touch each other, make sure to specify the Quadblock Ignore List to detach both path.";
		}
		pathId++;
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
				distToNextMap.insert({ distToNext, i });
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
				for (auto& mapIt = range.first; mapIt != range.second; ++mapIt)
				{
					if (mapIt->second == upIndex)
					{
						distToNextMap.erase(mapIt);
						break;
					}
				}

				currentDistances[upIndex] = newDist;
				distToNextMap.insert({ newDist, upIndex });
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
	return !anyOverlap;
}


bool Level::GenerateOceanVertices()
{
	const int brightCyclesTime = WaterAnimSettings::brightWaveCycle;
	const float baseBright = WaterAnimSettings::baseBrightness;
	const float waveLength = std::max(WaterAnimSettings::waveLength, 1.0f);
	const float waveK = 2.0f * MATH_PI / waveLength;

	for (Quadblock& quad : m_quadblocks)
	{
		if (!quad.GetWater())
			continue;
		const std::vector<Vertex>& vertices = quad.GetVertices();
		for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
		{
			Vec3 vPos = vertices[i].m_pos;
			const float baseU = vPos.x * WaterAnimSettings::sizeTex;
			const float baseV = vPos.z * WaterAnimSettings::sizeTex;
			const float spaceWave = (std::cos(waveK * vPos.x) + std::cos(waveK * vPos.z)) / 2;

			PSX::OceanVertex ov{};
			for (int f = 0; f < NUM_FRAME_OVERT; f++)
			{
				const float frac = static_cast<float>(f) / NUM_FRAME_OVERT;

				const float scrollU = WaterAnimSettings::ScrollULoops * 64.0f * frac;
				const float scrollV = WaterAnimSettings::ScrollVLoops * 64.0f * frac;
				const float waveU = WaterAnimSettings::waveAmplitude * spaceWave * std::sin(2.0f * MATH_PI * WaterAnimSettings::waveCyclesTimeU * frac);
				const float waveV = WaterAnimSettings::waveAmplitude * spaceWave * std::sin(2.0f * MATH_PI * WaterAnimSettings::waveCyclesTimeV * frac);
				const int u = static_cast<int>(std::round(baseU + scrollU + waveU));
				const int v = static_cast<int>(std::round(baseV + scrollV + waveV));

				const float brightTemporalPhase = 2.0f * MATH_PI * brightCyclesTime * frac;
				const float waveBright = WaterAnimSettings::brightAmp * std::sin(brightTemporalPhase) * spaceWave;
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


// Helper functions for GenerateMinimap (calculate how much area a triangle cover within a square)
// AI
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

bool Level::GenerateMinimap()
{
	int targetHeight = MinimapSettings::textureHeight;
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
		if (MinimapSettings::checkpointQuads && m_quadblocks[i].GetCheckpoint() != -1)
			usedQuadIds.push_back(i);
		else if (MinimapSettings::checkpointPathableQuads && m_quadblocks[i].GetCheckpointPathable() && m_quadblocks[i].GetCheckpointStatus())
			usedQuadIds.push_back(i);
		else
		{
			for (size_t f = 0; f < NUM_FACES_QUADBLOCK + 1; f++)
			{
				if (MinimapSettings::materials.contains(m_quadblocks[i].GetMaterial(f)))
				{
					usedQuadIds.push_back(i);
					break;
				}
			}
		}
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
	m_minimap.worldBox = BoundingBox::Empty();
	for (const Tri& t : tris)
	{
		for (int i = 0; i < 3; i++)
			m_minimap.worldBox.Expand(t.p[i].pos);
	}

	const float spanX = m_minimap.worldBox.AxisLength().x;
	const float spanZ = m_minimap.worldBox.AxisLength().z;

	// World -> pixel mapping
	if (static_cast<MinimapOrientation>(MinimapSettings::orientation) == MinimapOrientation::AUTO)
		if (spanX > spanZ)
			m_minimap.orientationMode = MinimapOrientation::DOWN;
		else
			m_minimap.orientationMode = MinimapOrientation::RIGHT;
	else
		m_minimap.orientationMode = static_cast<MinimapOrientation>(MinimapSettings::orientation);

	const bool swapped = (m_minimap.orientationMode == MinimapOrientation::DOWN || m_minimap.orientationMode == MinimapOrientation::UP);
	const float colSpanWorld = swapped ? spanZ : spanX;
	const float rowSpanWorld = swapped ? spanX : spanZ;
	constexpr float minimapStretchX = 1.6f;
	const int contentWidth = std::max(1, static_cast<int>(std::lround(contentHeight * (colSpanWorld * minimapStretchX) / rowSpanWorld)));
	const int targetWidth = contentWidth + 1; // One extra column reserved the same way as the padding row (see below).

	auto toPixelSpace = [&](const Vec3& worldPos) // Convert World Pos to Pixel coordinate on the image
		{
			float x = worldPos.x, z = worldPos.z;
			float colFrac = 0.0, rowFrac = 0.0;
			switch (m_minimap.orientationMode)
			{
			case MinimapOrientation::RIGHT: colFrac = (x - m_minimap.worldBox.min.x) / spanX; rowFrac = (z - m_minimap.worldBox.min.z) / spanZ; break;
			case MinimapOrientation::DOWN:  colFrac = (m_minimap.worldBox.max.z - z) / spanZ; rowFrac = (x - m_minimap.worldBox.min.x) / spanX; break;
			case MinimapOrientation::LEFT:  colFrac = (m_minimap.worldBox.max.x - x) / spanX; rowFrac = (m_minimap.worldBox.max.z - z) / spanZ; break;
			case MinimapOrientation::UP:    colFrac = (z - m_minimap.worldBox.min.z) / spanZ; rowFrac = (m_minimap.worldBox.max.x - x) / spanX; break;
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
	switch (m_minimap.orientationMode)
	{
	case MinimapOrientation::RIGHT: m_minimap.worldBox.max.x += extCol; m_minimap.worldBox.max.z += extRow; break;
	case MinimapOrientation::DOWN:  m_minimap.worldBox.min.z -= extCol; m_minimap.worldBox.max.x += extRow; break;
	case MinimapOrientation::LEFT:  m_minimap.worldBox.min.x -= extCol; m_minimap.worldBox.min.z -= extRow; break;
	case MinimapOrientation::UP:    m_minimap.worldBox.max.z += extCol; m_minimap.worldBox.min.x -= extRow; break;
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
		if (color == 255)
			a = 255;
		else if (color == 0)
			a = 0;
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

	m_minimap.texture = Texture(pngPath);
	if (m_minimap.texture.IsEmpty())
	{
		printf("ERROR: Failed to load generated minimap texture %s\n", pngPath.string().c_str());
		return false;
	}
	m_minimap.texture.SetBlendMode(static_cast<uint16_t>(PSX::BlendMode::ADDITIVE));
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
				if (m_materialToQuadFaces.contains(material))
				{
					if (json.contains(material + "_terrain"))
					{
						m_propTerrain.SetPreview(material, json[material + "_terrain"]);
						m_propTerrain.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
					}
					if (json.contains(material + "_quadflags"))
					{
						m_propQuadFlags.SetPreview(material, json[material + "_quadflags"]);
						m_propQuadFlags.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
					}
					if (json.contains(material + "_drawflags"))
					{
						m_propDoubleSided.SetPreview(material, json[material + "_drawflags"]);
						m_propDoubleSided.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
					}
					if (json.contains(material + "_checkpoint"))
					{
						m_propCheckpoints.SetPreview(material, json[material + "_checkpoint"]);
						m_propCheckpoints.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
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
						m_propSpeedImpact.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
					}
					if (json.contains(material + "_weatherIntensity"))
					{
						m_propWeatherIntensity.SetPreview(material, json[material + "_weatherIntensity"]);
						m_propWeatherIntensity.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
					}
					if (json.contains(material + "_weatherVanishRate"))
					{
						m_propWeatherVanishRate.SetPreview(material, json[material + "_weatherVanishRate"]);
						m_propWeatherVanishRate.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
					}
					if (json.contains(material + "_checkpointPathable"))
					{
						m_propCheckpointPathable.SetPreview(material, json[material + "_checkpointPathable"]);
						m_propCheckpointPathable.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
					}
					if (json.contains(material + "_visTreeTransparent"))
					{
						m_propVisTreeTransparent.SetPreview(material, json[material + "_visTreeTransparent"]);
						m_propVisTreeTransparent.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
					}
					if (json.contains(material + "_drawOrderHigh"))
					{
						m_propDrawOrderHigh.SetPreview(material, json[material + "_drawOrderHigh"]);
						m_propDrawOrderHigh.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
					}
					if (json.contains(material + "_water"))
					{
						m_propWater.SetPreview(material, json[material + "_water"]);
						m_propWater.Apply(material, m_materialToQuadFaces[material], m_quadblocks);
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
			m_minimap = json["minimap"];
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

	if (!m_materialToQuadFaces.empty())
	{
		nlohmann::json materialJson = {};
		materialJson["header"] = PresetHeader::MATERIAL;
		std::vector<std::string> materials; materials.reserve(m_materialToQuadFaces.size());
		for (const auto& [key, value] : m_materialToQuadFaces)
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
	minimapJson["minimap"] = m_minimap;
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

	m_parentPath = levFile.parent_path();
	m_name = levFile.filename().replace_extension().string() + "_edit";

	uint32_t offPointerMap;
	Read(file, offPointerMap);

	std::streampos offLev = file.tellg();
	PSX::LevHeader header = {};
	Read(file, header);

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


	// Load .vrm
	std::filesystem::path vrmPath = levFile;
	vrmPath.replace_extension(".vrm");
	std::vector<uint16_t> vram = ReadRawVRAM(vrmPath);
	std::filesystem::path tempDir = levFile.parent_path() / (levFile.stem().string() + "_textures");
	std::filesystem::create_directories(tempDir);
	m_hasRawTexture = true;

	// WATER
	if (header.offEnvironmentMap != 0)
	{
		file.seekg(offLev + std::streampos(header.offEnvironmentMap));
		Read(file, m_rawWaterLayout);
		LayoutKey waterkey(m_rawWaterLayout);
		PixelBounds waterBound{};
		waterBound.Update(RawUV(m_rawWaterLayout));
		m_envMapTex = Texture(waterkey, waterBound, vram, "envMap", tempDir);
	}
	//ICONS
	std::vector<PSX::Icon> levelIcons;
	Texture minimapTop; Texture minimapBottom; Texture minimapMerged;
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
			}
		}

		for (PSX::Icon& icon : levelIcons)
		{
			if (icon.globalIconArrayIndex == PSX::ICON_INDEX_MAP_TOP)
			{
				LayoutKey mapKey(icon.texLayout);
				PixelBounds bounds{};
				bounds.Update(RawUV(icon.texLayout));
				mapKey.blendMode = PSX::BlendMode::ADDITIVE; // Blend mode is not used for rendering minimaps, additive alwyas used ? (or a minimap specific one).
				minimapTop = Texture(mapKey, bounds, vram, "minimap_top", tempDir);
			}
			if (icon.globalIconArrayIndex == PSX::ICON_INDEX_MAP_BOTTOM)
			{
				LayoutKey mapKey(icon.texLayout);
				PixelBounds bounds{};
				bounds.Update(RawUV(icon.texLayout));
				mapKey.blendMode = PSX::BlendMode::ADDITIVE;
				minimapBottom = Texture(mapKey, bounds, vram, "minimap_bottom", tempDir);
			}
		}
		if (!minimapTop.IsEmpty() && !minimapBottom.IsEmpty())
		{
			minimapMerged = Texture(minimapTop, minimapBottom, "minimap", tempDir);
		}
	}

	// Load textures, AnimTex, and Quadblocks
	std::map<size_t, std::array<uint32_t, NUM_FACES_QUADBLOCK>> quadblockFaceToAnimOffset; // Map: quadblock index -> Array animTexOffset per face
	std::vector<uint32_t> quadblocksVisibleSetOff; // List of VisibleSetOffset for quadblock. Needed for vistree loading, parsed with quadblocks.
	std::set<uint32_t> parsedAnimTexOffset;
	int texCounter = 0;
	bool hasAnimData = header.offAnimTex > 0;
	size_t offAnimStart = header.offAnimTex;

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
			if (texOffset == 0)
				continue;
			uint32_t ptrAnimatedFlag = texOffset & 0x3;
			uint32_t realOffset = texOffset & ~0x3;

			if (hasAnimData && ptrAnimatedFlag > 0) // Anim Textures
			{
				if (f < NUM_FACES_QUADBLOCK)
				{
					if (!quadblockFaceToAnimOffset.contains(i))
						quadblockFaceToAnimOffset[i] = {}; // init array with 0s
					quadblockFaceToAnimOffset[i][f] = realOffset;
				}

				if (!parsedAnimTexOffset.contains(realOffset))
				{
					file.seekg(offLev + std::streampos(realOffset));
					PSX::AnimTex animTex;
					Read(file, animTex);
					parsedAnimTexOffset.insert(realOffset);

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
						const PSX::TextureLayout& layout = group.middle;
						LayoutKey key(layout);

						if (!m_materialCache.contains(key))
						{
							std::string newMatName = "tex_" + std::to_string(texCounter++);
							m_materialCache[key] = newMatName;
						}
						RawUV rawUV(layout);
						m_textureToPixelBounds[key].Update(rawUV);
					}
				}
			}
			else // Regular Textures
			{
				file.seekg(offLev + static_cast<std::streamoff>(texOffset));
				PSX::TextureGroup group = {};
				Read(file, group);
				const PSX::TextureLayout& layout = group.middle;
				LayoutKey key(layout);

				if (!m_materialCache.contains(key))
				{
					std::string newMatName = "tex_" + std::to_string(texCounter++);
					m_materialCache[key] = newMatName;
				}
				RawUV rawUV(layout);
				m_textureToPixelBounds[key].Update(rawUV);
			}
		}
		file.seekg(currentPosQuad);
	}

	// 3rd pass : Create PNGs and Materials
	for (const auto& [key, bounds] : m_textureToPixelBounds)
	{
		std::string newMatName = m_materialCache[key];
		Texture newTexture(key, bounds, vram, newMatName, tempDir);
		m_materialToTexture[newMatName] = newTexture;
	}

	std::map<uint32_t, std::vector<std::tuple<size_t, size_t>>> vertToQuad; // Map vertex absolute offset -> List of (quad, vert id) containing the vert.
	// 4th pass : create quadblocks with material, UVs and texture	
	file.seekg(offLev + std::streampos(meshInfo.offQuadblocks));
	for (uint32_t i = 0; i < meshInfo.numQuadblocks; i++)
	{
		file.seekg(offLev + std::streampos(meshInfo.offQuadblocks + i * sizeof(PSX::Quadblock)));
		PSX::Quadblock psxQuad = {};
		Read(file, psxQuad);
		quadblocksVisibleSetOff.push_back(psxQuad.offVisibleSet);
		Quadblock& qb = m_quadblocks.emplace_back(psxQuad, vertices, [this](const Quadblock& qb) { UpdateFilterRenderData(qb); });
		bool materialAssigned = false;
		std::string qbMatName = "default";
		for (int f = 0; f < NUM_FACES_QUADBLOCK + 1; f++)
		{
			uint32_t texOffset = f == NUM_FACES_QUADBLOCK ? psxQuad.offLowTexture : psxQuad.offMidTextures[f];
			if (texOffset == 0)
			{
				qb.SetMaterial(f, "default");
				m_materialToQuadFaces["default"].push_back(std::make_pair(i, f));
				continue;
			}

			uint32_t ptrAnimatedFlag = texOffset & 0x3;
			uint32_t realOffset = texOffset & ~0x3;

			if (!(hasAnimData && ptrAnimatedFlag > 0))
			{
				file.seekg(offLev + static_cast<std::streamoff>(texOffset));
				PSX::TextureGroup group = {};
				Read(file, group);

				const PSX::TextureLayout& layout = group.middle;
				LayoutKey key(layout);

				qbMatName = m_materialCache[key];
				qb.SetMaterial(f, qbMatName);
				qb.SetTexPath(f, m_materialToTexture[qbMatName].GetPath());
				m_materialToQuadFaces[qbMatName].push_back(std::make_pair(i, f));

				RawUV rawUV(layout);
				const PixelBounds& bounds = m_textureToPixelBounds[key];
				qb.SetFaceUVs(f, ConvertUV(bounds, rawUV));
			}
			else // for now assign mat and UVs of the 1st frame of animation
			{
				file.seekg(offLev + static_cast<std::streamoff>(realOffset));
				PSX::AnimTex animTex{};
				Read(file, animTex);
				if (animTex.frameCount > 0)
				{
					uint32_t frameTexOffset;
					Read(file, frameTexOffset);
					if (frameTexOffset > 0)
					{
						file.seekg(offLev + static_cast<std::streamoff>(frameTexOffset));
						PSX::TextureGroup group = {};
						Read(file, group);
						const PSX::TextureLayout& layout = group.middle;
						LayoutKey key(layout);

						qbMatName = m_materialCache[key];
						qb.SetMaterial(f, qbMatName);
						qb.SetTexPath(f, m_materialToTexture[qbMatName].GetPath());
						m_materialToQuadFaces[qbMatName].push_back(std::make_pair(i, f));

						RawUV rawUV(layout); // TODO VERIFY IF WE NEED THE draworderlow
						const PixelBounds& bounds = m_textureToPixelBounds[key];
						qb.SetFaceUVs(f, ConvertUV(bounds, rawUV));
						continue;
					}
				}
				qb.SetMaterial(f, "default");
				m_materialToQuadFaces["default"].push_back(std::make_pair(i, f));
			}
		}

		for (size_t j = 0; j < NUM_VERTICES_QUADBLOCK; j++)
		{
			vertToQuad[static_cast<uint32_t>(meshInfo.offVertices + psxQuad.index[j] * sizeof(PSX::Vertex))].push_back(std::make_tuple(i, j));
		}
	}

	//5th pass : Create AnimTexture objects for fully-animated quadblocks, assign to quads
	std::map<size_t, std::array<std::vector<std::pair<std::string, QuadUV>>, NUM_FACES_QUADBLOCK>> quadblockAnimFrames;

	if (hasAnimData)
	{
		for (const auto& [quadIdx, offsets] : quadblockFaceToAnimOffset)
		{
			std::array<std::vector<std::pair<std::string, QuadUV>>, NUM_FACES_QUADBLOCK> faceFrames;
			size_t frameCount = 0;
			bool validAnimation = true;

			for (size_t face = 0; face < NUM_FACES_QUADBLOCK && validAnimation; face++)
			{
				if (offsets[face] == 0) { validAnimation = false; break; }
				file.seekg(offLev + static_cast<std::streamoff>(offsets[face]));
				PSX::AnimTex animTex{};
				Read(file, animTex);
				std::streampos frameTexOffsetsPos = file.tellg();

				if (face == 0) { frameCount = animTex.frameCount; }
				else if (animTex.frameCount != frameCount) { validAnimation = false; break; }

				for (uint32_t frame = 0; frame < frameCount; frame++)
				{
					file.seekg(frameTexOffsetsPos + static_cast<std::streamoff>(frame * sizeof(uint32_t)));
					uint32_t frameTexOffset;
					Read(file, frameTexOffset);
					if (frameTexOffset == 0) { validAnimation = false; break; }

					file.seekg(offLev + static_cast<std::streamoff>(frameTexOffset));
					PSX::TextureGroup group = {};
					Read(file, group);

					const PSX::TextureLayout& layout = group.middle;
					LayoutKey key(layout);
					const PixelBounds& bounds = m_textureToPixelBounds[key];
					RawUV rawUV(layout);
					faceFrames[face].push_back({ m_materialCache[key], ConvertUV(bounds, rawUV) });
				}
			}
			if (!validAnimation || frameCount == 0) { continue; }
			quadblockAnimFrames[quadIdx] = std::move(faceFrames);
		}

		std::map<std::array<uint32_t, NUM_FACES_QUADBLOCK>, std::vector<size_t>> offsetsToQuadblocks;
		for (const auto& [quadIdx, offsets] : quadblockFaceToAnimOffset)
		{
			if (quadblockAnimFrames.contains(quadIdx))
				offsetsToQuadblocks[offsets].push_back(quadIdx);
		}

		for (const auto& [offsets, quadIndices] : offsetsToQuadblocks)
		{
			size_t repQuadIdx = quadIndices[0];

			file.seekg(offLev + std::streampos(offsets[0]));
			PSX::AnimTex animTex;
			Read(file, animTex);
			std::string animName = "AnimTex_" + std::to_string(m_animTextures.size());
			AnimTexture animTexture(animTex, animName, tempDir, quadblockAnimFrames.at(repQuadIdx), m_materialToTexture);

			if (animTexture.IsEmpty())
			{
				printf("WARNING : Empty animtex '%s'\n", animName.c_str());
				continue;
			}

			for (size_t quadIdx : quadIndices)
			{
				animTexture.AddQuadblockIndex(quadIdx);
				m_quadblocks[quadIdx].SetAnimated(true);
			}
			m_animTextures.push_back(animTexture);
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

	// Load BSP
	m_bsp.Clear();
	file.seekg(offLev + std::streampos(meshInfo.offBSPNodes));
	std::vector<BSP*> bspArray;
	for (uint32_t i = 0; i < meshInfo.numBSPNodes; i++) { bspArray.push_back(new BSP()); }
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
			bspArray[branch.id]->PopulateBranch(branch, bspArray, meshInfo.numBSPNodes);
		}
	}
	if (!bspArray.empty())
	{
		m_bsp = *(bspArray[0]);
		m_bsp.PopulateBranchQuadIndexes();
		if (m_bsp.IsValid()) { GenerateRenderBspData(); }
		else { m_bsp.Clear(); printf("ERROR : Couldn't load BSP Tree : Empty leaves\n"); }
	}
	else { m_bsp.Clear(); }
	std::set<size_t> validID;
	std::vector<const BSP*> tree = static_cast<const BSP&>(m_bsp).GetTree();
	for (const BSP* bsp : tree) { validID.insert(bsp->GetId()); }
	for (BSP* bsp : bspArray) { if (!validID.contains(bsp->GetId())) { m_bsp.Clear(); printf("ERROR : Couldn't load BSP Tree : Missing IDs\n"); break ; } }


	m_bspVis.Clear();
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
	}

	// Load checkpoints
	file.seekg(offLev + std::streampos(header.offCheckpointNodes));
	for (uint32_t i = 0; i < header.numCheckpointNodes; i++)
	{
		PSX::Checkpoint checkpoint = {};
		Read(file, checkpoint);
		m_checkpoints.emplace_back(checkpoint, static_cast<int>(i));
	}
	UpdateRenderCheckpointData();

	// Load Ghosts
	m_tropyGhost.clear();
	m_oxideGhost.clear();
	if (header.offExtra > 0)
	{
		file.seekg(offLev + std::streampos(header.offExtra));
		PSX::LevelExtraHeader extraHeader = {};
		Read(file, extraHeader);
		// Read N. Tropy Ghost
		if (extraHeader.count >= PSX::LevelExtra::N_TROPY_GHOST + 1 && extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST] > 0)
		{
			file.seekg(offLev + std::streampos(extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST]));
			m_tropyGhost.resize(GHOST_DATA_FILESIZE);
			file.read(reinterpret_cast<char*>(m_tropyGhost.data()), GHOST_DATA_FILESIZE);
		}
		// Read N. Oxide Ghost
		if (extraHeader.count >= PSX::LevelExtra::N_OXIDE_GHOST + 1 && extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] > 0)
		{
			file.seekg(offLev + std::streampos(extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST]));
			m_oxideGhost.resize(GHOST_DATA_FILESIZE);
			file.read(reinterpret_cast<char*>(m_oxideGhost.data()), GHOST_DATA_FILESIZE);
		}

		// Read minimap
		if (extraHeader.count > PSX::LevelExtra::MINIMAP && extraHeader.offsets[PSX::LevelExtra::MINIMAP] != 0)
		{
			file.seekg(offLev + std::streampos(extraHeader.offsets[PSX::LevelExtra::MINIMAP]));
			PSX::Minimap minimap{};
			Read(file, minimap);
			m_minimap = ConvertMinimap(minimap);
			m_minimap.texture = minimapMerged;
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

	// Load BotNodes
	if (header.offLevNavTable != 0)
	{
		file.seekg(offLev + std::streampos(header.offLevNavTable));
		PSX::levAINavTable navTable{};
		Read(file, navTable);
		for (int i = 0; i < 3; i++)
		{
			if (navTable.offAIPathArray[i] != 0)
			{
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
				}
				m_botPaths[i] = BotPath(navHeader, nodes);
			}
		}
		UpdateRenderBotData();
	}

	m_loaded = true;
	file.close();
	GenerateRenderLevData();
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
	*		- PointerMap
	*/
	m_hotReloadLevPath = path / (m_name + ".lev");
	std::ofstream file(m_hotReloadLevPath, std::ios::binary);


	if (m_bsp.IsEmpty()) { GenerateBSP(); }
	ReOrderBSP();

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
	size_t currOffset = sizeof(header);

	PSX::MeshInfo meshInfo = {};
	const size_t offMeshInfo = currOffset;
	currOffset += sizeof(meshInfo);

	const size_t offTexture = currOffset;
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
	std::unordered_map<std::string, LayoutKey> matToKey; // for useRawTex : material -> Layout Key
	for (const auto& [key, matName] : m_materialCache)
	{
		if (matToKey.contains(matName))
		{
			printf("WARNING : Material Cache have several Key with the same matName\n");
			continue;
		}
		matToKey[matName] = key;
	}
	if (useRawTextures || UpdateVRM())
	{
		for (Quadblock& quad : m_quadblocks)
		{
			if (quad.GetAnimated()) { continue; }
			for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
			{
				if (m_materialToTexture.contains(quad.GetMaterial(i)))
				{
					Texture& texture = m_materialToTexture[quad.GetMaterial(i)];
					if (!useRawTextures && (texture.IsEmpty() || !texture.IsPlaced())) { quad.SetTextureID(-1, i); continue; }
					size_t textureID = 0;
					const QuadUV& uvs = quad.GetQuadUV(i);
					PSX::TextureLayout layout{};
					if (useRawTextures)
					{
						if (matToKey.contains(quad.GetMaterial(i)))
						{
							LayoutKey& key = matToKey[quad.GetMaterial(i)];
							PixelBounds& bounds = m_textureToPixelBounds[key];
							layout = key.Serialize(quad.GetQuadUV(i), bounds);
						}
					}
					else
					{
						layout = texture.Serialize(uvs);
					}

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
					quad.SetTextureID(static_cast<int>(textureID), i);
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
					for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
					{
						if (i == NUM_FACES_QUADBLOCK && !firstFrame) { continue; }
						Texture& texture = const_cast<Texture&>(animTextures[frame.textureIndexes[i]]);
						size_t textureID = 0;
						const QuadUV& uvs = frame.uvs[i];

						PSX::TextureLayout layout{};
						if (useRawTextures)
						{
							std::string texName = texture.GetPath().filename().replace_extension().string();
							LayoutKey& key = matToKey[texName];
							PixelBounds& bounds = m_textureToPixelBounds[key];
							layout = key.Serialize(uvs, bounds);
						}
						else
						{
							layout = texture.Serialize(uvs);
						}

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
								m_quadblocks[index].SetTextureID(static_cast<int>(textureID), i);
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
						quadblock.SetAnimTextureOffset(static_cast<int>(animOffsetPerQuadblock[i][j] + offAnimData), j);
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

		if (!useRawTextures)
		{
			m_hotReloadVRMPath = path / (m_name + ".vrm");
			std::ofstream vrmFile(m_hotReloadVRMPath, std::ios::binary);
			Write(vrmFile, m_vrm.data(), m_vrm.size());
			vrmFile.close();
		}

	}
	else
	{
		texGroups.push_back(defaultTexGroup);
		offAnimData = currOffset + (sizeof(PSX::TextureGroup) * texGroups.size());
		for (size_t i = 0; i < sizeof(uint32_t); i++) { animData.push_back(0); }
		memcpy(&animData[0], &offAnimData, sizeof(uint32_t));
		animPtrMapOffsets.push_back(0);
	}

	currOffset += (sizeof(PSX::TextureGroup) * texGroups.size()) + animData.size();

	// Water texture
	PSX::TextureLayout envMapLayout{}; // must be 64x64, uvs don't matter
	if (useRawTextures)
		envMapLayout = m_rawWaterLayout;
	else
	{
		if (!m_envMapTex.IsEmpty())
		{
			envMapLayout = m_envMapTex.Serialize(QuadUV{ {Vec2(0.0f, 0.0f), Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f), Vec2(1.0f, 1.0f)} });
		}
	}
	const size_t offEnvMapLayout = currOffset;
	currOffset += sizeof(PSX::TextureLayout);

	const size_t offQuadblocks = currOffset;
	std::vector<std::vector<uint8_t>> serializedBSPs;
	std::vector<std::vector<uint8_t>> serializedQuads;
	std::vector<const Quadblock*> orderedQuads;
	std::unordered_map<Vertex, size_t> vertexMap;
	std::vector<Vertex> orderedVertices;
	std::unordered_map<PSX::OceanVertex, size_t> oVertexMap;
	std::vector<PSX::OceanVertex> orderedOVert;
	std::set<std::tuple<size_t, size_t>> waterVerticesIndexes; // (Vertex id, OVert id)
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
			for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
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
			visQuads = visibleQuadsAll; //Saves space, doesn't seem to cost performances. Vanilla does store all visible quad from visible leaves at this point
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
		visibleNodes.push_back({ visibleNodeAll, currOffset });
		uniqueVisNodes.push_back(visibleNodeAll);
		currOffset += visibleNodeAll.size() * sizeof(uint32_t);

		visibleQuads.push_back({ visibleQuadsAll, currOffset });
		uniqueVisQuads.push_back(visibleQuadsAll);
		currOffset += visibleQuadsAll.size() * sizeof(uint32_t);
	}

	std::vector<uint32_t> visibleInstancesDummy;
	visibleInstancesDummy.push_back(0);
	visibleInstances.push_back({ visibleInstancesDummy, currOffset });
	currOffset += visibleInstancesDummy.size() * sizeof(uint32_t);


	visibleExtra.push_back({ visibleExtraAll, currOffset });
	currOffset += visibleExtraAll.size() * sizeof(uint32_t);


	std::unordered_map<PSX::VisibleSet, size_t> visibleSetMap;
	std::vector<PSX::VisibleSet> visibleSets;
	const size_t offVisibleSet = currOffset;

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
		set.offVisibleInstances = static_cast<uint32_t>(std::get<size_t>(visibleInstances[0]));
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
	currOffset += visibleSets.size() * sizeof(PSX::VisibleSet);

	const size_t offWaterVertices = currOffset;
	std::vector<PSX::WaterVertex> waterVertices;
	std::vector<std::tuple<size_t, size_t>> orderedWaterVerticesIndexes;
	for (auto& tuple : waterVerticesIndexes)
	{
		PSX::WaterVertex waterVert{};
		waterVertices.push_back(waterVert);
		currOffset += sizeof(waterVert);
		orderedWaterVerticesIndexes.push_back(tuple);
	}

	const size_t offVertices = currOffset;
	std::vector<std::vector<uint8_t>> serializedVertices;
	for (const Vertex& vertex : orderedVertices)
	{
		serializedVertices.push_back(vertex.Serialize());
		currOffset += serializedVertices.back().size();
	}

	for (size_t i = 0; i < waterVertices.size(); i++)
	{
		std::tuple<size_t, size_t>& tuple = orderedWaterVerticesIndexes[i];
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
	std::vector<std::vector<uint8_t>> serializedCheckpoints;
	for (const Checkpoint& checkpoint : m_checkpoints)
	{
		serializedCheckpoints.push_back(checkpoint.Serialize());
		currOffset += serializedCheckpoints.back().size();
	}

	const size_t offTropyGhost = m_tropyGhost.empty() ? 0 : currOffset;
	currOffset += m_tropyGhost.size();

	const size_t offOxideGhost = m_oxideGhost.empty() ? 0 : currOffset;
	currOffset += m_oxideGhost.size();

	// Note: extraHeader.offsets[MINIMAP] will be updated later after minimap data is serialized
	PSX::LevelExtraHeader extraHeader = {};
	extraHeader.offsets[PSX::LevelExtra::MINIMAP] = 0;
	extraHeader.offsets[PSX::LevelExtra::SPAWN] = 0;
	extraHeader.offsets[PSX::LevelExtra::CAMERA_END_OF_RACE] = 0;
	extraHeader.offsets[PSX::LevelExtra::CAMERA_DEMO] = 0;
	extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST] = static_cast<uint32_t>(offTropyGhost);
	extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] = static_cast<uint32_t>(offOxideGhost);
	extraHeader.offsets[PSX::LevelExtra::CREDITS] = 0;

	// Determine count based on highest enabled entry index + 1
	// Count represents the number of valid entries in the offsets array
	if (offOxideGhost > 0) { extraHeader.count = PSX::LevelExtra::N_OXIDE_GHOST + 1; }
	else if (offTropyGhost > 0) { extraHeader.count = PSX::LevelExtra::N_TROPY_GHOST + 1; }
	// Note: minimap count will be set later if enabled and no ghosts present

	const size_t offExtraHeader = currOffset;
	currOffset += sizeof(extraHeader);

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
			serializedBotPaths.push_back(m_botPaths[i].Serialize());
			currOffset += serializedBotPaths.back().size();
		}
		else
		{
			navTable.offAIPathArray[i] = 0;
		}
	}

	std::vector<uint32_t> visMemNodesP1(visNodeSize);
	const size_t offVisMemNodesP1 = currOffset;
	currOffset += visMemNodesP1.size() * sizeof(uint32_t);

	std::vector<uint32_t> visMemQuadsP1(visQuadSize);
	const size_t offVisMemQuadsP1 = currOffset;
	currOffset += visMemQuadsP1.size() * sizeof(uint32_t);

	std::vector<uint32_t> visMemBSPP1(bspNodes.size() * 2);
	const size_t offVisMemBSPP1 = currOffset;
	currOffset += visMemBSPP1.size() * sizeof(uint32_t);

	std::vector<uint32_t> visMemOceanP1(visExtraSize);
	const size_t offvisMemOceanP1 = currOffset;
	currOffset += visMemOceanP1.size() * sizeof(uint32_t);

	PSX::VisualMem visMem = {};
	visMem.offNodes[0] = static_cast<uint32_t>(offVisMemNodesP1);
	visMem.offQuads[0] = static_cast<uint32_t>(offVisMemQuadsP1);
	visMem.offBSP[0] = static_cast<uint32_t>(offVisMemBSPP1);
	visMem.offOcean[0] = static_cast<uint32_t>(offvisMemOceanP1);
	const size_t offVisMem = currOffset;
	currOffset += sizeof(visMem);

	// Minimap data serialization
	size_t offMinimapStruct = 0;
	size_t offLevelIconHeader = 0;
	size_t offMinimapIcons = 0;
	std::vector<uint8_t> minimapData;
	std::vector<size_t> minimapPtrMapOffsets;

	if (!m_minimap.texture.IsEmpty())
	{
		// Map struct - this is what extraHeader.offsets[MINIMAP] will point to
		offMinimapStruct = currOffset;
		PSX::Minimap mapStruct = ConvertMinimap(m_minimap);
		size_t mapStructOffset = minimapData.size();
		minimapData.resize(minimapData.size() + sizeof(PSX::Minimap));
		memcpy(&minimapData[mapStructOffset], &mapStruct, sizeof(PSX::Minimap));
		currOffset += sizeof(PSX::Minimap);

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
		topIcon.texLayout = m_minimap.texture.Serialize(topUV);
		size_t topIconOffset = minimapData.size();
		minimapData.resize(minimapData.size() + sizeof(PSX::Icon));
		memcpy(&minimapData[topIconOffset], &topIcon, sizeof(PSX::Icon));
		currOffset += sizeof(PSX::Icon);

		// Bottom icon
		PSX::Icon bottomIcon = {};
		strncpy_s(bottomIcon.name, sizeof(bottomIcon.name), "minimap-bot", _TRUNCATE);
		bottomIcon.globalIconArrayIndex = PSX::ICON_INDEX_MAP_BOTTOM;
		bottomIcon.texLayout = m_minimap.texture.Serialize(bottomUV);
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
		
		// Set count if no ghosts are present (minimap is at index 0, so count = 1)
		if (extraHeader.count == 0) { extraHeader.count = PSX::LevelExtra::MINIMAP + 1; }
	}
	
	// Skybox data serialization
	size_t offSkyboxData = 0;
	std::vector<uint8_t> skyboxData;
	std::vector<size_t> skyboxPtrMapOffsets;

	if (m_skybox.IsReady())
	{
		offSkyboxData = currOffset;
		skyboxData = m_skybox.Serialize(offSkyboxData, skyboxPtrMapOffsets);
		currOffset += skyboxData.size();
	}

	const size_t offPointerMap = currOffset;

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
	header.offWaterVertices = static_cast<uint32_t>(offWaterVertices);
	header.numWaterVertices = static_cast<uint32_t>(waterVertices.size());
	header.offEnvironmentMap = static_cast<uint32_t>(offEnvMapLayout);


	// Set minimap pointers in header if enabled
	if (!m_minimap.texture.IsEmpty())
	{
		header.offIconsLookup = static_cast<uint32_t>(offLevelIconHeader);
		header.offIcons = static_cast<uint32_t>(offMinimapIcons);
	}

	// Set skybox pointer in header if enabled
	if (m_skybox.IsReady())
	{
		header.offSkybox = static_cast<uint32_t>(offSkyboxData);
	}

#define CALCULATE_OFFSET(s, m, b) static_cast<uint32_t>(offsetof(s, m) + b)

	std::vector<uint32_t> pointerMap =
	{
		CALCULATE_OFFSET(PSX::LevHeader, offMeshInfo, offHeader),
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
	
	// Add minimap header pointers to pointer map
	if (!m_minimap.texture.IsEmpty())
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::LevHeader, offIconsLookup, offHeader));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::LevHeader, offIcons, offHeader));
	}

	// Add skybox header pointer to pointer map
	if (m_skybox.IsReady())
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::LevHeader, offSkybox, offHeader));
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
		pointerMap.push_back(static_cast<uint32_t>(offVisMemBSPP1 + visMemListIndex * sizeof(uint32_t)));
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
	Write(file, &extraHeader, sizeof(extraHeader));
	Write(file, &navTable, sizeof(navTable));
	for (const std::vector<uint8_t>& serializedBotPath : serializedBotPaths) { Write(file, serializedBotPath.data(), serializedBotPath.size()); }
	Write(file, visMemNodesP1.data(), visMemNodesP1.size() * sizeof(uint32_t));
	Write(file, visMemQuadsP1.data(), visMemQuadsP1.size() * sizeof(uint32_t));
	Write(file, visMemBSPP1.data(), visMemBSPP1.size() * sizeof(uint32_t));
	Write(file, visMemOceanP1.data(), visMemOceanP1.size() * sizeof(uint32_t));
	Write(file, &visMem, sizeof(visMem));
	if (!minimapData.empty()) { Write(file, minimapData.data(), minimapData.size()); }
	if (!skyboxData.empty()) { Write(file, skyboxData.data(), skyboxData.size()); }
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
	std::vector<Point> globalVertices;
	std::vector<Vec2> uvs;
	std::unordered_map<std::string, bool> meshMap;
	std::unordered_set<std::string> materials;
	size_t quadblockCount = 0;

	std::string currQuadblockName;
	std::string activeMaterial;
	bool currQuadblockGoodUV = true;
	std::vector<Point> objPoints;
	std::unordered_map<size_t, size_t> objGlobalToLocal;
	std::vector<std::vector<size_t>> objFaceIndices;
	std::vector<std::vector<Vec2>> objFaceUVs;
	std::vector<std::string> objFaceMaterials;

	auto ResetObjState = [&]()
		{
			objPoints.clear();
			objGlobalToLocal.clear();
			objFaceIndices.clear();
			objFaceUVs.clear();
			objFaceMaterials.clear();
			currQuadblockGoodUV = true;
			activeMaterial.clear();
		};

	auto FinalizeCurrentObject = [&]()
		{
			if (currQuadblockName.empty() || objFaceIndices.empty()) { return; }
			for (const std::string& material : objFaceMaterials)
			{
				if (material.empty() || materials.contains(material)) { continue; }
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

			try
			{
				m_quadblocks.emplace_back(currQuadblockName, objPoints, objFaceIndices, objFaceUVs, objFaceMaterials, currQuadblockGoodUV,
					[this](const Quadblock& qb) { UpdateFilterRenderData(qb); });
				meshMap[currQuadblockName] = true;

				if (!currQuadblockGoodUV) { m_invalidQuadblocks.emplace_back(currQuadblockName, "Missing UVs."); }

				const Quadblock& qb = m_quadblocks.back();
				bool sameUVs = true;
				const Vec2& targetUV = qb.GetQuadUV(0)[0];
				for (size_t face = 0; face < NUM_FACES_QUADBLOCK && sameUVs; face++)
				{
					const QuadUV& faceUV = qb.GetQuadUV(face);
					for (size_t c = 0; c < 4; c++)
					{
						if (faceUV[c] != targetUV) { sameUVs = false; break; }
					}
				}
				if (sameUVs) { m_invalidQuadblocks.emplace_back(currQuadblockName, "Degenerated UV data."); }

				for (size_t face = 0; face < NUM_FACES_QUADBLOCK; face++)
				{
					const std::string& material = qb.GetMaterial(face);
					if (material.empty()) { continue; }
					m_materialToQuadFaces[material].push_back(std::make_pair(m_quadblocks.size() - 1, face));
				}
			}
			catch (const QuadException& e)
			{
				ret = false;
				m_invalidQuadblocks.emplace_back(currQuadblockName, e.what());
			}
		};

	while (std::getline(file, line))
	{
		std::vector<std::string> tokens = Split(line);
		if (tokens.empty()) { continue; }
		const std::string& command = tokens[0];
		if (command == "v")
		{
			if (tokens.size() < 4) { continue; }
			globalVertices.emplace_back(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3]));
			if (tokens.size() < 7) { continue; }
			globalVertices.back().color = Color(std::stof(tokens[4]), std::stof(tokens[5]), std::stof(tokens[6]));
		}
		else if (command == "vt")
		{
			if (tokens.size() < 3) { continue; }
			Vec2 uv = { std::stof(tokens[1]), std::stof(tokens[2]) };
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
			FinalizeCurrentObject();

			if (tokens.size() < 2 || meshMap.contains(tokens[1]))
			{
				ret = false;
				m_invalidQuadblocks.emplace_back(tokens.size() < 2 ? std::string("<unnamed>") : tokens[1], "Duplicated mesh name.");
				currQuadblockName.clear();
				ResetObjState();
				continue;
			}

			currQuadblockName = tokens[1];
			meshMap[currQuadblockName] = false;
			quadblockCount++;
			ResetObjState();
		}
		else if (command == "usemtl")
		{
			if (tokens.size() < 2) { continue; }
			if (currQuadblockName.empty()) { continue; }
			activeMaterial = tokens[1];
		}
		else if (command == "f")
		{
			if (currQuadblockName.empty()) { return false; }
			if (tokens.size() < 4) { continue; } // need at least 3 vertices for a face

			std::vector<size_t> faceLocalIndices;
			std::vector<Vec2> faceUVList;
			bool faceOk = true;
			for (size_t t = 1; t < tokens.size(); t++)
			{
				std::vector<std::string> tok = Split(tokens[t], '/');
				if (tok.size() < 3) // pos, uv?, normal - normal slot must exist, value itself unused
				{
					ret = false;
					m_invalidQuadblocks.emplace_back(currQuadblockName, "Missing vertex normals.");
					faceOk = false;
					break;
				}

				int posIdx = std::stoi(tok[0]) - 1;
				if (posIdx < 0 || static_cast<size_t>(posIdx) >= globalVertices.size())
				{
					ret = false;
					m_invalidQuadblocks.emplace_back(currQuadblockName, "Vertex index out of range.");
					faceOk = false;
					break;
				}
				size_t globalIdx = static_cast<size_t>(posIdx);

				size_t localIdx;
				auto it = objGlobalToLocal.find(globalIdx);
				if (it == objGlobalToLocal.end())
				{
					localIdx = objPoints.size();
					objGlobalToLocal[globalIdx] = localIdx;
					objPoints.push_back(globalVertices[globalIdx]);
				}
				else { localIdx = it->second; }
				faceLocalIndices.push_back(localIdx);

				Vec2 uv = Vec2();
				try
				{
					int uvIdx = std::stoi(tok[1]) - 1;
					if (uvIdx < 0 || static_cast<size_t>(uvIdx) >= uvs.size()) { throw std::out_of_range("uv index out of range"); }
					uv = uvs[static_cast<size_t>(uvIdx)];
				}
				catch (...) { currQuadblockGoodUV = false; }
				faceUVList.push_back(uv);
			}

			if (!faceOk) { continue; }

			objFaceIndices.push_back(faceLocalIndices);
			objFaceUVs.push_back(faceUVList);
			objFaceMaterials.push_back(activeMaterial);
		}
	}
	file.close();
	FinalizeCurrentObject();

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
		}
		for (Quadblock& quad : m_quadblocks)
		{
			bool stp = false;
			for (size_t face = 0; face < NUM_FACES_QUADBLOCK; face++)
			{
				Texture& tex = m_materialToTexture[quad.GetMaterial(face)];
				quad.SetTexPath(face, tex.GetPath());
				stp = stp || tex.IsSemiTransparent();
			}
			quad.SetVisTreeTransparent(stp);
		}
	}

	if (quadblockCount != m_quadblocks.size())
	{
		m_showLogWindow = true;
		m_logMessage = "Error: number of meshes does not equal number of quadblocks.\n\nNumber of meshes found: " + std::to_string(quadblockCount) + "\nNumber of quadblocks: " + std::to_string(m_quadblocks.size());
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
		GenerateRenderLevData();
		GenerateBSP();
	}
	return ret;
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
	struct HostSettings // RAW STRUCT TO EMIT FOR HOT RELOAD SETTINGS
	{
		int32_t magic;          // HOST_SETTINGS_MAGIC once the editor has written here
		int32_t sequence;       // bumped by the editor on every push
		int32_t relicSapphire;  // ms
		int32_t relicGold;      // ms
		int32_t relicPlatinum;  // ms
		int32_t crystalTime;    // ms
		int32_t introCutscene;  // 1 plays the intro cam, 0 skips it
		int32_t ghost;          // 1 leaves the ghost replay alone, 0 kills its thread
	};

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
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, relicSapphire)) = static_cast<int32_t>(HotReloadSettings::relicSapphire * 1000.0f);
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, relicGold)) = static_cast<int32_t>(HotReloadSettings::relicGold * 1000.0f);
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, relicPlatinum)) = static_cast<int32_t>(HotReloadSettings::relicPlatinum * 1000.0f);
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, crystalTime)) = static_cast<int32_t>(HotReloadSettings::crystalTime * 1000.0f);
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, introCutscene)) = HotReloadSettings::introCutscene ? 1 : 0;
		Process::At<int32_t>(HOST_SETTINGS_LOCATION + offsetof(HostSettings, ghost)) = HotReloadSettings::ghost ? 1 : 0;
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
		for (size_t face = 0; face < NUM_FACES_QUADBLOCK + 1; face++)
			usedMaterials.insert(quad.GetMaterial(face));
	}

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
			for (size_t f = 0; f < NUM_FACES_QUADBLOCK + 1; f++)
			{
				Texture* texture = const_cast<Texture*>(&animTextures[frame.textureIndexes[f]]);
				for (Texture* addedTexture : textures)
				{
					if (*texture == *addedTexture)
					{
						copyTextureAttributes.push_back({ addedTexture, texture });
						foundEqual = true;
						break;
					}
				}
				if (foundEqual) { continue; }
				textures.push_back(texture);
			}
		}
	}

	// Add water texture
	if (!m_envMapTex.IsEmpty())
	{
		Texture* tex = &m_envMapTex;
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

	// Add minimap textures if enabled
	if (!m_minimap.texture.IsEmpty())
	{
		Texture* tex = &m_minimap.texture;
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

			std::array<std::filesystem::path, NUM_FACES_QUADBLOCK> texPaths{};
			for (size_t f = 0; f < NUM_FACES_QUADBLOCK; f++)
			{
				texPaths[f] = textures[frame.textureIndexes[f]].GetPath();
			}
			std::vector<Primitive> qbTriangles = qb.ToGeometry(false, &uvs, &texPaths);
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

	constexpr float labelHeightOffset = 1.5f;
	std::vector<Primitive> botTriangles;

	for (int pathIndex = 0; pathIndex < 3; pathIndex++)
	{
		const BotPath& path = m_botPaths[pathIndex];
		const Color& c = BotPathSettings::pathColor[pathIndex];

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

	if (m_minimap.texture.IsEmpty())
	{
		m_models[LevelModels::MINIMAP_BOUNDS]->GetMesh().Clear();
		return;
	}

	// Magenta color for minimap bounds
	Color c = Color(static_cast<unsigned char>(255), static_cast<unsigned char>(0), static_cast<unsigned char>(255));

	std::vector<Primitive> triangles = m_minimap.worldBox.ToGeometry();
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
	const std::array<std::filesystem::path, NUM_FACES_QUADBLOCK> emptyTexturePaths = { emptyTexturePath, emptyTexturePath, emptyTexturePath, emptyTexturePath };
	const std::array<QuadUV, NUM_FACES_QUADBLOCK + 1> emptyUvs = {};
	for (size_t index : m_rendererSelectedQuadblockIndexes)
	{
		const Quadblock& qb = m_quadblocks[index];
		std::vector<Primitive> qbTriangles = qb.ToGeometry(false, &emptyUvs, &emptyTexturePaths);
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

	if (GuiRenderSettings::showVisTree && !m_bspVis.IsEmpty())
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
					std::vector<Primitive> qbTriangles = qb.ToGeometry(false, &emptyUvs, &emptyTexturePaths);
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
