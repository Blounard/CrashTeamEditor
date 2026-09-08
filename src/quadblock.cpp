#include "quadblock.h"
#include "utils.h"
#include "settings.h"

#include <unordered_map>
#include <unordered_set>
#include <cstring>

Vec3 ComputeNewellNormal(const std::vector<Vec3>& poly)
{
	Vec3 n = Vec3();
	const size_t count = poly.size();
	for (size_t i = 0; i < count; i++)
	{
		const Vec3& curr = poly[i];
		const Vec3& next = poly[(i + 1) % count];
		n.x += (curr.y - next.y) * (curr.z + next.z);
		n.y += (curr.z - next.z) * (curr.x + next.x);
		n.z += (curr.x - next.x) * (curr.y + next.y);
	}
	const float len = n.Length();
	if (len > 0.0f) { n = n / len; }
	return n;
}

Quadblock::Quadblock(const std::string& name,
	const std::vector<Point>& points,
	const std::vector<std::vector<size_t>>& faceIndices,
	const std::vector<std::vector<Vec2>>& faceUVs,
	const std::vector<std::string>& faceMaterials,
	bool hasUV, UpdateFilterCallback filterCallback)
{
	constexpr size_t INVALID = std::numeric_limits<size_t>::max();
	constexpr size_t EXPECTED_FACE_COUNT = NUM_FACES_QUADBLOCK;

	if (faceIndices.size() < EXPECTED_FACE_COUNT)
	{
		throw QuadException("Expected at least " + std::to_string(EXPECTED_FACE_COUNT) + " faces to build a quadblock, found " + std::to_string(faceIndices.size()) + ".");
	}
	if (faceIndices.size() > EXPECTED_FACE_COUNT)
	{
		// Not explicitly specified by design; >4 faces has no defined grid model yet, so
		// treated as an error for now rather than silently ignoring the extras.
		throw QuadException("Expected exactly " + std::to_string(EXPECTED_FACE_COUNT) + " faces, found " + std::to_string(faceIndices.size()) + ".");
	}
	if (faceUVs.size() != faceIndices.size() || faceMaterials.size() != faceIndices.size())
	{
		throw QuadException("Internal error: face data arrays have mismatched sizes.");
	}

	// Step 1: reference-count every point referenced by any face.
	std::vector<size_t> refCount(points.size(), 0);
	for (const auto& face : faceIndices)
	{
		for (size_t idx : face)
		{
			if (idx >= points.size()) { throw QuadException("Face references a vertex index out of range."); }
			refCount[idx]++;
		}
	}

	size_t centerIdx = INVALID;
	size_t centerCount = 0;
	for (size_t i = 0; i < points.size(); i++)
	{
		if (refCount[i] == faceIndices.size())
		{
			centerIdx = i;
			centerCount++;
		}
	}

	if (centerCount == 0)
	{
		// No vertex shared by all 4 faces: this object doesn't have quadblock topology at all
		// (a triblock, under the old 9-vertex encoding). Real triblock support is being
		// reworked separately; for now, build a trivial placeholder rather than failing the
		// whole object, so the rest of the level still loads. Revisit once triblocks are
		// properly redesigned.
		printf("NOTE: Object '%s' has no vertex shared by all faces (looks like a triblock); building a placeholder quadblock for it.\n", name.c_str());
		const size_t placeholderIdx = faceIndices[0][0];
		for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++) { m_p[i] = Vertex(points[placeholderIdx]); }
		ResetUVs();
		m_name = name;
		for (size_t i = 0; i < NUM_FACES_QUADBLOCK; i++) { m_materials[i] = faceMaterials[i]; }
		m_materials[NUM_FACES_QUADBLOCK] = faceMaterials[0];
		m_triblock = true;
		m_filterCallback = filterCallback;
		SetDefaultValues();
		return;
	}
	if (centerCount > 1)
	{
		throw QuadException("More than one candidate center vertex found (" + std::to_string(centerCount) + ").");
	}

	// Step 2: classify each face's own vertices relative to the center. cornersInFace == 0
	// is allowed (a triangle face missing its corner) - synthesized in Step 5.
	std::vector<size_t> faceCorner(faceIndices.size(), INVALID);
	std::vector<std::array<size_t, 2>> faceEdges(faceIndices.size());
	for (size_t f = 0; f < faceIndices.size(); f++)
	{
		size_t cornersInFace = 0, edgesInFace = 0, centersInFace = 0;
		for (size_t idx : faceIndices[f])
		{
			if (idx == centerIdx) { centersInFace++; }
			else if (refCount[idx] == 1) { faceCorner[f] = idx; cornersInFace++; }
			else if (refCount[idx] == 2) { faceEdges[f][edgesInFace++] = idx; }
			else
			{
				throw QuadException("Face " + std::to_string(f) + " references vertex " + std::to_string(idx) +
					" which is shared by an unexpected number of faces (" + std::to_string(refCount[idx]) + ").");
			}
		}
		if (centersInFace != 1 || edgesInFace != 2 || cornersInFace > 1)
		{
			throw QuadException("Face " + std::to_string(f) + " does not have the composition of a quadblock quadrant (found " +
				std::to_string(cornersInFace) + " corner(s), " + std::to_string(edgesInFace) + " edge(s), " +
				std::to_string(centersInFace) + " center(s)).");
		}
	}

	// Step 3: resolve winding using face 0's own declared vertex order around the CENTER
	// (not the corner, which might be missing). The center's two immediate neighbours in a
	// face's own cyclic vertex list are always that face's 2 edge vertices, whether the face
	// has 4 vertices (quad) or 3 (corner-missing triangle) - so this anchor works either way.
	const std::vector<size_t>& face0 = faceIndices[0];
	size_t centerSlot = INVALID;
	for (size_t s = 0; s < face0.size(); s++)
	{
		if (face0[s] == centerIdx) { centerSlot = s; break; }
	}
	if (centerSlot == INVALID) { throw QuadException("Internal error: face 0's center vertex not found in its own vertex list."); }
	const size_t L = face0.size();
	const size_t ccwNextIdx = face0[(centerSlot + 1) % L];
	const size_t ccwPrevIdx = face0[(centerSlot + L - 1) % L];

	std::array<size_t, NUM_FACES_QUADBLOCK> faceForRole = { 0, INVALID, INVALID, INVALID };
	for (size_t f = 1; f < faceIndices.size(); f++)
	{
		bool sharesNext = (faceEdges[f][0] == ccwNextIdx || faceEdges[f][1] == ccwNextIdx);
		bool sharesPrev = (faceEdges[f][0] == ccwPrevIdx || faceEdges[f][1] == ccwPrevIdx);
		if (sharesNext && !sharesPrev) { faceForRole[1] = f; }
		else if (sharesPrev && !sharesNext) { faceForRole[2] = f; }
		else if (!sharesNext && !sharesPrev) { faceForRole[3] = f; }
		else { throw QuadException("Face " + std::to_string(f) + " shares both of face 0's edges; malformed grid topology."); }
	}
	if (faceForRole[1] == INVALID || faceForRole[2] == INVALID || faceForRole[3] == INVALID)
	{
		throw QuadException("Could not uniquely resolve the 4 faces into a 2x2 grid; malformed topology.");
	}

	// Step 4: place center + the 4 edges (always present - only corners can be missing).
	std::array<size_t, NUM_VERTICES_QUADBLOCK> gridPointIdx = {};
	gridPointIdx[4] = centerIdx;
	auto FindSharedEdge = [&](size_t faceA, size_t faceB) -> size_t
		{
			for (size_t a : faceEdges[faceA])
			{
				for (size_t b : faceEdges[faceB])
				{
					if (a == b) { return a; }
				}
			}
			throw QuadException("Faces expected to be adjacent share no edge; malformed topology.");
		};
	gridPointIdx[1] = FindSharedEdge(faceForRole[0], faceForRole[1]);
	gridPointIdx[3] = FindSharedEdge(faceForRole[0], faceForRole[2]);
	gridPointIdx[5] = FindSharedEdge(faceForRole[1], faceForRole[3]);
	gridPointIdx[7] = FindSharedEdge(faceForRole[2], faceForRole[3]);

	// Step 5: place the 4 corners, synthesizing any missing one from a fixed neighbour
	// (ring order 0->1, 2->5, 8->7, 6->3 - deterministic regardless of rotation or file order).
	auto ResolveCorner = [&](size_t role, size_t substituteGridPos) -> size_t
		{
			size_t f = faceForRole[role];
			if (faceCorner[f] != INVALID) { return faceCorner[f]; }
			return gridPointIdx[substituteGridPos];
		};
	gridPointIdx[0] = ResolveCorner(0, 1);
	gridPointIdx[2] = ResolveCorner(1, 5);
	gridPointIdx[6] = ResolveCorner(2, 3);
	gridPointIdx[8] = ResolveCorner(3, 7);

	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++) { m_p[i] = Vertex(points[gridPointIdx[i]]); }

	// Step 6: UVs. A synthesized corner's grid index equals its substitute edge's grid index,
	// so this lookup naturally finds that edge's own real, authored UV for it automatically -
	// no special-casing needed for the missing-corner case.
	constexpr size_t uvVertInd[NUM_FACES_QUADBLOCK][4] =
	{
		{0, 1, 3, 4},
		{1, 2, 4, 5},
		{3, 4, 6, 7},
		{4, 5, 7, 8},
	};

	if (hasUV)
	{
		for (size_t role = 0; role < NUM_FACES_QUADBLOCK; role++)
		{
			const size_t origFace = faceForRole[role];
			for (size_t j = 0; j < 4; j++)
			{
				const size_t wantedGlobalIdx = gridPointIdx[uvVertInd[role][j]];
				size_t srcSlot = INVALID;
				for (size_t k = 0; k < faceIndices[origFace].size(); k++)
				{
					if (faceIndices[origFace][k] == wantedGlobalIdx) { srcSlot = k; break; }
				}
				if (srcSlot == INVALID) { throw QuadException("Internal error mapping UVs for face " + std::to_string(origFace) + "."); }
				m_uvs[role][j] = faceUVs[origFace][srcSlot];
			}
		}

		float uMin = std::numeric_limits<float>::max(); float vMin = std::numeric_limits<float>::max();
		float uMax = -std::numeric_limits<float>::max(); float vMax = -std::numeric_limits<float>::max();
		for (size_t i = 0; i < 4; i++)
		{
			for (size_t j = 0; j < 4; j++)
			{
				uMin = std::min(uMin, m_uvs[i][j].x); vMin = std::min(vMin, m_uvs[i][j].y);
				uMax = std::max(uMax, m_uvs[i][j].x); vMax = std::max(vMax, m_uvs[i][j].y);
			}
		}
		bool indexPicked[4] = { false, false, false, false };
		bool boundPicked[4] = { false, false, false, false };
		const QuadUV uvBounds = { Vec2(uMin, vMin), Vec2(uMax, vMin), Vec2(uMin, vMax), Vec2(uMax, vMax) };
		for (size_t indexCount = 0; indexCount < 4; indexCount++)
		{
			size_t bestIndex = 0, bestBound = 0;
			float bestDistance = std::numeric_limits<float>::max();
			for (size_t i = 0; i < 4; i++)
			{
				if (indexPicked[i]) { continue; }
				for (size_t j = 0; j < 4; j++)
				{
					if (boundPicked[j]) { continue; }
					float dist = ((m_uvs[0][i].x - uvBounds[j].x) * (m_uvs[0][i].x - uvBounds[j].x)) + ((m_uvs[0][i].y - uvBounds[j].y) * (m_uvs[0][i].y - uvBounds[j].y));
					if (dist < bestDistance) { bestIndex = i; bestBound = j; bestDistance = dist; }
				}
			}
			indexPicked[bestIndex] = true;
			boundPicked[bestBound] = true;
			m_uvs[4][bestIndex] = uvBounds[bestBound];
		}
	}
	else { ResetUVs(); }

	m_name = name;
	for (size_t role = 0; role < NUM_FACES_QUADBLOCK; role++) { m_materials[role] = faceMaterials[faceForRole[role]]; }
	m_materials[NUM_FACES_QUADBLOCK] = m_materials[0];
	m_triblock = false;
	m_filterCallback = filterCallback;
	SetDefaultValues();
}

Quadblock::Quadblock(const PSX::Quadblock& quadblock, const std::vector<PSX::Vertex>& vertices, UpdateFilterCallback filterCallback)
{
	uint16_t reverseIndexMapping[NUM_VERTICES_QUADBLOCK] = {0, 2, 6, 8, 1, 3, 4, 5, 7};
	std::unordered_set<uint16_t> indexes;
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
	{
		uint16_t index = quadblock.index[i];
		indexes.insert(index);
		const PSX::Vertex& vertex = vertices[index];
		m_p[reverseIndexMapping[i]] = Vertex(vertex);
	}
	SetDefaultValues();
	ResetUVs();
	m_hasRawNormalData = true;
	m_hasRawTexture = true;
	m_triNormalVecBitshift = quadblock.triNormalVecBitshift;
	for (int i = 0; i < 10; i++) { m_triNormalVecDividend[i] = quadblock.triNormalVecDividend[i]; }
	m_bbox.max = ConvertPSXVec3(quadblock.bbox.max, FP_ONE_GEO);
	m_bbox.min = ConvertPSXVec3(quadblock.bbox.min, FP_ONE_GEO);

	m_name = "Quadblock." + std::to_string(quadblock.id);
	m_flags = quadblock.flags;
	m_doubleSided = (quadblock.drawOrderLow & (1 << 31)) != 0;
	for (size_t i = 0; i < NUM_FACES_QUADBLOCK; i++)
	{
		uint32_t packedFace = (quadblock.drawOrderLow >> (8 + i * 5)) & 0b11111;
		m_faceRotateFlip[i] = packedFace & 0b111;
		m_faceDrawMode[i] = (packedFace >> 3) & 0b11;
		if (quadblock.drawOrderHigh[i] != 0) { m_drawOrderHigh = static_cast<int>(quadblock.drawOrderHigh[i]); }
		m_offTextures[i] = quadblock.offMidTextures[i];
	}
	m_offTextures[NUM_FACES_QUADBLOCK] = quadblock.offLowTexture;
	m_terrain = quadblock.terrain;
	m_downforce = static_cast<int>(quadblock.speedImpact);
	m_weatherIntensity = quadblock.weatherIntensity;
	m_weatherVanishRate = quadblock.weatherVanishRate;
	m_checkpointIndex = quadblock.checkpointIndex;
	if (m_checkpointIndex == std::numeric_limits<uint8_t>::max()) { m_checkpointIndex = -1; }
	else { m_checkpointStatus = true; }
	for (size_t face = 0; face < NUM_FACES_QUADBLOCK + 1; face++)
	{
		m_materials[face] = "default";
	}	
	m_triblock = indexes.size() == 6;
	m_filterCallback = filterCallback;
		
}

const std::string& Quadblock::GetName() const
{
	return m_name;
}

bool Quadblock::IsQuadblock() const
{
	return !m_triblock;
}

Vec3 Quadblock::GetCenter() const
{
	if (m_triblock)
	{
		Vec3 v1 = m_p[1].m_pos;
		Vec3 v2 = m_p[3].m_pos;
		Vec3 v3 = m_p[4].m_pos;
		return (v1 + v2 + v3) / (3.0f);
	}
	else
	{
		return m_p[4].m_pos;
	}
}

Vec3 Quadblock::GetNormal() const
{
	Vec3 normal = ComputeNormalVector(0, 2, 6);
	normal.Normalize();
	return normal;
}

const std::vector<std::array<size_t, 3>>& Quadblock::GetCollTriFacesIndexes() const
{
	return m_collTriFaces;
}

std::vector<std::array<size_t, 3>> Quadblock::GetTriFacesIndexes() const
{
	// Return a list of (size_t, size_t, size_t) containing vertex ID
	// of every triface composing the quad, ordered clockwise
	std::vector<std::array<size_t, 3>> triFaces;

	if (!m_triblock)
	{
		triFaces = {
			{0, 1, 3},
			{1, 4, 3},
			{1, 2, 4},
			{2, 5, 4},
			{3, 4, 6},
			{4, 7, 6},
			{4, 5, 7},
			{5, 8, 7}
		};
	}
	else
	{
		triFaces = {
			{0, 1, 3},
			{1, 2, 4},
			{1, 4, 3},
			{4, 6, 3}
		};
	}
	return triFaces;
}

std::array<Vec3, 3> Quadblock::GetTriFace(size_t id0, size_t id1, size_t id2) const
{
	return {
		m_p[id0].m_pos,
		m_p[id1].m_pos,
		m_p[id2].m_pos
	};
}

uint8_t Quadblock::GetTerrain() const
{
	return m_terrain;
}

uint16_t Quadblock::GetFlags() const
{
	return m_flags;
}

bool Quadblock::GetWater() const
{
	return m_water;
}

QuadblockTrigger Quadblock::GetTrigger() const
{
	return m_trigger;
}

size_t Quadblock::GetTurboPadIndex() const
{
	return m_turboPadIndex;
}

size_t Quadblock::GetBSPID() const
{
	return m_bspID;
}

void Quadblock::SetBSPID(size_t id) const
{
	m_bspID = id;
}

bool Quadblock::GetHide() const
{
	return m_hide;
}

bool Quadblock::GetAnimated() const
{
	return m_animated;
}

bool Quadblock::GetFilter() const
{
	return m_filter;
}

const Color& Quadblock::GetFilterColor() const
{
	return m_filterColor;
}

bool Quadblock::GetDrawDoubleSided() const
{
	return m_doubleSided;
}

bool Quadblock::GetCheckpointStatus() const
{
	return m_checkpointStatus;
}

bool Quadblock::GetCheckpointPathable() const
{
	return m_checkpointPathable;
}

bool Quadblock::GetVisTreeTransparent() const
{
	return m_visTreeTransparent;
}

int Quadblock::GetDrawOrderHigh() const
{
	return m_drawOrderHigh;
}

uint32_t Quadblock::GetFaceRotateFlip(size_t face) const
{
	if (face >= NUM_FACES_QUADBLOCK) { return 0; }
	return m_faceRotateFlip[face];
}

int Quadblock::GetWeatherIntensity() const
{
	return m_weatherIntensity;
}

int Quadblock::GetWeatherVanishRate() const
{
	return m_weatherVanishRate;
}

const QuadUV& Quadblock::GetQuadUV(size_t quad) const
{
	return m_uvs[quad];
}

const std::filesystem::path& Quadblock::GetTexPath(size_t face) const
{
	return m_texPaths[face];
}

const std::array<QuadUV, NUM_FACES_QUADBLOCK + 1>& Quadblock::GetUVs() const
{
	return m_uvs;
}

uint32_t Quadblock::GetRawTexOffset(size_t i) const
{
	return m_offTextures[i];
}

size_t Quadblock::GetRenderPrimitiveIndex() const
{
	return m_renderPrimitiveIndex;
}

const std::string& Quadblock::GetMaterial(size_t face) const
{
	return m_materials[face];
}

void Quadblock::SetRenderPrimitiveIndex(size_t primitiveIndex)
{
	m_renderPrimitiveIndex = primitiveIndex;
}

void Quadblock::SetTerrain(uint8_t terrain)
{
	m_terrain = terrain;
}

void Quadblock::SetFlag(uint16_t flag)
{
	m_flags = flag;
}

void Quadblock::SetWater(bool isWater)
{
	m_water = isWater;
}

void Quadblock::SetCheckpoint(int index)
{
	m_checkpointIndex = index;
}

int Quadblock::GetCheckpoint() const
{
	return m_checkpointIndex;
}

void Quadblock::SetDrawDoubleSided(bool active)
{
	m_doubleSided = active;
}

void Quadblock::SetCheckpointStatus(bool active)
{
	m_checkpointStatus = active;
}

void Quadblock::SetCheckpointPathable(bool pathable)
{
	m_checkpointPathable = pathable;
}

void Quadblock::SetVisTreeTransparent(bool transparent)
{
	m_visTreeTransparent = transparent;
}

void Quadblock::SetDrawOrderHigh(int drawOrderHigh)
{
	m_drawOrderHigh = drawOrderHigh;
}

void Quadblock::SetName(const std::string& name)
{
	m_name = name;
}

void Quadblock::SetTurboPadIndex(size_t index)
{
	m_turboPadIndex = index;
}

void Quadblock::SetHide(bool active)
{
	m_hide = active;
}

void Quadblock::SetTextureID(size_t id, size_t quad)
{
	m_textureIDs[quad] = id;
}

void Quadblock::SetAnimTextureOffset(size_t relOffset, size_t levOffset, size_t quad)
{
	m_animTexOffset[quad] = relOffset + levOffset;
}

void Quadblock::SetTrigger(QuadblockTrigger trigger)
{
	m_trigger = trigger;
}

void Quadblock::SetTexPath(size_t face, const std::filesystem::path& path)
{
	m_texPaths[face] = path;
}

void Quadblock::SetAnimated(bool animated)
{
	m_animated = animated;
}

void Quadblock::SetFilter(bool filter)
{
	m_filter = filter;
	m_filterCallback(*this);
}

void Quadblock::SetFilterColor(const Color& color)
{
	m_filterColor = color;
	m_filterCallback(*this);
}

void Quadblock::SetSpeedImpact(int speed)
{
	m_downforce = speed;
}

void Quadblock::SetUVs(const QuadUV& uvs)
{
	for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
	{
		m_uvs[i] = uvs;
	}
}

void Quadblock::SetFaceUVs(size_t faceIndex, const QuadUV& uvs)
{
	if (faceIndex < m_uvs.size())
	{
		m_uvs[faceIndex] = uvs;
	}
}

void Quadblock::SetMaterial(size_t face, const std::string& material) 
{ 
	m_materials[face] = material;
}

void Quadblock::SetOceanVertex(PSX::OceanVertex overt, size_t vertId)
{
	m_oVert[vertId] = overt;
}

PSX::OceanVertex Quadblock::GetOceanVertex(size_t vertId) const
{
	return m_oVert[vertId];
}

void Quadblock::SetWeatherIntensity(int intensity)
{
	m_weatherIntensity = intensity;
}

void Quadblock::SetWeatherVanishRate(int vanishRate)
{
	m_weatherVanishRate = vanishRate;
}

void Quadblock::Translate(float ratio, const Vec3& direction)
{
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++) { m_p[i].m_pos += direction * ratio; }
	ComputeBoundingBox();
}

const BoundingBox& Quadblock::GetBoundingBox() const
{
	return m_bbox;
}

std::vector<Primitive> Quadblock::ToGeometry(bool filterTriangles, const std::array<QuadUV, NUM_FACES_QUADBLOCK + 1>* overrideUvs, const std::array<std::filesystem::path, NUM_FACES_QUADBLOCK>* overrideTexturePaths) const
{
	if (GetHide()) { return std::vector<Primitive>(); } /* Turbo Pads */

	constexpr int NUM_VERTICES_QUAD = 4;
	constexpr int uvVertInd[NUM_FACES_QUADBLOCK][NUM_VERTICES_QUAD] =
	{
		{0, 1, 3, 4},
		{1, 2, 4, 5},
		{3, 4, 6, 7},
		{4, 5, 7, 8},
	};

	const bool isQuadblock = IsQuadblock();
	const std::array<QuadUV, NUM_FACES_QUADBLOCK + 1>& uvs = overrideUvs ? *overrideUvs : m_uvs;
	const Color filterColor = GetFilter() ? GetFilterColor() : Color(static_cast<unsigned char>(0u), static_cast<unsigned char>(0u), static_cast<unsigned char>(0u));

	auto GetUVForVertex = [&](int quadInd, int vertInd) -> Vec2
		{
			const QuadUV& quv = uvs[quadInd];
			int vertIndInUvs = 0;
			for (int i = 0; i < NUM_VERTICES_QUAD; i++)
			{
				if (vertInd == uvVertInd[quadInd][i]) { vertIndInUvs = i; break; }
			}
			return quv[vertIndInUvs];
		};

	std::vector<Primitive> primitives;
	if (isQuadblock)
	{
		primitives.reserve(NUM_FACES_QUADBLOCK);
		for (int faceId = 0; faceId < NUM_FACES_QUADBLOCK; faceId++)
		{
			const std::filesystem::path& texPath = overrideTexturePaths ? (*overrideTexturePaths)[faceId] : m_texPaths[faceId];
			const std::string textureString = filterTriangles ? std::string() : texPath.string();
			Quad quad;
			quad.texture = textureString;
			for (int i = 0; i < NUM_VERTICES_QUAD; i++)
			{
				const int vertIndex = uvVertInd[faceId][i];
				const Vertex& vert = m_p[vertIndex];
				quad.p[i].pos = vert.m_pos;
				quad.p[i].normal = Vec3(0.0f, 1.0f, 0.0f);
				quad.p[i].color = filterTriangles ? filterColor : vert.GetColor(true);
				quad.p[i].uv = filterTriangles ? Vec2() : GetUVForVertex(faceId, vertIndex);
			}
			primitives.push_back(quad);
		}
	}
	else
	{
		constexpr int triCount = 4;
		primitives.reserve(triCount);
		constexpr int triblockVertArrangements[triCount][3] =
		{
			{ 0, 1, 3 },
			{ 1, 2, 4 },
			{ 3, 4, 6 },
			{ 1, 4, 3 },
		};
		constexpr int triblockQuadIndex[triCount] = {0, 1, 2, 0};
		for (int triIndex = 0; triIndex < triCount; triIndex++)
		{
			const int faceId = triblockQuadIndex[triIndex];
			const int* triVerts = triblockVertArrangements[triIndex];
			const std::filesystem::path& texPath = overrideTexturePaths ? (*overrideTexturePaths)[faceId] : m_texPaths[faceId];
			const std::string textureString = filterTriangles ? std::string() : texPath.string();
			Tri tri;
			tri.texture = textureString;
			for (int i = 0; i < 3; i++)
			{
				const int vertIndex = triVerts[i];
				const Vertex& vert = m_p[vertIndex];
				tri.p[i].pos = vert.m_pos;
				tri.p[i].normal = Vec3(0.0f, 1.0f, 0.0f);
				tri.p[i].color = filterTriangles ? filterColor : vert.GetColor(true);
				tri.p[i].uv = filterTriangles ? Vec2() : GetUVForVertex(faceId, vertIndex);
			}
			primitives.push_back(tri);
		}
	}

	return primitives;
}

std::vector<Vertex> Quadblock::GetVertices() const
{
	/*                                 0       1       2       3       4       5       6       7       8    */
	std::vector<Vertex> vertices = {m_p[0], m_p[2], m_p[6], m_p[8], m_p[1], m_p[3], m_p[4], m_p[5], m_p[7]};
	return vertices;
}

const Vertex* const Quadblock::GetUnswizzledVertices() const
{
	return m_p;
}

float Quadblock::DistanceClosestVertex(Vec3& out, const Vec3& v) const
{
	float minDist = std::numeric_limits<float>::max();
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
	{
		float dist = (v - m_p[i].m_pos).Length();
		if (dist < minDist)
		{
			minDist = dist;
			out = m_p[i].m_pos;
		}
	}
	return minDist;
}

bool Quadblock::IntersectRay(const Vec3& point, const Vec3& projectDir, float& outdist, Vec3& outnormal, float barycentricTolerance) const
{
	for (std::array<size_t, 3> tri : m_collTriFaces)
	{
		const Vec3& A = m_p[tri[0]].m_pos;
		const Vec3& B = m_p[tri[1]].m_pos;
		const Vec3& C = m_p[tri[2]].m_pos;
		if (TestBarycentric(A, B, C, point, projectDir, outdist, outnormal, barycentricTolerance))
			return true;
	}
	return false;
}

bool Quadblock::SnapPoint(Vec3& pos, Vec3& rot, const Vec3& projectDir, float barycentricTolerance) const
{
	for (std::array<size_t, 3> tri : m_collTriFaces)
	{
		const Vec3& A = m_p[tri[0]].m_pos;
		const Vec3& B = m_p[tri[1]].m_pos;
		const Vec3& C = m_p[tri[2]].m_pos;
		if (SnapTriangle(A, B, C, pos, rot, projectDir, barycentricTolerance))
			return true;
	}
	return false;
}

bool Quadblock::Neighbours(const Quadblock& quadblock, float threshold) const
{
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
	{
		for (size_t j = 0; j < NUM_VERTICES_QUADBLOCK; j++)
		{
			if ((m_p[i].m_pos - quadblock.m_p[j].m_pos).Length() < threshold) { return true; }
		}
	}
	return false;
}

std::vector<uint8_t> Quadblock::Serialize(size_t id, size_t offTextures, const std::vector<size_t>& vertexIndexes) const
{
	PSX::Quadblock quadblock = {};
	std::vector<uint8_t> buffer(sizeof(quadblock));
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
	{
		quadblock.index[i] = static_cast<uint16_t>(vertexIndexes[i]);
	}
	quadblock.flags = m_flags;
	quadblock.drawOrderLow = m_doubleSided ? (1 << 31) : 0;
	for (size_t i = 0; i < NUM_FACES_QUADBLOCK; i++)
	{
		uint32_t packedFace = m_faceRotateFlip[i] | (m_faceDrawMode[i] << 3);
		quadblock.drawOrderLow |= packedFace << (8 + i * 5);
		quadblock.drawOrderHigh[i] = static_cast<int8_t>(m_drawOrderHigh);
	}
	if (m_animated)
	{
		for (size_t f = 0; f < NUM_FACES_QUADBLOCK; f++)
		{
			if (m_animTexOffset[f] >= 0)
				quadblock.offMidTextures[f] = static_cast<uint32_t>(m_animTexOffset[f] | 1);
		}
	}
	else
	{
		if (m_textureIDs[0] >= 0)
			quadblock.offMidTextures[0] = static_cast<uint32_t>(offTextures + (m_textureIDs[0] * sizeof(PSX::TextureGroup)));
		if (m_textureIDs[1] >= 0)
			quadblock.offMidTextures[1] = static_cast<uint32_t>(offTextures + (m_textureIDs[1] * sizeof(PSX::TextureGroup)));
		if (m_textureIDs[2] >= 0)
			quadblock.offMidTextures[2] = static_cast<uint32_t>(offTextures + (m_textureIDs[2] * sizeof(PSX::TextureGroup)));
		if (m_textureIDs[3] >= 0)
			quadblock.offMidTextures[3] = static_cast<uint32_t>(offTextures + (m_textureIDs[3] * sizeof(PSX::TextureGroup)));
	}
	if (m_textureIDs[4] >= 0)
		quadblock.offLowTexture = static_cast<uint32_t>(offTextures + (m_textureIDs[4] * sizeof(PSX::TextureGroup)));

	quadblock.bbox.min = ConvertVec3(m_bbox.min, FP_ONE_GEO);
	quadblock.bbox.max = ConvertVec3(m_bbox.max, FP_ONE_GEO);
	quadblock.terrain = m_terrain;
	quadblock.weatherIntensity = 0;
	quadblock.weatherVanishRate = 0;
	quadblock.speedImpact = static_cast<int8_t>(m_downforce);
	quadblock.weatherIntensity = static_cast<uint8_t>(m_weatherIntensity);
	quadblock.weatherVanishRate = static_cast<uint8_t>(m_weatherVanishRate);
	const size_t idVis = id / 32;
	quadblock.id = static_cast<uint16_t>((32 * idVis) + (31 - (id % 32)));
	quadblock.checkpointIndex = static_cast<uint8_t>(m_checkpointIndex);

	if (!m_hasRawNormalData)
	{
		quadblock.triNormalVecBitshift = static_cast<uint8_t>(std::round(std::log2(std::max(ComputeNormalVector(0, 2, 6).Length(), ComputeNormalVector(2, 8, 6).Length()) * 512.0f)));
		auto CalculateNormalDividend = [this](size_t id0, size_t id1, size_t id2, float scaler) -> int16_t
			{
				return static_cast<int16_t>(std::round(scaler / ComputeNormalVector(id0, id1, id2).Length()));
			};

		float scaler = static_cast<float>(1 << quadblock.triNormalVecBitshift);
		quadblock.triNormalVecDividend[0] = CalculateNormalDividend(0, 1, 3, scaler);
		quadblock.triNormalVecDividend[1] = CalculateNormalDividend(1, 4, 3, scaler);
		quadblock.triNormalVecDividend[2] = CalculateNormalDividend(4, 1, 2, scaler);
		quadblock.triNormalVecDividend[3] = CalculateNormalDividend(3, 4, 6, scaler);
		quadblock.triNormalVecDividend[4] = CalculateNormalDividend(7, 4, 5, scaler);
		quadblock.triNormalVecDividend[5] = CalculateNormalDividend(5, 8, 7, scaler);
		quadblock.triNormalVecDividend[6] = CalculateNormalDividend(2, 5, 4, scaler);
		quadblock.triNormalVecDividend[7] = CalculateNormalDividend(6, 4, 7, scaler);
		quadblock.triNormalVecDividend[9] = CalculateNormalDividend(2, 8, 6, scaler); /* low LoD */
		quadblock.triNormalVecDividend[8] = CalculateNormalDividend(0, 2, 6, scaler); /* low LoD */
	}
	else
	{
		quadblock.triNormalVecBitshift = m_triNormalVecBitshift;
		for (int i = 0; i < 10; i++) { quadblock.triNormalVecDividend[i] = m_triNormalVecDividend[i]; }
	}

	std::memcpy(buffer.data(), &quadblock, sizeof(quadblock));
	return buffer;
}

void Quadblock::SetDefaultValues()
{
	ComputeBoundingBox();
	m_checkpointIndex = -1;
	m_flags = QuadFlags::DEFAULT;
	m_terrain = TerrainType::LABELS.at(TerrainType::DEFAULT);

	for (size_t i = 0; i < NUM_FACES_QUADBLOCK; i++)
	{
		m_faceDrawMode[i] = FaceDrawMode::DRAW_BOTH;
		m_faceRotateFlip[i] = FaceRotateFlip::NONE;
	}

	const bool equivalentDiagonal = std::abs((m_p[2].m_pos - m_p[6].m_pos).Length() - ((m_p[2].m_pos - m_p[4].m_pos).Length() + (m_p[4].m_pos - m_p[6].m_pos).Length())) <= EPSILON;
	const bool equivalentSide02 = std::abs((m_p[0].m_pos - m_p[2].m_pos).Length() - ((m_p[0].m_pos - m_p[1].m_pos).Length() + (m_p[1].m_pos - m_p[2].m_pos).Length())) <= EPSILON;
	const bool equivalentSide06 = std::abs((m_p[0].m_pos - m_p[6].m_pos).Length() - ((m_p[0].m_pos - m_p[3].m_pos).Length() + (m_p[3].m_pos - m_p[6].m_pos).Length())) <= EPSILON;
	if (equivalentDiagonal && equivalentSide02 && equivalentSide06) { m_collTriFaces = {{0, 2, 6}}; }
	else
	{
		m_collTriFaces = {
			{0, 1, 3},
			{1, 2, 4},
			{1, 4, 3},
			{4, 6, 3}
		};
	}

	if (!m_triblock)
	{
		const bool equivalentSide28 = std::abs((m_p[2].m_pos - m_p[8].m_pos).Length() - ((m_p[2].m_pos - m_p[5].m_pos).Length() + (m_p[5].m_pos - m_p[8].m_pos).Length())) <= EPSILON;
		const bool equivalentSide68 = std::abs((m_p[6].m_pos - m_p[8].m_pos).Length() - ((m_p[6].m_pos - m_p[7].m_pos).Length() + (m_p[7].m_pos - m_p[8].m_pos).Length())) <= EPSILON;
		if (equivalentDiagonal && equivalentSide28 && equivalentSide68) { m_collTriFaces.push_back({2, 8, 6}); }
		else
		{
			m_collTriFaces.push_back({2, 5, 4});
			m_collTriFaces.push_back({4, 7, 6});
			m_collTriFaces.push_back({4, 5, 7});
			m_collTriFaces.push_back({5, 8, 7});
		}
	}

	m_doubleSided = false;
	m_checkpointPathable = true;
	m_checkpointStatus = false;
	m_visTreeTransparent = false;
	m_drawOrderHigh = 0x0;
	m_trigger = QuadblockTrigger::NONE;
	m_turboPadIndex = TURBO_PAD_INDEX_NONE;
	m_hide = false;
	m_animated = false;
	m_filter = false;
	m_downforce = 0;
	m_hasRawNormalData = false;
	m_hasRawTexture = false;
	m_weatherIntensity = 0;
	m_weatherVanishRate = 0;
	m_filterColor = GuiRenderSettings::defaultFilterColor;
	m_renderPrimitiveIndex = RENDER_INDEX_NONE;
	m_water = false;
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
	{
		m_oVert[i] = PSX::OceanVertex{};
	}
}

Vec3 Quadblock::ComputeNormalVector(size_t id0, size_t id1, size_t id2) const
{
	Vec3 a = m_p[id0].m_pos - m_p[id1].m_pos;
	Vec3 b = m_p[id2].m_pos - m_p[id0].m_pos;
	return a.Cross(b);
}

void Quadblock::ResetUVs()
{
	m_uvs[0] = {Vec2(0.0f, 0.0f), Vec2(0.5f, 0.0f), Vec2(0.0f, 0.5f), Vec2(0.5f, 0.5f)};
	m_uvs[1] = {Vec2(0.5f, 0.0f), Vec2(1.0f, 0.0f), Vec2(0.5f, 0.5f), Vec2(1.0f, 0.5f)};
	m_uvs[2] = {Vec2(0.0f, 0.5f), Vec2(0.5f, 0.5f), Vec2(0.0f, 1.0f), Vec2(0.5f, 1.0f)};
	m_uvs[3] = {Vec2(0.5f, 0.5f), Vec2(1.0f, 0.5f), Vec2(0.5f, 1.0f), Vec2(1.0f, 1.0f)};
	m_uvs[4] = {Vec2(0.0f, 0.0f), Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f), Vec2(1.0f, 1.0f)};
}

void Quadblock::ComputeBoundingBox()
{
	Vec3 min = Vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	Vec3 max = Vec3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
	{
		min.x = std::min(min.x, m_p[i].m_pos.x); max.x = std::max(max.x, m_p[i].m_pos.x);
		min.y = std::min(min.y, m_p[i].m_pos.y); max.y = std::max(max.y, m_p[i].m_pos.y);
		min.z = std::min(min.z, m_p[i].m_pos.z); max.z = std::max(max.z, m_p[i].m_pos.z);
	}
	m_bbox.min = min;
	m_bbox.max = max;
	m_hasRawNormalData = false;
}

int SnapToClosestQuad(const std::vector<Quadblock>& quadblocks, const std::vector<size_t>quadIndexes, Vec3& outpos, Vec3& outrot, const Vec3& projectDir, float negSnapLimit, float posSnapLimit, float barycentricTolerance)
{
	int quadId = -1;
	float minDist = std::numeric_limits<float>::max();
	Vec3 pos = outpos;
	for (size_t i : quadIndexes)
	{
		const Quadblock& quad = quadblocks[i];
		float dist;
		Vec3 normal;
		if (quad.IntersectRay(pos, projectDir, dist, normal, barycentricTolerance))
		{
			if (dist > negSnapLimit && dist < posSnapLimit)
			{
				if (std::fabs(dist) < minDist)
				{
					minDist = std::fabs(dist);
					quadId = static_cast<int>(i);
					quad.SnapPoint(outpos, outrot, projectDir, barycentricTolerance);
				}
			}
		}
	}
	return quadId;
}