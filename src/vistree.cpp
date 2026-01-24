#include "vistree.h"
#include <omp.h>
#include <cmath>

bool BitMatrix::Get(size_t x, size_t y) const
{
	return m_data[(y * m_width) + x];
}

size_t BitMatrix::GetWidth() const
{
	return m_width;
}

size_t BitMatrix::GetHeight() const
{
	return m_height;
}

void BitMatrix::Set(bool value, size_t x, size_t y)
{
	m_data[(y * m_width) + x] = value;
}

bool BitMatrix::IsEmpty() const
{
	return m_data.empty();
}

void BitMatrix::Clear()
{
	m_width = 0;
	m_height = 0;
	m_data.clear();
}

static bool WorldspaceRayTriIntersection(const Vec3& worldSpaceRayOrigin, const Vec3& worldSpaceRayDir, const Vec3* tri, float& dist)
{
	constexpr float epsilon = 0.00001f;

	//moller-trumbore intersection test
	//https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

	Vec3 edge_1 = tri[1] - tri[0], edge_2 = tri[2] - tri[0];
	Vec3 ray_cross_e2 = worldSpaceRayDir.Cross(edge_2);
	float det = edge_1.Dot(ray_cross_e2);

	if (std::abs(det) < epsilon) { return false; } // ray is parallel to plane, couldn't possibly intersect with triangle.

	float inv_det = 1.0f / det;
	Vec3 s = worldSpaceRayOrigin - tri[0];
	float u = inv_det * s.Dot(ray_cross_e2);

	if ((u < 0 && std::abs(u) > epsilon) || (u > 1 && std::abs(u - 1.0f) > epsilon)) { return false; } //fails a barycentric test

	Vec3 s_cross_e1 = s.Cross(edge_1);
	float v = inv_det * worldSpaceRayDir.Dot(s_cross_e1);

	if ((v < 0 && std::abs(v) > epsilon) || (u + v > 1 && std::abs(u + v - 1) > epsilon)) { return false; } //fails a barycentric test

	float t = inv_det * edge_2.Dot(s_cross_e1); // time value (interpolant)
	if (t > epsilon) { dist = std::abs(t); return true; }
	return false;
}

static bool RayIntersectQuadblockTest(const Vec3& worldSpaceRayOrigin, const Vec3& worldSpaceRayDir, const Quadblock& qb, float& dist)
{
	bool isQuadblock = qb.IsQuadblock();
	const Vertex* verts = qb.GetUnswizzledVertices();

	Vec3 tris[3];
	if (isQuadblock)
	{
		for (int triIndex = 0; triIndex < 8; triIndex++)
		{
			tris[0] = Vec3(verts[FaceIndexConstants::quadHLODVertArrangements[triIndex][0]].m_pos.x, verts[FaceIndexConstants::quadHLODVertArrangements[triIndex][0]].m_pos.y, verts[FaceIndexConstants::quadHLODVertArrangements[triIndex][0]].m_pos.z);
			tris[1] = Vec3(verts[FaceIndexConstants::quadHLODVertArrangements[triIndex][1]].m_pos.x, verts[FaceIndexConstants::quadHLODVertArrangements[triIndex][1]].m_pos.y, verts[FaceIndexConstants::quadHLODVertArrangements[triIndex][1]].m_pos.z);
			tris[2] = Vec3(verts[FaceIndexConstants::quadHLODVertArrangements[triIndex][2]].m_pos.x, verts[FaceIndexConstants::quadHLODVertArrangements[triIndex][2]].m_pos.y, verts[FaceIndexConstants::quadHLODVertArrangements[triIndex][2]].m_pos.z);
			if (WorldspaceRayTriIntersection(worldSpaceRayOrigin, worldSpaceRayDir, tris, dist)) { return true; }
		}
	}
	else
	{
		for (int triIndex = 0; triIndex < 4; triIndex++)
		{
			tris[0] = Vec3(verts[FaceIndexConstants::triHLODVertArrangements[triIndex][0]].m_pos.x, verts[FaceIndexConstants::triHLODVertArrangements[triIndex][0]].m_pos.y, verts[FaceIndexConstants::triHLODVertArrangements[triIndex][0]].m_pos.z);
			tris[1] = Vec3(verts[FaceIndexConstants::triHLODVertArrangements[triIndex][1]].m_pos.x, verts[FaceIndexConstants::triHLODVertArrangements[triIndex][1]].m_pos.y, verts[FaceIndexConstants::triHLODVertArrangements[triIndex][1]].m_pos.z);
			tris[2] = Vec3(verts[FaceIndexConstants::triHLODVertArrangements[triIndex][2]].m_pos.x, verts[FaceIndexConstants::triHLODVertArrangements[triIndex][2]].m_pos.y, verts[FaceIndexConstants::triHLODVertArrangements[triIndex][2]].m_pos.z);
			if (WorldspaceRayTriIntersection(worldSpaceRayOrigin, worldSpaceRayDir, tris, dist)) { return true; }
		}
	}
	return false;
}

// Ray-AABB intersection test using the slab method
// Returns true if the ray intersects the bounding box, and sets dist to the distance to the nearest intersection
static bool RayIntersectBoundingBox(const Vec3& rayOrigin, const Vec3& rayDir, const BoundingBox& bbox, float& dist)
{
	constexpr float epsilon = 0.00001f;

	// Compute inverse ray direction for efficiency
	float invDirX = 1.0f / (std::abs(rayDir.x) < epsilon ? epsilon : rayDir.x);
	float invDirY = 1.0f / (std::abs(rayDir.y) < epsilon ? epsilon : rayDir.y);
	float invDirZ = 1.0f / (std::abs(rayDir.z) < epsilon ? epsilon : rayDir.z);

	// Compute intersection distances for each pair of slabs
	float t1 = (bbox.min.x - rayOrigin.x) * invDirX;
	float t2 = (bbox.max.x - rayOrigin.x) * invDirX;
	float t3 = (bbox.min.y - rayOrigin.y) * invDirY;
	float t4 = (bbox.max.y - rayOrigin.y) * invDirY;
	float t5 = (bbox.min.z - rayOrigin.z) * invDirZ;
	float t6 = (bbox.max.z - rayOrigin.z) * invDirZ;

	// Find the min and max for each axis
	float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
	float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

	// No intersection if tmax < 0 (box is behind ray) or tmin > tmax (ray misses box)
	if (tmax < 0.0f || tmin > tmax)
	{
		return false;
	}

	// Set dist to the near intersection distance (tmin), or 0 if ray origin is inside box
	dist = tmin < 0.0f ? 0.0f : tmin;
	return true;
}

// Recursively traverse BSP tree to find quadblock indexes that could potentially intersect the ray
static std::vector<size_t> GetPotentialQuadblockIndexes(const std::vector<Quadblock>& quadblocks, const BSP* node, const Vec3& rayOrigin, const Vec3& rayDir)
{
	std::vector<size_t> result;

	// Check if the ray intersects this node's bounding box
	float dist;
	if (!RayIntersectBoundingBox(rayOrigin, rayDir, node->GetBoundingBox(), dist))
	{
		// No intersection with bounding box - return empty list
		return result;
	}

	// If this is a leaf node, return all quadblock indexes in this leaf
	if (!node->IsBranch())
	{
		return node->GetQuadblockIndexes();
	}

	// This is a branch node - recursively get indexes from both children
	const BSP* leftChild = node->GetLeftChildren();
	const BSP* rightChild = node->GetRightChildren();

	if (leftChild != nullptr)
	{
		std::vector<size_t> leftIndexes = GetPotentialQuadblockIndexes(quadblocks, leftChild, rayOrigin, rayDir);
		result.insert(result.end(), leftIndexes.begin(), leftIndexes.end());
	}

	if (rightChild != nullptr)
	{
		std::vector<size_t> rightIndexes = GetPotentialQuadblockIndexes(quadblocks, rightChild, rayOrigin, rayDir);
		result.insert(result.end(), rightIndexes.begin(), rightIndexes.end());
	}

	return result;
}

static std::vector<Vec3> GenerateSamplePointLeaf(const std::vector<Quadblock>& quadblocks, const BSP& leaf, float camera_raise = 0)
{
	// For a leaf node, generate all the points for the vis ray test.
	// Originally the center of each quad in a node.
	// The camera raise is currently done with quad's normal. Might change to up vector later
	std::vector<Vec3> samples;
	const std::vector<size_t>& quadIndexes = leaf.GetQuadblockIndexes();
	for (size_t quadID : quadIndexes)
	{
		const Quadblock& quad = quadblocks[quadID];
		Vec3 center = quad.GetCenter();
		samples.push_back(center + (quad.GetNormal() * camera_raise));
	}
	return samples;
}

BitMatrix GenerateVisTree(const std::vector<Quadblock>& quadblocks, const BSP* root, float maxDistanceSquared)
{
	std::vector<const BSP*> leaves = root->GetLeaves();
	BitMatrix vizMatrix = BitMatrix(leaves.size(), leaves.size());

	const float cameraHeight = 20.0f;

	const int quadCount = static_cast<int>(quadblocks.size());
	std::vector<size_t> quadIndexesToLeaves(quadCount);
	for (size_t i = 0; i < leaves.size(); i++)
	{
		const std::vector<size_t>& quadIndexes = leaves[i]->GetQuadblockIndexes();
		for (size_t index : quadIndexes) { quadIndexesToLeaves[index] = i; }
	}

	for (size_t leafA = 0; leafA < leaves.size(); leafA++)
	{
		printf("Prog: %d/%d\n", static_cast<int>(leafA + 1), static_cast<int>(leaves.size()));
		vizMatrix.Set(true, leafA, leafA);
		const std::vector<Vec3> sampleA = GenerateSamplePointLeaf(quadblocks, *leaves[leafA], cameraHeight);
		for (size_t leafB = leafA + 1; leafB < leaves.size(); leafB++)
		{
			bool foundLeafABHit = false;
			const std::vector<Vec3> sampleB = GenerateSamplePointLeaf(quadblocks, *leaves[leafB], 0.0f);

			for (const Vec3& pointA : sampleA)
			{
				if (foundLeafABHit) { break; }

				for (const Vec3& pointB : sampleB)
				{
					if (foundLeafABHit) { break; }
					Vec3 directionVector = pointB - pointA;
					if (directionVector.LengthSquared() > maxDistanceSquared) { continue; }
					directionVector.Normalize();

					float closestDist = std::numeric_limits<float>::max();
					size_t closestLeaf = leafA;
					std::vector<size_t> potentialQuads = GetPotentialQuadblockIndexes(quadblocks, root, pointA, directionVector);

					// Create thread-local storage for distances to avoid conflicts
					std::vector<float> localDists(potentialQuads.size(), std::numeric_limits<float>::max());

#pragma omp parallel for
					for (int i = 0; i < static_cast<int>(potentialQuads.size()); i++)
					{
						size_t testQuadIndex = potentialQuads[i];
						float dist = 0.0f;
						const Quadblock& testQuad = quadblocks[testQuadIndex];

						if (RayIntersectQuadblockTest(pointA, directionVector, testQuad, dist))
						{
							localDists[i] = dist;
						}
					}

					// Find the closest hit among the potential quads
					for (size_t i = 0; i < potentialQuads.size(); i++)
					{
						if (localDists[i] < closestDist)
						{
							closestDist = localDists[i];
							closestLeaf = quadIndexesToLeaves[potentialQuads[i]];
						}
					}

					if (closestLeaf == leafB)
					{
						foundLeafABHit = true;
					}
				}
			}
			if (foundLeafABHit)
			{
				vizMatrix.Set(true, leafA, leafB);
				vizMatrix.Set(true, leafB, leafA);
			}
		}
	}
	int count = 0;
	for (size_t leafA = 0; leafA < leaves.size(); leafA++)
	{
		for (size_t leafB = 0; leafB < leaves.size(); leafB++)
		{
			if (vizMatrix.Get(leafA, leafB))
			{
				count++;
			}
		}
	}
	printf("Count: %d\n", count);
	return vizMatrix;
}

BitMatrix GenerateVisTreeOLD(const std::vector<Quadblock>& quadblocks, const BSP* root, float maxDistanceSquared)
{
	std::vector<const BSP*> leaves = root->GetLeaves();
	BitMatrix vizMatrix = BitMatrix(leaves.size(), leaves.size());

	const float cameraHeight = 20.0f;

	const int quadCount = static_cast<int>(quadblocks.size());
	std::vector<float> dists(quadCount, std::numeric_limits<float>::max());
	std::vector<size_t> quadIndexesToLeaves(quadCount);
	for (size_t i = 0; i < leaves.size(); i++)
	{
		const std::vector<size_t>& quadIndexes = leaves[i]->GetQuadblockIndexes();
		for (size_t index : quadIndexes) { quadIndexesToLeaves[index] = i; }
	}

	for (size_t leafA = 0; leafA < leaves.size(); leafA++)
	{
		printf("Prog: %d/%d\n", static_cast<int>(leafA + 1), static_cast<int>(leaves.size()));
		vizMatrix.Set(true, leafA, leafA);
		const std::vector<Vec3> sampleA = GenerateSamplePointLeaf(quadblocks, *leaves[leafA], cameraHeight);
		for (size_t leafB = leafA + 1; leafB < leaves.size(); leafB++)
		{
			bool foundLeafABHit = false;
			const std::vector<Vec3> sampleB = GenerateSamplePointLeaf(quadblocks, *leaves[leafB], 0.0f);

			for (const Vec3& pointA : sampleA)
			{
				if (foundLeafABHit) { break; }

				for (const Vec3& pointB : sampleB)
				{
					if (foundLeafABHit) { break; }
					Vec3 directionVector = pointB - pointA;
					if (directionVector.LengthSquared() > maxDistanceSquared) { continue; }
					directionVector.Normalize();
					float closestDist = std::numeric_limits<float>::max();
					size_t closestLeaf = leafA;
					std::vector<size_t> potentialQuads = GetPotentialQuadblockIndexes(quadblocks, root, pointA, directionVector);
#pragma omp parallel for
					// Would be better to first calculate the distance between centerA and the Bbox of leafB.
					// If we ever find a quad from neither leafA or leafB with a smaller distance, we can break here : no visibility from centerA to leafB, because there is an obstacle.
					/*for (int i = 0; i < static_cast<int>(potentialQuads.size()); i++)
					{
						size_t testQuadIndex = potentialQuads[i];*/
					for (int testQuadIndex = 0; testQuadIndex < quadCount; testQuadIndex++)
					{
						float dist = 0.0f;
						const Quadblock& testQuad = quadblocks[testQuadIndex];

						if (RayIntersectQuadblockTest(pointA, directionVector, testQuad, dist))
						{
							dists[testQuadIndex] = dist;
						}
						else
						{
							dists[testQuadIndex] = std::numeric_limits<float>::max();
						}
					}

					for (int i = 0; i < quadCount; i++)
					{
						if (dists[i] < closestDist)
						{
							closestDist = dists[i];
							closestLeaf = quadIndexesToLeaves[i];
						}
					}

					if (closestLeaf == leafB)
					{
						foundLeafABHit = true;
					}
				}
			}
			if (foundLeafABHit)
			{
				vizMatrix.Set(true, leafA, leafB);
				vizMatrix.Set(true, leafB, leafA);
			}
		}
	}
	int count = 0;
	for (size_t leafA = 0; leafA < leaves.size(); leafA++)
	{
		for (size_t leafB = 0; leafB < leaves.size(); leafB++)
		{
			if (vizMatrix.Get(leafA, leafB))
			{
				count++;
			}
		}
	}
	printf("Count: %d\n", count);
	return vizMatrix;
}