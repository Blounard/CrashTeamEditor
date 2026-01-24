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

BitMatrix GenerateVisTree(const std::vector<Quadblock>& quadblocks, const std::vector<const BSP*>& leaves, float maxDistance)
{
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
		for (size_t leafB = leafA + 1; leafB < leaves.size(); leafB++)
		{
			bool foundLeafABHit = false;
			const std::vector<size_t>& quadIndexesA = leaves[leafA]->GetQuadblockIndexes();
			const std::vector<size_t>& quadIndexesB = leaves[leafB]->GetQuadblockIndexes();

			for (size_t quadIDA : quadIndexesA)
			{
				if (foundLeafABHit) { break; }

				Quadblock quadA = quadblocks[quadIDA];
				quadA.Translate(cameraHeight, quadA.GetNormal());
				const Vec3& centerA = quadA.GetCenter();

				for (size_t quadIDB : quadIndexesB)
				{
					if (foundLeafABHit) { break; }

					const Quadblock& quadB = quadblocks[quadIDB];
					const Vec3& centerB = quadB.GetCenter();
					Vec3 directionVector = centerB - centerA;
					if (directionVector.LengthSquared() > maxDistance) { continue; }
					directionVector.Normalize();
					float closestDist = std::numeric_limits<float>::max();
					size_t closestLeaf = leafA;

#pragma omp parallel for
					// Would be better to first calculate the distance between centerA and the Bbox of leafB.
					// If we ever find a quad from neither leafA or leafB with a smaller distance, we can break here : no visibility from centerA to leafB, because there is an obstacle.
					for (int testQuadIndex = 0; testQuadIndex < quadCount; testQuadIndex++)
					{
						float dist = 0.0f;
						const Quadblock& testQuad = quadblocks[testQuadIndex];

						if (RayIntersectQuadblockTest(centerA, directionVector, testQuad, dist))
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
