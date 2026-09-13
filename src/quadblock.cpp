#include "quadblock.h"
#include "utils.h"
#include "settings.h"

#include <unordered_map>
#include <map>
#include <unordered_set>
#include <cstring>

Quadblock::Quadblock(const std::string& name,
	const std::vector<Point>& points,
	std::vector<std::vector<size_t>>& facesIndexes,
	std::vector<std::vector<Vec2>>& faceUVs,
	std::vector<std::string>& faceMaterials,
	bool hasUV, UpdateFilterCallback filterCallback)
{
	constexpr size_t INVALID = std::numeric_limits<size_t>::max();
	constexpr size_t MIN_VERTS_PER_FACE = 3;
	constexpr size_t MAX_VERTS_PER_FACE = 4;

	if (facesIndexes.empty() || facesIndexes.size() > NUM_FACES_QUADBLOCK)
	{
		throw QuadException("OBJ error: expected between 1 and " + std::to_string(NUM_FACES_QUADBLOCK) +
			" faces, found " + std::to_string(facesIndexes.size()) + ".");
	}
	if (faceUVs.size() != facesIndexes.size() || faceMaterials.size() != facesIndexes.size())
	{
		throw QuadException("OBJ error: face data arrays have mismatched sizes.");
	}
	for (size_t objFaceId = 0; objFaceId < facesIndexes.size(); objFaceId++)
	{
		const size_t faceVertCount = facesIndexes[objFaceId].size();
		if (faceVertCount < MIN_VERTS_PER_FACE || faceVertCount > MAX_VERTS_PER_FACE)
		{
			throw QuadException("OBJ error: face " + std::to_string(objFaceId) + " has " + std::to_string(faceVertCount) +
				" vertices, expected between " + std::to_string(MIN_VERTS_PER_FACE) + " and " + std::to_string(MAX_VERTS_PER_FACE) + ".");
		}
		if (faceUVs[objFaceId].size() != faceVertCount)
		{
			throw QuadException("OBJ error: face " + std::to_string(objFaceId) + " has " + std::to_string(faceVertCount) +
				" vertices but " + std::to_string(faceUVs[objFaceId].size()) + " UVs.");
		}
	}
	if (points.size() > NUM_VERTICES_QUADBLOCK)
		throw QuadException("OBJ error : Expected 9 uniques vertices or less, found " + std::to_string(points.size()));

	std::map<std::tuple<size_t, size_t>, size_t> objFaceVertIdMap; // (objFaceId, objVertId) -> objFaceVertId
	std::vector<size_t> refCount(points.size(), 0);
	for (size_t objFaceId = 0; objFaceId < facesIndexes.size(); objFaceId++)
	{
		const auto& face = facesIndexes[objFaceId];
		for (size_t objFaceVertId = 0; objFaceVertId < face.size(); objFaceVertId++)
		{
			size_t objVertId = face[objFaceVertId];
			if (objVertId >= points.size()) { throw QuadException("Face references a vertex index out of range."); }
			refCount[objVertId]++;
			objFaceVertIdMap[std::make_tuple(objFaceId, objVertId)] = objFaceVertId;
		}
	}

	std::vector<size_t> objCenterIds;
	for (size_t objVertId = 0; objVertId < points.size(); objVertId++)
	{
		if (refCount[objVertId] == facesIndexes.size())
			objCenterIds.push_back(objVertId);
	}

	m_triblock = false;
	if (objCenterIds.empty()) // Test if this is a legacy triblock.
	{
		if (facesIndexes.size() != 4)
			throw QuadException("Not a quadblock (no center) and not a legacy triblock (not 4 trifaces)");
		for (size_t objFaceId = 0; objFaceId < facesIndexes.size(); objFaceId++)
		{
			if (facesIndexes[objFaceId].size() != 3)
				throw QuadException("Not a quadblock (no center) and not a legacy triblock (not 4 trifaces)");
		}
		// Find center face
		size_t centerFace = INVALID;
		for (size_t objFaceId = 0; objFaceId < facesIndexes.size(); objFaceId++)
		{
			bool isCenter = true;
			for (size_t objVertId : facesIndexes[objFaceId])
			{
				if (refCount[objVertId] != 3)
					isCenter = false;
			}
			if (isCenter) { centerFace = objFaceId; break; }
		}
		if (centerFace == INVALID)
			throw QuadException("Couldn't assimilate to a valid triblock");

		// Find matching UVs	
		bool foundMatching = false;
		for (size_t faceVertId = 0; faceVertId < 3; faceVertId++)
		{
			if (foundMatching) { break; }
			size_t edgeVert1 = facesIndexes[centerFace][faceVertId];
			size_t edgeVert2 = facesIndexes[centerFace][(faceVertId + 1) % 3];
			for (size_t outFaceId = 0; outFaceId < facesIndexes.size(); outFaceId++)
			{
				if (outFaceId == centerFace) { continue; }
				if (!objFaceVertIdMap.contains(std::make_tuple(outFaceId, edgeVert1))) { continue; }
				if (!objFaceVertIdMap.contains(std::make_tuple(outFaceId, edgeVert2))) { continue; }
				size_t outFaceEdgeVert1Id = objFaceVertIdMap[std::make_tuple(outFaceId, edgeVert1)];
				size_t outFaceEdgeVert2Id = objFaceVertIdMap[std::make_tuple(outFaceId, edgeVert2)];
				if (outFaceEdgeVert1Id != (outFaceEdgeVert2Id + 1) % 3) { continue; }
				// Correct face, now check UVs
				auto UVsMatch = [](const Vec2& a, const Vec2& b) -> bool
					{
						constexpr float UV_EPS = 1/256.0f; 
						return std::abs(a.x - b.x) < UV_EPS && std::abs(a.y - b.y) < UV_EPS;
					};
				if (UVsMatch(faceUVs[centerFace][faceVertId], faceUVs[outFaceId][outFaceEdgeVert1Id]) &&
					UVsMatch(faceUVs[centerFace][(faceVertId + 1) % 3], faceUVs[outFaceId][outFaceEdgeVert2Id])) 
				{
					// Found matching, merge CenterFace and OutFaceId into a quadface.
					foundMatching = true;
					std::vector<size_t> newFaceIndexes; 
					std::vector<Vec2> newFaceUVs;
					newFaceIndexes.push_back(facesIndexes[centerFace][(faceVertId + 1) % 3]);
					newFaceUVs.push_back(faceUVs[centerFace][(faceVertId + 1) % 3]);
					newFaceIndexes.push_back(facesIndexes[centerFace][(faceVertId + 2) % 3]);
					newFaceUVs.push_back(faceUVs[centerFace][(faceVertId + 2) % 3]);
					newFaceIndexes.push_back(facesIndexes[centerFace][faceVertId]);
					newFaceUVs.push_back(faceUVs[centerFace][faceVertId]);
					newFaceIndexes.push_back(facesIndexes[outFaceId][(outFaceEdgeVert1Id + 1) % 3]);
					newFaceUVs.push_back(faceUVs[outFaceId][(outFaceEdgeVert1Id + 1) % 3]);
					std::string newMaterial = faceMaterials[centerFace];
					size_t firstEraseIdx = std::max(centerFace, outFaceId);
					size_t secondEraseIdx = std::min(centerFace, outFaceId);
					facesIndexes.erase(std::next(facesIndexes.begin(), firstEraseIdx));
					facesIndexes.erase(std::next(facesIndexes.begin(), secondEraseIdx));
					faceUVs.erase(std::next(faceUVs.begin(), firstEraseIdx));
					faceUVs.erase(std::next(faceUVs.begin(), secondEraseIdx));
					faceMaterials.erase(std::next(faceMaterials.begin(), firstEraseIdx));
					faceMaterials.erase(std::next(faceMaterials.begin(), secondEraseIdx));
					facesIndexes.push_back(newFaceIndexes);
					faceUVs.push_back(newFaceUVs);
					faceMaterials.push_back(newMaterial);

					// Re compute the start of this constructor with those new params : 
					objFaceVertIdMap.clear();
					for (size_t& count : refCount) { count = 0 ; }
					for (size_t objFaceId = 0; objFaceId < facesIndexes.size(); objFaceId++)
					{
						const auto& face = facesIndexes[objFaceId];
						for (size_t objFaceVertId = 0; objFaceVertId < face.size(); objFaceVertId++)
						{
							size_t objVertId = face[objFaceVertId];
							refCount[objVertId]++;
							objFaceVertIdMap[std::make_tuple(objFaceId, objVertId)] = objFaceVertId;
						}
					}
					for (size_t objVertId = 0; objVertId < points.size(); objVertId++)
					{
						if (refCount[objVertId] == facesIndexes.size())
							objCenterIds.push_back(objVertId);
					}
					if (objCenterIds.empty())
						throw QuadException("Couldn't assimilate to a valid triblock");
					m_triblock = true;
					break;
				}
			}
		}
		if (!foundMatching)
		{
			throw QuadException("Wrong triblock UVs : Make sure 2 trifaces can be merged into a quadFace without breaking UVs");
		}
	}

	size_t objCenterId = objCenterIds[0]; // any valid center works, we fix one.
	std::vector<size_t> faceOBJtoQuad(facesIndexes.size(), INVALID); // OBJ face index -> Quadblock face index
	std::array<size_t, NUM_FACES_QUADBLOCK> faceQuadtoOBJ{}; faceQuadtoOBJ.fill(INVALID); // Quadblock face index -> OBJ face index
	std::array<size_t, NUM_VERTICES_QUADBLOCK> vertQuadtoOBJ{}; vertQuadtoOBJ.fill(INVALID); // Quadblock vert index -> OBJ vert index

	constexpr size_t quadFaceVertOrder[NUM_FACES_QUADBLOCK][4] = 
	{
		{4, 3, 0, 1},
		{4, 1, 2, 5},
		{4, 7, 6, 3},
		{4, 5, 8, 7}
	};
	constexpr size_t quadFaceOrder[NUM_FACES_QUADBLOCK] = { 0, 1 , 3 , 2 };


	auto FindRelativeFaceQuad = [&](size_t faceId, int offset) -> size_t // other quadface than faceId that share the edge with center and center+offset 
		{
			size_t facePos = INVALID;
			for (size_t i = 0; i < NUM_FACES_QUADBLOCK; i++)
			{
				if (quadFaceOrder[i] == faceId) 
					facePos = i;
			}
			return quadFaceOrder[(((static_cast<int>(facePos) - offset) % NUM_FACES_QUADBLOCK) + NUM_FACES_QUADBLOCK) % NUM_FACES_QUADBLOCK];
		};

	auto FindRelativePointQuad = [&](size_t quadFaceId, size_t quadVertId, int offset) -> size_t
		{
			size_t quadVertInFace = INVALID;
			for (quadVertInFace = 0; quadVertInFace < 4; quadVertInFace++)
			{
				if (quadFaceVertOrder[quadFaceId][quadVertInFace] == quadVertId)
					break;
			}
			return quadFaceVertOrder[quadFaceId][(((static_cast<int>(quadVertInFace) + offset) % 4) + 4) % 4];
		};

	auto FindRelativePointOBJ = [&](size_t objFaceId, size_t objGlobalVertId, int offset) -> size_t
		{
			// facesIndexes[i] ; vert ID (quad ID, not face) ; Offset (like +1 for next in face) -> vert ID (quad ID not face)
			if (!objFaceVertIdMap.contains(std::make_tuple(objFaceId, objGlobalVertId)))
				return INVALID;
			size_t objFaceVertId = objFaceVertIdMap[std::make_tuple(objFaceId, objGlobalVertId)];				
			int facesSize = static_cast<int>(facesIndexes[objFaceId].size());
			return facesIndexes[objFaceId][(((static_cast<int>(objFaceVertId) - offset) % facesSize) + facesSize) % facesSize];
		};

	// Step 1 : Assign quadFace0 with any face in the OBJ. We pick a 4-vert face if any to avoid rotations later
	// We assign it to quadFace0 since face it can't always be collapsed, so it's better if it's not INVALID.
	size_t objFace0 = 0;
	for (size_t objFaceId = 0; objFaceId < facesIndexes.size(); objFaceId++)
	{
		if (facesIndexes[objFaceId].size() == 4) { objFace0 = objFaceId; break; } 
	}
	faceOBJtoQuad[objFace0] = 0;
	faceQuadtoOBJ[0] = objFace0;

	const size_t centerQuadVertId = 4;
	vertQuadtoOBJ[centerQuadVertId] = objCenterId;

	// Resolve quad faces from 1 connected component
	std::vector<size_t> objFaceIdToVisit = { objFace0 };
	while (!objFaceIdToVisit.empty())
	{
		size_t objFaceId = objFaceIdToVisit.back();
		objFaceIdToVisit.pop_back();
		size_t quadFaceId = faceOBJtoQuad[objFaceId];
		for (int offset : {-1, 1})
		{
			size_t relQuadVertId = FindRelativePointQuad(quadFaceId, centerQuadVertId, offset);
			size_t relOBJVertId = FindRelativePointOBJ(objFaceId, objCenterId, offset);
			size_t relQuadFaceId = FindRelativeFaceQuad(quadFaceId, offset);
			for (size_t otherObjFaceId = 0; otherObjFaceId < facesIndexes.size(); otherObjFaceId++)
			{
				if (FindRelativePointOBJ(otherObjFaceId, objCenterId, -offset) == relOBJVertId)
				{
					if (faceOBJtoQuad[otherObjFaceId] == INVALID)
					{
						faceOBJtoQuad[otherObjFaceId] = relQuadFaceId;
						faceQuadtoOBJ[relQuadFaceId] = otherObjFaceId;
						objFaceIdToVisit.push_back(otherObjFaceId);
						break;
					}
				}
			}
		}
	}


	// This step is very tricky : Assign not assigned objFaceId. Needs empirical verification.
	std::vector<size_t> notAssignedOBJFaceId;
	for (size_t objFaceId = 0; objFaceId < facesIndexes.size(); objFaceId++)
	{
		if (faceOBJtoQuad[objFaceId] == INVALID)
			notAssignedOBJFaceId.push_back(objFaceId);
	}
	if (notAssignedOBJFaceId.size() > 2)
		throw QuadException("3 faces not reachable from face " + std::to_string(faceQuadtoOBJ[0])); // I think this isn't possible from a quadblock ?
	if (notAssignedOBJFaceId.size() == 2)
	{
		// 2 not assigned. They must touch eachother. Let's see which one is ahead/behind the other in face winding order.
		size_t objFaceA = notAssignedOBJFaceId[0];
		size_t objFaceB = notAssignedOBJFaceId[1];
		size_t prevObjFace = INVALID; size_t nextObjFace = INVALID;
		if (FindRelativePointOBJ(objFaceA, objCenterId, 1) == FindRelativePointOBJ(objFaceB, objCenterId, -1))
		{
			nextObjFace = objFaceA; 
			prevObjFace = objFaceB;
		}
		else if (FindRelativePointOBJ(objFaceA, objCenterId, -1) == FindRelativePointOBJ(objFaceB, objCenterId, 1))
		{
			nextObjFace = objFaceB;
			prevObjFace = objFaceA;
		}
		else
			throw QuadException("More than 3 edge based connected component"); // Should be impossible with correctly formed data
		// The used faces are either 0 and 1 or 0 and 2. Unused are 1 and 3 or 2 and 3
		if (faceQuadtoOBJ[2] == INVALID)
		{
			faceQuadtoOBJ[2] = nextObjFace;
			faceOBJtoQuad[nextObjFace] = 2;
			faceQuadtoOBJ[3] = prevObjFace;
			faceOBJtoQuad[prevObjFace] = 3;
		}
		else
		{
			faceQuadtoOBJ[3] = nextObjFace;
			faceOBJtoQuad[nextObjFace] = 3;
			faceQuadtoOBJ[1] = prevObjFace;
			faceOBJtoQuad[prevObjFace] = 1;
		}
	}
	if (notAssignedOBJFaceId.size() == 1)
	{
		if (facesIndexes.size() == 4) // 4 total faces, 3 + 1. Invalid. For the same reason that 1 + 3 was invalid.
			throw QuadException("3 faces not reachable from face " + std::to_string(notAssignedOBJFaceId[0])); // I think this isn't possible from a quadblock ?
		faceOBJtoQuad[notAssignedOBJFaceId[0]] = 3;
		faceQuadtoOBJ[3] = notAssignedOBJFaceId[0];
	}
	for (size_t objFaceId = 0; objFaceId < facesIndexes.size(); objFaceId++)
	{
		if (faceOBJtoQuad[objFaceId] == INVALID)
			throw QuadException("Can't resolve the objFace " + std::to_string(objFaceId));
	}

	// Resolve quad vertices
	for (size_t objFaceId = 0; objFaceId < facesIndexes.size(); objFaceId++)
	{
		size_t quadFaceId = faceOBJtoQuad[objFaceId];
		for (int offset : {-1, 1, 2})
		{
			if (offset == 2 && facesIndexes[objFaceId].size() == 3) continue; // No offset 2 for triface, since it's equivalent to -1
			size_t relQuadVertId = FindRelativePointQuad(quadFaceId, centerQuadVertId, offset);
			size_t oppQuadVertId = FindRelativePointQuad(quadFaceId, centerQuadVertId, 2); // opposite corner of the quad
			size_t relOBJVertId = FindRelativePointOBJ(objFaceId, objCenterId, offset);

			if (refCount[relOBJVertId] == 2 || (offset != 2 && faceQuadtoOBJ[FindRelativeFaceQuad(quadFaceId, offset)] == INVALID))
				vertQuadtoOBJ[relQuadVertId] = relOBJVertId;
			else
				vertQuadtoOBJ[oppQuadVertId] = relOBJVertId;			
		}
	}

	// Adjust some geometry (to potentially avoid a rotation later)
	// The idea is : If we can free a sharedVert (1,5,7,3) onto a unique vert collapsable (2, 6), we do it. So collapsing missing face don't trigger a rotation.
	for (size_t quadFaceId : {1, 2})
	{
		size_t objFaceId = faceQuadtoOBJ[quadFaceId];
		if (objFaceId != INVALID)
		{
			size_t prevOBJVertId = FindRelativePointOBJ(objFaceId, objCenterId, -1);
			size_t nextOBJVertId = FindRelativePointOBJ(objFaceId, objCenterId, 1);
			size_t prevQuadVertId = FindRelativePointQuad(quadFaceId, centerQuadVertId, -1);
			size_t nextQuadVertId = FindRelativePointQuad(quadFaceId, centerQuadVertId, 1);
			size_t oppQuadVertId = FindRelativePointQuad(quadFaceId, centerQuadVertId, 2);
			if (vertQuadtoOBJ[oppQuadVertId] == INVALID)
			{
				if (refCount[prevOBJVertId] == 1)
				{
					vertQuadtoOBJ[oppQuadVertId] = vertQuadtoOBJ[prevQuadVertId];
					vertQuadtoOBJ[prevQuadVertId] = INVALID;
				}
				else if (refCount[nextOBJVertId] == 1)
				{
					vertQuadtoOBJ[oppQuadVertId] = vertQuadtoOBJ[nextQuadVertId];
					vertQuadtoOBJ[nextQuadVertId] = INVALID;
				}
			}
		}
	}


	bool needRotation = false;
	bool noRotation = false;
	// Collapse missing face's unique vert
	for (size_t quadFaceId = 0; quadFaceId < NUM_FACES_QUADBLOCK; quadFaceId++)
	{
		if (faceQuadtoOBJ[quadFaceId] == INVALID)
		{
			int uniqueVertOffset = 2; 
			size_t uniqueQuadVertInFace = FindRelativePointQuad(quadFaceId, centerQuadVertId, uniqueVertOffset);
			int prevEdgeOffset = -1;
			size_t prevQuadFaceId = FindRelativeFaceQuad(quadFaceId, prevEdgeOffset);
			size_t prevEdgeQuadVertInFace = FindRelativePointQuad(quadFaceId, centerQuadVertId, prevEdgeOffset);
			int nextEdgeOffset = 1;
			size_t nextQuadFaceId = FindRelativeFaceQuad(quadFaceId, nextEdgeOffset);
			size_t nextEdgeQuadVertInFace = FindRelativePointQuad(quadFaceId, centerQuadVertId, nextEdgeOffset);
			
			vertQuadtoOBJ[uniqueQuadVertInFace] = vertQuadtoOBJ[centerQuadVertId];
			if (faceQuadtoOBJ[prevQuadFaceId] != INVALID && faceQuadtoOBJ[nextQuadFaceId] != INVALID) // Check that logic
			{
				if (vertQuadtoOBJ[prevEdgeQuadVertInFace] != INVALID && vertQuadtoOBJ[nextEdgeQuadVertInFace] != INVALID)
				{
					if (uniqueQuadVertInFace == 0 || uniqueQuadVertInFace == 8)
					{
						printf("WARNING : When collapsing missing face in %s, we collapsed the vert %zu\n", name.c_str(), uniqueQuadVertInFace);
						needRotation = true;
					}
					else
						noRotation = true;
				}
			}
		}
	}

	if (needRotation && noRotation)
		throw QuadException("No valid rotation found that preserves geometry"); // Need more research maybe ? Or actually impossible ? Can this even happen ?
	if (needRotation)
	{
		printf("Rotation done on %s\n", name.c_str());
		std::array<size_t, NUM_FACES_QUADBLOCK> rotation90FaceMap = { 1, 3, 0, 2 };
		std::array<size_t, NUM_VERTICES_QUADBLOCK> rotation90VertMap = { 2, 5, 8, 1, 4, 7, 0, 3, 6};
		std::vector<size_t> rotatedFaceOBJtoQuad(facesIndexes.size(), INVALID); 
		std::array<size_t, NUM_FACES_QUADBLOCK> rotatedFaceQuadtoOBJ{}; rotatedFaceQuadtoOBJ.fill(INVALID);
		std::array<size_t, NUM_VERTICES_QUADBLOCK> rotatedVertQuadtoOBJ{}; rotatedVertQuadtoOBJ.fill(INVALID);
		
		for (size_t quadFaceId = 0; quadFaceId < NUM_FACES_QUADBLOCK; quadFaceId++)
		{
			rotatedFaceQuadtoOBJ[quadFaceId] = faceQuadtoOBJ[rotation90FaceMap[quadFaceId]];
			if (faceQuadtoOBJ[rotation90FaceMap[quadFaceId]] != INVALID)
				rotatedFaceOBJtoQuad[faceQuadtoOBJ[rotation90FaceMap[quadFaceId]]] = quadFaceId;
		}
		for (size_t quadVertId = 0; quadVertId < NUM_VERTICES_QUADBLOCK; quadVertId++)
		{
			rotatedVertQuadtoOBJ[quadVertId] = vertQuadtoOBJ[rotation90VertMap[quadVertId]];
		}
		faceOBJtoQuad = rotatedFaceOBJtoQuad;
		faceQuadtoOBJ = rotatedFaceQuadtoOBJ;
		vertQuadtoOBJ = rotatedVertQuadtoOBJ;
	}
	if (noRotation)
		printf("No-rotation requested on %s\n", name.c_str());
	// collapse missing edges to center
	for (size_t quadFaceId = 0; quadFaceId < NUM_FACES_QUADBLOCK; quadFaceId++)
	{
		int prevEdgeOffset = -1; // Gets the previous edge starting from center
		size_t prevEdgeQuadVertInFace = FindRelativePointQuad(quadFaceId, centerQuadVertId, prevEdgeOffset);
		if (vertQuadtoOBJ[prevEdgeQuadVertInFace] == INVALID)
			vertQuadtoOBJ[prevEdgeQuadVertInFace] = vertQuadtoOBJ[centerQuadVertId];
	}
	// collapse missing unique vert to edges
	for (size_t quadFaceId = 0; quadFaceId < NUM_FACES_QUADBLOCK; quadFaceId++)
	{
		int prevEdgeOffset = -1;
		int uniqueVertOffset = 2;
		size_t uniqueQuadVertInFace = FindRelativePointQuad(quadFaceId, centerQuadVertId, uniqueVertOffset);
		size_t prevEdgeQuadVertInFace = FindRelativePointQuad(quadFaceId, centerQuadVertId, prevEdgeOffset);
		if (vertQuadtoOBJ[uniqueQuadVertInFace] == INVALID)
			vertQuadtoOBJ[uniqueQuadVertInFace] = vertQuadtoOBJ[prevEdgeQuadVertInFace];
	}
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++) 
	{ 
		if (vertQuadtoOBJ[i] == INVALID)
			throw QuadException("Quadblock Vert " + std::to_string(i) + " couldn't be identified");	
			
		m_p[i] = Vertex(points[vertQuadtoOBJ[i]]);
	}

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

	ResetUVs();
	if (hasUV)
	{
		for (size_t quadFaceId = 0; quadFaceId < NUM_FACES_QUADBLOCK; quadFaceId++)
		{
			const size_t objFaceId = faceQuadtoOBJ[quadFaceId];
			if (objFaceId == INVALID) continue;
			for (size_t faceVertId = 0; faceVertId < 4; faceVertId++)
			{
				size_t quadVertId = uvVertInd[quadFaceId][faceVertId];
				size_t objGlobalVertId = vertQuadtoOBJ[quadVertId];
				size_t objVertFaceId = objFaceVertIdMap[std::make_tuple(objFaceId, objGlobalVertId)];
				m_uvs[quadFaceId][faceVertId] = faceUVs[objFaceId][objVertFaceId];
			}
			// Note : Low LOD UVs are assigned some default Reset UVs values.
		}
	}

	m_name = name;
	for (size_t quadFaceId = 0; quadFaceId < NUM_FACES_QUADBLOCK; quadFaceId++) 
	{ 
		size_t objFaceId = faceQuadtoOBJ[quadFaceId];
		if (objFaceId == INVALID) 
			m_materials[quadFaceId] = faceMaterials[0];
		else
			m_materials[quadFaceId] = faceMaterials[objFaceId];
	}
	m_materials[NUM_FACES_QUADBLOCK] = m_materials[0];
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
	m_triblock = false;
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
	m_textureIDs[quad] = static_cast<int>(id);
}

void Quadblock::SetAnimTextureOffset(size_t relOffset, size_t levOffset, size_t quad)
{
	m_animTexOffset[quad] = static_cast<int>(relOffset + levOffset);
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
		std::vector<float> normalLengths;
		for (std::array<size_t, 3> triFace : GetTriFacesIndexes())
			normalLengths.push_back(ComputeNormalVector(triFace[0], triFace[1], triFace[2]).Length());
		normalLengths.push_back(ComputeNormalVector(0, 2, 6).Length());
		normalLengths.push_back(ComputeNormalVector(2, 8, 6).Length());
		float maxNormalLength = *std::max_element(normalLengths.begin(), normalLengths.end());

		quadblock.triNormalVecBitshift = static_cast<uint8_t>(std::round(std::log2(maxNormalLength * 512.0f)));

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
void Quadblock::ComputeCollTrifaces()
{
	const bool equivalentDiagonal = std::abs((m_p[2].m_pos - m_p[6].m_pos).Length() - ((m_p[2].m_pos - m_p[4].m_pos).Length() + (m_p[4].m_pos - m_p[6].m_pos).Length())) <= EPSILON;
	const bool equivalentSide02 = std::abs((m_p[0].m_pos - m_p[2].m_pos).Length() - ((m_p[0].m_pos - m_p[1].m_pos).Length() + (m_p[1].m_pos - m_p[2].m_pos).Length())) <= EPSILON;
	const bool equivalentSide06 = std::abs((m_p[0].m_pos - m_p[6].m_pos).Length() - ((m_p[0].m_pos - m_p[3].m_pos).Length() + (m_p[3].m_pos - m_p[6].m_pos).Length())) <= EPSILON;
	if (equivalentDiagonal && equivalentSide02 && equivalentSide06) { m_collTriFaces = { {0, 2, 6} }; }
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
		if (equivalentDiagonal && equivalentSide28 && equivalentSide68) { m_collTriFaces.push_back({ 2, 8, 6 }); }
		else
		{
			m_collTriFaces.push_back({ 2, 5, 4 });
			m_collTriFaces.push_back({ 4, 7, 6 });
			m_collTriFaces.push_back({ 4, 5, 7 });
			m_collTriFaces.push_back({ 5, 8, 7 });
		}
	}
}

void Quadblock::SetDefaultValues()
{
	ComputeBoundingBox();
	ComputeCollTrifaces();
	m_checkpointIndex = -1;
	m_flags = QuadFlags::DEFAULT;
	m_terrain = TerrainType::LABELS.at(TerrainType::DEFAULT);

	for (size_t i = 0; i < NUM_FACES_QUADBLOCK; i++)
	{
		m_faceDrawMode[i] = FaceDrawMode::DRAW_BOTH;
		m_faceRotateFlip[i] = FaceRotateFlip::NONE;
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