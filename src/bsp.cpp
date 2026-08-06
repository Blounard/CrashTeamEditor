#include "bsp.h"
#include "psx_types.h"

#include <cstring>
#include <set>
#include <map>

static size_t g_id = 0;
static std::set<size_t> g_idSet = {};

size_t FindAvailableID()
{
	size_t id = 0;
	for (const size_t usedId : g_idSet)
	{
		if (usedId != id) { break; }
		id++;
	}
	if (!g_idSet.contains(id))
	{
		g_idSet.insert(id);
		return id;
	}
	else
	{
		printf("Error in BSP ID allocation : %d\n", id);
		return 0;
	}
}

void ResetAllBSPID()
{
	g_idSet.clear();
}

BSP::BSP()
{
	m_parent = nullptr;
	m_id = FindAvailableID();
	m_idFlag = 0;
	m_leftFlag = 0;
	m_rightFlag = 0;
	m_node = BSPNode::BRANCH;
	m_axis = AxisSplit::NONE;
	m_flags = BSPFlags::NONE;
	m_left = nullptr;
	m_right = nullptr;
	m_bbox = BoundingBox();
	m_quadblockIndexes = std::vector<size_t>();
}

BSP::BSP(BSPNode type, const std::vector<size_t>& quadblockIndexes, BSP* parent, const std::vector<Quadblock>& quadblocks)
{
	bool isLeaf = type == BSPNode::LEAF;
	m_parent = parent;
	m_id = FindAvailableID();
	m_idFlag = 0;
	m_leftFlag = 0;
	m_rightFlag = 0;
	m_node = type;
	m_axis = AxisSplit::NONE;
	m_flags = isLeaf ? BSPFlags::LEAF : BSPFlags::NONE;
	m_left = nullptr;
	m_right = nullptr;
	m_bbox = BoundingBox();
	m_quadblockIndexes = quadblockIndexes;
	if (isLeaf)
	{
		for (size_t index : m_quadblockIndexes) { quadblocks[index].SetBSPID(m_id); }
	}
}

BSP::~BSP()
{
	g_idSet.erase(m_id);
}

void BSP::PopulateBranch(PSX::BSPBranch& branch, std::vector<BSP*>& bspArray, size_t global_id)
{
	//g_id = global_id;
	g_idSet.insert(branch.id);
	m_id = branch.id;
	m_node = BSPNode::BRANCH;
	if (branch.axis.x != 0) { m_axis = AxisSplit::X; }
	else if (branch.axis.y != 0) { m_axis = AxisSplit::Y; }
	else if (branch.axis.z != 0) { m_axis = AxisSplit::Z; }
	else { m_axis = AxisSplit::NONE; }
	m_flags = branch.flag;
	if (branch.leftChild != BSPID::EMPTY)
	{
		uint16_t leftId = branch.leftChild & (BSPID::LEAF-1);
		if (leftId < bspArray.size())
		{
			m_left = bspArray[leftId];
			m_left->SetParent(this);
			m_leftFlag = branch.leftChild - leftId;
		}
	}
	if (branch.rightChild != BSPID::EMPTY)
	{
		uint16_t rightId = branch.rightChild & (BSPID::LEAF - 1);
		if (rightId < bspArray.size())
		{
			m_right = bspArray[rightId];
			m_right->SetParent(this);
			m_rightFlag = branch.rightChild - rightId;
		}
	}
	m_bbox = BoundingBox();
	m_bbox.min = ConvertPSXVec3(branch.bbox.min, FP_ONE_GEO);
	m_bbox.max = ConvertPSXVec3(branch.bbox.max, FP_ONE_GEO);
	m_quadblockIndexes = std::vector<size_t>(); // Need to be set later
}

void BSP::PopulateLeaf(PSX::BSPLeaf& leaf, std::vector<BSP*>& bspArray, const std::vector<Quadblock>& quadblocks, uint32_t offQuadblocks, size_t global_id)
{
	//g_id = global_id;
	g_idSet.insert(leaf.id);
	m_id = leaf.id;
	m_node = BSPNode::LEAF;
	m_axis = AxisSplit::NONE;
	m_flags = leaf.flag;
	m_left = nullptr; 
	m_right = nullptr; 
	m_bbox = BoundingBox();
	m_bbox.min = ConvertPSXVec3(leaf.bbox.min, FP_ONE_GEO);
	m_bbox.max = ConvertPSXVec3(leaf.bbox.max, FP_ONE_GEO);
	m_quadblockIndexes = std::vector<size_t>();
	for (size_t i = 0; i < leaf.numQuads; i++)
	{
		uint32_t relative_offset = leaf.offQuads - offQuadblocks;
		m_quadblockIndexes.push_back(i + relative_offset/sizeof(PSX::Quadblock));
	}
	for (size_t i : m_quadblockIndexes)
	{
		if (i < quadblocks.size()) { quadblocks[i].SetBSPID(m_id); }
	}
}


void BSP::PopulateBranchQuadIndexes()
{
	if (m_node == BSPNode::BRANCH)
	{
		m_quadblockIndexes.clear();
		if (m_left != nullptr)
		{
			m_left->PopulateBranchQuadIndexes();
			const std::vector<size_t>& leftIndexes = m_left->GetQuadblockIndexes();
			m_quadblockIndexes.insert(m_quadblockIndexes.end(), leftIndexes.begin(), leftIndexes.end());
		}
		if (m_right != nullptr)
		{
			m_right->PopulateBranchQuadIndexes();
			const std::vector<size_t>& rightIndexes = m_right->GetQuadblockIndexes();
			m_quadblockIndexes.insert(m_quadblockIndexes.end(), rightIndexes.begin(), rightIndexes.end());
		}
	}
}

size_t BSP::GetId() const
{
	return m_id;
}

void BSP::SetId(size_t id)
{
	g_idSet.erase(m_id);
	if (g_idSet.contains(id))
	{
		printf("ERROR : CAN'T ASSIGN ID %d to BSP BECAUSE IT'S ALREADY USED\n", id);
		m_id = FindAvailableID();
	}
	m_id = id;
	g_idSet.insert(id);
}

bool BSP::IsEmpty() const
{
	return m_quadblockIndexes.empty();
}

bool BSP::IsValid() const
{
	for (const BSP* bsp : GetTree())
	{
		if (bsp->m_node == BSPNode::BRANCH) { if (!bsp->m_right && !bsp->m_left) { return false; } }
		else if (bsp->IsEmpty()) { return false; }
	}
	return true;
}

bool BSP::IsBranch() const
{
	return m_node == BSPNode::BRANCH;
}

uint16_t BSP::GetFlags() const
{
	return m_flags;
}

const std::string& BSP::GetType() const
{
	const static std::string sBranch = "Branch";
	const static std::string sLeaf = "Leaf";
	return m_node == BSPNode::BRANCH ? sBranch : sLeaf;
}

const std::string& BSP::GetAxis() const
{
	static std::string sX = "X";
	static std::string sY = "Y";
	static std::string sZ = "Z";
	static std::string sNone = "None";
	switch (m_axis)
	{
	case AxisSplit::X: return sX;
	case AxisSplit::Y: return sY;
	case AxisSplit::Z: return sZ;
	}
	return sNone;
}

const BoundingBox& BSP::GetBoundingBox() const
{
	return m_bbox;
}

const std::vector<size_t>& BSP::GetQuadblockIndexes() const
{
	return m_quadblockIndexes;
}

const BSP* BSP::GetLeftChildren() const
{
	return m_left;
}

const BSP* BSP::GetRightChildren() const
{
	return m_right;
}

const BSP* BSP::GetParent() const
{
	return m_parent;
}

const std::vector<const BSP*> BSP::GetTree() const
{
	size_t i = 0;
	std::vector<const BSP*> bspNodes = {this};
	while (i < bspNodes.size())
	{
		const BSP* currNode = bspNodes[i++];
		const BSP* leftNode = currNode->m_left;
		if (leftNode) { bspNodes.push_back(leftNode); }
		const BSP* rightNode = currNode->m_right;
		if (rightNode) { bspNodes.push_back(rightNode); }
	}
	return bspNodes;
}

std::vector<BSP*> BSP::GetTree()
{
	size_t i = 0;
	std::vector<BSP*> bspNodes = { this };
	while (i < bspNodes.size())
	{
		BSP* currNode = bspNodes[i++];
		BSP* leftNode = currNode->m_left;
		if (leftNode) { bspNodes.push_back(leftNode); }
		BSP* rightNode = currNode->m_right;
		if (rightNode) { bspNodes.push_back(rightNode); }
	}
	return bspNodes;
}

std::vector<const BSP*> BSP::GetLeaves() const
{
	std::vector<const BSP*> ret;
	const std::vector<const BSP*> tree = GetTree();
	for (const BSP* bsp : tree)
	{
		if (bsp->IsBranch()) { continue; }
		ret.push_back(bsp);
	}
	return ret;
}

void BSP::SetQuadblockIndexes(const std::vector<size_t>& quadblockIndexes)
{
	m_quadblockIndexes = quadblockIndexes;
}

void BSP::SetParent(BSP* parent)
{
	m_parent = parent;
}

void BSP::Clear()
{
	std::vector<BSP*> vBSP = {this};
	size_t currIndex = 0;
	while (currIndex < vBSP.size())
	{
		BSP* pBSP = vBSP[currIndex++];
		if (pBSP->m_right) { vBSP.push_back(pBSP->m_right); }
		if (pBSP->m_left) { vBSP.push_back(pBSP->m_left); }
	}
	for (size_t i = 1; i < vBSP.size(); i++) { delete vBSP[i]; }
	m_right = nullptr;
	m_left = nullptr;
	m_axis = AxisSplit::NONE;
	m_flags = BSPFlags::NONE;
	m_quadblockIndexes.clear();
	//g_id = 1;
}

bool BSP::SplitLeafGeometry(const std::vector<Quadblock>& quadblocks, const AxisSplit axis, const float midpoint)
{
	// Split a leaf using axis and midpoint.
	// Do nothing if it's a branch, or the midpoint doesn't split into 2 non empty leaves.
	if (IsBranch()) { return false; } 

	std::vector<size_t> left_quad_indexes;
	std::vector<size_t> right_quad_indexes;
	for (size_t quad_index : m_quadblockIndexes)
	{
		const Quadblock& quad = quadblocks[quad_index];
		float centerValue = 0.0f;
		switch (axis)
		{
			case AxisSplit::X: centerValue = quad.GetCenter().x; break;
			case AxisSplit::Y: centerValue = quad.GetCenter().y; break;
			case AxisSplit::Z: centerValue = quad.GetCenter().z; break;
			default: return false; // Invalid axis
		}

		if (centerValue >= midpoint)
			left_quad_indexes.push_back(quad_index);
		else
			right_quad_indexes.push_back(quad_index);
	}

	if (left_quad_indexes.empty() || right_quad_indexes.empty()) { return false;}

	m_node = BSPNode::BRANCH;
	m_flags &= ~BSPFlags::LEAF;
	m_axis = axis;

	m_left = new BSP(BSPNode::LEAF, left_quad_indexes, this, quadblocks);
	m_left->m_bbox = ComputeBoundingBox(quadblocks, left_quad_indexes);
	
	m_right = new BSP(BSPNode::LEAF, right_quad_indexes, this, quadblocks);
	m_right->m_bbox = ComputeBoundingBox(quadblocks, right_quad_indexes);

	//printf("Split : %d -> %d + %d\n", m_quadblockIndexes.size(), left_quad_indexes.size(), right_quad_indexes.size());
	//float parent;
	//float left;
	//float right;
	//switch (axis)
	//{
	//case AxisSplit::X: parent = m_bbox.AxisLength().x; left = m_left->m_bbox.AxisLength().x; right = m_right->m_bbox.AxisLength().x;  break;
	//case AxisSplit::Y: parent = m_bbox.AxisLength().y; left = m_left->m_bbox.AxisLength().y; right = m_right->m_bbox.AxisLength().y;  break;
	//case AxisSplit::Z: parent = m_bbox.AxisLength().z; left = m_left->m_bbox.AxisLength().z; right = m_right->m_bbox.AxisLength().z;  break;
	//default: return false; // Invalid axis
	//}
	//printf("Split %.2f -> %.2f + %.2f\n\n", parent, left, right);

	return true;
}

bool BSP::SplitLeafMaterial(const std::vector<Quadblock>& quadblocks)
{
	//Split a leaf into a subtree, separating all quad by material.

	if (IsBranch()) { return false; }

	std::map<std::string, std::vector<size_t>> materialGroups;
	for (size_t idx : m_quadblockIndexes)
	{
		materialGroups[quadblocks[idx].GetMaterial()].push_back(idx);
	}

	std::vector<size_t> left_quad_indexes;
	std::vector<size_t> right_quad_indexes;

	int c = 0;
	for (auto& [mat, indexes] : materialGroups)
	{
		if (c%2)
			left_quad_indexes.insert(left_quad_indexes.end(), indexes.begin(), indexes.end());
		else
			right_quad_indexes.insert(right_quad_indexes.end(), indexes.begin(), indexes.end());
		c++;
	}
	if (left_quad_indexes.empty() || right_quad_indexes.empty()) 
		return false;

	m_node = BSPNode::BRANCH;
	m_flags &= ~BSPFlags::LEAF;
	m_axis = AxisSplit::NONE;

	m_left = new BSP(BSPNode::LEAF, left_quad_indexes, this, quadblocks);
	m_left->m_bbox = ComputeBoundingBox(quadblocks, left_quad_indexes);

	m_right = new BSP(BSPNode::LEAF, right_quad_indexes, this, quadblocks);
	m_right->m_bbox = ComputeBoundingBox(quadblocks, right_quad_indexes);

	m_right->SplitLeafMaterial(quadblocks);
	m_left->SplitLeafMaterial(quadblocks);

	return true;
}


void BSP::MergeBranch()
{
	// Merge all children from a branch into a single leaf
	if (!IsBranch()) { return; }
	std::vector<BSP*> toDel = { this };
	size_t currIndex = 0;
	while (currIndex < toDel.size())
	{
		BSP* pBSP = toDel[currIndex++];
		if (pBSP->m_left) { toDel.push_back(pBSP->m_left); }
		if (pBSP->m_right) { toDel.push_back(pBSP->m_right); }
	}
	for (size_t i = 1; i < toDel.size(); i++) { delete toDel[i]; }

	m_left = nullptr;
	m_right = nullptr;
	m_axis = AxisSplit::NONE;
	m_node = BSPNode::LEAF;
	m_flags |= BSPFlags::LEAF;
}


bool BSP::NeedSplitGeometry(const BSPTreeSettings settings, std::vector<AxisSplit>& allowedAxis)
{
	if (m_quadblockIndexes.size() <= 1) { return false; }
	if ((int)m_quadblockIndexes.size() > settings.maxQuadPerLeaf) 
	{ 
		allowedAxis.push_back(AxisSplit::X);
		allowedAxis.push_back(AxisSplit::Y);
		allowedAxis.push_back(AxisSplit::Z);
		return true; 
	}
	if (m_bbox.max.x - m_bbox.min.x > settings.maxAxisDistance) { allowedAxis.push_back(AxisSplit::X); }
	if (m_bbox.max.y - m_bbox.min.y > settings.maxAxisDistance) { allowedAxis.push_back(AxisSplit::Y); }
	if (m_bbox.max.z - m_bbox.min.z > settings.maxAxisDistance) { allowedAxis.push_back(AxisSplit::Z); }
	return !allowedAxis.empty();
}

float BSP::SplitScoreLeft(float scoreQuadWeight)
{  
	if (!IsBranch() || !m_left)
		return 0.0f;

	float quadCount = static_cast<float>(m_quadblockIndexes.size());
	float quadCountLeft = static_cast<float>(m_left->GetQuadblockIndexes().size());
	float quadScoreLeft = std::log2(quadCount / quadCountLeft);

	float bboxLength = m_bbox.AxisLength().Length();
	float bboxLengthLeft = m_left->GetBoundingBox().AxisLength().Length();
	bboxLengthLeft = std::max(bboxLengthLeft, EPSILON);
	float bboxScoreLeft = std::log2(bboxLength / bboxLengthLeft);

	return scoreQuadWeight * quadScoreLeft + (1 - scoreQuadWeight) * bboxScoreLeft;
}

float BSP::SplitScoreRight(float scoreQuadWeight)
{
	if (!IsBranch() || !m_right)
		return 0.0f;

	float quadCount = static_cast<float>(m_quadblockIndexes.size());
	float quadCountRight = static_cast<float>(m_right->GetQuadblockIndexes().size());
	float quadScoreRight = std::log2(quadCount / quadCountRight);

	float bboxLength = m_bbox.AxisLength().Length();
	float bboxLengthRight = m_right->GetBoundingBox().AxisLength().Length();
	bboxLengthRight = std::max(bboxLengthRight, EPSILON);
	float bboxScoreRight = std::log2(bboxLength / bboxLengthRight);

	return scoreQuadWeight * quadScoreRight + (1 - scoreQuadWeight) * bboxScoreRight;
}


float BSP::SplitScore(float scoreQuadWeight)
{
	// Rate how good is a split. Score of 0 is the worst possible split. Score of 1 is a great balanced split.
	if (!IsBranch() || !m_left || !m_right)
		return 0.0f;
	return std::min(SplitScoreLeft(scoreQuadWeight), SplitScoreRight(scoreQuadWeight));
}

float BSP::FindBestSplitCandidates(const std::vector<Quadblock>& quadblocks, AxisSplit& axis, std::vector<float>& candidates, float scoreQuadWeight, float& score)
{	// Reccursive helper for FindBestSplit, dichotomic search instead of linear. candidates must be sorted ascending, non empty
	// Return the float midpoint value. Modify score. If no split are found, score is -1, and midpoint irrelevant.

	if (candidates.empty())
	{
		score = -1.0f;
		return 0.0f;
	}
	if (candidates.size() == 1)
	{
		float candidate = candidates[0];
		if (SplitLeafGeometry(quadblocks, axis, candidate))
		{
			score = SplitScore(scoreQuadWeight);
			MergeBranch();
		}
		else
			score = -1.0f;
		return candidate;
	}
		
	size_t middleId = candidates.size() / 2;
	float candidate = candidates[middleId];
	if (SplitLeafGeometry(quadblocks, axis, candidate))
	{
		float leftScore = SplitScoreLeft(scoreQuadWeight);
		float rightScore = SplitScoreRight(scoreQuadWeight);
		MergeBranch();
		float minScore = std::min(leftScore, rightScore);

		float recursiveScore = -1.0f;
		float recursiveMidpoint = 0.0f;
		if (leftScore < rightScore)
		{
			std::vector<float> rightCandidates(candidates.begin() + middleId + 1, candidates.end());
			recursiveMidpoint = FindBestSplitCandidates(quadblocks, axis, rightCandidates, scoreQuadWeight, recursiveScore);
		}
		else
		{
			std::vector<float> leftCandidates(candidates.begin(), candidates.begin() + middleId);
			recursiveMidpoint = FindBestSplitCandidates(quadblocks, axis, leftCandidates, scoreQuadWeight, recursiveScore);
		}
		if (minScore >= recursiveScore)
		{
			score = minScore;
			return candidate;
		}
		else
		{
			score = recursiveScore;
			return recursiveMidpoint;
		}
	}
	std::vector<float> leftCandidates(candidates.begin(), candidates.begin() + middleId);
	std::vector<float> rightCandidates(candidates.begin() + middleId + 1, candidates.end());
	float leftBestScore, rightBestScore;
	float leftBestMidpoint = FindBestSplitCandidates(quadblocks, axis, leftCandidates, scoreQuadWeight, leftBestScore);
	float rightBestMidpoint = FindBestSplitCandidates(quadblocks, axis, rightCandidates, scoreQuadWeight, rightBestScore);
	if (leftBestScore >= rightBestScore)
	{
		score = leftBestScore;
		return leftBestMidpoint;
	}
	score = rightBestScore;
	return rightBestMidpoint;
}

bool BSP::FindBestSplit(const std::vector<Quadblock>& quadblocks, AxisSplit& outAxis, float& outMidpoint, BSPTreeSettings settings)
{	// Must return false when the node doesn't need split.
	
	float bestScore = std::numeric_limits<float>::lowest();
	float bestMidpoint = 0.0f;
	AxisSplit bestAxis = AxisSplit::NONE;

	bool hasTooMuchQuads = (int)m_quadblockIndexes.size() > settings.maxQuadPerLeaf;
	std::vector<AxisSplit> allowedAxis;
	if (m_bbox.max.x - m_bbox.min.x > settings.maxAxisDistance) { allowedAxis.push_back(AxisSplit::X); }
	if (m_bbox.max.y - m_bbox.min.y > settings.maxAxisDistance) { allowedAxis.push_back(AxisSplit::Y); }
	if (m_bbox.max.z - m_bbox.min.z > settings.maxAxisDistance) { allowedAxis.push_back(AxisSplit::Z); }

	float scoreQuadWeight = settings.scoreWeight;
	if (allowedAxis.empty() && hasTooMuchQuads) // Need geometry split because too much quad, but BBox constraint already matched
	{
		allowedAxis.push_back(AxisSplit::X);
		allowedAxis.push_back(AxisSplit::Y);
		allowedAxis.push_back(AxisSplit::Z);
		scoreQuadWeight = 0.9f; // 0.9f is better than 1.0f because 1.0f will generate atleast 3 score ties.
	}
	if (allowedAxis.empty() && !hasTooMuchQuads)
	{
		return false;
	}
	if (!allowedAxis.empty() && !hasTooMuchQuads) // Need spatial split, but quad constraint already matched
	{
		scoreQuadWeight = 0.0f;
	}

	for (AxisSplit axis : allowedAxis)
	{
		std::vector<float> centers;
		for (size_t idx : m_quadblockIndexes)
		{
			const Vec3& c = quadblocks[idx].GetCenter();
			switch (axis)
			{
			case AxisSplit::X: centers.push_back(c.x); break;
			case AxisSplit::Y: centers.push_back(c.y); break;
			case AxisSplit::Z: centers.push_back(c.z); break;
			default: break;
			}
		}
		std::sort(centers.begin(), centers.end());
		std::vector<float> candidates;
		for (size_t i = 0; i < m_quadblockIndexes.size() - 1; i++)
		{
			candidates.push_back((centers[i] + centers[i + 1]) / 2);
		}
			
		float score = 0.0f;
		float midPoint = FindBestSplitCandidates(quadblocks, axis, candidates, scoreQuadWeight, score);
		if (score > bestScore && score >= 0.0f)
		{
			bestAxis = axis;
			bestScore = score;
			bestMidpoint = midPoint;
		}
	}

	if (bestAxis != AxisSplit::NONE)
	{
		outMidpoint = bestMidpoint;
		outAxis = bestAxis;
		return true;
	}
	else
	{
		// Can happen when a quad is bigger than maxAxisLength, or when all center's of a leaf or in the same spot (overlapping quads)
		printf("BSP WARNING : Needed a split for Leaf %d (%d quads) but couldn't make a valid one\n", m_id, m_quadblockIndexes.size());
		return false;
	}
}


void BSP::GenerateTree(const std::vector<Quadblock>& quadblocks, const BSPTreeSettings settings)
{
	MergeBranch();
	if (m_quadblockIndexes.size() < 2)
		return;

	std::vector<BSP*> worklist = { this };
	while (!worklist.empty())
	{
		BSP* node = worklist.back();
		worklist.pop_back();

		AxisSplit axis; float midpoint;
		if (node->FindBestSplit(quadblocks, axis, midpoint, settings))
		{
			if (node->SplitLeafGeometry(quadblocks, axis, midpoint))
			{
				if (node->m_left) { worklist.push_back(node->m_left); }
				if (node->m_right) { worklist.push_back(node->m_right); }
				continue;
			}
			else
				printf("BSP TREE GENERATION MISTAKE : FiNDBESTSPLIT TRUE, SPLIT LEAF FALSE ?\n");
		}

		if (settings.separateMaterial)
		{
			node->SplitLeafMaterial(quadblocks);
		}
	}
}

void BSP::Generate(const std::vector<Quadblock>& quadblocks, const size_t maxQuadsPerLeaf, const float maxAxisLength)
{
	m_bbox = ComputeBoundingBox(quadblocks, m_quadblockIndexes);

	bool isLeaf = !IsBranch();
	if (isLeaf)
	{
		if (m_bbox.AxisLength() < maxAxisLength || m_quadblockIndexes.size() == 1) { return; }
		m_node = BSPNode::BRANCH;
		m_flags &= ~BSPFlags::LEAF;
	}

	if (m_quadblockIndexes.size() == 1)
	{
		std::vector<size_t> empty = {};
		GenerateOffspring(m_quadblockIndexes, empty, quadblocks, maxQuadsPerLeaf, maxAxisLength);
		return;
	}

	std::vector<size_t> x_right, x_left, y_right, y_left, z_right, z_left;
	float x_score = Split(x_left, x_right, AxisSplit::X, quadblocks);
	float y_score = Split(y_left, y_right, AxisSplit::Y, quadblocks);
	float z_score = Split(z_left, z_right, AxisSplit::Z, quadblocks);
	float bestScore = std::min(std::min(x_score, y_score), z_score);
	if (bestScore == std::numeric_limits<float>::max())
	{
		if (isLeaf)
		{
			m_node = BSPNode::LEAF;
			m_flags |= BSPFlags::LEAF;
		}
		return;
	}
	if (bestScore == x_score)
	{
		m_axis = AxisSplit::X;
		GenerateOffspring(x_left, x_right, quadblocks, maxQuadsPerLeaf, maxAxisLength);
	}
	else if (bestScore == z_score)
	{
		m_axis = AxisSplit::Z;
		GenerateOffspring(z_left, z_right, quadblocks, maxQuadsPerLeaf, maxAxisLength);
	}
	else
	{
		m_axis = AxisSplit::Y;
		GenerateOffspring(y_left, y_right, quadblocks, maxQuadsPerLeaf, maxAxisLength);
	}
}

std::vector<uint8_t> BSP::Serialize(size_t offQuads) const
{
	return m_node == BSPNode::BRANCH ? SerializeBranch() : SerializeLeaf(offQuads);
}

float BSP::GetAxisMidpoint(const AxisSplit axis) const
{
	float midpoint = 0.0f;
	switch (axis)
	{
	case AxisSplit::X: return m_bbox.Midpoint().x;
	case AxisSplit::Y: return m_bbox.Midpoint().y;
	case AxisSplit::Z: return m_bbox.Midpoint().z;
	}
	return midpoint;
}

BoundingBox BSP::ComputeBoundingBox(const std::vector<Quadblock>& quadblocks, const std::vector<size_t>& quadblockIndexes) const
{
	if (quadblockIndexes.empty()) { return {}; }

	Vec3 min = Vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	Vec3 max = Vec3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
	for (size_t index : quadblockIndexes)
	{
		const Quadblock& quad = quadblocks[index];
		const BoundingBox& quadBbox = quad.GetBoundingBox();
		min.x = std::min(min.x, quadBbox.min.x); max.x = std::max(max.x, quadBbox.max.x);
		min.y = std::min(min.y, quadBbox.min.y); max.y = std::max(max.y, quadBbox.max.y);
		min.z = std::min(min.z, quadBbox.min.z); max.z = std::max(max.z, quadBbox.max.z);
	}
	BoundingBox bbox = {min, max};
	return bbox;
}

float BSP::Split(std::vector<size_t>& left, std::vector<size_t>& right, const AxisSplit axis, const std::vector<Quadblock>& quadblocks) const
{
	float midpoint = GetAxisMidpoint(axis);
	for (size_t index : m_quadblockIndexes)
	{
		const Quadblock& quad = quadblocks[index];
		switch (axis)
		{
		case AxisSplit::X:
			if (quad.GetCenter().x < midpoint) { right.push_back(index); }
			else { left.push_back(index); }
			break;
		case AxisSplit::Y:
			if (quad.GetCenter().y < midpoint) { right.push_back(index); }
			else { left.push_back(index); }
			break;
		case AxisSplit::Z:
			if (quad.GetCenter().z < midpoint) { right.push_back(index); }
			else { left.push_back(index); }
			break;
		}
	}
	if (left.empty() || right.empty()) { return std::numeric_limits<float>::max(); }
	float perimeterLeft = ComputeBoundingBox(quadblocks, left).SemiPerimeter();
	float perimeterRight = ComputeBoundingBox(quadblocks, right).SemiPerimeter();
	return perimeterLeft + perimeterRight;
}

void BSP::GenerateOffspring(std::vector<size_t>& left, std::vector<size_t>& right, const std::vector<Quadblock>& quadblocks, const size_t maxQuadsPerLeaf, const float maxAxisLength)
{
	if (left.size() <= maxQuadsPerLeaf) { if (!left.empty()) { m_left = new BSP(BSPNode::LEAF, left, this, quadblocks); } }
	else { m_left = new BSP(BSPNode::BRANCH, left, this, quadblocks); }
	if (m_left) { m_left->Generate(quadblocks, maxQuadsPerLeaf, maxAxisLength); }

	if (right.size() <= maxQuadsPerLeaf) { if (!right.empty()) { m_right = new BSP(BSPNode::LEAF, right, this, quadblocks); } }
	else { m_right = new BSP(BSPNode::BRANCH, right, this, quadblocks); }
	if (m_right) { m_right->Generate(quadblocks, maxQuadsPerLeaf, maxAxisLength); }
}

std::vector<uint8_t> BSP::SerializeBranch() const
{
	PSX::BSPBranch branch = {};
	std::vector<uint8_t> buffer(sizeof(branch));
	branch.flag = m_flags;
	branch.id = static_cast<uint16_t>(m_id);
	branch.bbox.min = ConvertVec3(m_bbox.min, FP_ONE_GEO);
	branch.bbox.max = ConvertVec3(m_bbox.max, FP_ONE_GEO);
	branch.axis = {0, 0, 0};
	switch (m_axis)
	{
	case AxisSplit::X: branch.axis.x = 0x1000; break;
	case AxisSplit::Y: branch.axis.y = 0x1000; break;
	case AxisSplit::Z: branch.axis.z = 0x1000; break;
	}
	if (m_left)
	{
		branch.leftChild = static_cast<uint16_t>(m_left->m_id) + static_cast<uint16_t>(m_leftFlag);
		if (!m_left->IsBranch() && m_leftFlag == 0) { branch.leftChild |= BSPID::LEAF; }
	}
	else { branch.leftChild = BSPID::EMPTY; }
	if (m_right)
	{
		branch.rightChild = static_cast<uint16_t>(m_right->m_id) + static_cast<uint16_t>(m_rightFlag);
		if (!m_right->IsBranch() && m_rightFlag == 0) { branch.rightChild |= BSPID::LEAF; }
	}
	else { branch.rightChild = BSPID::EMPTY; }
	branch.unk1 = 0x00;
	switch (m_axis)
	{
	case AxisSplit::X: branch.unk1 = ConvertFloat((m_bbox.min.x + m_bbox.max.x) / 4, FP_ONE_GEO); break;
	case AxisSplit::Y: branch.unk1 = ConvertFloat((m_bbox.min.y + m_bbox.max.y) / 4, FP_ONE_GEO); break;
	case AxisSplit::Z: branch.unk1 = ConvertFloat((m_bbox.min.z + m_bbox.max.z) / 4, FP_ONE_GEO); break;
	}
	branch.unk2 = 0;
	branch.unk3 = 0;
	std::memcpy(buffer.data(), &branch, sizeof(branch));
	return buffer;
}

std::vector<uint8_t> BSP::SerializeLeaf(size_t offQuads) const
{
	PSX::BSPLeaf leaf = {};
	std::vector<uint8_t> buffer(sizeof(leaf));
	leaf.flag = m_flags;
	leaf.id = static_cast<uint16_t>(m_id);
	leaf.bbox.min = ConvertVec3(m_bbox.min, FP_ONE_GEO);
	leaf.bbox.max = ConvertVec3(m_bbox.max, FP_ONE_GEO);
	leaf.offHitbox = 0;
	leaf.numQuads = static_cast<uint32_t>(m_quadblockIndexes.size());
	leaf.offQuads = static_cast<uint32_t>(offQuads);
	leaf.unk1 = 0;
	std::memcpy(buffer.data(), &leaf, sizeof(leaf));
	return buffer;
}
