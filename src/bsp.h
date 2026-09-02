#pragma once

#include "geo.h"
#include "quadblock.h"
#include "settings.h"


#include <vector>



enum class BSPNode
{
	BRANCH,
	LEAF
};

enum class AxisSplit
{
	NONE, X, Y, Z
};
static const char* AxisSplitNames[] = { "NONE", "X", "Y", "Z" };
inline float ProjectionAxis(const Vec3& vec, const AxisSplit& axis)
{
	switch (axis)
	{
	case AxisSplit::X: return vec.x;
	case AxisSplit::Y: return vec.y;
	case AxisSplit::Z: return vec.z;
	default:           return 0.0f;
	}
}

struct BSPFlags
{
	static constexpr uint16_t NONE = 0;
	static constexpr uint16_t LEAF = 1 << 0;
	static constexpr uint16_t WATER = 1 << 1;
	static constexpr uint16_t SUBDIV_4_1 = 1 << 3;
	static constexpr uint16_t SUBDIV_4_2 = 1 << 4;
	static constexpr uint16_t SUBDIV_DYNAMIC = 1 << 5;
	static constexpr uint16_t INVISIBLE = 1 << 6; // ??
	static constexpr uint16_t NO_COLLISION = 1 << 7;
};

struct BSPID
{
	static constexpr uint16_t ID_MASK = 0x3FFF;
	static constexpr uint16_t LEAF = 0x4000;
	static constexpr uint16_t INVISIBLE = 0x8000;
	static constexpr uint16_t EMPTY = 0xFFFF;
};

class BSP
{
public:
	BSP();
	BSP(BSPNode type, const std::vector<size_t>& quadblockIndexes, BSP* parent, const std::vector<Quadblock>& quadblocks);

	void PopulateLeaf(PSX::BSPLeaf& leaf, std::vector<BSP*>& bspArray, std::vector<Quadblock>& quadblocks, uint32_t offQuadblocks, size_t global_id);

	~BSP();

	void PopulateBranch(PSX::BSPBranch& branch, std::vector<BSP*>& bspArray, size_t global_id);
	void PopulateBranchQuadIndexes();
	size_t GetId() const;
	void SetId(size_t id);
	bool IsEmpty() const;
	bool IsValid() const;
	bool IsBranch() const;
	uint16_t GetFlags() const;
	const std::string& GetType() const;
	const std::string& GetAxis() const;
	const BoundingBox& GetBoundingBox() const;
	const std::vector<size_t>& GetQuadblockIndexes() const;
	const std::vector<size_t>& GetInstanceIndexes() const;
	std::vector<size_t>& GetInstanceIndexes();
	const BSP* GetLeftChildren() const;
	const BSP* GetRightChildren() const;
	const BSP* GetParent() const;
	const std::vector<const BSP*> GetTree() const;
	std::vector<BSP*> GetTree();
	std::vector<const BSP*> GetLeaves() const;
	void SetQuadblockIndexes(const std::vector<size_t>& quadblockIndexes, std::vector<Quadblock>& quadblocks);
	void SetInstanceIndexes(const std::vector<size_t>& instanceIndexes);
	void ComputeBoundingBox(const std::vector<Quadblock>& quadblocks);
	void UpdateBoundingBox(const BoundingBox& bbox);
	void SetParent(BSP* parent);
	void Clear();
	bool SplitLeafGeometry(const std::vector<Quadblock>& quadblocks, const AxisSplit axis, const float midpoint);
	bool SplitLeafMaterial(const std::vector<Quadblock>& quadblocks);
	bool SplitLeafWater(const std::vector<Quadblock>& quadblocks);
	void MergeBranch();
	bool FindBestSplit(const std::vector<Quadblock>& quadblocks, AxisSplit& outAxis, float& outMidpoint);
	void Generate(const std::vector<Quadblock>& quadblocks);
	std::vector<uint8_t> Serialize(size_t offQuads, const std::vector<Quadblock>& quadblocks) const;
	void RenderUI(const std::vector<Quadblock>& quadblocks);

	uint32_t GetOffHitbox() const { return m_offHitbox; }

private:

	bool IsInvisible(const std::vector<Quadblock>& quadblocks);
	bool HasWater(const std::vector<Quadblock>& quadblocks) const;

	std::vector<uint8_t> SerializeBranch(const std::vector<Quadblock>& quadblocks) const;
	std::vector<uint8_t> SerializeLeaf(size_t offQuads, const std::vector<Quadblock>& quadblocks) const;



private:
	size_t m_id;
	size_t m_idFlag;
	BSPNode m_node;
	AxisSplit m_axis;
	float m_splitPoint;
	uint16_t m_flags;
	BSP* m_left;
	BSP* m_right;
	BSP* m_parent;
	BoundingBox m_bbox;
	std::vector<size_t> m_quadblockIndexes;
	std::vector<size_t> m_instanceIndexes;
	uint32_t m_offHitbox = 0;
};
void ResetAllBSPID();