#pragma once

#include "geo.h"
#include "psx_types.h"

#include <cstdint>

struct VertexFlags
{
	static constexpr uint16_t NONE = 0;
};

class Vertex
{
public:
	Vertex();
	Vertex(const Point& point);
	Vertex(const PSX::Vertex& vertex);
	void RenderUI(size_t index, bool& editedPos);
	std::vector<uint8_t> Serialize() const;
	Color GetColor(bool high) const;
	inline bool operator==(const Vertex& v) const {
		PSX::Vec3 pos1 = ConvertVec3(m_pos, FP_ONE_GEO);
		PSX::Vec3 pos2 = ConvertVec3(v.m_pos, FP_ONE_GEO);
		return (pos1.x == pos2.x && pos1.y == pos2.y && pos1.z == pos2.z) &&
			(m_flags == v.m_flags) &&
			(m_colorHigh == v.m_colorHigh) &&
			(m_colorLow == v.m_colorLow);
	}

public:
	Vec3 m_pos;
	Vec3 m_normal;

private:
	uint16_t m_flags;
	Color m_colorHigh;
	Color m_colorLow;

	friend std::hash<Vertex>;
};

template<>
struct std::hash<Vertex>
{
	inline std::size_t operator()(const Vertex& key) const
	{
		PSX::Vec3 pos = ConvertVec3(key.m_pos, FP_ONE_GEO);
		size_t pos_hash = std::hash<int16_t>()(pos.x) ^
			(std::hash<int16_t>()(pos.y) << 1) ^
			(std::hash<int16_t>()(pos.z) << 2);
		return ((((pos_hash ^ (std::hash<uint16_t>()(key.m_flags) << 1)) >> 1) ^
			(std::hash<Color>()(key.m_colorHigh) << 1)) >> 2) ^
			(std::hash<Color>()(key.m_colorLow) << 2);
	}
};