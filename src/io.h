#pragma once

#include "lev.h"
#include "path.h"
#include "quadblock.h"
#include "animtexture.h"
#include "minimap.h"
#include "checkpoint.h"
#include "instance.h"
#include "bsp.h"
#include "vistree.h"


#include <nlohmann/json.hpp>

#include <fstream>
#include <vector>
#include <cstdint>

void to_json(nlohmann::json& json, const Vec3& v);
void from_json(const nlohmann::json& json, Vec3& v);

void to_json(nlohmann::json& json, const BoundingBox& bbox);
void from_json(const nlohmann::json& json, BoundingBox& bbox);

void to_json(nlohmann::json& json, const Color& c);
void from_json(const nlohmann::json& json, Color& c);

void to_json(nlohmann::json& json, const Spawn& spawn);
void from_json(const nlohmann::json& json, Spawn& spawn);

void to_json(nlohmann::json& json, const ColorGradient& spawn);
void from_json(const nlohmann::json& json, ColorGradient& spawn);

void to_json(nlohmann::json& json, const Stars& stars);
void from_json(const nlohmann::json& json, Stars& stars);

void to_json(nlohmann::json& json, const Weather& weather);
void from_json(const nlohmann::json& json, Weather& weather);

void to_json(nlohmann::json& json, const Minimap& minimap);
void from_json(const nlohmann::json& json, Minimap& minimap);

void to_json(nlohmann::json& json, const InstanceHitbox& hitbox);
void from_json(const nlohmann::json& json, InstanceHitbox& hitbox);

void ReadBinaryFile(std::vector<uint8_t>& v, const std::filesystem::path& path);

template<typename T> static inline void Read(std::ifstream& file, T& data)
{
	if (file.rdstate() == std::ios_base::goodbit)
	{
		file.read(reinterpret_cast<char*>(&data), sizeof(data));
		if (file.rdstate() != std::ios_base::goodbit)
		{
			printf("Error : File not in correct state\n");
		}
	}
}

template<typename T> static inline void Write(std::ofstream& file, T* data, size_t size)
{
	file.write(reinterpret_cast<const char*>(data), size);
}