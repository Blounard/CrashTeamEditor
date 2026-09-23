#include "io.h"
#include "geo.h"
#include "utils.h"

#include <unordered_map>
#include <filesystem>

void to_json(nlohmann::json& json, const Vec3& v)
{
	json = {{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

void from_json(const nlohmann::json& json, Vec3& v)
{
	if (json.contains("x")) { v.x = json["x"]; }
	if (json.contains("y")) { v.y = json["y"]; }
	if (json.contains("z")) { v.z = json["z"]; }
}

void to_json(nlohmann::json& json, const BoundingBox& box)
{
	json = { {"min", box.min}, {"max", box.max} };
}
void from_json(const nlohmann::json& json, BoundingBox& box)
{
	if (json.contains("min")) { json.at("min").get_to(box.min); }
	if (json.contains("max")) { json.at("max").get_to(box.max); }
}

void to_json(nlohmann::json& json, const Color& c)
{
	json = {{"r", c.Red()}, {"g", c.Green()}, {"b", c.Blue()}, {"a", c.a}};
}

void from_json(const nlohmann::json& json, Color& c)
{
	float r = 0.0f;
	float g = 0.0f;
	float b = 0.0f;
	float a = 0.0f;
	if (json.contains("r")) { json.at("r").get_to(r); }
	if (json.contains("g")) { json.at("g").get_to(g); }
	if (json.contains("b")) { json.at("b").get_to(b); }
	if (json.contains("a"))
	{
		const nlohmann::json& alpha = json.at("a");
		if (alpha.is_boolean())
		{
			a = alpha.get<bool>() ? 1.0f : 0.0f;
		}
		else if (alpha.is_number_float()) { alpha.get_to(a); }
	}
	c = Color(r, g, b, a);
}

void to_json(nlohmann::json& json, const Spawn& spawn)
{
	json = {{"pos", spawn.pos}, {"rot", spawn.rot}};
}

void from_json(const nlohmann::json& json, Spawn& spawn)
{
	if (json.contains("pos")) { json.at("pos").get_to(spawn.pos); }
	if (json.contains("rot")) { json.at("rot").get_to(spawn.rot); }
}

void to_json(nlohmann::json& json, const ColorGradient& spawn)
{
	json = {{"posFrom", spawn.posFrom}, {"posTo", spawn.posTo}, {"colorFrom", spawn.colorFrom}, {"colorTo", spawn.colorTo}};
}

void from_json(const nlohmann::json& json, ColorGradient& spawn)
{
	if (json.contains("posFrom")) { json.at("posFrom").get_to(spawn.posFrom); }
	if (json.contains("posTo")) { json.at("posTo").get_to(spawn.posTo); }
	if (json.contains("colorFrom")) { json.at("colorFrom").get_to(spawn.colorFrom); }
	if (json.contains("colorTo")) { json.at("colorTo").get_to(spawn.colorTo); }
}

void to_json(nlohmann::json& json, const Stars& stars)
{
    json = {
        {"numStars", stars.numStars},
        {"spread", stars.spread},
        {"seed", stars.seed},
        {"depth", stars.zDepth}
    };
}

void from_json(const nlohmann::json& json, Stars& stars)
{
    if (json.contains("numStars")) { stars.numStars = json["numStars"]; }
    if (json.contains("spread")) { stars.spread = json["spread"]; }
    if (json.contains("seed")) { stars.seed = json["seed"]; }
    if (json.contains("depth")) { stars.zDepth = json["depth"]; }
}

void to_json(nlohmann::json& json, const Weather& weather)
{
	json = {
		{"velocity", weather.velocity},
		{"colorTop", weather.colorTop},
		{"colorBottom", weather.colorBottom},
		{"fillMode", weather.fillMode},
		{"OTindex", weather.OTindex}
	};
}

void from_json(const nlohmann::json& json, Weather& weather)
{
	if (json.contains("velocity")) { weather.velocity = json["velocity"]; }
	if (json.contains("colorTop")) { weather.colorTop = json["colorTop"]; }
	if (json.contains("colorBottom")) { weather.colorBottom = json["colorBottom"]; }
	if (json.contains("fillMode")) { weather.fillMode = json["fillMode"]; }
	if (json.contains("OTindex")) { weather.OTindex = json["OTindex"]; }
}

void to_json(nlohmann::json& json, const Minimap& minimap)
{
	json = {
		{"worldBox", minimap.worldBox},
		{"orientationMode", minimap.orientationMode},
		{"texturePath", minimap.texture.GetPath()},
	};
}

void from_json(const nlohmann::json& json, Minimap& minimap)
{
	if (json.contains("worldBox")) { json.at("worldBox").get_to(minimap.worldBox); }
	if (json.contains("orientationMode")) { json.at("orientationMode").get_to(minimap.orientationMode); }

	if (json.contains("texturePath"))
	{
		std::string path;
		json.at("texturePath").get_to(path);
		if (!path.empty()) { minimap.texture.UpdateTexture(path); }
	}
}

void to_json(nlohmann::json& json, const InstanceHitbox& hitbox)
{
	json["enabled"] = hitbox.enabled;
	json["preset"] = hitbox.preset;
	json["flags"] = hitbox.flags;
	json["halfExtent"] = hitbox.halfExtent;
	json["yOffset"] = hitbox.yOffset;
}

void to_json(nlohmann::json& json, const BotFlags& flags)
{
	json["turboPad"] = flags.turboPad;
	json["skidmarkFront"] = flags.skidmarkFront;
	json["skidmarkBack"] = flags.skidmarkBack;
	json["turboPadLow"] = flags.turboPadLow;
	json["maskGrabSTP"] = flags.maskGrabSTP;
	json["jump"] = flags.jump;
	json["driftLeft"] = flags.driftLeft;
	json["driftRight"] = flags.driftRight;
	json["echo"] = flags.echo;
	json["midAir"] = flags.midAir;
	json["sink"] = flags.sink;
	json["lowGrav"] = flags.lowGrav;
}

void from_json(const nlohmann::json& json, BotFlags& flags)
{
	if (json.contains("turboPad")) { json.at("turboPad").get_to(flags.turboPad); }
	if (json.contains("skidmarkFront")) { json.at("skidmarkFront").get_to(flags.skidmarkFront); }
	if (json.contains("skidmarkBack")) { json.at("skidmarkBack").get_to(flags.skidmarkBack); }
	if (json.contains("turboPadLow")) { json.at("turboPadLow").get_to(flags.turboPadLow); }
	if (json.contains("maskGrabSTP")) { json.at("maskGrabSTP").get_to(flags.maskGrabSTP); }
	if (json.contains("jump")) { json.at("jump").get_to(flags.jump); }
	if (json.contains("driftLeft")) { json.at("driftLeft").get_to(flags.driftLeft); }
	if (json.contains("driftRight")) { json.at("driftRight").get_to(flags.driftRight); }
	if (json.contains("echo")) { json.at("echo").get_to(flags.echo); }
	if (json.contains("midAir")) { json.at("midAir").get_to(flags.midAir); }
	if (json.contains("sink")) { json.at("sink").get_to(flags.sink); }
	if (json.contains("lowGrav")) { json.at("lowGrav").get_to(flags.lowGrav); }
}

void from_json(const nlohmann::json& json, InstanceHitbox& hitbox)
{
	if (json.contains("enabled")) { json.at("enabled").get_to(hitbox.enabled); }
	if (json.contains("preset")) { json.at("preset").get_to(hitbox.preset); }
	if (json.contains("flags")) { json.at("flags").get_to(hitbox.flags); }
	if (json.contains("halfExtent")) { json.at("halfExtent").get_to(hitbox.halfExtent); }
	if (json.contains("yOffset")) { json.at("yOffset").get_to(hitbox.yOffset); }
}

void ReadBinaryFile(std::vector<uint8_t>& v, const std::filesystem::path& path)
{
	std::ifstream file(path, std::ios::binary);
	file.seekg(0, std::ios::end);
	size_t size = file.tellg();
	v.resize(size);
	file.seekg(0, std::ios::beg);
	file.read(reinterpret_cast<char*>(v.data()), size);
	file.close();
}

void Path::ToJson(nlohmann::json& json, const std::vector<Quadblock>& quadblocks) const
{
	json = {{"index", m_index}, {"hasLeft", m_left != nullptr}, {"hasRight", m_right != nullptr}};

	std::vector<std::string> quadStart, quadEnd, quadIgnore;
	for (size_t index : m_quadIndexesStart) { quadStart.push_back(quadblocks[index].GetName()); }
	for (size_t index : m_quadIndexesEnd) { quadEnd.push_back(quadblocks[index].GetName()); }
	for (size_t index : m_quadIndexesIgnore) { quadIgnore.push_back(quadblocks[index].GetName()); }
	json["quadStart"] = quadStart;
	json["quadEnd"] = quadEnd;
	json["quadIgnore"] = quadIgnore;
	json["color"] = m_color;

	if (m_left) { json["left"] = nlohmann::json(); m_left->ToJson(json["left"], quadblocks); }
	if (m_right) { json["right"] = nlohmann::json(); m_right->ToJson(json["right"], quadblocks); }
}

void Path::FromJson(const nlohmann::json& json, const std::vector<Quadblock>& quadblocks)
{
	std::vector<std::string> quadStart, quadEnd, quadIgnore;
	if (json.contains("color")) { json.at("color").get_to(m_color); }
	if (json.contains("index")) { json.at("index").get_to(m_index); }
	if (json.contains("hasLeft") && json.contains("left"))
	{
		bool hasLeft = false;
		json.at("hasLeft").get_to(hasLeft);
		if (hasLeft)
		{
			m_left = new Path();
			m_left->FromJson(json.at("left"), quadblocks);
		}
	}
	if (json.contains("hasRight") && json.contains("right"))
	{
		bool hasRight = false;
		json.at("hasRight").get_to(hasRight);
		if (hasRight)
		{
			m_right = new Path();
			m_right->FromJson(json.at("right"), quadblocks);
		}
	}
	if (json.contains("quadStart")) { json.at("quadStart").get_to(quadStart); }
	if (json.contains("quadEnd")) { json.at("quadEnd").get_to(quadEnd); }
	if (json.contains("quadIgnore")) { json.at("quadIgnore").get_to(quadIgnore); }

	std::unordered_map<std::string, size_t> quadNameMap;
	for (size_t i = 0; i < quadblocks.size(); i++) { quadNameMap[quadblocks[i].GetName()] = i; }
	for (const std::string& name : quadStart) { if (quadNameMap.contains(name)) { m_quadIndexesStart.push_back(quadNameMap[name]); } }
	for (const std::string& name : quadEnd) { if (quadNameMap.contains(name)) { m_quadIndexesEnd.push_back(quadNameMap[name]); } }
	for (const std::string& name : quadIgnore) { if (quadNameMap.contains(name)) { m_quadIndexesIgnore.push_back(quadNameMap[name]); } }
}

void AnimTexture::FromJson(const nlohmann::json& json, std::vector<Quadblock>& quadblocks, const std::filesystem::path& parentPath)
{
	if (!json.contains("path")) { return; }
	std::filesystem::path path = json["path"];
	if (!std::filesystem::exists(path))
	{
		path = parentPath / path.filename();
		if (!std::filesystem::exists(path)) { return; }
	}
	if (!ReadAnimation(path)) { ClearAnimation(); return; }

	m_path = path;
	if (json.contains("name")) { m_name = json["name"]; }
	if (json.contains("startAt")) { m_startAtFrame = json["startAt"]; }
	if (json.contains("duration")) { m_duration = json["duration"]; }
	if (json.contains("rotation")) { m_rotation = json["rotation"]; }
	if (json.contains("horMirror")) { m_horMirror = json["horMirror"]; }
	if (json.contains("verMirror")) { m_verMirror = json["verMirror"]; }

	if (m_horMirror) { MirrorFrames(true); }
	if (m_verMirror) { MirrorFrames(false); }
	RotateFrames(m_rotation);

	if (json.contains("blendModes"))
	{
		std::vector<uint16_t> blendModes = json["blendModes"];
		if (blendModes.size() == m_textures.size())
		{
			for (size_t i = 0; i < m_textures.size(); i++) { m_textures[i].SetBlendMode(blendModes[i]); }
		}
	}
	if (json.contains("quads"))
	{
		std::unordered_set<std::string> quadNames = json["quads"];
		for (size_t i = 0; i < quadblocks.size(); i++)
		{
			if (quadNames.contains(quadblocks[i].GetName()))
			{
				m_quadblockIndexes.push_back(i);
				quadblocks[i].SetAnimated(true);
			}
		}
	}
}

void AnimTexture::ToJson(nlohmann::json& json, const std::vector<Quadblock>& quadblocks) const
{
	json["path"] = m_path;
	json["name"] = m_name;
	json["startAt"] = m_startAtFrame;
	json["duration"] = m_duration;
	json["rotation"] = m_rotation;
	json["horMirror"] = m_horMirror;
	json["verMirror"] = m_verMirror;

	std::unordered_set<std::string> quadNames;
	for (size_t index : m_quadblockIndexes) { quadNames.insert(quadblocks[index].GetName()); }
	json["quads"] = quadNames;

	std::vector<uint16_t> blendModes;
	for (const Texture& tex : m_textures) { blendModes.push_back(tex.GetBlendMode()); }
	json["blendModes"] = blendModes;
}

void Quadblock::ToJsonMetadata(nlohmann::json& json) const
{
	json["checkpointPathable"] = m_checkpointPathable;
	json["checkpointStatus"] = m_checkpointStatus;
	json["visTreeTransparent"] = m_visTreeTransparent;
	json["drawOrderHigh"] = m_drawOrderHigh;
	json["checkpointIndex"] = m_checkpointIndex;
	json["doubleSided"] = m_doubleSided;
	json["flags"] = m_flags;
	json["terrain"] = m_terrain;
	json["water"] = m_water;
	json["downforce"] = m_downforce;
	json["weatherIntensity"] = m_weatherIntensity;
	json["weatherVanishRate"] = m_weatherVanishRate;
}

void Quadblock::FromJsonMetadata(const nlohmann::json& json)
{
	if (json.contains("checkpointPathable")) { json.at("checkpointPathable").get_to(m_checkpointPathable); }
	if (json.contains("checkpointStatus")) { json.at("checkpointStatus").get_to(m_checkpointStatus); }
	if (json.contains("visTreeTransparent")) { json.at("visTreeTransparent").get_to(m_visTreeTransparent); }
	if (json.contains("drawOrderHigh")) { json.at("drawOrderHigh").get_to(m_drawOrderHigh); }
	if (json.contains("checkpointIndex")) { json.at("checkpointIndex").get_to(m_checkpointIndex); }
	if (json.contains("doubleSided")) { json.at("doubleSided").get_to(m_doubleSided); }
	if (json.contains("flags")) { json.at("flags").get_to(m_flags); }
	if (json.contains("terrain")) { json.at("terrain").get_to(m_terrain); }
	if (json.contains("water")) { json.at("water").get_to(m_water); }
	if (json.contains("downforce")) { json.at("downforce").get_to(m_downforce); }
	if (json.contains("weatherIntensity")) { json.at("weatherIntensity").get_to(m_weatherIntensity); }
	if (json.contains("weatherVanishRate")) { json.at("weatherVanishRate").get_to(m_weatherVanishRate); }
}

void Quadblock::ToJsonGeometry(nlohmann::json& json) const
{
	json["bspID"] = m_bspID;
	json["bbox"] = m_bbox;
	json["hasRawNormalData"] = m_hasRawNormalData;
	if (m_hasRawNormalData)
	{
		json["triNormalVecBitshift"] = m_triNormalVecBitshift;
		json["triNormalVecDividend"] = std::vector<int16_t>(m_triNormalVecDividend, m_triNormalVecDividend + 10);
	}
}

void Quadblock::FromJsonGeometry(const nlohmann::json& json)
{
	if (json.contains("bspID")) { json.at("bspID").get_to(m_bspID); }
	if (json.contains("bbox")) { json.at("bbox").get_to(m_bbox); }
	if (json.contains("hasRawNormalData")) { json.at("hasRawNormalData").get_to(m_hasRawNormalData); }
	if (m_hasRawNormalData)
	{
		if (json.contains("triNormalVecBitshift")) { json.at("triNormalVecBitshift").get_to(m_triNormalVecBitshift); }
		if (json.contains("triNormalVecDividend"))
		{
			const std::vector<int16_t> dividend = json.at("triNormalVecDividend").get<std::vector<int16_t>>();
			for (size_t i = 0; i < dividend.size() && i < 10; i++) { m_triNormalVecDividend[i] = dividend[i]; }
		}
	}
}

void Checkpoint::ToJson(nlohmann::json& json) const
{
	json["pos"] = m_pos;
	json["distToFinish"] = m_distToFinish;
	json["up"] = m_up;
	json["down"] = m_down;
	json["left"] = m_left;
	json["right"] = m_right;
}

void Checkpoint::FromJson(const nlohmann::json& json)
{
	if (json.contains("pos")) { json.at("pos").get_to(m_pos); }
	if (json.contains("distToFinish")) { json.at("distToFinish").get_to(m_distToFinish); }
	if (json.contains("up")) { json.at("up").get_to(m_up); }
	if (json.contains("down")) { json.at("down").get_to(m_down); }
	if (json.contains("left")) { json.at("left").get_to(m_left); }
	if (json.contains("right")) { json.at("right").get_to(m_right); }
}

void Instance::ToJson(nlohmann::json& json) const
{
	json["name"] = m_name;
	json["scale"] = m_scale;
	json["pos"] = m_pos;
	json["rot"] = m_rot;
	json["modelID"] = static_cast<int16_t>(m_modelID);
	json["color"] = m_color;
	json["modelKey"] = m_modelKey;
	json["flags"] = m_flags;
	json["hitbox"] = m_hitbox;
}

void Instance::FromJson(const nlohmann::json& json)
{
	if (json.contains("name")) { json.at("name").get_to(m_name); }
	if (json.contains("scale")) { json.at("scale").get_to(m_scale); }
	if (json.contains("pos")) { json.at("pos").get_to(m_pos); }
	if (json.contains("rot")) { json.at("rot").get_to(m_rot); }
	if (json.contains("modelID")) { m_modelID = static_cast<ModelId>(json.at("modelID").get<int16_t>()); }
	if (json.contains("color")) { json.at("color").get_to(m_color); }
	if (json.contains("flags")) { json.at("flags").get_to(m_flags); }
	if (json.contains("hitbox")) { json.at("hitbox").get_to(m_hitbox); }
}

void BSP::ToJson(nlohmann::json& json) const
{
	json["id"] = m_id;
	json["node"] = static_cast<int>(m_node);
	json["axis"] = static_cast<int>(m_axis);
	json["splitPoint"] = m_splitPoint;
	json["flags"] = m_flags;
	json["bbox"] = m_bbox;

	if (m_left)
	{ 
		json["left"] = nlohmann::json(); 
		m_left->ToJson(json["left"]);
	}
	if (m_right)
	{ 
		json["right"] = nlohmann::json(); 
		m_right->ToJson(json["right"]);
	}
}

void BSP::FromJson(const nlohmann::json& json)
{
	if (json.contains("id")) { json.at("id").get_to(m_id); }
	if (json.contains("node")) { m_node = static_cast<BSPNode>(json.at("node").get<int>()); }
	if (json.contains("axis")) { m_axis = static_cast<AxisSplit>(json.at("axis").get<int>()); }
	if (json.contains("splitPoint")) { json.at("splitPoint").get_to(m_splitPoint); }
	if (json.contains("flags")) { json.at("flags").get_to(m_flags); }
	if (json.contains("bbox")) { json.at("bbox").get_to(m_bbox); }

	if (json.contains("left"))
	{
		m_left = new BSP();
		m_left->FromJson(json.at("left"));
		m_left->SetParent(this);
	}
	if (json.contains("right"))
	{
		m_right = new BSP();
		m_right->FromJson(json.at("right"));
		m_right->SetParent(this);
	}
}

void BitMatrix::ToJson(nlohmann::json& json) const
{
	json["width"] = m_width;
	json["height"] = m_height;
	json["data"] = m_data;
}

void BitMatrix::FromJson(const nlohmann::json& json)
{
	if (json.contains("width")) { json.at("width").get_to(m_width); }
	if (json.contains("height")) { json.at("height").get_to(m_height); }
	m_data.assign(m_width * m_height, 0);

	if (json.contains("data"))
	{
		std::vector<uint8_t> data = json.at("data").get<std::vector<uint8_t>>();
		if (data.size() == m_data.size()) { m_data = data; }
	}
}

void BotNode::ToJson(nlohmann::json& json) const
{
	json["pos"] = m_pos;
	json["rot"] = m_rot;
	json["flags"] = m_flags;
	json["specialBits"] = static_cast<int>(m_specialBits);
	json["splitLineID"] = m_splitLineID;
	json["ramPhysID"] = m_ramPhysID;
	json["shadow"] = m_shadow;
	json["terrain"] = m_terrain;
	json["pathChange"] = m_pathChange;
	json["pathChangeIndex"] = m_pathChangeIndex;
	json["checkpoint"] = m_checkpoint;
}

void BotNode::FromJson(const nlohmann::json& json)
{
	if (json.contains("pos")) { json.at("pos").get_to(m_pos); }
	if (json.contains("rot")) { json.at("rot").get_to(m_rot); }
	if (json.contains("flags")) { json.at("flags").get_to(m_flags); }
	if (json.contains("specialBits")) { m_specialBits = static_cast<BotSpecialBits>(json.at("specialBits").get<int>()); }
	if (json.contains("splitLineID")) { json.at("splitLineID").get_to(m_splitLineID); }
	if (json.contains("ramPhysID")) { json.at("ramPhysID").get_to(m_ramPhysID); }
	if (json.contains("shadow")) { json.at("shadow").get_to(m_shadow); }
	if (json.contains("terrain")) { json.at("terrain").get_to(m_terrain); }
	if (json.contains("pathChange")) { json.at("pathChange").get_to(m_pathChange); }
	if (json.contains("pathChangeIndex")) { json.at("pathChangeIndex").get_to(m_pathChangeIndex); }
	if (json.contains("checkpoint")) { json.at("checkpoint").get_to(m_checkpoint); }
}

void BotPath::ToJson(nlohmann::json& json) const
{
	json = nlohmann::json::array();
	for (const BotNode& node : m_nodes)
	{
		nlohmann::json nodeJson = nlohmann::json();
		node.ToJson(nodeJson);
		json.push_back(nodeJson);
	}
}

void BotPath::FromJson(const nlohmann::json& json)
{
	m_nodes.clear();
	for (const nlohmann::json& nodeJson : json)
	{
		BotNode node;
		node.FromJson(nodeJson);
		m_nodes.push_back(node);
	}
}
