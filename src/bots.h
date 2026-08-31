#pragma once

#include "geo.h"
#include "psx_types.h"
#include "quadblock.h"
#include "instance.h"
#include <filesystem>
#include <string>
#include <cstdint>
#include <vector>


static constexpr uint16_t BOT_PATH_MAGIC = 0xECFD;
struct PSXBotNodeFlags
{
    static constexpr uint16_t NONE = 0;
    static constexpr uint16_t TURBO_PAD_HIGH = 1 << 0;
    static constexpr uint16_t SKIDMARKS_FRONT = 1 << 1;
    static constexpr uint16_t SKIDMARKS_BACK = 1 << 2;
    static constexpr uint16_t TERRAIN_MASK = 0x00F8;
    static constexpr uint16_t TURBO_PAD_LOW = 1 << 8;
    static constexpr uint16_t MASK_GRAB_STP = 1 << 9;
    static constexpr uint16_t JUMP = 1 << 10;
    static constexpr uint16_t DRIFT_LEFT = 1 << 11;
    static constexpr uint16_t DRIFT_RIGHT = 1 << 12;
    static constexpr uint16_t ENGINE_ECHO = 1 << 13;
    static constexpr uint16_t MID_AIR = 1 << 14;
    static constexpr uint16_t SINK_KART = 1 << 15;
};

struct PSXBotNodeFlags2
{
    static constexpr uint16_t NONE = 0;
    static constexpr uint16_t SPECIAL_MASK = 0xF; // if USE_RAMPHYS : id of RamPhys ; if REFLECTION : id of splitline (0 or 1) ; else : transparency
    static constexpr uint16_t USE_RAMPHYS = 1 << 4;
    static constexpr uint16_t USE_REFLECTION = 1 << 5;
    static constexpr uint16_t INSTANCE_COLL = 1 << 6;
    static constexpr uint16_t MOON_GRAV = 1 << 7;
};

struct BotPathSettings
{
    bool  useManualPath = false;
    bool  normalizeNodeDist = true;
    float nodeDistance = 4.0f;
    float sidewayOffset = 6.0f;
};

enum class BotSpecialBits : int
{
    RAM_PHYS = 0,
    REFLECTION = 1,
    TRANSPARENCY = 2,
};

struct BotFlags
{
    bool turboPad = false;
    bool skidmarkFront = false;
    bool skidmarkBack = false;
    bool turboPadLow = false;
    bool maskGrabSTP = false;
    bool jump = false;
    bool driftLeft = false;
    bool driftRight = false;
    bool echo = false;
    bool midAir = false;
    bool sink = false;
    bool lowGrav = false;
};

class BotNode
{
public:

    BotNode() = default;
    BotNode(const PSX::NavFrame& frame);
  
    std::vector<uint8_t> Serialize(const Vec3& nextPos, std::vector<Instance>& instances) const;
    void RenderUI(int index, bool& deleteRequested);
    
    const Vec3& GetPos() const { return m_pos; }
    void SetPos(const Vec3& pos) { m_pos = pos; }

    const Vec3& GetRot() const { return m_rot; }
    void SetRot(const Vec3& rot) { m_rot = rot; }

    const BotFlags& GetFlags() const { return m_flags; }
    void SetFlags(const BotFlags& flags) { m_flags = flags; }

    uint8_t GetTerrain() const { return m_terrain; }
    void SetTerrain(uint8_t v) { m_terrain = v; }

    uint8_t GetCheckpoint() const { return m_checkpoint; }
    void SetCheckpoint(uint8_t v) { m_checkpoint = v; }

    int GetPathChange() const { return m_pathChange; }
    void SetPathChange(int v) { m_pathChange = v; }

    int GetPathChangeIndex() const { return m_pathChangeIndex; }
    void SetPathChangeIndex(int v) { m_pathChangeIndex = v; }

private:
    Vec3 m_pos = {};
    Vec3 m_rot = {};

    BotFlags m_flags = {};
    BotSpecialBits m_specialBits = BotSpecialBits::TRANSPARENCY;
    int m_splitLineID = 0;
    int m_ramPhysID = 0;
    int m_transparency = 0;

    uint8_t m_terrain = TerrainType::ASPHALT;
    int m_pathChange = 0;
    int m_pathChangeIndex = 0;
    uint8_t  m_checkpoint = 0;
};

class BotPath
{
public:
    BotPath() = default;

    BotPath(const PSX::NavHeader& header, const std::vector<PSX::NavFrame>& frames);
    void Clear();
    bool IsValid();
    bool LoadFromOBJ(const std::filesystem::path& path, std::vector<Quadblock>& quadblocks);
    bool GeneratePath(std::vector<Vec3>& nodesPos, std::vector<Quadblock>& quadblocks);

    std::vector<uint8_t> Serialize(std::vector<Instance>& instances) const;
    void RenderUI(int pathIndex);

    const std::vector<BotNode>& GetNodes() const { return m_nodes; }
    std::vector<BotNode>& GetNodes() { return m_nodes; }

    size_t GetNodeCount() const { return m_nodes.size(); }

    const BotNode& GetNode(size_t index) const { return m_nodes.at(index); }
    BotNode& GetNode(size_t index) { return m_nodes.at(index); }

    void AddNode(const BotNode& node) { m_nodes.push_back(node); }
    void InsertNode(size_t index, const BotNode& node) { m_nodes.insert(m_nodes.begin() + index, node); }
    void RemoveNode(size_t index) { m_nodes.erase(m_nodes.begin() + index); }

    uint16_t GetPhysUnk(size_t i) const { return m_physUnk[i]; }
    void     SetPhysUnk(size_t i, uint16_t v) { m_physUnk[i] = v; }

private:
    std::vector<BotNode> m_nodes;
    uint16_t m_physUnk[0x20] = {};
};

std::vector<Vec3> GenerateLateralPath(const std::vector<BotNode>& nodes, float lateralOffset, std::vector<Quadblock>& quadblocks);