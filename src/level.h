#pragma once

#include "vertex.h"
#include "quadblock.h"
#include "checkpoint.h"
#include "lev.h"
#include "bsp.h"
#include "path.h"
#include "material.h"
#include "texture.h"
#include "renderer.h"
#include "animtexture.h"
#include "model.h"
#include "vistree.h"
#include "minimap.h"
#include "skybox.h"
#include "bots.h"
#include "instance.h"
#include "settings.h"

#include <nlohmann/json.hpp>
#include <vector>
#include <array>
#include <unordered_map>
#include <map>
#include <filesystem>
#include <tuple>
#include <cstdint>

static constexpr size_t REND_NO_SELECTED_QUADBLOCK = std::numeric_limits<size_t>::max();

namespace LevelModels
{
	static constexpr size_t LEVEL = 0;
	static constexpr size_t BSP = 1;
	static constexpr size_t SPAWN = 2;
	static constexpr size_t CHECKPOINT = 3;
	static constexpr size_t SELECTED = 4;
	static constexpr size_t MULTI_SELECTED = 5;
	static constexpr size_t FILTER = 6;
	static constexpr size_t SKYBOX = 7;
	static constexpr size_t BOT = 8;
	static constexpr size_t INSTANCES = 9;
	static constexpr size_t MINIMAP_BOUNDS = 10;
	static constexpr size_t COUNT = 11;
};

struct HostSettings // RAW STRUCT TO EMIT FOR HOT RELOAD SETTINGS
{
	int32_t magic;          // HOST_SETTINGS_MAGIC once the editor has written here
	int32_t sequence;       // bumped by the editor on every push
	int32_t relicSapphire;  // ms
	int32_t relicGold;      // ms
	int32_t relicPlatinum;  // ms
	int32_t crystalTime;    // ms
	int32_t introCutscene;  // 1 plays the intro cam, 0 skips it
	int32_t ghost;          // 1 leaves the ghost replay alone, 0 kills its thread
};



class Level
{
public:
	bool Load(const std::filesystem::path& filename, bool isLevel);
	bool Save(const std::filesystem::path& path);
	bool SaveLEV(const std::filesystem::path& path, bool useRawTexture);
	bool SaveOBJ(const std::filesystem::path& objFile);
	bool IsLoaded() const;
	void Clear(bool clearErrors);
	//bool ImportModel(const std::filesystem::path& ctrmodelPath);
	const std::string& GetName() const;
	std::vector<Quadblock>& GetQuadblocks();
	BSP& GetBSP();
	BitMatrix& GetVisTree();
	std::vector<Checkpoint>& GetCheckpoints();
	std::vector<Path>& GetCheckpointPaths();
	std::vector<BotNode>& GetBotPath(int i);
	const std::filesystem::path& GetParentPath() const;
	std::vector<std::string> GetMaterialNames() const;
	std::vector<size_t> GetMaterialQuadblockIndexes(const std::string& material) const;
	std::tuple<std::vector<Quadblock*>, Vec3> GetRendererSelectedData();
	Model* GetLevelModel();
	Model* GetBspModel();
	Model* GetSpawnModel();
	Model* GetCheckpointModel();
	Model* GetBotModel();
	Model* GetSelectedModel();
	Model* GetMultiSelectedModel();
	Model* GetFilterModel();
	Model* GetInstancesModel();
	bool LoadPreset(const std::filesystem::path& filename);
	bool SavePreset(const std::filesystem::path& path);
	void ResetFilter();
	void ResetRendererSelection();
	void UpdateRenderCheckpointData();
	void UpdateRenderBotData();
	void GenerateRenderLevData();
	bool GenerateVisTreeOnly();
	void GenerateBotPathLeft();
	bool HasRawTexture() const { return m_hasRawTexture; }

private:
	void ManageTurbopad(Quadblock& quadblock);
	bool LoadLEV(const std::filesystem::path& levFile);
	bool LoadOBJ(const std::filesystem::path& objFile, bool isLevel);


	bool StartEmuIPC(const std::string& emulator);
	bool HotReload(const std::string& levPath, const std::string& vrmPath, const std::string& emulator);
	bool SaveGhostData(const std::string& emulator, const std::filesystem::path& path);
	bool SetGhostData(const std::filesystem::path& path, bool tropy);
	bool UpdateVRM();
	std::vector<uint16_t> ReadRawVRAM(std::filesystem::path vrmPath);
	bool EmplaceInstanceBSP();
	void GenerateBotPathChangeCode();
	bool GenerateSpawn(float colSpacing, float rowSpacing, float centerOffset);
	bool GenerateInstanceRow(int checkpointIndex, size_t instanceIndex, int numInstances, float spacing, bool deleteAfter);
	bool QueryGround(const Vec3& pos, float& height, Vec3& normal) const;
	std::string GenerateUniqueInstanceName(const std::string& name) const;
	bool GenerateCheckpoints();
	bool GenerateBSP();
	bool ReOrderBSP();
	bool GenerateOceanVertices();
	bool GenerateMinimap();
	void OpenHotReloadWindow();
	void RenderUI(Renderer& renderer);
	void InitModels(Renderer& renderer);
	void UpdateAnimationRenderData();
	void UpdateFilterRenderData(const Quadblock& qb);
	void GenerateRenderBspData();
	void GenerateRenderInstanceData();
	void GenerateRenderStartpointData();
	void GenerateRenderMinimapBoundsData();
	void GenerateRenderSkyboxData();
	void GenerateRenderSelectedBlockData(const Quadblock& quadblock, const Vec3& queryPoint);
	bool UpdateAnimTextures(float deltaTime);
	void ViewportClickHandleBlockSelection(int pixelX, int pixelY, bool appendSelection, const Renderer& rend);

	friend class UI;

private:
	bool m_saveScript;
	bool m_showLogWindow;
	bool m_showHotReloadWindow;
	bool m_loaded;

	std::vector<std::tuple<std::string, std::string>> m_invalidQuadblocks;
	std::string m_logMessage;
	std::string m_name;

	std::filesystem::path m_parentPath;
	std::filesystem::path m_hotReloadLevPath;
	std::filesystem::path m_hotReloadVRMPath;

	std::array<Spawn, NUM_DRIVERS> m_spawn;
	uint32_t m_configFlags;
	std::array<ColorGradient, NUM_GRADIENT> m_skyGradient;
	Color m_clearColor;
	Weather m_weather;
	Stars m_stars;
	float m_splitLines[2];
	int m_jumpYSpeedCap;
	std::vector<uint8_t> m_tropyGhost;
	std::vector<uint8_t> m_oxideGhost;
	std::vector<Quadblock> m_quadblocks;
	std::vector<Checkpoint> m_checkpoints;
	BSP m_bsp;
	std::vector<Path> m_checkpointPaths;
	std::string m_pythonScript = "print('CrashTeamEditor Python console ready!')\nprint('Level:', m_lev.name)";
	std::string m_pythonConsole;
	std::vector<AnimTexture> m_animTextures;
	BitMatrix m_bspVis;
	std::vector<uint8_t> m_vrm;
	Minimap m_minimap;
	Skybox m_skybox;
	BotPath m_botPaths[3];
	Texture m_envMapTex;

	bool m_hasRawTexture;
	std::unordered_map<uint32_t, PSX::TextureGroup> m_rawTextureGroup;
	std::unordered_map<LayoutKey, std::string> m_materialCache; // Layout Key -> matName
	std::unordered_map<LayoutKey, PixelBounds> m_textureToPixelBounds; // Map Layout key -> Pixels bounds of the texture.
	std::unordered_map<uint32_t, PSX::AnimTex> m_rawAnimTex;
	std::unordered_map<uint32_t, std::vector<uint32_t>> m_rawAnimTexFrames;
	PSX::TextureLayout m_rawWaterLayout;
	//std::vector<PSX::Vertex> m_rawWaterVertices;
	//std::vector<PSX::OceanVertex> m_rawWaterColors;
	std::map<std::string, std::vector<std::pair<size_t, size_t>>> m_materialToQuadFaces; // MaterialName -> List of (QuadId, FaceId)
	std::unordered_map<std::string, Texture> m_materialToTexture;
	MaterialProperty<std::string, MaterialType::TERRAIN> m_propTerrain;
	MaterialProperty<uint16_t, MaterialType::QUAD_FLAGS> m_propQuadFlags;
	MaterialProperty<bool, MaterialType::DRAW_FLAGS> m_propDoubleSided;
	MaterialProperty<bool, MaterialType::CHECKPOINT> m_propCheckpoints;
	MaterialProperty<QuadblockTrigger, MaterialType::TURBO_PAD> m_propTurboPads;
	MaterialProperty<int, MaterialType::SPEED_IMPACT> m_propSpeedImpact;
	MaterialProperty<int, MaterialType::WEATHER_INTENSITY> m_propWeatherIntensity;
	MaterialProperty<int, MaterialType::WEATHER_VANISH_RATE> m_propWeatherVanishRate;
	MaterialProperty<bool, MaterialType::CHECKPOINT_PATHABLE> m_propCheckpointPathable;
	MaterialProperty<bool, MaterialType::VISTREE_TRANSPARENT> m_propVisTreeTransparent;
	MaterialProperty<int, MaterialType::DRAW_ORDER_HIGH> m_propDrawOrderHigh;
	MaterialProperty<bool, MaterialType::WATER> m_propWater;

	std::array<Model*, LevelModels::COUNT> m_models;

	Vec3 m_rendererQueryPoint;
	std::vector<size_t> m_rendererSelectedQuadblockIndexes;
	size_t m_lastAnimTextureCount;

	

	// VRAM data parsed from .vrm file (for model texture extraction)
	std::vector<uint16_t> m_vramData;

	std::unordered_map<size_t, InstanceModel> m_instanceModels; //Not using vector, so model can be deleted without affecting instance's model key.
	std::vector<Instance> m_instances;
	std::vector<std::vector<Vec3>> m_spawntypes;
	std::vector<std::vector<Spawn>> m_spawntypesPosRot;
	int m_openInstanceIndex = -1;
	int m_closeInstanceIndex = -1;
};
