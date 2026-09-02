#pragma once

#include "geo.h"

#include <set>
#include <vector>

struct GuiRenderSettings
{
	// note: updating this also requires updating all shader source.
  enum RenderType
	{
    Default,
    Texture,
    VertexColor,
    Normals,
  };

  static int renderType, bspTreeTopDepth, bspTreeBottomDepth, bspTreeMaxDepth;
  static float camFovDeg, camZoomMult, camRotateMult, camMoveMult, camSprintMult;
  static int camKeyForward, camKeyBack, camKeyLeft, camKeyRight, camKeyUp, camKeyDown, camKeySprint;
  static int camOrbitMouseButton;
  static bool showLowLOD, showWireframe, showVerts, showBackfaces, showBspRectTree, showLevel, showCheckpoints, showStartpoints, showVisTree, filterActive, showSelectedQuadblockInfo, showSkybox, showBots, showInstances, showMinimapBounds;
  static Color defaultFilterColor, selectedCheckpointColor;
  static const std::vector<const char*> renderTypeLabels;
};

struct MinimapSettings
{
	static int textureHeight;
	static int orientation;
	static bool checkpointQuads;
	static bool checkpointPathableQuads;
	static std::set<std::string> materials;
};

struct HotReloadSettings	// HOT RELOAD PARAMETERS
{
	static float relicSapphire;	// seconds
	static float relicGold;		// seconds
	static float relicPlatinum;	// seconds
	static float crystalTime;		// seconds
	static bool introCutscene;		// 1 plays the intro cam, 0 skips it
	static bool ghost;				// 1 leaves the ghost replay alone, 0 kills its thread
};

struct BSPTreeSettings
{
	static int maxQuadPerLeaf;
	static float maxAxisDistance;
	static bool separateMaterial;
};

struct VisTreeSettings
{
	static bool centerOnlySamples;
	static bool commutativeRays;
	static bool selfTargetNearClip;
	static float nearClipDistance;
	static float farClipDistance;
};

struct BotPathSettings
{
	static bool useManualPath;
	static bool normalizeNodeDist;
	static float nodeDistance;
	static float sidewayOffset;
	static float negSnapDist; // MUST BE NEGATIVE
	static float posSnapDist; // MUST BE POSITIVE
	static float ghostStart;
	static float ghostEnd;
	static Color pathColor[3];
};

struct WaterAnimSettings
{
	static float waveLength;	// world units
	// For BaseUV (static tex part)
	static float sizeTex;		// Size of the full texture in world units
	// For ScrollUV (UV scrolling like a conveyer belt)
	static int ScrollULoops;	// Speed
	static int ScrollVLoops;
	// For WaveUV (small perturbation)
	static int waveCyclesTimeU;	// Speed
	static int waveCyclesTimeV;
	static float waveAmplitude;	// Size of the UV perturbation (in pixels)
	// For brightness:
	static float baseBrightness;	// Base
	static float brightAmp;		// Amplitude
	static int brightWaveCycle;	// Speed
};

struct InstanceLoadPathSettings
{
	static bool normalize;
	static float normalizeDist;
	static bool groundSnap;
	static float negSnapDist; // MUST BE NEGATIVE
	static float posSnapDist; // MUST BE POSITIVE
	static float radius; // Only for Pos+Rot
	static bool loop;
	static bool rolling;
};