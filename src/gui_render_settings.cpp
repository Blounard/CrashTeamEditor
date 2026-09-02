#include "gui_render_settings.h"
#include <imgui.h>

float GuiRenderSettings::camFovDeg = 70.0f;
float GuiRenderSettings::camZoomMult = 1.0f;
float GuiRenderSettings::camRotateMult = 1.5f;
float GuiRenderSettings::camMoveMult = 2.5f;
float GuiRenderSettings::camSprintMult = 3.0f;
int GuiRenderSettings::camKeyForward = ImGuiKey_W;
int GuiRenderSettings::camKeyBack = ImGuiKey_S;
int GuiRenderSettings::camKeyLeft = ImGuiKey_A;
int GuiRenderSettings::camKeyRight = ImGuiKey_D;
int GuiRenderSettings::camKeyUp = ImGuiKey_E;
int GuiRenderSettings::camKeyDown = ImGuiKey_Q;
int GuiRenderSettings::camKeySprint = ImGuiKey_ModShift;
int GuiRenderSettings::camOrbitMouseButton = ImGuiMouseButton_Middle;
bool GuiRenderSettings::showLowLOD = false;
bool GuiRenderSettings::showWireframe = false;
bool GuiRenderSettings::showVerts = false;
bool GuiRenderSettings::showBackfaces = true;
bool GuiRenderSettings::showBspRectTree = false;
bool GuiRenderSettings::showLevel = true;
bool GuiRenderSettings::showCheckpoints = false;
bool GuiRenderSettings::showStartpoints = false;
bool GuiRenderSettings::showVisTree = false;
bool GuiRenderSettings::showBots = false;
bool GuiRenderSettings::filterActive = true;
bool GuiRenderSettings::showSelectedQuadblockInfo = true;
bool GuiRenderSettings::showMinimapBounds = false;
bool GuiRenderSettings::showSkybox = true;
bool GuiRenderSettings::showInstances = true;
Color GuiRenderSettings::defaultFilterColor = Color(static_cast<unsigned char>(255), static_cast<unsigned char>(128), static_cast<unsigned char>(0));
Color GuiRenderSettings::selectedCheckpointColor = Color(static_cast<unsigned char>(0), static_cast < unsigned char>(255), static_cast < unsigned char>(255));
int GuiRenderSettings::renderType = 0;
int GuiRenderSettings::bspTreeTopDepth = 0;
int GuiRenderSettings::bspTreeBottomDepth = 0;
int GuiRenderSettings::bspTreeMaxDepth = 0;
const std::vector<const char*> GuiRenderSettings::renderTypeLabels = { "Default", "Texture", "Vertex Color", "Normals"};

int MinimapSettings::textureHeight = 87;
int MinimapSettings::orientation = 4;
bool MinimapSettings::checkpointQuads = true;
bool MinimapSettings::checkpointPathableQuads = true;
std::set<std::string> MinimapSettings::materials = {};


float HotReloadSettings::relicSapphire = 60.0f;
float HotReloadSettings::relicGold = 60.0f;
float HotReloadSettings::relicPlatinum = 60.0f;
float HotReloadSettings::crystalTime = 60.0f;
bool HotReloadSettings::introCutscene = false;
bool HotReloadSettings::ghost = false;

int BSPTreeSettings::maxQuadPerLeaf = 32;
float BSPTreeSettings::maxAxisDistance = 64.0f;
bool BSPTreeSettings::separateMaterial = false;

bool VisTreeSettings::centerOnlySamples = true;
bool VisTreeSettings::commutativeRays = false;
bool VisTreeSettings::selfTargetNearClip = true;
float VisTreeSettings::nearClipDistance = -1.0f;
float VisTreeSettings::farClipDistance = 1000.0f;

bool BotPathSettings::useManualPath = false;
bool BotPathSettings::normalizeNodeDist = true;
float BotPathSettings::nodeDistance = 4.0f;
float BotPathSettings::sidewayOffset = 6.0f;
float BotPathSettings::negSnapDist = -6.0f;
float BotPathSettings::posSnapDist = 6.0f;
float BotPathSettings::ghostStart = 0.0f;
float BotPathSettings::ghostEnd = 60.0f;
Color BotPathSettings::pathColor[3] = {
	Color(0.86f, 0.31f, 0.31f),
	Color(0.31f, 0.78f, 0.31f),
	Color(0.31f, 0.51f, 0.86f)
};

float WaterAnimSettings::waveLength = 20.0f;
float WaterAnimSettings::sizeTex = 20.0f;
int WaterAnimSettings::ScrollULoops = 1;
int WaterAnimSettings::ScrollVLoops = 1;
int WaterAnimSettings::waveCyclesTimeU = 3;
int WaterAnimSettings::waveCyclesTimeV = 3;
float WaterAnimSettings::waveAmplitude = 16.0f;
float WaterAnimSettings::baseBrightness = 4.0f;
float WaterAnimSettings::brightAmp = 2.5f;
int WaterAnimSettings::brightWaveCycle = 1;

bool InstanceLoadPathSettings::normalize = true;
float InstanceLoadPathSettings::normalizeDist = 5.0f;
bool InstanceLoadPathSettings::groundSnap = true;
float InstanceLoadPathSettings::negSnapDist = -10.0f;
float InstanceLoadPathSettings::posSnapDist = 8.0f;
float InstanceLoadPathSettings::radius = 10.0f;
bool InstanceLoadPathSettings::loop = true;
bool InstanceLoadPathSettings::rolling = true;
