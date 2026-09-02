#include "app.h"
#include "ui.h"
#include "gui_render_settings.h"
#include "io.h"

#include "globalimguiglglfw.h"
#include "renderer.h"

#include <nlohmann/json.hpp>
#include <fstream>

bool App::Init()
{
	bool success = true;
	InitUISettings();
	success &= InitGLFW();
	success &= InitImGui();
	return  success;
}

void App::Run()
{
	UI ui;
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableKeyboard;
	ImVec4 clearColor = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	while (!glfwWindowShouldClose(m_window))
	{
		glfwPollEvents();
		if (glfwGetWindowAttrib(m_window, GLFW_ICONIFIED) != 0)
		{
			ImGui_ImplGlfw_Sleep(10);
			continue;
		}

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		GL_CHECK(glClearColor(clearColor.x * clearColor.w, clearColor.y * clearColor.w, clearColor.z * clearColor.w, clearColor.w));
		GL_CHECK(glClear(GL_COLOR_BUFFER_BIT));
		ImGui::NewFrame();
		GL_CHECK(glViewport(0, 0, static_cast<int>(io.DisplaySize.x), static_cast<int>(io.DisplaySize.y)));
		glfwGetWindowSize(m_window, &Settings::w_width, &Settings::w_height);
		ui.Render();
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(m_window);
	}
}

void App::Close()
{
	SaveUISettings(false);
	CloseImGui();
}

static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

bool App::InitGLFW()
{
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit()) { return false; }

	// Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
		// GL ES 2.0 + GLSL 100 (WebGL 1.0)
	m_glslVer = "#version 100";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(IMGUI_IMPL_OPENGL_ES3)
		// GL ES 3.0 + GLSL 300 es (WebGL 2.0)
	m_glslVer = "#version 300 es";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
		// GL 3.2 + GLSL 150
	m_glslVer = "#version 150";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
		// GL 3.0 + GLSL 130
	m_glslVer = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

	// Create window with graphics context
	const std::string title = "Crash Team Editor " + m_version;
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE); //resizeable window
	glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE); //high dpi
	m_window = glfwCreateWindow(Settings::w_width, Settings::w_height, title.c_str(), nullptr, nullptr);
	if (m_window == nullptr) { return false; }
	glfwSetWindowSize(m_window, Settings::w_width, Settings::w_height);
	glfwSetWindowPos(m_window, Settings::w_x, Settings::w_y);
	if (Settings::w_maximized) { glfwMaximizeWindow(m_window); }
	glfwMakeContextCurrent(m_window);
	glfwSwapInterval(1); // Enable vsync

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		fprintf(stderr, "Failed to initialize OpenGL loader!");
		return false;
	}
	return true;
}

bool App::InitImGui()
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	(void)io; //supress "variable not used" warnings, idk, the template code does this.
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

	ImGui::StyleColorsDark();

	bool success = true;
	success &= ImGui_ImplGlfw_InitForOpenGL(m_window, true);
#ifdef __EMSCRIPTEN__
	success &= ImGui_ImplGlfw_InstallEmscriptenCallbacks(window, "#canvas");
#endif
	success &= ImGui_ImplOpenGL3_Init(m_glslVer.c_str());
	ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	return success;
}

void App::InitUISettings()
{
	if (!std::filesystem::exists(m_configFile))
	{
		SaveUISettings(true);
		return;
	}
	nlohmann::json json = nlohmann::json::parse(std::ifstream(m_configFile));
	if (json.contains("Width")) { Settings::w_width = json["Width"]; }
	if (json.contains("Height")) { Settings::w_height = json["Height"]; }
	if (json.contains("WindowX")) { Settings::w_x = json["WindowX"]; }
	if (json.contains("WindowY")) { Settings::w_y = json["WindowY"]; }
	if (json.contains("Maximized")) { Settings::w_maximized = json["Maximized"]; }
	if (json.contains("AnimTex")) { Settings::w_animtex = json["AnimTex"]; }
	if (json.contains("BSP")) { Settings::w_bsp = json["BSP"]; }
	if (json.contains("Checkpoints")) { Settings::w_checkpoints = json["Checkpoints"]; }
	if (json.contains("Ghost")) { Settings::w_ghost = json["Ghost"]; }
	if (json.contains("Level")) { Settings::w_level = json["Level"]; }
	if (json.contains("Material")) { Settings::w_material = json["Material"]; }
	if (json.contains("Quadblocks")) { Settings::w_quadblocks = json["Quadblocks"]; }
	if (json.contains("Renderer")) { Settings::w_renderer = json["Renderer"]; }
	if (json.contains("Spawn")) { Settings::w_spawn = json["Spawn"]; }
	if (json.contains("LastOpenedFolder")) { Settings::m_lastOpenedFolder = json["LastOpenedFolder"]; }
	if (json.contains("LastOpenedScriptFolder")) { Settings::m_lastOpenedScriptFolder = json["LastOpenedScriptFolder"]; }
	if (json.contains("LastOpenedModelFolder")) { Settings::m_lastOpenedModelFolder = json["LastOpenedModelFolder"]; }
	if (json.contains("Script")) { Settings::w_python = json["Script"]; }
	if (json.contains("Instances")) { Settings::w_modelImporter = json["Instances"]; }
	if (json.contains("Bot")) { Settings::w_bot = json["Bot"]; }
	if (json.contains("CameraBindings"))
	{
		const nlohmann::json& bindings = json["CameraBindings"];
		if (bindings.contains("Forward")) { GuiRenderSettings::camKeyForward = bindings["Forward"]; }
		if (bindings.contains("Back")) { GuiRenderSettings::camKeyBack = bindings["Back"]; }
		if (bindings.contains("Left")) { GuiRenderSettings::camKeyLeft = bindings["Left"]; }
		if (bindings.contains("Right")) { GuiRenderSettings::camKeyRight = bindings["Right"]; }
		if (bindings.contains("Up")) { GuiRenderSettings::camKeyUp = bindings["Up"]; }
		if (bindings.contains("Down")) { GuiRenderSettings::camKeyDown = bindings["Down"]; }
		if (bindings.contains("Sprint")) { GuiRenderSettings::camKeySprint = bindings["Sprint"]; }
		if (bindings.contains("OrbitMouseButton")) { GuiRenderSettings::camOrbitMouseButton = bindings["OrbitMouseButton"]; }
	}
	if (json.contains("CameraSettings"))
	{
		const nlohmann::json& camSettings = json["CameraSettings"];
		if (camSettings.contains("FovDeg")) { GuiRenderSettings::camFovDeg = camSettings["FovDeg"]; }
		if (camSettings.contains("ZoomMult")) { GuiRenderSettings::camZoomMult = camSettings["ZoomMult"]; }
		if (camSettings.contains("RotateMult")) { GuiRenderSettings::camRotateMult = camSettings["RotateMult"]; }
		if (camSettings.contains("MoveMult")) { GuiRenderSettings::camMoveMult = camSettings["MoveMult"]; }
		if (camSettings.contains("SprintMult")) { GuiRenderSettings::camSprintMult = camSettings["SprintMult"]; }
	}
	if (json.contains("RendererSettings"))
	{
		const nlohmann::json& renderSettings = json["RendererSettings"];
		if (renderSettings.contains("FilterActive")) { GuiRenderSettings::filterActive = renderSettings["FilterActive"]; }
		if (renderSettings.contains("FilterColor"))
		{
			const nlohmann::json& color = renderSettings["FilterColor"];
			if (color.is_array() && color.size() == 3)
			{
				GuiRenderSettings::defaultFilterColor = Color(static_cast<unsigned char>(color[0]), static_cast<unsigned char>(color[1]), static_cast<unsigned char>(color[2]));
			}
		}
		if (renderSettings.contains("RenderType")) { GuiRenderSettings::renderType = renderSettings["RenderType"]; }
		if (renderSettings.contains("ShowLowLOD")) { GuiRenderSettings::showLowLOD = renderSettings["ShowLowLOD"]; }
		if (renderSettings.contains("ShowWireframe")) { GuiRenderSettings::showWireframe = renderSettings["ShowWireframe"]; }
		if (renderSettings.contains("ShowVerts")) { GuiRenderSettings::showVerts = renderSettings["ShowVerts"]; }
		if (renderSettings.contains("ShowBackfaces")) { GuiRenderSettings::showBackfaces = renderSettings["ShowBackfaces"]; }
		if (renderSettings.contains("ShowBspRectTree")) { GuiRenderSettings::showBspRectTree = renderSettings["ShowBspRectTree"]; }
		if (renderSettings.contains("ShowLevel")) { GuiRenderSettings::showLevel = renderSettings["ShowLevel"]; }
		if (renderSettings.contains("ShowCheckpoints")) { GuiRenderSettings::showCheckpoints = renderSettings["ShowCheckpoints"]; }
		if (renderSettings.contains("ShowStartpoints")) { GuiRenderSettings::showStartpoints = renderSettings["ShowStartpoints"]; }
		if (renderSettings.contains("ShowVisTree")) { GuiRenderSettings::showVisTree = renderSettings["ShowVisTree"]; }
		if (renderSettings.contains("ShowSelectedQuadblockInfo")) { GuiRenderSettings::showSelectedQuadblockInfo = renderSettings["ShowSelectedQuadblockInfo"]; }
		if (renderSettings.contains("ShowSkybox")) { GuiRenderSettings::showSkybox = renderSettings["ShowSkybox"]; }
		if (renderSettings.contains("ShowBots")) { GuiRenderSettings::showBots = renderSettings["ShowBots"]; }
		if (renderSettings.contains("ShowInstances")) { GuiRenderSettings::showInstances = renderSettings["ShowInstances"]; }
		if (renderSettings.contains("ShowMinimapBounds")) { GuiRenderSettings::showMinimapBounds = renderSettings["ShowMinimapBounds"]; }
		if (renderSettings.contains("SelectedCheckpointColor"))
		{
			const nlohmann::json& color = renderSettings["SelectedCheckpointColor"];
			if (color.is_array() && color.size() == 3)
			{
				GuiRenderSettings::selectedCheckpointColor = Color(static_cast<unsigned char>(color[0]), static_cast<unsigned char>(color[1]), static_cast<unsigned char>(color[2]));
			}
		}
	}
	if (json.contains("MinimapSettings"))
	{
		const nlohmann::json& minimapSettings = json["MinimapSettings"];
		if (minimapSettings.contains("TextureHeight")) { MinimapSettings::textureHeight = minimapSettings["TextureHeight"]; }
		if (minimapSettings.contains("Orientation")) { MinimapSettings::orientation = minimapSettings["Orientation"]; }
		if (minimapSettings.contains("CheckpointQuads")) { MinimapSettings::checkpointQuads = minimapSettings["CheckpointQuads"]; }
		if (minimapSettings.contains("CheckpointPathableQuads")) { MinimapSettings::checkpointPathableQuads = minimapSettings["CheckpointPathableQuads"]; }
		// materials intentionally not loaded
	}
	if (json.contains("HotReloadSettings"))
	{
		const nlohmann::json& hotReloadSettings = json["HotReloadSettings"];
		if (hotReloadSettings.contains("RelicSapphire")) { HotReloadSettings::relicSapphire = hotReloadSettings["RelicSapphire"]; }
		if (hotReloadSettings.contains("RelicGold")) { HotReloadSettings::relicGold = hotReloadSettings["RelicGold"]; }
		if (hotReloadSettings.contains("RelicPlatinum")) { HotReloadSettings::relicPlatinum = hotReloadSettings["RelicPlatinum"]; }
		if (hotReloadSettings.contains("CrystalTime")) { HotReloadSettings::crystalTime = hotReloadSettings["CrystalTime"]; }
		if (hotReloadSettings.contains("IntroCutscene")) { HotReloadSettings::introCutscene = hotReloadSettings["IntroCutscene"]; }
		if (hotReloadSettings.contains("Ghost")) { HotReloadSettings::ghost = hotReloadSettings["Ghost"]; }
	}
	if (json.contains("BSPTreeSettings"))
	{
		const nlohmann::json& bspTreeSettings = json["BSPTreeSettings"];
		if (bspTreeSettings.contains("MaxQuadPerLeaf")) { BSPTreeSettings::maxQuadPerLeaf = bspTreeSettings["MaxQuadPerLeaf"]; }
		if (bspTreeSettings.contains("MaxAxisDistance")) { BSPTreeSettings::maxAxisDistance = bspTreeSettings["MaxAxisDistance"]; }
		if (bspTreeSettings.contains("SeparateMaterial")) { BSPTreeSettings::separateMaterial = bspTreeSettings["SeparateMaterial"]; }
	}
	if (json.contains("VisTreeSettings"))
	{
		const nlohmann::json& visTreeSettings = json["VisTreeSettings"];
		if (visTreeSettings.contains("CenterOnlySamples")) { VisTreeSettings::centerOnlySamples = visTreeSettings["CenterOnlySamples"]; }
		if (visTreeSettings.contains("CommutativeRays")) { VisTreeSettings::commutativeRays = visTreeSettings["CommutativeRays"]; }
		if (visTreeSettings.contains("SelfTargetNearClip")) { VisTreeSettings::selfTargetNearClip = visTreeSettings["SelfTargetNearClip"]; }
		if (visTreeSettings.contains("NearClipDistance")) { VisTreeSettings::nearClipDistance = visTreeSettings["NearClipDistance"]; }
		if (visTreeSettings.contains("FarClipDistance")) { VisTreeSettings::farClipDistance = visTreeSettings["FarClipDistance"]; }
	}
	if (json.contains("BotPathSettings"))
	{
		const nlohmann::json& botPathSettings = json["BotPathSettings"];
		if (botPathSettings.contains("UseManualPath")) { BotPathSettings::useManualPath = botPathSettings["UseManualPath"]; }
		if (botPathSettings.contains("NormalizeNodeDist")) { BotPathSettings::normalizeNodeDist = botPathSettings["NormalizeNodeDist"]; }
		if (botPathSettings.contains("NodeDistance")) { BotPathSettings::nodeDistance = botPathSettings["NodeDistance"]; }
		if (botPathSettings.contains("SidewayOffset")) { BotPathSettings::sidewayOffset = botPathSettings["SidewayOffset"]; }
		if (botPathSettings.contains("NegSnapDist")) { BotPathSettings::negSnapDist = botPathSettings["NegSnapDist"]; }
		if (botPathSettings.contains("PosSnapDist")) { BotPathSettings::posSnapDist = botPathSettings["PosSnapDist"]; }
		if (botPathSettings.contains("GhostStart")) { BotPathSettings::ghostStart = botPathSettings["GhostStart"]; }
		if (botPathSettings.contains("GhostEnd")) { BotPathSettings::ghostEnd = botPathSettings["GhostEnd"]; }
		if (botPathSettings.contains("PathColor"))
		{
			const nlohmann::json& colors = botPathSettings["PathColor"];
			if (colors.is_array())
			{
				for (size_t i = 0; i < colors.size() && i < 3; ++i)
				{
					colors[i].get_to(BotPathSettings::pathColor[i]);
				}
			}
		}
	}
	if (json.contains("WaterAnimSettings"))
	{
		const nlohmann::json& waterAnimSettings = json["WaterAnimSettings"];
		if (waterAnimSettings.contains("WaveLength")) { WaterAnimSettings::waveLength = waterAnimSettings["WaveLength"]; }
		if (waterAnimSettings.contains("SizeTex")) { WaterAnimSettings::sizeTex = waterAnimSettings["SizeTex"]; }
		if (waterAnimSettings.contains("ScrollULoops")) { WaterAnimSettings::ScrollULoops = waterAnimSettings["ScrollULoops"]; }
		if (waterAnimSettings.contains("ScrollVLoops")) { WaterAnimSettings::ScrollVLoops = waterAnimSettings["ScrollVLoops"]; }
		if (waterAnimSettings.contains("WaveCyclesTimeU")) { WaterAnimSettings::waveCyclesTimeU = waterAnimSettings["WaveCyclesTimeU"]; }
		if (waterAnimSettings.contains("WaveCyclesTimeV")) { WaterAnimSettings::waveCyclesTimeV = waterAnimSettings["WaveCyclesTimeV"]; }
		if (waterAnimSettings.contains("WaveAmplitude")) { WaterAnimSettings::waveAmplitude = waterAnimSettings["WaveAmplitude"]; }
		if (waterAnimSettings.contains("BaseBrightness")) { WaterAnimSettings::baseBrightness = waterAnimSettings["BaseBrightness"]; }
		if (waterAnimSettings.contains("BrightAmp")) { WaterAnimSettings::brightAmp = waterAnimSettings["BrightAmp"]; }
		if (waterAnimSettings.contains("BrightWaveCycle")) { WaterAnimSettings::brightWaveCycle = waterAnimSettings["BrightWaveCycle"]; }
	}
	if (json.contains("InstanceLoadPathSettings"))
	{
		const nlohmann::json& instanceLoadPathSettings = json["InstanceLoadPathSettings"];
		if (instanceLoadPathSettings.contains("Normalize")) { InstanceLoadPathSettings::normalize = instanceLoadPathSettings["Normalize"]; }
		if (instanceLoadPathSettings.contains("NormalizeDist")) { InstanceLoadPathSettings::normalizeDist = instanceLoadPathSettings["NormalizeDist"]; }
		if (instanceLoadPathSettings.contains("GroundSnap")) { InstanceLoadPathSettings::groundSnap = instanceLoadPathSettings["GroundSnap"]; }
		if (instanceLoadPathSettings.contains("NegSnapDist")) { InstanceLoadPathSettings::negSnapDist = instanceLoadPathSettings["NegSnapDist"]; }
		if (instanceLoadPathSettings.contains("PosSnapDist")) { InstanceLoadPathSettings::posSnapDist = instanceLoadPathSettings["PosSnapDist"]; }
		if (instanceLoadPathSettings.contains("Radius")) { InstanceLoadPathSettings::radius = instanceLoadPathSettings["Radius"]; }
		if (instanceLoadPathSettings.contains("Loop")) { InstanceLoadPathSettings::loop = instanceLoadPathSettings["Loop"]; }
		if (instanceLoadPathSettings.contains("Rolling")) { InstanceLoadPathSettings::rolling = instanceLoadPathSettings["Rolling"]; }
	}
}
	
void App::SaveUISettings(bool useDefault)
{
	int width, height, xpos, ypos;
	bool maximized = false;
	if (useDefault)
	{
		width = 600;
		height = 400;
		ypos = 50;
		xpos = 50;
		maximized = false;
	}
	else
	{
		if (glfwGetWindowAttrib(m_window, GLFW_ICONIFIED))
		{
			width = Settings::w_width;
			height = Settings::w_height;
			xpos = Settings::w_x;
			ypos = Settings::w_y;
			maximized = Settings::w_maximized;
		}
		else
		{
			glfwGetWindowSize(m_window, &width, &height);
			glfwGetWindowPos(m_window, &xpos, &ypos);
			maximized = glfwGetWindowAttrib(m_window, GLFW_MAXIMIZED);
		}
	}

	nlohmann::json json;
	json["Width"] = width;
	json["Height"] = height;
	json["WindowX"] = xpos;
	json["WindowY"] = ypos;
	json["Maximized"] = maximized;
	json["AnimTex"] = Settings::w_animtex;
	json["BSP"] = Settings::w_bsp;
	json["Checkpoints"] = Settings::w_checkpoints;
	json["Ghost"] = Settings::w_ghost;
	json["Level"] = Settings::w_level;
	json["Material"] = Settings::w_material;
	json["Quadblocks"] = Settings::w_quadblocks;
	json["Renderer"] = Settings::w_renderer;
	json["Spawn"] = Settings::w_spawn;
	json["LastOpenedFolder"] = Settings::m_lastOpenedFolder;
	json["LastOpenedScriptFolder"] = Settings::m_lastOpenedScriptFolder;
	json["LastOpenedModelFolder"] = Settings::m_lastOpenedModelFolder;
	json["Script"] = Settings::w_python;
	json["Instances"] = Settings::w_modelImporter;
	json["Bot"] = Settings::w_bot;
	json["CameraBindings"] = {
		{"Forward", GuiRenderSettings::camKeyForward},
		{"Back", GuiRenderSettings::camKeyBack},
		{"Left", GuiRenderSettings::camKeyLeft},
		{"Right", GuiRenderSettings::camKeyRight},
		{"Up", GuiRenderSettings::camKeyUp},
		{"Down", GuiRenderSettings::camKeyDown},
		{"Sprint", GuiRenderSettings::camKeySprint},
		{"OrbitMouseButton", GuiRenderSettings::camOrbitMouseButton},
	};
	json["CameraSettings"] = {
	{"FovDeg", GuiRenderSettings::camFovDeg},
	{"ZoomMult", GuiRenderSettings::camZoomMult},
	{"RotateMult", GuiRenderSettings::camRotateMult},
	{"MoveMult", GuiRenderSettings::camMoveMult},
	{"SprintMult", GuiRenderSettings::camSprintMult},
	};
	json["RendererSettings"] = {
		{"FilterActive", GuiRenderSettings::filterActive},
		{"FilterColor", {GuiRenderSettings::defaultFilterColor.r, GuiRenderSettings::defaultFilterColor.g, GuiRenderSettings::defaultFilterColor.b}},
		{"RenderType", GuiRenderSettings::renderType},
		{"ShowLowLOD", GuiRenderSettings::showLowLOD},
		{"ShowWireframe", GuiRenderSettings::showWireframe},
		{"ShowVerts", GuiRenderSettings::showVerts},
		{"ShowBackfaces", GuiRenderSettings::showBackfaces},
		{"ShowBspRectTree", GuiRenderSettings::showBspRectTree},
		{"ShowLevel", GuiRenderSettings::showLevel},
		{"ShowCheckpoints", GuiRenderSettings::showCheckpoints},
		{"ShowStartpoints", GuiRenderSettings::showStartpoints},
		{"ShowVisTree", GuiRenderSettings::showVisTree},
		{"ShowSelectedQuadblockInfo", GuiRenderSettings::showSelectedQuadblockInfo},
		{"ShowSkybox", GuiRenderSettings::showSkybox},
		{"ShowBots", GuiRenderSettings::showBots},
		{"ShowInstances", GuiRenderSettings::showInstances},
		{"ShowMinimapBounds", GuiRenderSettings::showMinimapBounds},
		{"SelectedCheckpointColor", {GuiRenderSettings::selectedCheckpointColor.r, GuiRenderSettings::selectedCheckpointColor.g, GuiRenderSettings::selectedCheckpointColor.b}},
	};
	json["MinimapSettings"] = {
		{"TextureHeight", MinimapSettings::textureHeight},
		{"Orientation", MinimapSettings::orientation},
		{"CheckpointQuads", MinimapSettings::checkpointQuads},
		{"CheckpointPathableQuads", MinimapSettings::checkpointPathableQuads},
	};
	json["HotReloadSettings"] = {
		{"RelicSapphire", HotReloadSettings::relicSapphire},
		{"RelicGold", HotReloadSettings::relicGold},
		{"RelicPlatinum", HotReloadSettings::relicPlatinum},
		{"CrystalTime", HotReloadSettings::crystalTime},
		{"IntroCutscene", HotReloadSettings::introCutscene},
		{"Ghost", HotReloadSettings::ghost},
	};
	json["BSPTreeSettings"] = {
		{"MaxQuadPerLeaf", BSPTreeSettings::maxQuadPerLeaf},
		{"MaxAxisDistance", BSPTreeSettings::maxAxisDistance},
		{"SeparateMaterial", BSPTreeSettings::separateMaterial},
	};
	json["VisTreeSettings"] = {
		{"CenterOnlySamples", VisTreeSettings::centerOnlySamples},
		{"CommutativeRays", VisTreeSettings::commutativeRays},
		{"SelfTargetNearClip", VisTreeSettings::selfTargetNearClip},
		{"NearClipDistance", VisTreeSettings::nearClipDistance},
		{"FarClipDistance", VisTreeSettings::farClipDistance},
	};
	json["BotPathSettings"] = {
		{"UseManualPath", BotPathSettings::useManualPath},
		{"NormalizeNodeDist", BotPathSettings::normalizeNodeDist},
		{"NodeDistance", BotPathSettings::nodeDistance},
		{"SidewayOffset", BotPathSettings::sidewayOffset},
		{"NegSnapDist", BotPathSettings::negSnapDist},
		{"PosSnapDist", BotPathSettings::posSnapDist},
		{"GhostStart", BotPathSettings::ghostStart},
		{"GhostEnd", BotPathSettings::ghostEnd},
		{"PathColor", {BotPathSettings::pathColor[0], BotPathSettings::pathColor[1], BotPathSettings::pathColor[2]}},
	};
	json["WaterAnimSettings"] = {
		{"WaveLength", WaterAnimSettings::waveLength},
		{"SizeTex", WaterAnimSettings::sizeTex},
		{"ScrollULoops", WaterAnimSettings::ScrollULoops},
		{"ScrollVLoops", WaterAnimSettings::ScrollVLoops},
		{"WaveCyclesTimeU", WaterAnimSettings::waveCyclesTimeU},
		{"WaveCyclesTimeV", WaterAnimSettings::waveCyclesTimeV},
		{"WaveAmplitude", WaterAnimSettings::waveAmplitude},
		{"BaseBrightness", WaterAnimSettings::baseBrightness},
		{"BrightAmp", WaterAnimSettings::brightAmp},
		{"BrightWaveCycle", WaterAnimSettings::brightWaveCycle},
	};
	json["InstanceLoadPathSettings"] = {
		{"Normalize", InstanceLoadPathSettings::normalize},
		{"NormalizeDist", InstanceLoadPathSettings::normalizeDist},
		{"GroundSnap", InstanceLoadPathSettings::groundSnap},
		{"NegSnapDist", InstanceLoadPathSettings::negSnapDist},
		{"PosSnapDist", InstanceLoadPathSettings::posSnapDist},
		{"Radius", InstanceLoadPathSettings::radius},
		{"Loop", InstanceLoadPathSettings::loop},
		{"Rolling", InstanceLoadPathSettings::rolling},
	};
	std::ofstream file = std::ofstream(m_configFile);
	file << std::setw(4) << json << std::endl;
	file.close();
}

void App::CloseImGui()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}
