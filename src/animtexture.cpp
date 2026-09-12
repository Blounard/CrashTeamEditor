#include "animtexture.h"
#include "level.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <iostream>
#include <fstream>

AnimTexture::AnimTexture(const std::filesystem::path& path, const std::vector<std::string>& usedNames)
{
	m_path = path;
	std::string origName = path.filename().replace_extension().string();
	m_name = origName;
	int repetitionCount = 1;
	while (true)
	{
		bool validName = true;
		for (const std::string& usedName : usedNames)
		{
			if (m_name == usedName) { validName = false; break; }
		}
		if (validName) { break; }
		m_name = origName + " (" + std::to_string(repetitionCount++) + ")";
	}
	if (!ReadAnimation(path)) { ClearAnimation(); }
}


AnimTexture::AnimTexture(const PSX::AnimTex& animTex, const std::string& animName, const std::filesystem::path& tempDir,
	const std::array<std::vector<std::pair<std::string, QuadUV>>, NUM_FACES_QUADBLOCK>& faceFrames,
	const std::unordered_map<std::string, Texture>& materialToTexture)
{
	std::filesystem::path animDir = tempDir / animName;
	std::filesystem::create_directories(animDir);
	std::filesystem::path tempObjPath = animDir / (animName + ".obj");
	std::ofstream objFile(tempObjPath);

	const size_t frameCount = faceFrames[0].size();

	objFile << "mtllib " << animName << ".mtl\n";

	for (size_t frameIdx = 0; frameIdx < frameCount; frameIdx++)
	{
		std::string objName = "Frame_" + std::to_string(frameIdx + 1);
		float z = static_cast<float>(frameIdx);

		// 9 vertices, normalized coordinates
		objFile << "v 0.0 0.0 " << z << "\n";
		objFile << "v 0.0 0.5 " << z << "\n";
		objFile << "v 0.0 1.0 " << z << "\n";
		objFile << "v 0.5 0.0 " << z << "\n";
		objFile << "v 0.5 0.5 " << z << "\n";
		objFile << "v 0.5 1.0 " << z << "\n";
		objFile << "v 1.0 0.0 " << z << "\n";
		objFile << "v 1.0 0.5 " << z << "\n";
		objFile << "v 1.0 1.0 " << z << "\n";

		// 3 normals, placeholders
		objFile << "vn 0.0 1.0 0.0\n";
		objFile << "vn 0.0 1.0 0.0\n";
		objFile << "vn 0.0 1.0 0.0\n";

		for (size_t faceIdx = 0; faceIdx < NUM_FACES_QUADBLOCK; faceIdx++)
		{
			const QuadUV& uvs = faceFrames[faceIdx][frameIdx].second;
			objFile << "vt " << uvs[0].x << " " << (1.0f - uvs[0].y) << "\n";
			objFile << "vt " << uvs[1].x << " " << (1.0f - uvs[1].y) << "\n"; // original, sounds good but there is an uv bug
			objFile << "vt " << uvs[2].x << " " << (1.0f - uvs[2].y) << "\n";
			objFile << "vt " << uvs[3].x << " " << (1.0f - uvs[3].y) << "\n";
		}

		objFile << "o " << objName << "\n";

		int vOffset = static_cast<int>(frameIdx * 9) + 1;
		int vtOffset = static_cast<int>(frameIdx * 16) + 1;
		int vnOffset = static_cast<int>(frameIdx * 3) + 1;
		// TODO VERIFY FACE ORDER : quadblock constructor should put face0 from obj to face0 in quadblock. So face0 should be correct to start with...
		// 

	// Face 0: Top-Left Quadrant (Reversed: 3 -> 4 -> 1 -> 0)
		objFile << "usemtl " << faceFrames[0][frameIdx].first << "\n";
		objFile << "f " << (vOffset + 3) << "/" << (vtOffset + 2) << "/" << (vnOffset + 0) << " "
			<< (vOffset + 4) << "/" << (vtOffset + 3) << "/" << (vnOffset + 0) << " "
			<< (vOffset + 1) << "/" << (vtOffset + 1) << "/" << (vnOffset + 0) << " "
			<< (vOffset + 0) << "/" << (vtOffset + 0) << "/" << (vnOffset + 0) << "\n";

		// Face 1: Top-Right Quadrant (Reversed: 4 -> 5 -> 2 -> 1)
		objFile << "usemtl " << faceFrames[1][frameIdx].first << "\n";
		objFile << "f " << (vOffset + 4) << "/" << (vtOffset + 6) << "/" << (vnOffset + 1) << " "
			<< (vOffset + 5) << "/" << (vtOffset + 7) << "/" << (vnOffset + 1) << " "
			<< (vOffset + 2) << "/" << (vtOffset + 5) << "/" << (vnOffset + 1) << " "
			<< (vOffset + 1) << "/" << (vtOffset + 4) << "/" << (vnOffset + 1) << "\n";

		// Face 2: Bottom-Left Quadrant (Reversed: 6 -> 7 -> 4 -> 3)
		objFile << "usemtl " << faceFrames[2][frameIdx].first << "\n";
		objFile << "f " << (vOffset + 6) << "/" << (vtOffset + 10) << "/" << (vnOffset + 2) << " "
			<< (vOffset + 7) << "/" << (vtOffset + 11) << "/" << (vnOffset + 2) << " "
			<< (vOffset + 4) << "/" << (vtOffset + 9) << "/" << (vnOffset + 2) << " "
			<< (vOffset + 3) << "/" << (vtOffset + 8) << "/" << (vnOffset + 2) << "\n";

		// Face 3: Bottom-Right Quadrant (Reversed: 7 -> 8 -> 5 -> 4)
		objFile << "usemtl " << faceFrames[3][frameIdx].first << "\n";
		objFile << "f " << (vOffset + 7) << "/" << (vtOffset + 14) << "/" << (vnOffset + 2) << " "
			<< (vOffset + 8) << "/" << (vtOffset + 15) << "/" << (vnOffset + 2) << " "
			<< (vOffset + 5) << "/" << (vtOffset + 13) << "/" << (vnOffset + 2) << " "
			<< (vOffset + 4) << "/" << (vtOffset + 12) << "/" << (vnOffset + 2) << "\n";
	}
	objFile.close();

	// Write MTL, copying each distinct material's already-created PNG locally alongside it
	std::filesystem::path tempMtlPath = animDir / (animName + ".mtl");
	std::ofstream mtlFile(tempMtlPath);
	std::set<std::string> writtenMaterials;

	for (const auto& frames : faceFrames)
	{
		for (const auto& [mat, uv] : frames)
		{
			if (writtenMaterials.count(mat) || mat == "default") { continue; }
			writtenMaterials.insert(mat);

			if (!materialToTexture.count(mat)) { continue; }

			const std::filesystem::path& sourcePath = materialToTexture.at(mat).GetPath();
			std::filesystem::path localPath = animDir / sourcePath.filename();
			std::error_code ec;
			std::filesystem::copy_file(sourcePath, localPath, std::filesystem::copy_options::overwrite_existing, ec);
			if (ec) { printf("WARNING : Failed to copy texture '%s' for AnimTexture '%s': %s\n", sourcePath.string().c_str(), animName.c_str(), ec.message().c_str()); }

			mtlFile << "newmtl " << mat << "\n";
			mtlFile << "map_Kd " << sourcePath.filename().string() << "\n";
		}
	}
	mtlFile.close();

	m_path = tempObjPath;
	m_name = animName;
	if (!ReadAnimation(tempObjPath)) { ClearAnimation(); }
	m_startAtFrame = static_cast<int>(animTex.startAtFrame);
	m_duration = static_cast<int>(animTex.frameDuration);
}

bool AnimTexture::IsEmpty() const
{
	return m_frames.empty();
}

bool AnimTexture::IsTriblock() const
{
	return m_triblock;
}

const std::vector<AnimTextureFrame>& AnimTexture::GetFrames() const
{
	return m_frames;
}

const std::vector<Texture>& AnimTexture::GetTextures() const
{
	return m_textures;
}

bool AnimTexture::AdvanceRender(float deltaTime)
{
	deltaTime = std::max(deltaTime, 0.0f);
	if (m_frames.empty())
	{
		ResetRenderState();
		return false;
	}

	const size_t frameCount = m_frames.size();
	bool reset = false;
	if (m_renderDirty)
	{
		m_renderDirty = false;
		reset = true;
	}

	const float frameDuration = static_cast<float>(m_duration + 1) / 30.0f;
	if (frameCount <= 1) { return reset; }

	m_renderFrameTimer += deltaTime;
	bool advanced = false;
	if (m_renderFrameTimer >= frameDuration)
	{
		m_renderFrameTimer = 0.0f;
		m_renderFrameIndex++;
		if (m_renderFrameIndex >= frameCount)
		{
			m_renderFrameIndex = static_cast<size_t>(m_startAtFrame);
		}
		advanced = true;
	}

	return reset || advanced;
}

const AnimTextureFrame& AnimTexture::GetRenderFrame() const
{
	const int frameCount = static_cast<int>(m_frames.size());
	if (frameCount <= 0)
	{
		static AnimTextureFrame emptyFrame = {};
		return emptyFrame;
	}

	size_t frameIndex = m_renderFrameIndex;
	if (frameIndex >= frameCount)
	{
		frameIndex = static_cast<size_t>(std::clamp(m_startAtFrame, 0, static_cast<int>(frameCount - 1)));
	}
	return m_frames[frameIndex];
}

std::vector<uint8_t> AnimTexture::Serialize(size_t offsetFirstFrame, size_t offTextures) const
{
	std::vector<uint8_t> buffer(sizeof(PSX::AnimTex));
	PSX::AnimTex animTex = {};
	animTex.offActiveFrame = static_cast<uint32_t>(offTextures + (offsetFirstFrame * sizeof(PSX::TextureGroup)));
	animTex.frameCount = static_cast<uint16_t>(m_frames.size());
	animTex.startAtFrame = static_cast<int16_t>(m_startAtFrame);
	animTex.frameDuration = static_cast<int16_t>(m_duration);
	animTex.frameIndex = 0;
	memcpy(buffer.data(), &animTex, sizeof(PSX::AnimTex));
	return buffer;
}

const std::string& AnimTexture::GetName() const
{
	return m_name;
}

bool AnimTexture::IsPopulated() const
{
	return !IsEmpty() && !m_quadblockIndexes.empty();
}

void AnimTexture::AddQuadblockIndex(size_t index)
{
	for (size_t currIndex : m_quadblockIndexes) { if (index == currIndex) { return; } }
	m_quadblockIndexes.push_back(index);
}

void AnimTexture::CopyParameters(const AnimTexture& animTex)
{
	m_startAtFrame = animTex.m_startAtFrame;
	m_duration = animTex.m_duration;
	m_rotation = animTex.m_rotation;
	m_textures = animTex.m_textures;
	m_frames = animTex.m_frames;
	m_renderDirty = true;
}

bool AnimTexture::IsEquivalent(const AnimTexture& animTex) const
{
	if (m_triblock != animTex.m_triblock) { return false; }
	if (m_startAtFrame != animTex.m_startAtFrame) { return false; }
	if (m_duration != animTex.m_duration) { return false; }
	if (m_frames.size() != animTex.m_frames.size()) { return false; }
	if (m_textures.size() != animTex.m_textures.size()) { return false; }
	for (size_t i = 0; i < m_frames.size(); i++)
	{
		if (m_frames[i].textureIndexes != animTex.m_frames[i].textureIndexes) { return false; }
		if (m_frames[i].uvs != animTex.m_frames[i].uvs) { return false; }
	}
	for (size_t i = 0; i < m_textures.size(); i++)
	{
		if (m_textures[i].GetBlendMode() != animTex.m_textures[i].GetBlendMode()) { return false; }
		if (m_textures[i] != animTex.m_textures[i]) { return false; }
	}
	return true;
}

const std::vector<size_t>& AnimTexture::GetQuadblockIndexes() const
{
	return m_quadblockIndexes;
}

bool AnimTexture::ReadAnimation(const std::filesystem::path& path)
{
	Level dummy;
	if (!dummy.Load(path, false)) { return false; }

	std::unordered_map<std::filesystem::path, size_t> loadedPaths;
	const std::vector<Quadblock>& quadblocks = dummy.GetQuadblocks();
	for (const Quadblock& quadblock : quadblocks)
	{
		AnimTextureFrame frame = {};
		frame.uvs = quadblock.GetUVs();
		for (size_t face = 0; face < NUM_FACES_QUADBLOCK + 1; face++)
		{
			const std::filesystem::path& texPath = quadblock.GetTexPath(face);
			if (texPath.empty())
			{
				if (face == NUM_FACES_QUADBLOCK) { frame.textureIndexes[face] = frame.textureIndexes[0]; continue; }
				return false;
			}
			if (!loadedPaths.contains(texPath))
			{
				size_t index = m_textures.size();
				loadedPaths.insert({ texPath, index });
				m_textures.emplace_back(texPath);
			}
			frame.textureIndexes[face] = loadedPaths.at(texPath);
		}
		m_frames.push_back(frame);
	}
	SetDefaultParams();
	return true;
}

void AnimTexture::ClearAnimation()
{
	SetDefaultParams();
	m_name.clear(); m_path.clear();
	m_frames.clear(); m_textures.clear();
	m_quadblockIndexes.clear();
	m_previewQuadName.clear();
	m_previewMaterialName.clear();
	m_lastAppliedMaterialName.clear();
}

void AnimTexture::SetStartFrame(int frame) 
{ 
	m_startAtFrame = frame; 
	m_renderDirty = true; 
}

void AnimTexture::SetDuration(int duration) 
{ 
	m_duration = duration;
	m_renderDirty = true; 
}

void AnimTexture::SetDefaultParams()
{
	m_startAtFrame = 0;
	m_duration = 0;
	m_rotation = 0;
	m_horMirror = false;
	m_verMirror = false;
	m_previewQuadIndex = 0;
	m_manualOrientation = false;
	ResetRenderState();
}

void AnimTexture::MirrorQuadUV(bool horizontal, std::array<QuadUV, 5>& uvs)
{
	if (horizontal)
	{
		auto SwapHor = [](QuadUV& uv1, QuadUV& uv2)
			{
				Swap(uv1[0], uv2[1]);
				Swap(uv1[1], uv2[0]);
				Swap(uv1[2], uv2[3]);
				Swap(uv1[3], uv2[2]);
			};

		SwapHor(uvs[0], uvs[1]);
		SwapHor(uvs[2], uvs[3]);
		Swap(uvs[4][0], uvs[4][1]);
		Swap(uvs[4][2], uvs[4][3]);
	}
	else
	{
		auto SwapVer = [](QuadUV& uv1, QuadUV& uv2)
			{
				Swap(uv1[0], uv2[2]);
				Swap(uv1[2], uv2[0]);
				Swap(uv1[1], uv2[3]);
				Swap(uv1[3], uv2[1]);
			};

		SwapVer(uvs[0], uvs[2]);
		SwapVer(uvs[1], uvs[3]);
		Swap(uvs[4][0], uvs[4][2]);
		Swap(uvs[4][1], uvs[4][3]);
	}
}

void AnimTexture::RotateQuadUV(std::array<QuadUV, 5>& uvs)
{
	auto SwapQuadUV = [this](QuadUV& dst, const QuadUV& tgt)
		{
			dst[0] = tgt[2];
			dst[1] = tgt[0];
			if (m_triblock) { dst[2] = tgt[1]; }
			else
			{
				dst[2] = tgt[3];
				dst[3] = tgt[1];
			}
		};

	QuadUV tmp = uvs[0];
	SwapQuadUV(uvs[0], uvs[2]);
	if (m_triblock) { SwapQuadUV(uvs[2], uvs[1]); }
	else
	{
		SwapQuadUV(uvs[2], uvs[3]);
		SwapQuadUV(uvs[3], uvs[1]);
	}
	SwapQuadUV(uvs[1], tmp);

	Vec2 lowTmp = uvs[4][0];
	uvs[4][0] = uvs[4][2];
	if (m_triblock) { uvs[4][2] = uvs[4][1]; }
	else
	{
		uvs[4][2] = uvs[4][3];
		uvs[4][3] = uvs[4][1];
	}
	uvs[4][1] = lowTmp;
}

void AnimTexture::MirrorFrames(bool horizontal)
{
	if (m_triblock) { return; }
	for (AnimTextureFrame& frame : m_frames)
	{
		MirrorQuadUV(horizontal, frame.uvs);
	}
	m_renderDirty = true;
}

void AnimTexture::RotateFrames(int targetRotation)
{
	if (targetRotation == 0) { return; }

	int rotTimes = targetRotation / 90;
	if (rotTimes < 0) { rotTimes = 4 - (rotTimes * -1); }
	for (AnimTextureFrame& frame : m_frames)
	{
		for (int i = 0; i < rotTimes; i++) { RotateQuadUV(frame.uvs); }
	}
	m_renderDirty = true;
}

void AnimTexture::ResetRenderState()
{
	m_renderFrameTimer = 0.0f;
	m_renderFrameIndex = static_cast<size_t>(m_startAtFrame);
	m_renderDirty = true;
}
