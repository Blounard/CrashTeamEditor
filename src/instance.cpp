#include "instance.h"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace
{
	// --- Parsing helpers, file-local to this translation unit ---

	int ParseIntSafe(const std::string& s)
	{
		if (s.empty()) { return 0; }
		try { return std::stoi(s); }
		catch (...) { return 0; }
	}

	struct ParsedMaterial
	{
		std::string localName;
		std::string textureFile; // relative filename from map_Kd, empty if none
	};

	std::vector<ParsedMaterial> ParseMtlFile(const std::filesystem::path& mtlPath)
	{
		std::vector<ParsedMaterial> materials;
		std::ifstream file(mtlPath);
		if (!file) { return materials; }

		std::string line;
		while (std::getline(file, line))
		{
			std::istringstream iss(line);
			std::string token;
			iss >> token;

			if (token == "newmtl")
			{
				ParsedMaterial mat;
				iss >> mat.localName;
				materials.push_back(mat);
			}
			else if (token == "map_Kd" && !materials.empty())
			{
				iss >> materials.back().textureFile;
			}
		}
		return materials;
	}

	struct ParsedOBJVertex
	{
		Vec3 pos;
		Color color = Color(static_cast<unsigned char>(128), 128, 128);
	};

	struct ParsedFace
	{
		int vIdx[3] = { 0, 0, 0 };
		int vtIdx[3] = { 0, 0, 0 };
		int vnIdx[3] = { 0, 0, 0 };
		std::string material; // local material name from the last "usemtl"
	};

	void ParseFaceVertexToken(const std::string& token, int& v, int& vt, int& vn)
	{
		v = 0; vt = 0; vn = 0;
		size_t firstSlash = token.find('/');
		if (firstSlash == std::string::npos)
		{
			v = ParseIntSafe(token);
			return;
		}
		v = ParseIntSafe(token.substr(0, firstSlash));

		size_t secondSlash = token.find('/', firstSlash + 1);
		if (secondSlash == std::string::npos)
		{
			vt = ParseIntSafe(token.substr(firstSlash + 1));
			return;
		}
		vt = ParseIntSafe(token.substr(firstSlash + 1, secondSlash - firstSlash - 1));
		vn = ParseIntSafe(token.substr(secondSlash + 1));
	}

	void ParseObjFile(const std::filesystem::path& objPath,
		std::vector<ParsedOBJVertex>& outVerts,
		std::vector<Vec2>& outUVs,
		std::vector<Vec3>& outNormals,
		std::vector<ParsedFace>& outFaces)
	{
		std::ifstream file(objPath);
		if (!file) { return; }

		std::string currentMaterial;
		std::string line;
		while (std::getline(file, line))
		{
			if (line.empty() || line[0] == '#') { continue; }
			std::istringstream iss(line);
			std::string token;
			iss >> token;

			if (token == "v")
			{
				ParsedOBJVertex v;
				float r = 0.5f, g = 0.5f, b = 0.5f;
				iss >> v.pos.x >> v.pos.y >> v.pos.z;
				iss >> r >> g >> b; // our exporter always writes these; default gray if absent
				v.color = Color(
					static_cast<unsigned char>(std::clamp(r, 0.0f, 1.0f) * 255.0f),
					static_cast<unsigned char>(std::clamp(g, 0.0f, 1.0f) * 255.0f),
					static_cast<unsigned char>(std::clamp(b, 0.0f, 1.0f) * 255.0f));
				outVerts.push_back(v);
			}
			else if (token == "vt")
			{
				Vec2 uv;
				iss >> uv.x >> uv.y;
				outUVs.push_back(uv);
			}
			else if (token == "vn")
			{
				Vec3 n;
				iss >> n.x >> n.y >> n.z;
				outNormals.push_back(n);
			}
			else if (token == "usemtl")
			{
				iss >> currentMaterial;
			}
			else if (token == "f")
			{
				ParsedFace face;
				face.material = currentMaterial;
				for (int i = 0; i < 3; i++)
				{
					std::string vertToken;
					iss >> vertToken;
					ParseFaceVertexToken(vertToken, face.vIdx[i], face.vtIdx[i], face.vnIdx[i]);
				}
				outFaces.push_back(face);
			}
		}
	}

	// Ensures 'baseName' doesn't collide with an existing key in materialToTexture,
	// appending a numeric suffix ("wood" -> "wood1") until it's unique.
	std::string MakeUniqueMaterialName(const std::string& baseName,
		const std::unordered_map<std::string, Texture>& materialToTexture)
	{
		if (!materialToTexture.contains(baseName)) { return baseName; }
		int suffix = 1;
		std::string candidate;
		do
		{
			candidate = baseName + std::to_string(suffix);
			suffix++;
		} while (materialToTexture.contains(candidate));
		return candidate;
	}

	// Loads an .obj + .mtl pair into a flat triangle list, creating a new Texture
	// object (with a globally-unique material name) for every material that has
	// a texture, and adding it to materialToTexture.
	std::vector<Tri> LoadFacesFromObjMtl(const std::filesystem::path& objPath,
		const std::filesystem::path& mtlPath,
		std::unordered_map<std::string, Texture>& materialToTexture)
	{
		std::vector<Tri> faces;

		std::vector<ParsedMaterial> materials = ParseMtlFile(mtlPath);

		// Local (per-header, as written in the .mtl) material name -> globally-unique name
		std::unordered_map<std::string, std::string> localToGlobalMaterial;
		for (const ParsedMaterial& mat : materials)
		{
			if (mat.textureFile.empty())
			{
				// No texture (e.g. "notex") — nothing to add to the shared map,
				// so no uniqueness concern; keep the name as-is.
				localToGlobalMaterial[mat.localName] = mat.localName;
				continue;
			}

			std::string globalName = MakeUniqueMaterialName(mat.localName, materialToTexture);
			std::filesystem::path pngPath = mtlPath.parent_path() / mat.textureFile;
			materialToTexture.emplace(globalName, Texture(pngPath));
			localToGlobalMaterial[mat.localName] = globalName;
		}

		std::vector<ParsedOBJVertex> verts;
		std::vector<Vec2> uvs;
		std::vector<Vec3> normals;
		std::vector<ParsedFace> parsedFaces;
		ParseObjFile(objPath, verts, uvs, normals, parsedFaces);

		faces.reserve(parsedFaces.size());
		for (const ParsedFace& pf : parsedFaces)
		{
			Point p[3];
			bool hasNormal[3] = { false, false, false };

			for (int i = 0; i < 3; i++)
			{
				int vIndex = pf.vIdx[i] - 1;
				if (vIndex >= 0 && vIndex < static_cast<int>(verts.size()))
				{
					p[i].pos = verts[vIndex].pos;
					p[i].color = verts[vIndex].color;
				}

				int vtIndex = pf.vtIdx[i] - 1;
				if (vtIndex >= 0 && vtIndex < static_cast<int>(uvs.size()))
				{
					Vec2 rawUV = uvs[vtIndex]; // as stored in the .obj file (bottom-left origin)
					if (rawUV.x < 0.0f || rawUV.x > 1.0f || rawUV.y < 0.0f || rawUV.y > 1.0f)
					{
						printf("WARNING: UV (%.4f, %.4f) out of [0,1] range in %s, clamping\n",
							rawUV.x, rawUV.y, objPath.string().c_str());
						rawUV.x = std::clamp(rawUV.x, 0.0f, 1.0f);
						rawUV.y = std::clamp(rawUV.y, 0.0f, 1.0f);
					}
					// Invert ExportOBJ's `1.0f - v` to get back to top-left origin
					p[i].uv = Vec2(rawUV.x, 1.0f - rawUV.y);
				}

				int vnIndex = pf.vnIdx[i] - 1;
				if (vnIndex >= 0 && vnIndex < static_cast<int>(normals.size()))
				{
					p[i].normal = normals[vnIndex];
					hasNormal[i] = true;
				}
			}

			// Fallback: recompute a flat face normal if the .obj didn't provide one
			if (!hasNormal[0] || !hasNormal[1] || !hasNormal[2])
			{
				Vec3 e1 = p[1].pos - p[0].pos;
				Vec3 e2 = p[2].pos - p[0].pos;
				Vec3 n = e1.Cross(e2);
				if (n.LengthSquared() > 0.0001f) { n.Normalize(); }
				p[0].normal = n; p[1].normal = n; p[2].normal = n;
			}

			Tri tri(p[0], p[1], p[2]);
			auto matIt = localToGlobalMaterial.find(pf.material);
			tri.texture = (matIt != localToGlobalMaterial.end()) ? matIt->second : std::string();
			faces.push_back(tri);
		}

		return faces;
	}
}








InstanceModelHeader::InstanceModelHeader(const nlohmann::json& headerJson, const std::filesystem::path& modelDir,
	std::unordered_map<std::string, Texture>& materialToTexture)
	: m_name(headerJson.value("name", std::string()))
	, m_maxDistLOD(headerJson.value("maxDistanceLOD", 0.0f))
	, m_flags(headerJson.value("flags", static_cast<uint16_t>(0)))
	, m_scale()
	, m_scaleOrPad(headerJson.value("scaleOrPad", static_cast<int16_t>(0)))
	, m_unk1(headerJson.value("unk1", static_cast<uint32_t>(0)))
	, m_unk3(headerJson.value("unk3", static_cast<uint32_t>(0)))
	, m_unkNum(headerJson.value("unkNum", static_cast<uint32_t>(0)))
{
	if (headerJson.contains("scale"))
	{
		const nlohmann::json& scaleJson = headerJson["scale"];
		m_scale.x = scaleJson.value("x", 0.0f);
		m_scale.y = scaleJson.value("y", 0.0f);
		m_scale.z = scaleJson.value("z", 0.0f);
	}

	std::string objFile = headerJson.value("objFile", std::string());
	std::string mtlFile = headerJson.value("mtlFile", std::string());
	if (objFile.empty() || mtlFile.empty()) { return; }

	m_faces = LoadFacesFromObjMtl(modelDir / objFile, modelDir / mtlFile, materialToTexture);
}

InstanceModel::InstanceModel(const std::filesystem::path& jsonPath, std::unordered_map<std::string, Texture>& materialToTexture)
{
	std::ifstream jsonFile(jsonPath);
	if (!jsonFile) { return; }

	nlohmann::json json = nlohmann::json::parse(jsonFile);
	m_name = json.value("name", std::string());
	m_id = json.value("id", static_cast<int16_t>(0));
	m_valid = true;

	std::filesystem::path modelDir = jsonPath.parent_path();

	if (json.contains("headers") && json["headers"].is_array())
	{
		for (const nlohmann::json& headerJson : json["headers"])
		{
			m_headers.emplace_back(headerJson, modelDir, materialToTexture);
		}
	}
}









InstanceModelHeader::InstanceModelHeader(PSX::ModelHeader modelHeader, std::vector<Tri> triangles, uint32_t unkNum)
{
	m_name = std::string(modelHeader.name, strnlen(modelHeader.name, sizeof(modelHeader.name)));
	m_maxDistLOD = ConvertFP(modelHeader.maxDistanceLOD, FP_ONE_GEO);
	m_flags = modelHeader.flags;
	m_scale = ConvertPSXVec3(modelHeader.scale, FP_ONE);
	m_faces = triangles;
	m_scaleOrPad = modelHeader.maybeScaleMaybePadding;
	m_unk1 = modelHeader.unk1;
	m_unk3 = modelHeader.unk3;
	m_unkNum = unkNum;
}

nlohmann::json InstanceModelHeader::WriteMetadataJson(const std::string& objFile, const std::string& mtlFile) const
{
	nlohmann::json json;
	json["name"] = m_name;
	json["objFile"] = objFile;
	json["mtlFile"] = mtlFile;
	json["triangleCount"] = m_faces.size();
	json["maxDistanceLOD"] = m_maxDistLOD;
	json["flags"] = m_flags;
	json["scale"] = { {"x", m_scale.x}, {"y", m_scale.y}, {"z", m_scale.z} };
	json["scaleOrPad"] = m_scaleOrPad;
	json["unk1"] = m_unk1;
	json["unk3"] = m_unk3;
	json["unkNum"] = m_unkNum;
	return json;
}

void InstanceModelHeader::ExportOBJ(const std::filesystem::path& modelDir, std::string baseFileName, std::unordered_map<std::string, Texture>& materialToTexture)
{
	std::unordered_map<std::string, std::vector<size_t>> materialToTris; //material name -> list of triangle index
	for (size_t i = 0; i < m_faces.size(); i++)
		materialToTris[m_faces[i].texture].push_back(i);

	// --- .mtl ---
	std::ofstream mtl(modelDir / (baseFileName + ".mtl"));
	if (mtl)
	{
		for (const auto& [matName, indices] : materialToTris)
		{
			std::filesystem::path sourcePath = materialToTexture[matName].GetPath();
			std::filesystem::path destPath = modelDir / sourcePath.filename();

			mtl << "newmtl " << matName << "\nKd 1 1 1\n";
			 mtl << "map_Kd " << sourcePath.filename() << "\n";
			mtl << "\n";

			// Copy .png to the modelDir aswell. (not directly extracted there, so they are initially extracted once if several quad/models share the same texture)
			std::filesystem::copy_file(sourcePath, destPath, std::filesystem::copy_options::overwrite_existing);
		}
	}

	// --- .obj ---
	std::ofstream obj(modelDir / (baseFileName + ".obj"));
	if (!obj) { return; }

	obj << "# Auto-exported from .ctrmodel (triangle soup, no shared vertex indices)\n";
	obj << "mtllib " << baseFileName << ".mtl\n";
	obj << "o " << baseFileName << "\n\n";

	size_t runningIndex = 0; // 1-based OBJ v/vt index, advances by 3 per triangle
	for (const auto& [matName, indices] : materialToTris)
	{
		obj << "usemtl " << matName << "\n";
		for (size_t triIdx : indices)
		{
			const Tri& tri = m_faces[triIdx];

			Vec3 e1 = tri.p[1].pos - tri.p[0].pos;
			Vec3 e2 = tri.p[2].pos - tri.p[0].pos;
			Vec3 n = e1.Cross(e2);
			if (n.LengthSquared() > 0.0001f) { n.Normalize(); }

			for (int i = 0; i < 3; i++)
			{
				obj << "v " << tri.p[i].pos.x << " " << tri.p[i].pos.y << " " << tri.p[i].pos.z
					<< " " << (tri.p[i].color.r / 255.0f) << " " << (tri.p[i].color.g / 255.0f)
					<< " " << (tri.p[i].color.b / 255.0f) << "\n"; // nonstandard v+rgb extension (Blender/MeshLab)
			}
			for (int i = 0; i < 3; i++)
			{
				// PNG/PSX v origin is top-left, OBJ vt origin is bottom-left
				obj << "vt " << tri.p[i].uv.x << " " << (1.0f - tri.p[i].uv.y) << "\n";
			}
			obj << "vn " << n.x << " " << n.y << " " << n.z << "\n";

			size_t i0 = runningIndex + 1, i1 = runningIndex + 2, i2 = runningIndex + 3;
			size_t vn = runningIndex / 3 + 1;
			obj << "f " << i0 << "/" << i0 << "/" << vn
				<< " " << i1 << "/" << i1 << "/" << vn
				<< " " << i2 << "/" << i2 << "/" << vn << "\n";
			runningIndex += 3;
		}
		obj << "\n";
	}
}

InstanceModel::InstanceModel(std::string name, std::vector<uint8_t> rawData)
	: m_name(std::move(name))
	, m_rawData(std::move(rawData))
{
	m_valid = false;
}

InstanceModel::InstanceModel(PSX::Model model, std::string modelName)
{
	m_name = modelName;
	m_id = model.id;
	m_valid = true;
	m_headers.clear();
}


void InstanceModel::Export(const std::filesystem::path& exportDir, std::unordered_map<std::string, Texture>& materialToTexture)
{
	std::filesystem::path modelDir = exportDir / m_name;
	std::filesystem::create_directories(modelDir);

	std::vector<std::string> objFiles(m_headers.size());
	std::vector<std::string> mtlFiles(m_headers.size());

	for (size_t headerID = 0; headerID < m_headers.size(); headerID++)
	{
		InstanceModelHeader& header = m_headers[headerID];
		std::string baseFileName = m_name + "LOD" + std::to_string(headerID);
		header.ExportOBJ(modelDir, baseFileName, materialToTexture);
		objFiles[headerID] = baseFileName + ".obj";
		mtlFiles[headerID] = baseFileName + ".mtl";
	}

	nlohmann::json json;
	json["name"] = m_name;
	json["id"] = m_id;
	json["numHeaders"] = m_headers.size();

	nlohmann::json headersArray = nlohmann::json::array();
	for (size_t headerID = 0; headerID < m_headers.size(); headerID++)
	{
		headersArray.push_back(m_headers[headerID].WriteMetadataJson(objFiles[headerID], mtlFiles[headerID]));
	}
	json["headers"] = headersArray;

	std::ofstream file(modelDir / "metadata.json");
	file << std::setw(4) << json << std::endl;
	file.close();
}


namespace
{
	void AppendBytes(std::vector<uint8_t>& buffer, const void* data, size_t size)
	{
		const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
		buffer.insert(buffer.end(), bytes, bytes + size);
	}

	template <typename T>
	void AppendValue(std::vector<uint8_t>& buffer, const T& value)
	{
		AppendBytes(buffer, &value, sizeof(T));
	}

	void AppendPadding(std::vector<uint8_t>& buffer, size_t alignment)
	{
		size_t remainder = buffer.size() % alignment;
		if (remainder != 0)
		{
			buffer.insert(buffer.end(), alignment - remainder, uint8_t(0));
		}
	}

	// Inverse of the decoder's per-axis dequantization:
	//   value = ((rawByte / 255.0f) + origin) * scale
	uint8_t QuantizeVertexAxis(float value, float scale, float origin)
	{
		if (std::fabs(scale) < 0.0001f) { return 0; } // degenerate (flat) axis
		float normalized = (value / scale) - origin;
		float raw = std::round(normalized * 255.0f);
		return static_cast<uint8_t>(std::clamp(raw, 0.0f, 255.0f));
	}

	// Encodes an engine-space position into the PSX format's byte triple:
	// axis-shuffled (X,Z,Y storage order) and sign-flipped on X/Z, mirroring
	// DecodeModelHeaderTriangles' vertex decode in reverse.
	void EncodeVertexBytes(const Vec3& pos, const Vec3& scale, const Vec3& origin, uint8_t outBytes[3])
	{
		float preFlipX = -pos.x;
		float preFlipY = pos.y;
		float preFlipZ = -pos.z;

		outBytes[0] = QuantizeVertexAxis(preFlipX, scale.x, origin.x); // decode reads src[0] -> pos.x
		outBytes[2] = QuantizeVertexAxis(preFlipY, scale.y, origin.y); // decode reads src[2] -> pos.y
		outBytes[1] = QuantizeVertexAxis(preFlipZ, scale.z, origin.z); // decode reads src[1] -> pos.z
	}

	// Useless function, must remove to use directly ConvertColor
	uint32_t PackColor(const Color& c)
	{
		PSX::Color psxColor = ConvertColor(c);
		return (uint32_t(psxColor.r) << 0) | (uint32_t(psxColor.g) << 8) |
			(uint32_t(psxColor.b) << 16) | (uint32_t(psxColor.a) << 24);
	}

	// colorCoordIndex is only 7 bits wide (max 128 entries per header), so
	// colors are deduplicated by exact value. If a header genuinely needs more
	// than 128 distinct colors, we warn once and reuse the last slot rather
	// than silently corrupt the index via bitfield truncation.
	uint32_t GetOrAddColorIndex(std::vector<uint32_t>& palette, std::unordered_map<uint32_t, uint32_t>& lookup,
		const Color& color, const std::string& headerName, bool& warnedOverflow)
	{
		uint32_t packed = PackColor(color);
		auto it = lookup.find(packed);
		if (it != lookup.end()) { return it->second; }

		if (palette.size() >= 128)
		{
			if (!warnedOverflow)
			{
				printf("WARNING: header '%s' needs more than 128 unique colors; some colors will be approximated\n", headerName.c_str());
				warnedOverflow = true;
			}
			return static_cast<uint32_t>(palette.size() - 1);
		}

		uint32_t index = static_cast<uint32_t>(palette.size());
		palette.push_back(packed);
		lookup[packed] = index;
		return index;
	}
}



void InstanceModelHeader::SerializeInto(std::vector<uint8_t>& output, uint32_t modelOffset, size_t headerStructOffset,
	std::unordered_map<std::string, Texture>& materialToTexture,
	std::vector<uint32_t>& outPointerLocations) const
{
	PSX::ModelHeader header{};
	std::memset(header.name, 0, sizeof(header.name));
	std::memcpy(header.name, m_name.data(), std::min(m_name.size(), sizeof(header.name)));
	//std::strncpy(header.name, m_name.c_str(), sizeof(header.name) - 1);
	header.unk1 = m_unk1;
	header.maxDistanceLOD = ConvertFloat(m_maxDistLOD, FP_ONE_GEO);
	header.flags = m_flags;
	header.maybeScaleMaybePadding = m_scaleOrPad;
	header.unk3 = m_unk3;
	header.numAnimations = 0; // this editable format has no animation support; not a guess, an invariant
	header.offAnimations = 0;
	header.offAnimtex = 0;

	// --- Bounding box refit: scale/origin are DERIVED from the current mesh,
	// not taken from m_scale (which was only the value read at import time and
	// may no longer fit an edited mesh). See design notes below. ---
	Vec3 preFlipMin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	Vec3 preFlipMax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
	for (const Tri& tri : m_faces)
	{
		for (int i = 0; i < 3; i++)
		{
			Vec3 preFlip(-tri.p[i].pos.x, tri.p[i].pos.y, -tri.p[i].pos.z);
			preFlipMin.x = std::min(preFlipMin.x, preFlip.x); preFlipMax.x = std::max(preFlipMax.x, preFlip.x);
			preFlipMin.y = std::min(preFlipMin.y, preFlip.y); preFlipMax.y = std::max(preFlipMax.y, preFlip.y);
			preFlipMin.z = std::min(preFlipMin.z, preFlip.z); preFlipMax.z = std::max(preFlipMax.z, preFlip.z);
		}
	}
	if (m_faces.empty()) { preFlipMin = Vec3(0, 0, 0); preFlipMax = Vec3(0, 0, 0); }

	Vec3 boxSize(
		std::max(preFlipMax.x - preFlipMin.x, 0.0001f),
		std::max(preFlipMax.y - preFlipMin.y, 0.0001f),
		std::max(preFlipMax.z - preFlipMin.z, 0.0001f)
	);
	Vec3 originF(preFlipMin.x / boxSize.x, preFlipMin.y / boxSize.y, preFlipMin.z / boxSize.z);

	// Round into the actual PSX fixed-point fields, then recompute the
	// "effective" float values FROM those rounded integers (not the
	// pre-rounding floats), so vertex quantization below matches exactly
	// what the decoder will reconstruct when it reads these fields back.
	float factor = static_cast<float>(FP_ONE);
	header.scale.x = static_cast<int16_t>(std::round(boxSize.x * factor)); // 960 factor: see ParseCtrModelGeometry's modelScale comment
	header.scale.y = static_cast<int16_t>(std::round(boxSize.y * factor));
	header.scale.z = static_cast<int16_t>(std::round(boxSize.z * factor));
	Vec3 effScale(header.scale.x / factor, header.scale.y / factor, header.scale.z / factor);

	PSX::ModelFrame frame{};
	frame.pos.x = static_cast<int16_t>(std::round(originF.x * 256.0f));
	frame.pos.y = static_cast<int16_t>(std::round(originF.y * 256.0f));
	frame.pos.z = static_cast<int16_t>(std::round(originF.z * 256.0f));
	frame.maybePosMaybePadding = 0; // default, per struct comment ("usually 0x0")
	std::memset(frame.unk16, 0, sizeof(frame.unk16)); // default, per struct comment ("sixteen 0x0")
	frame.vertexOffset = sizeof(PSX::ModelFrame); // standard layout, no mystery padding

	Vec3 effOrigin(frame.pos.x / 256.0f, frame.pos.y / 256.0f, frame.pos.z / 256.0f);

	// --- Build command list + vertex data + texture layouts + colors ---
	std::vector<PSX::InstDrawCommand> commands;
	std::vector<uint8_t> vertexBytes;
	std::vector<PSX::TextureLayout> layouts;
	std::vector<uint32_t> colorPalette;
	std::unordered_map<uint32_t, uint32_t> colorLookup;
	bool warnedColorOverflow = false;
	bool warnedTexOverflow = false;

	for (const Tri& tri : m_faces)
	{
		// Push order is reversed (p2, p1, p0) relative to the triangle's own
		// corner order -- required so the decoder's stack machine (which
		// emits temp[3],temp[2],temp[1] as tri.p[0],p[1],p[2]) reproduces the
		// exact corner order we stored, with no swap/normalFlip trick needed.
		uint32_t colorIdx[3];
		colorIdx[2] = GetOrAddColorIndex(colorPalette, colorLookup, tri.p[0].color, m_name, warnedColorOverflow); // read by cmdC
		colorIdx[1] = GetOrAddColorIndex(colorPalette, colorLookup, tri.p[1].color, m_name, warnedColorOverflow); // read by cmdB
		colorIdx[0] = GetOrAddColorIndex(colorPalette, colorLookup, tri.p[2].color, m_name, warnedColorOverflow); // read by cmdA

		uint32_t texCoordIndex = 0;
		auto texIt = materialToTexture.find(tri.texture);
		if (!tri.texture.empty() && texIt != materialToTexture.end())
		{
			// TextureLayout is a 4-corner quad struct reused for 3-corner
			// triangles; the 4th corner is never read on decode, so we fill
			// it with the centroid rather than leave it at (0,0).
			Vec2 centroid(
				(tri.p[0].uv.x + tri.p[1].uv.x + tri.p[2].uv.x) / 3.0f,
				(tri.p[0].uv.y + tri.p[1].uv.y + tri.p[2].uv.y) / 3.0f
			);
			QuadUV quadUV = { tri.p[2].uv, tri.p[1].uv, tri.p[0].uv, centroid };

			layouts.push_back(texIt->second.Serialize(quadUV));

			if (layouts.size() > 511)
			{
				if (!warnedTexOverflow)
				{
					printf("WARNING: header '%s' needs more than 511 texture layouts; reusing last one for the rest\n", m_name.c_str());
					warnedTexOverflow = true;
				}
				layouts.pop_back();
				texCoordIndex = 511;
			}
			else
			{
				texCoordIndex = static_cast<uint32_t>(layouts.size()); // 1-based
			}
		}

		uint8_t bytes[3];
		for (int pushOrder = 0; pushOrder < 3; pushOrder++)
		{
			int cornerIdx = 2 - pushOrder; // 2, 1, 0 -- matches command push order below
			EncodeVertexBytes(tri.p[cornerIdx].pos, effScale, effOrigin, bytes);
			vertexBytes.push_back(bytes[0]);
			vertexBytes.push_back(bytes[1]);
			vertexBytes.push_back(bytes[2]);
		}

		PSX::InstDrawCommand cmdA{};
		cmdA.stackWriteLocationIndex = 87; // was bugged and vanilla used 87 for crates. NEED UNDERSTANDING.
		cmdA.readNextVertFromStackIndexFlag = 0;
		cmdA.resetFlag = 1; // starts this triangle as an independent strip
		cmdA.colorCoordIndex = colorIdx[0];
		cmdA.texCoordIndex = texCoordIndex; 
		cmdA.colorFromScratchpadOrRamFlag = static_cast<uint32_t>(!tri.texture.empty());
		cmdA.noBackfaceFlag = 0; // Will need to store this per tri instead of hardcoding 
		commands.push_back(cmdA);

		PSX::InstDrawCommand cmdB{};
		cmdB.stackWriteLocationIndex = 87; 
		cmdB.readNextVertFromStackIndexFlag = 0;
		cmdB.resetFlag = 0;
		cmdB.colorCoordIndex = colorIdx[1];
		cmdB.texCoordIndex = texCoordIndex;
		cmdB.colorFromScratchpadOrRamFlag = static_cast<uint32_t>(!tri.texture.empty());
		cmdB.noBackfaceFlag = 0;
		commands.push_back(cmdB);

		PSX::InstDrawCommand cmdC{};
		cmdC.stackWriteLocationIndex = 87;
		cmdC.readNextVertFromStackIndexFlag = 0;
		cmdC.resetFlag = 0;
		cmdC.colorCoordIndex = colorIdx[2];
		cmdC.texCoordIndex = texCoordIndex; 
		cmdC.colorFromScratchpadOrRamFlag = static_cast<uint32_t>(!tri.texture.empty());
		cmdC.noBackfaceFlag = 0;
		commands.push_back(cmdC);
	}

	// --- Write command list section: unkNum + commands + terminator ---
	const size_t commandListOffset = output.size();
	//unkNum is actually colorPalette size ??
	AppendValue(output, static_cast<uint32_t>(colorPalette.size()));
	for (const PSX::InstDrawCommand& cmd : commands) { AppendValue(output, cmd); }
	PSX::InstDrawCommand terminator{};
	terminator.command = 0xFFFFFFFF;
	AppendValue(output, terminator);

	// --- Write frame + vertex data ---
	const size_t frameDataOffset = output.size();
	AppendValue(output, frame);
	AppendBytes(output, vertexBytes.data(), vertexBytes.size());
	AppendPadding(output, 4);

	// --- Write texture layouts + pointer array ---
	size_t texLayoutPtrArrayOffset = 0;
	if (!layouts.empty())
	{
		const size_t texLayoutsOffset = output.size();
		for (const PSX::TextureLayout& layout : layouts) { AppendValue(output, layout); }

		texLayoutPtrArrayOffset = output.size();
		for (size_t i = 0; i < layouts.size(); i++)
		{
			uint32_t layoutPtr = static_cast<uint32_t>(modelOffset + texLayoutsOffset + i * sizeof(PSX::TextureLayout));
			AppendValue(output, layoutPtr);

			// Each entry here is itself a pointer stored in the .lev file, so
			// its own location goes in the relocation table too -- mirrors
			// ExtractModels' patchTable.push_back for this same array.
			outPointerLocations.push_back(static_cast<uint32_t>(modelOffset + texLayoutPtrArrayOffset + i * sizeof(uint32_t)));
		}
	}

	// --- Write colors ---
	size_t colorsOffset = 0;
	if (!colorPalette.empty())
	{
		colorsOffset = output.size();
		for (uint32_t packed : colorPalette) { AppendValue(output, packed); }
	}

	// --- Patch the ModelHeader reserved earlier by InstanceModel::Serialize ---
	header.offCommandList = static_cast<uint32_t>(modelOffset + commandListOffset);
	header.offFrameData = static_cast<uint32_t>(modelOffset + frameDataOffset);
	header.offTexLayout = layouts.empty() ? 0 : static_cast<uint32_t>(modelOffset + texLayoutPtrArrayOffset);
	header.offColors = colorPalette.empty() ? 0 : static_cast<uint32_t>(modelOffset + colorsOffset);

	std::memcpy(output.data() + headerStructOffset, &header, sizeof(PSX::ModelHeader));

	// --- Record this header's own pointer-field locations ---
	const uint32_t headerAbsoluteOffset = static_cast<uint32_t>(modelOffset + headerStructOffset);
	outPointerLocations.push_back(CALCULATE_OFFSET(PSX::ModelHeader, offCommandList, headerAbsoluteOffset));
	outPointerLocations.push_back(CALCULATE_OFFSET(PSX::ModelHeader, offFrameData, headerAbsoluteOffset));
	if (!layouts.empty())
	{
		outPointerLocations.push_back(CALCULATE_OFFSET(PSX::ModelHeader, offTexLayout, headerAbsoluteOffset));
	}
	if (!colorPalette.empty())
	{
		outPointerLocations.push_back(CALCULATE_OFFSET(PSX::ModelHeader, offColors, headerAbsoluteOffset));
	}
}


std::vector<uint8_t> InstanceModel::Serialize(uint32_t modelOffset, std::unordered_map<std::string, Texture>& materialToTexture,
	std::vector<uint32_t>& outPointerLocations) const
{
	std::vector<uint8_t> output;
	outPointerLocations.clear();

	// --- Model ---
	PSX::Model model{};
	std::memset(model.name, 0, sizeof(model.name));
	std::memcpy(model.name, m_name.data(), std::min(m_name.size(), sizeof(model.name)));
	//std::strncpy(model.name, m_name.c_str(), sizeof(model.name) - 1);
	model.id = m_id;
	model.numHeaders = static_cast<uint16_t>(m_headers.size());
	model.offHeaders = 0; // patched below

	AppendValue(output, model);

	// --- ModelHeader[] block, reserved contiguously ---
	const size_t headersBlockOffset = output.size();
	for (size_t h = 0; h < m_headers.size(); h++)
	{
		PSX::ModelHeader placeholder{};
		AppendValue(output, placeholder);
	}

	{
		PSX::Model* modelPtr = reinterpret_cast<PSX::Model*>(output.data());
		modelPtr->offHeaders = static_cast<uint32_t>(modelOffset + headersBlockOffset);
	}
	outPointerLocations.push_back(CALCULATE_OFFSET(PSX::Model, offHeaders, modelOffset));

	for (size_t h = 0; h < m_headers.size(); h++)
	{
		const size_t headerStructOffset = headersBlockOffset + h * sizeof(PSX::ModelHeader);
		m_headers[h].SerializeInto(output, modelOffset, headerStructOffset, materialToTexture, outPointerLocations);
	}

	AppendPadding(output, 4);

	return output;
}

Instance::Instance(std::string model)
{
	m_name = "NewInstance";
	m_scale = Vec3(1.0f, 1.0f, 1.0f);
	m_pos = Vec3(0.0f, 0.0f, 0.0f);
	m_rot = Vec3(0.0f, 0.0f, 0.0f);
	m_modelID = ModelId::NONE;
	m_color = Color(0.0f, 0.0f, 0.0f);
	m_modelName = model;
	m_flags = 0xB;
	m_unk24 = 0;
	m_unk28 = 0;
	m_hitbox = InstanceHitbox();
}

Instance::Instance(PSX::InstDef inst)
{
	m_name = std::string(inst.name, strnlen(inst.name, sizeof(inst.name)));
	m_scale = ConvertPSXVec3(inst.scale, FP_ONE);
	m_pos = ConvertPSXVec3(inst.pos, FP_ONE_GEO);
	m_rot = ConvertPSXAngle(inst.rot);
	m_rot.x = -m_rot.x;
	m_rot.y += 180.0f;
	m_rot.z = -m_rot.z;
	m_modelID = static_cast<ModelId>(inst.modelID);
	m_color = ConvertColor(inst.colorRGBA);
	m_flags = inst.flags;
	m_unk24 = inst.unk24;
	m_unk28 = inst.unk28;

	m_modelName = "";
	m_hitbox = InstanceHitbox();
}

void Instance::SetHitbox(const PSX::InstHitbox& hitbox)
{
	m_hitbox.enabled = true;
	m_hitbox.flags = hitbox.flags;
	m_hitbox.halfExtent = ConvertFP(hitbox.halfExtent, FP_ONE_GEO);
	m_hitbox.yOffset = ConvertFP(hitbox.center.y, FP_ONE_GEO) - m_pos.y;
}

std::vector<uint8_t> Instance::Serialize(uint32_t offModel) const
{
	PSX::InstDef inst = {};
	std::memset(inst.name, 0, sizeof(inst.name));
	std::memcpy(inst.name, m_name.data(), std::min(m_name.size(), sizeof(inst.name)));
	inst.offModel = offModel; 
	inst.scale = ConvertVec3(m_scale, FP_ONE); // 0x1000 is 1.0 scaling
	inst.maybeScaleMaybePadding = 0; 
	inst.colorRGBA = ConvertColor(m_color);
	inst.flags = m_flags;
	inst.unk24 = m_unk24;
	inst.unk28 = m_unk28;
	inst.offInstance = 0; // Probably unused data
	inst.pos = ConvertVec3(m_pos, FP_ONE_GEO);
	inst.rot = ConvertAngle(m_rot);
	inst.modelID = static_cast<int32_t>(m_modelID);

	std::vector<uint8_t> buffer(sizeof(PSX::InstDef));
	std::memcpy(buffer.data(), &inst, sizeof(PSX::InstDef));
	return buffer;
}

BoundingBox Instance::ComputeBBox()
{
	Vec3 center = m_pos + Vec3(0.0f, m_hitbox.yOffset, 0.0f);
	Vec3 half_ext = Vec3(m_hitbox.halfExtent, m_hitbox.halfExtent, m_hitbox.halfExtent);
	BoundingBox bbox{};
	bbox.min = center - half_ext;
	bbox.max = center + half_ext;
	return bbox;
}


PSX::InstHitbox Instance::SerializeHitbox(uint32_t insatnceOffset) const
{	// Don't call on Instances that have hitbox disabled. 
	// Serialization will still work, but shouldn't be called.

	Vec3 center = m_pos + Vec3(0.0f, m_hitbox.yOffset, 0.0f);
	Vec3 half_ext = Vec3(m_hitbox.halfExtent, m_hitbox.halfExtent, m_hitbox.halfExtent);

	PSX::InstHitbox hitbox = {};
	hitbox.flags = m_hitbox.flags;
	hitbox.bbox.min = ConvertVec3(center - half_ext, FP_ONE_GEO);
	hitbox.bbox.max = ConvertVec3(center + half_ext, FP_ONE_GEO);
	hitbox.center = ConvertVec3(center, FP_ONE_GEO);
	hitbox.halfExtent = ConvertFloat(m_hitbox.halfExtent, FP_ONE_GEO);
	hitbox.halfExtentSq = hitbox.halfExtent * hitbox.halfExtent;
	hitbox.padding = 0;
	hitbox.offInstDef = insatnceOffset;

	return hitbox;
}