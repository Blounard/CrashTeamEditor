#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_STB_IMAGE
#include "tiny_gltf.h"
#undef far
#undef near

#include "instance.h"

#include <filesystem>
#include <fstream>
#include <iostream>

#include <numeric>
#include <functional>



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
namespace detail
{
	// ---- low-level helpers ----

	template <typename T>
	void writePOD(std::ostream& os, const T& value)
	{
		static_assert(std::is_trivially_copyable<T>::value, "writePOD requires trivially copyable type");
		os.write(reinterpret_cast<const char*>(&value), sizeof(T));
	}

	template <typename T>
	void readPOD(std::istream& is, T& value)
	{
		static_assert(std::is_trivially_copyable<T>::value, "readPOD requires trivially copyable type");
		is.read(reinterpret_cast<char*>(&value), sizeof(T));
		if (!is)
			throw std::runtime_error("Unexpected end of file while reading POD value");
	}

	void writeString(std::ostream& os, const std::string& s)
	{
		uint32_t len = static_cast<uint32_t>(s.size());
		writePOD(os, len);
		if (len > 0)
			os.write(s.data(), len);
	}

	std::string readString(std::istream& is)
	{
		uint32_t len = 0;
		readPOD(is, len);
		std::string s(len, '\0');
		if (len > 0)
		{
			is.read(&s[0], len);
			if (!is)
				throw std::runtime_error("Unexpected end of file while reading string data");
		}
		return s;
	}

	void writeVec3(std::ostream& os, const Vec3& v)
	{
		writePOD(os, v.x);
		writePOD(os, v.y);
		writePOD(os, v.z);
	}

	Vec3 readVec3(std::istream& is)
	{
		Vec3 v{};
		readPOD(is, v.x);
		readPOD(is, v.y);
		readPOD(is, v.z);
		return v;
	}
}




namespace
{
	static std::vector<Vec3> ReadVec3Accessor(const tinygltf::Model& model, int accessorIdx)
	{
		const tinygltf::Accessor& acc = model.accessors[accessorIdx];
		const tinygltf::BufferView& bv = model.bufferViews[acc.bufferView];
		const tinygltf::Buffer& buf = model.buffers[bv.buffer];
		size_t stride = bv.byteStride != 0 ? bv.byteStride : sizeof(float) * 3;
		const uint8_t* base = buf.data.data() + bv.byteOffset + acc.byteOffset;
		std::vector<Vec3> out(acc.count);
		for (size_t i = 0; i < acc.count; i++)
		{
			const float* f = reinterpret_cast<const float*>(base + i * stride);
			out[i] = Vec3(f[0], f[1], f[2]);
		}
		return out;
	}

	static std::vector<Vec2> ReadVec2Accessor(const tinygltf::Model& model, int accessorIdx)
	{
		const tinygltf::Accessor& acc = model.accessors[accessorIdx];
		const tinygltf::BufferView& bv = model.bufferViews[acc.bufferView];
		const tinygltf::Buffer& buf = model.buffers[bv.buffer];
		size_t stride = bv.byteStride != 0 ? bv.byteStride : sizeof(float) * 2;
		const uint8_t* base = buf.data.data() + bv.byteOffset + acc.byteOffset;
		std::vector<Vec2> out(acc.count);
		for (size_t i = 0; i < acc.count; i++)
		{
			const float* f = reinterpret_cast<const float*>(base + i * stride);
			out[i] = Vec2(f[0], f[1]);
		}
		return out;
	}

	static std::vector<float> ReadScalarAccessor(const tinygltf::Model& model, int accessorIdx)
	{
		const tinygltf::Accessor& acc = model.accessors[accessorIdx];
		const tinygltf::BufferView& bv = model.bufferViews[acc.bufferView];
		const tinygltf::Buffer& buf = model.buffers[bv.buffer];
		size_t stride = bv.byteStride != 0 ? bv.byteStride : sizeof(float);
		const uint8_t* base = buf.data.data() + bv.byteOffset + acc.byteOffset;
		std::vector<float> out(acc.count);
		for (size_t i = 0; i < acc.count; i++)
			out[i] = *reinterpret_cast<const float*>(base + i * stride);
		return out;
	}

	// COLOR_0 may be VEC3/VEC4 and FLOAT / normalized UBYTE / normalized
	// USHORT per spec (Blender commonly exports normalized UBYTE VEC4).
	// Alpha, if present, is dropped -- we don't track per-vertex alpha.
	static std::vector<Vec3> ReadColorAccessor(const tinygltf::Model& model, int accessorIdx)
	{
		const tinygltf::Accessor& acc = model.accessors[accessorIdx];
		const tinygltf::BufferView& bv = model.bufferViews[acc.bufferView];
		const tinygltf::Buffer& buf = model.buffers[bv.buffer];
		int numComponents = acc.type == TINYGLTF_TYPE_VEC4 ? 4 : 3;
		size_t compSize = acc.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT ? 4
			: acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT ? 2 : 1;
		size_t stride = bv.byteStride != 0 ? bv.byteStride : compSize * numComponents;
		const uint8_t* base = buf.data.data() + bv.byteOffset + acc.byteOffset;

		std::vector<Vec3> out(acc.count);
		for (size_t i = 0; i < acc.count; i++)
		{
			const uint8_t* p = base + i * stride;
			float c[3];
			for (int k = 0; k < 3; k++)
			{
				if (acc.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
					c[k] = reinterpret_cast<const float*>(p)[k];
				else if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
					c[k] = reinterpret_cast<const uint16_t*>(p)[k] / 65535.0f;
				else
					c[k] = p[k] / 255.0f;
			}
			out[i] = Vec3(c[0], c[1], c[2]);
		}
		return out;
	}

	static std::vector<uint32_t> ReadIndices(const tinygltf::Model& model, int accessorIdx)
	{
		const tinygltf::Accessor& acc = model.accessors[accessorIdx];
		const tinygltf::BufferView& bv = model.bufferViews[acc.bufferView];
		const tinygltf::Buffer& buf = model.buffers[bv.buffer];
		const uint8_t* base = buf.data.data() + bv.byteOffset + acc.byteOffset; // spec: no byteStride on index bufferViews

		std::vector<uint32_t> out(acc.count);
		for (size_t i = 0; i < acc.count; i++)
		{
			switch (acc.componentType)
			{
			case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:  out[i] = base[i]; break;
			case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: out[i] = reinterpret_cast<const uint16_t*>(base)[i]; break;
			case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:   out[i] = reinterpret_cast<const uint32_t*>(base)[i]; break;
			default: out[i] = 0; break;
			}
		}
		return out;
	}

	// Column-major 4x4, matching glTF's own convention.
	struct Mat4 { float m[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 }; };

	static Mat4 Mat4Multiply(const Mat4& a, const Mat4& b)
	{
		Mat4 r{};
		for (int col = 0; col < 4; col++)
			for (int row = 0; row < 4; row++)
			{
				float sum = 0.0f;
				for (int k = 0; k < 4; k++) sum += a.m[k * 4 + row] * b.m[col * 4 + k];
				r.m[col * 4 + row] = sum;
			}
		return r;
	}

	static Mat4 Mat4FromTRS(const std::vector<double>& t, const std::vector<double>& r, const std::vector<double>& s)
	{
		float tx = t.size() == 3 ? (float)t[0] : 0, ty = t.size() == 3 ? (float)t[1] : 0, tz = t.size() == 3 ? (float)t[2] : 0;
		float qx = r.size() == 4 ? (float)r[0] : 0, qy = r.size() == 4 ? (float)r[1] : 0, qz = r.size() == 4 ? (float)r[2] : 0, qw = r.size() == 4 ? (float)r[3] : 1;
		float sx = s.size() == 3 ? (float)s[0] : 1, sy = s.size() == 3 ? (float)s[1] : 1, sz = s.size() == 3 ? (float)s[2] : 1;

		float xx = qx * qx, yy = qy * qy, zz = qz * qz;
		float xy = qx * qy, xz = qx * qz, yz = qy * qz;
		float wx = qw * qx, wy = qw * qy, wz = qw * qz;

		Mat4 out{};
		out.m[0] = (1 - 2 * (yy + zz)) * sx; out.m[1] = (2 * (xy + wz)) * sx;  out.m[2] = (2 * (xz - wy)) * sx;  out.m[3] = 0;
		out.m[4] = (2 * (xy - wz)) * sy;   out.m[5] = (1 - 2 * (xx + zz)) * sy; out.m[6] = (2 * (yz + wx)) * sy;  out.m[7] = 0;
		out.m[8] = (2 * (xz + wy)) * sz;   out.m[9] = (2 * (yz - wx)) * sz;   out.m[10] = (1 - 2 * (xx + yy)) * sz; out.m[11] = 0;
		out.m[12] = tx; out.m[13] = ty; out.m[14] = tz; out.m[15] = 1;
		return out;
	}

	static Mat4 Mat4FromNode(const tinygltf::Node& node)
	{
		if (node.matrix.size() == 16)
		{
			Mat4 out{};
			for (int i = 0; i < 16; i++) out.m[i] = (float)node.matrix[i];
			return out;
		}
		return Mat4FromTRS(node.translation, node.rotation, node.scale);
	}

	static Vec3 Mat4TransformPoint(const Mat4& m, const Vec3& p)
	{
		return Vec3(
			m.m[0] * p.x + m.m[4] * p.y + m.m[8] * p.z + m.m[12],
			m.m[1] * p.x + m.m[5] * p.y + m.m[9] * p.z + m.m[13],
			m.m[2] * p.x + m.m[6] * p.y + m.m[10] * p.z + m.m[14]
		);
	}

	// Linear part only -- no translation. Used for morph-target deltas, which
	// are directions, not points.
	static Vec3 Mat4TransformVector(const Mat4& m, const Vec3& v)
	{
		return Vec3(
			m.m[0] * v.x + m.m[4] * v.y + m.m[8] * v.z,
			m.m[1] * v.x + m.m[5] * v.y + m.m[9] * v.z,
			m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z
		);
	}


	static bool FindMeshNodeRecursive(const tinygltf::Model& model, int nodeIdx, const Mat4& parentTransform,
		int& outMeshIdx, int& outMeshNodeIdx, Mat4& outAncestorTransform)
	{
		const tinygltf::Node& node = model.nodes[nodeIdx];
		if (node.mesh >= 0)
		{
			outMeshIdx = node.mesh;
			outMeshNodeIdx = nodeIdx;
			outAncestorTransform = parentTransform; // deliberately excludes this node's own local matrix
			return true;
		}
		Mat4 world = Mat4Multiply(parentTransform, Mat4FromNode(node));
		for (int child : node.children)
			if (FindMeshNodeRecursive(model, child, world, outMeshIdx, outMeshNodeIdx, outAncestorTransform)) { return true; }
		return false;
	}

	static bool FindMeshNode(const tinygltf::Model& model, int& outMeshIdx, int& outMeshNodeIdx, Mat4& outAncestorTransform)
	{
		if (model.scenes.empty()) { return false; }
		int sceneIdx = model.defaultScene >= 0 ? model.defaultScene : 0;
		for (int rootNode : model.scenes[sceneIdx].nodes)
			if (FindMeshNodeRecursive(model, rootNode, Mat4{}, outMeshIdx, outMeshNodeIdx, outAncestorTransform)) { return true; }
		return false;
	}

	static std::vector<AnimatedFace> WrapFaces(const std::vector<Tri>& faces, const std::vector<bool>& doubleSided)
	{
		std::vector<AnimatedFace> out(faces.size());
		for (size_t i = 0; i < faces.size(); i++)
		{
			out[i].tri = faces[i];
			out[i].doubleSided = (i < doubleSided.size()) && doubleSided[i];
		}
		return out;
	}
}

namespace
{
	struct ChannelSampler
	{
		std::vector<float> times;
		std::vector<float> values; // flat, numComponents per sample
		int numComponents = 3;
		std::string interpolation = "LINEAR"; // STEP, LINEAR; CUBICSPLINE unsupported (see below)
		bool valid = false;
	};

	// Evaluates a channel at time t via binary search + STEP/LINEAR
	// interpolation between the two surrounding real keyframes. Clamps to
	// the first/last value outside the authored time range.
	void EvaluateChannel(const ChannelSampler& ch, float t, float* out)
	{
		if (!ch.valid || ch.times.empty())
		{
			std::fill(out, out + ch.numComponents, 0.0f);
			return;
		}
		if (t <= ch.times.front())
		{
			std::copy(ch.values.begin(), ch.values.begin() + ch.numComponents, out);
			return;
		}
		if (t >= ch.times.back())
		{
			size_t last = (ch.times.size() - 1) * ch.numComponents;
			std::copy(ch.values.begin() + last, ch.values.begin() + last + ch.numComponents, out);
			return;
		}
		size_t hi = std::upper_bound(ch.times.begin(), ch.times.end(), t) - ch.times.begin();
		size_t lo = hi - 1;
		if (ch.interpolation == "STEP")
		{
			std::copy(ch.values.begin() + lo * ch.numComponents, ch.values.begin() + lo * ch.numComponents + ch.numComponents, out);
			return;
		}
		float t0 = ch.times[lo], t1 = ch.times[hi];
		float alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0f;
		for (int c = 0; c < ch.numComponents; c++)
		{
			float a = ch.values[lo * ch.numComponents + c];
			float b = ch.values[hi * ch.numComponents + c];
			out[c] = a + (b - a) * alpha;
		}
	}

	// Quaternion lerp+renormalize ("nlerp"), not true slerp. A standard,
	// widely-used approximation -- adequate at our fixed 30Hz sample rate
	// for ordinary rotation content; only meaningfully diverges from true
	// slerp on very large angular deltas between adjacent keyframes.
	void EvaluateQuatChannel(const ChannelSampler& ch, float t, float outQuat[4])
	{
		if (!ch.valid || ch.times.empty()) { outQuat[0] = outQuat[1] = outQuat[2] = 0; outQuat[3] = 1; return; }
		if (t <= ch.times.front() || t >= ch.times.back() || ch.interpolation == "STEP")
		{
			EvaluateChannel(ch, t, outQuat);
			return;
		}
		size_t hi = std::upper_bound(ch.times.begin(), ch.times.end(), t) - ch.times.begin();
		size_t lo = hi - 1;
		float t0 = ch.times[lo], t1 = ch.times[hi];
		float alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0f;
		float a[4], b[4];
		std::copy(ch.values.begin() + lo * 4, ch.values.begin() + lo * 4 + 4, a);
		std::copy(ch.values.begin() + hi * 4, ch.values.begin() + hi * 4 + 4, b);
		float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
		float sign = (dot < 0.0f) ? -1.0f : 1.0f; // shortest-path fix
		float q[4];
		for (int i = 0; i < 4; i++) { q[i] = a[i] + (sign * b[i] - a[i]) * alpha; }
		float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		if (len < 0.0001f) { outQuat[0] = outQuat[1] = outQuat[2] = 0; outQuat[3] = 1; }
		else { for (int i = 0; i < 4; i++) outQuat[i] = q[i] / len; }
	}

	std::vector<float> ReadVec4Flat(const tinygltf::Model& model, int accessorIdx)
	{
		const tinygltf::Accessor& acc = model.accessors[accessorIdx];
		const tinygltf::BufferView& bv = model.bufferViews[acc.bufferView];
		const tinygltf::Buffer& buf = model.buffers[bv.buffer];
		size_t stride = bv.byteStride != 0 ? bv.byteStride : sizeof(float) * 4;
		const uint8_t* base = buf.data.data() + bv.byteOffset + acc.byteOffset;
		std::vector<float> out(acc.count * 4);
		for (size_t i = 0; i < acc.count; i++)
		{
			const float* f = reinterpret_cast<const float*>(base + i * stride);
			for (int c = 0; c < 4; c++) out[i * 4 + c] = f[c];
		}
		return out;
	}

	ChannelSampler ReadChannelSampler(const tinygltf::Model& model, const tinygltf::Animation& anim,
		const std::string& targetPath, int targetNode, int numComponents)
	{
		ChannelSampler cs; cs.numComponents = numComponents;
		for (const auto& ch : anim.channels)
		{
			if (ch.target_node != targetNode || ch.target_path != targetPath) { continue; }
			const tinygltf::AnimationSampler& sampler = anim.samplers[ch.sampler];
			if (sampler.interpolation == "CUBICSPLINE")
			{
				printf("WARNING: %s channel on node %d uses CUBICSPLINE -- unsupported, skipping this channel\n",
					targetPath.c_str(), targetNode);
				return cs; // valid stays false -> caller falls back to the node's static value
			}
			cs.times = ReadScalarAccessor(model, sampler.input);
			cs.values = (numComponents == 4) ? ReadVec4Flat(model, sampler.output) : [&] {
				std::vector<float> flat;
				if (numComponents == 3)
					for (const Vec3& v : ReadVec3Accessor(model, sampler.output)) { flat.push_back(v.x); flat.push_back(v.y); flat.push_back(v.z); }
				return flat;
				}();
			cs.interpolation = sampler.interpolation;
			cs.valid = true;
			return cs;
		}
		return cs;
	}
}

namespace
{


	struct PrimData
	{
		std::vector<Vec3> basePositions; // per-corner, post index-expansion + node transform
		std::vector<Vec3> localPositions;  // mesh-local space, pre-node-transform -- only used by the TRS bake path
		std::vector<Vec2> uvs;
		std::vector<Vec3> colors;
		std::string materialName; // key into materialToTexture, or "" if untextured
		bool doubleSided = false;
		std::vector<uint32_t> cornerToVertex;
		std::vector<std::vector<Vec3>> targetDeltasByVertex; // [target][vertex], pre-expansion
	};


	constexpr float GAME_FPS = 30.0f; // hardcoded: the PSX format has no fps field; frames are consumed 1-per-tick at the engine's fixed rate

	std::vector<ModelAnimation> BuildAnimationsFromTRS(const tinygltf::Model& model, const std::vector<PrimData>& prims,
		int meshNodeIdx, const Mat4& ancestorTransform,
		const std::function<std::vector<AnimatedFace>(const std::function<Vec3(size_t, size_t)>&)>& buildFaces)
	{
		std::vector<ModelAnimation> out;
		const tinygltf::Node& restNode = model.nodes[meshNodeIdx];

		for (const tinygltf::Animation& anim : model.animations)
		{
			ChannelSampler tCh = ReadChannelSampler(model, anim, "translation", meshNodeIdx, 3);
			ChannelSampler rCh = ReadChannelSampler(model, anim, "rotation", meshNodeIdx, 4);
			ChannelSampler sCh = ReadChannelSampler(model, anim, "scale", meshNodeIdx, 3);
			if (!tCh.valid && !rCh.valid && !sCh.valid) { continue; } // this Animation doesn't touch our node at all

			float minTime = FLT_MAX, maxTime = -FLT_MAX;
			for (const ChannelSampler* c : { &tCh, &rCh, &sCh })
			{
				if (!c->valid || c->times.empty()) { continue; }
				minTime = std::min(minTime, c->times.front());
				maxTime = std::max(maxTime, c->times.back());
			}
			if (minTime > maxTime) { continue; }

			size_t frameCount = static_cast<size_t>(std::round((maxTime - minTime) * GAME_FPS)) + 1;
			frameCount = std::clamp<size_t>(frameCount, 1, 4096); // sanity cap, matching decode-side bounds elsewhere

			ModelAnimation animation{};
			animation.name = !anim.name.empty() ? anim.name : ("anim_" + std::to_string(out.size()));
			animation.interpolated = (tCh.interpolation == "LINEAR" || rCh.interpolation == "LINEAR" || sCh.interpolation == "LINEAR");
			animation.hasRawNumFrames = false;

			std::vector<double> restT = restNode.translation, restR = restNode.rotation, restS = restNode.scale;

			for (size_t f = 0; f < frameCount; f++)
			{
				float t = minTime + f * (1.0f / GAME_FPS);

				float tv[3], rv[4], sv[3];
				if (tCh.valid) { EvaluateChannel(tCh, t, tv); }
				else { tv[0] = restT.size() == 3 ? (float)restT[0] : 0; tv[1] = restT.size() == 3 ? (float)restT[1] : 0; tv[2] = restT.size() == 3 ? (float)restT[2] : 0; }
				if (rCh.valid) { EvaluateQuatChannel(rCh, t, rv); }
				else { rv[0] = restR.size() == 4 ? (float)restR[0] : 0; rv[1] = restR.size() == 4 ? (float)restR[1] : 0; rv[2] = restR.size() == 4 ? (float)restR[2] : 0; rv[3] = restR.size() == 4 ? (float)restR[3] : 1; }
				if (sCh.valid) { EvaluateChannel(sCh, t, sv); }
				else { sv[0] = restS.size() == 3 ? (float)restS[0] : 1; sv[1] = restS.size() == 3 ? (float)restS[1] : 1; sv[2] = restS.size() == 3 ? (float)restS[2] : 1; }

				Mat4 localMat = Mat4FromTRS({ tv[0],tv[1],tv[2] }, { rv[0],rv[1],rv[2],rv[3] }, { sv[0],sv[1],sv[2] });
				Mat4 frameMat = Mat4Multiply(ancestorTransform, localMat);

				animation.frames.push_back(buildFaces([&](size_t p, size_t c) {
					return Mat4TransformPoint(frameMat, prims[p].localPositions[c]);
					}));
			}
			out.push_back(std::move(animation));
		}
		return out;
	}


	bool LoadGLTFHeaderData(const std::filesystem::path& gltfPath,
		std::unordered_map<std::string, Texture>& materialToTexture,
		std::vector<ModelAnimation>& outAnimations,
		bool& outIsAnimated)
	{
		auto AlwaysTrue = [](auto&&... [[maybe_unused]] args) constexpr -> bool {
			return true;
			};

		tinygltf::Model model;
		tinygltf::TinyGLTF loader;
		loader.SetImageLoader(AlwaysTrue, nullptr);
		std::string err, warn;
		bool ok = (gltfPath.extension() == ".glb")
			? loader.LoadBinaryFromFile(&model, &err, &warn, gltfPath.string())
			: loader.LoadASCIIFromFile(&model, &err, &warn, gltfPath.string());
		if (!warn.empty()) { printf("glTF warning (%s): %s\n", gltfPath.string().c_str(), warn.c_str()); }
		if (!ok) { printf("ERROR loading glTF %s: %s\n", gltfPath.string().c_str(), err.c_str()); return false; }

		int meshIdx = -1; 
		int meshNodeIdx = -1;
		Mat4 nodeTransform{};
		if (!FindMeshNode(model, meshIdx, meshNodeIdx, nodeTransform))
		{
			if (model.meshes.empty()) { printf("ERROR: no mesh in %s\n", gltfPath.string().c_str()); return false; }
			meshIdx = 0; // no scene graph present -- fall back to the first mesh, identity transform
		}
		const tinygltf::Mesh& mesh = model.meshes[meshIdx];
		std::filesystem::path gltfDir = gltfPath.parent_path();

		
		std::vector<PrimData> prims;
		size_t globalNumTargets = SIZE_MAX;

		for (const tinygltf::Primitive& prim : mesh.primitives)
		{
			if (prim.mode != TINYGLTF_MODE_TRIANGLES)
			{
				printf("WARNING: skipping non-triangle primitive in %s (mode %d)\n", gltfPath.string().c_str(), prim.mode);
				continue;
			}
			auto posIt = prim.attributes.find("POSITION");
			if (posIt == prim.attributes.end()) { continue; }

			PrimData pd;
			std::vector<Vec3> rawPositions = ReadVec3Accessor(model, posIt->second);

			if (prim.indices >= 0) { pd.cornerToVertex = ReadIndices(model, prim.indices); }
			else { pd.cornerToVertex.resize(rawPositions.size()); std::iota(pd.cornerToVertex.begin(), pd.cornerToVertex.end(), 0); }

			pd.basePositions.reserve(pd.cornerToVertex.size());
			for (uint32_t vi : pd.cornerToVertex)
				pd.basePositions.push_back(Mat4TransformPoint(nodeTransform, rawPositions[vi]));

			pd.localPositions.reserve(pd.cornerToVertex.size());
			for (uint32_t vi : pd.cornerToVertex)
				pd.localPositions.push_back(rawPositions[vi]);

			auto uvIt = prim.attributes.find("TEXCOORD_0");
			std::vector<Vec2> rawUVs = uvIt != prim.attributes.end() ? ReadVec2Accessor(model, uvIt->second) : std::vector<Vec2>();
			auto colIt = prim.attributes.find("COLOR_0");
			std::vector<Vec3> rawColors = colIt != prim.attributes.end() ? ReadColorAccessor(model, colIt->second) : std::vector<Vec3>();

			pd.uvs.reserve(pd.cornerToVertex.size());
			pd.colors.reserve(pd.cornerToVertex.size());
			for (uint32_t vi : pd.cornerToVertex)
			{
				pd.uvs.push_back(vi < rawUVs.size() ? rawUVs[vi] : Vec2(0, 0));
				pd.colors.push_back(vi < rawColors.size() ? rawColors[vi] : Vec3(0.5f, 0.5f, 0.5f));
			}

			if (prim.material >= 0 && prim.material < (int)model.materials.size())
			{
				const tinygltf::Material& mat = model.materials[prim.material];
				pd.doubleSided = mat.doubleSided;
				int texIdx = mat.pbrMetallicRoughness.baseColorTexture.index;
				if (texIdx >= 0 && texIdx < (int)model.textures.size())
				{
					int imgIdx = model.textures[texIdx].source;
					if (imgIdx >= 0 && imgIdx < (int)model.images.size() && !model.images[imgIdx].uri.empty())
					{
						std::filesystem::path pngPath = gltfDir / model.images[imgIdx].uri;
						std::string baseName = !mat.name.empty() ? mat.name : std::filesystem::path(model.images[imgIdx].uri).stem().string();
						std::string globalName = MakeUniqueMaterialName(baseName, materialToTexture);
						materialToTexture.emplace(globalName, Texture(pngPath));
						pd.materialName = globalName;
					}
				}
			}

			size_t numTargets = prim.targets.size();
			if (globalNumTargets == SIZE_MAX) { globalNumTargets = numTargets; }
			else if (numTargets != globalNumTargets)
			{
				printf("WARNING: primitive morph target count mismatch in %s (%zu vs %zu)\n",
					gltfPath.string().c_str(), numTargets, globalNumTargets);
			}
			for (const auto& target : prim.targets)
			{
				auto tPosIt = target.find("POSITION");
				std::vector<Vec3> deltas = tPosIt != target.end() ? ReadVec3Accessor(model, tPosIt->second)
					: std::vector<Vec3>(rawPositions.size(), Vec3(0, 0, 0));
				for (Vec3& d : deltas) { d = Mat4TransformVector(nodeTransform, d); }
				pd.targetDeltasByVertex.push_back(std::move(deltas));
			}

			prims.push_back(std::move(pd));
		}

		if (prims.empty()) { printf("ERROR: no usable triangle primitives in %s\n", gltfPath.string().c_str()); return false; }
		if (globalNumTargets == SIZE_MAX) { globalNumTargets = 0; }

		auto BuildFaces = [&](const std::function<Vec3(size_t primIdx, size_t corner)>& getPos) -> std::vector<AnimatedFace>
			{
				std::vector<AnimatedFace> faces;
				for (size_t p = 0; p < prims.size(); p++)
				{
					const PrimData& pd = prims[p];
					for (size_t c = 0; c + 2 < pd.basePositions.size(); c += 3)
					{
						AnimatedFace af;
						af.doubleSided = pd.doubleSided;
						af.tri.texture = pd.materialName;
						for (int k = 0; k < 3; k++)
						{
							af.tri.p[k].pos = getPos(p, c + k);
							af.tri.p[k].uv = pd.uvs[c + k];
							const Vec3& col = pd.colors[c + k];
							af.tri.p[k].color = Color(
								static_cast<unsigned char>(std::clamp(col.x, 0.0f, 1.0f) * 255.0f),
								static_cast<unsigned char>(std::clamp(col.y, 0.0f, 1.0f) * 255.0f),
								static_cast<unsigned char>(std::clamp(col.z, 0.0f, 1.0f) * 255.0f));
						}
						Vec3 e1 = af.tri.p[1].pos - af.tri.p[0].pos;
						Vec3 e2 = af.tri.p[2].pos - af.tri.p[0].pos;
						Vec3 n = e1.Cross(e2);
						if (n.LengthSquared() > 0.0001f) { n.Normalize(); }
						af.tri.p[0].normal = af.tri.p[1].normal = af.tri.p[2].normal = n;
						faces.push_back(af);
					}
				}
				return faces;
			};

		std::vector<AnimatedFace> baseFaces = BuildFaces([&](size_t p, size_t c) { return prims[p].basePositions[c]; });

		auto BuildBlendedFrame = [&](const float* weights, size_t numWeights) -> std::vector<AnimatedFace>
			{
				return BuildFaces([&](size_t p, size_t c) -> Vec3
					{
						const PrimData& pd = prims[p];
						uint32_t vi = pd.cornerToVertex[c];
						Vec3 pos = pd.basePositions[c];
						size_t n = std::min(numWeights, pd.targetDeltasByVertex.size());
						for (size_t t = 0; t < n; t++)
						{
							float w = weights[t];
							if (w == 0.0f) { continue; }
							const Vec3& delta = pd.targetDeltasByVertex[t][vi];
							pos = Vec3(pos.x + delta.x * w, pos.y + delta.y * w, pos.z + delta.z * w);
						}
						return pos;
					});
			};

		outAnimations.clear();

		if (!model.animations.empty() && globalNumTargets > 0)
		{
			outIsAnimated = true;
			for (const tinygltf::Animation& anim : model.animations)
			{
				const tinygltf::AnimationChannel* weightsChannel = nullptr;
				for (const auto& ch : anim.channels)
					if (ch.target_path == "weights") { weightsChannel = &ch; break; }
				if (weightsChannel == nullptr) { continue; }

				const tinygltf::AnimationSampler& sampler = anim.samplers[weightsChannel->sampler];
				if (sampler.interpolation == "CUBICSPLINE")
				{
					printf("WARNING: animation '%s' uses CUBICSPLINE interpolation, which isn't supported -- skipping\n",
						anim.name.empty() ? "?" : anim.name.c_str());
					continue;
				}
				std::vector<float> times = ReadScalarAccessor(model, sampler.input);
				std::vector<float> weightsFlat = ReadScalarAccessor(model, sampler.output);
				if (times.empty() || weightsFlat.size() != times.size() * globalNumTargets) { continue; }

				ModelAnimation animation{};
				animation.name = !anim.name.empty() ? anim.name : ("anim_" + std::to_string(outAnimations.size()));
				animation.interpolated = (sampler.interpolation == "LINEAR");
				animation.hasRawNumFrames = false;

				for (size_t f = 0; f < times.size(); f++)
				{
					const float* w = &weightsFlat[f * globalNumTargets];
					animation.frames.push_back(BuildBlendedFrame(w, globalNumTargets));
				}
				if (!animation.frames.empty()) { outAnimations.push_back(std::move(animation)); }
			}
		}

		// Falls through here both when there were never any morph targets at all,
		// AND when there were morph targets but no usable weights channel was
		// found on them -- either way, try baking node TRS animation before
		// giving up and calling the model static.
		if (outAnimations.empty() || (outAnimations.size() == 1 && outAnimations[0].frames.size() <= 1))
		{
			if (meshNodeIdx >= 0 && !model.animations.empty())
			{
				// TODO : change the fps setting so 1 frame is blender = 1 frame in game, regardless or blender scene fps
				std::vector<ModelAnimation> trsAnims = BuildAnimationsFromTRS(model, prims, meshNodeIdx, nodeTransform, BuildFaces);
				if (!trsAnims.empty())
				{
					outAnimations = std::move(trsAnims);
					outIsAnimated = true;
				}
			}
		}

		if (outAnimations.empty())
		{
			ModelAnimation staticAnim{};
			staticAnim.frames.push_back(baseFaces);
			outAnimations.push_back(std::move(staticAnim));
			outIsAnimated = false;
		}
		return true;
	}
}



static size_t Align4(size_t value)
{
	return (value + 3) & ~static_cast<size_t>(3);
}

// Component-wise divide with a guard: an axis with truly zero scale (shouldn't
// happen after MIN_BOX_SIZE clamping, but a raw m_hasScale override could still
// supply one) maps to origin 0 rather than producing inf/UB.
static Vec3 SafeDivide(const Vec3& num, const Vec3& denom)
{
	return Vec3(
		std::fabs(denom.x) < 0.0001f ? 0.0f : num.x / denom.x,
		std::fabs(denom.y) < 0.0001f ? 0.0f : num.y / denom.y,
		std::fabs(denom.z) < 0.0001f ? 0.0f : num.z / denom.z
	);
}


InstanceModelHeader::InstanceModelHeader(PSX::ModelHeader& modelHeader, PSX::ModelFrame& baseFrame, uint32_t colorCount,
	std::vector<ModelAnimation> animations, bool isAnimated)
{
	m_name = std::string(modelHeader.name, strnlen(modelHeader.name, sizeof(modelHeader.name)));
	m_maxDistLOD = ConvertFP(modelHeader.maxDistanceLOD, FP_ONE_GEO);
	m_flags = modelHeader.flags;
	m_scale = ConvertPSXVec3(modelHeader.scale, FP_ONE);
	m_scaleOrPad = modelHeader.maybeScaleMaybePadding;
	m_origin = ConvertPSXVec3(baseFrame.pos, FP_ONE_GEO);
	m_originOrPad = baseFrame.maybePosMaybePadding;
	m_unk1 = modelHeader.unk1;
	m_colorCount = colorCount;
	m_hasScale = true;
	m_hasOrigin = true;
	m_animations = std::move(animations);
	m_isAnimated = isAnimated;
}


InstanceModelHeader::InstanceModelHeader(const nlohmann::json& headerJson, const std::filesystem::path& modelDir,
	std::unordered_map<std::string, Texture>& materialToTexture)
	: m_name(headerJson.value("name", std::string()))
	, m_maxDistLOD(headerJson.value("maxDistanceLOD", 0.0f))
	, m_flags(headerJson.value("flags", static_cast<uint16_t>(0)))
	, m_scale()
	, m_origin()
	, m_scaleOrPad(headerJson.value("scaleOrPad", static_cast<int16_t>(0)))
	, m_originOrPad(headerJson.value("originOrPad", static_cast<int16_t>(0)))
	, m_unk1(headerJson.value("unk1", static_cast<uint32_t>(0)))
	, m_colorCount(headerJson.value("colorCount", static_cast<uint32_t>(0)))
{
	m_hasScale = false;
	if (headerJson.contains("scale"))
	{
		const nlohmann::json& scaleJson = headerJson["scale"];
		m_scale.x = scaleJson.value("x", 0.0f);
		m_scale.y = scaleJson.value("y", 0.0f);
		m_scale.z = scaleJson.value("z", 0.0f);
		m_hasScale = true;
	}
	m_hasOrigin = false;
	if (headerJson.contains("m_origin"))
	{
		const nlohmann::json& scaleJson = headerJson["m_origin"];
		m_origin.x = scaleJson.value("x", 0.0f);
		m_origin.y = scaleJson.value("y", 0.0f);
		m_origin.z = scaleJson.value("z", 0.0f);
		m_hasOrigin = true;
	}
	

	if (headerJson.value("animated", false))
	{
		std::filesystem::path gltfPath = modelDir / headerJson.value("gltfFile", std::string());
		std::vector<ModelAnimation> animations;
		bool isAnimated = false;
		if (LoadGLTFHeaderData(gltfPath, materialToTexture, animations, isAnimated))
		{
			m_animations = std::move(animations);
			m_isAnimated = isAnimated;
		}
		else
		{
			printf("ERROR: failed to import %s -- header will have empty geometry\n", gltfPath.string().c_str());
			ModelAnimation empty{};
			empty.frames.push_back({});
			m_animations.push_back(std::move(empty));
			m_isAnimated = false;
		}
	}
	else
	{
		std::string objFile = headerJson.value("objFile", std::string());
		std::string mtlFile = headerJson.value("mtlFile", std::string());
		std::vector<bool> doubleSided;
		//std::vector<Tri> faces = LoadFacesFromObjMtl(modelDir / objFile, modelDir / mtlFile, materialToTexture, doubleSided);
		ModelAnimation staticAnim{};
		//staticAnim.frames.push_back(WrapFaces(faces, doubleSided));
		m_animations.push_back(std::move(staticAnim));
		m_isAnimated = false;
	}

}



void InstanceModelHeader::Clear()
{
	m_name = "NewLOD";
	m_maxDistLOD = 100.0f;
	m_flags = 0;
	m_hasScale = false;
	m_scaleOrPad = 0;
	m_hasOrigin = false;
	m_originOrPad = 0;
	m_unk1 = 0;
	m_colorCount = 0;
	m_animations.clear();
}

const std::string& InstanceModelHeader::GetName() const 
{ 
	return m_name; 
}
std::vector<AnimatedFace>& InstanceModelHeader::GetGeometry()
{
	return m_animations[0].frames[0];;
}

void InstanceModelHeader::LoadOBJ(const std::filesystem::path& objFilename, std::unordered_map<std::string, Texture>& materialToTexture)
{
	std::filesystem::path mtlFilename = objFilename;
	mtlFilename.replace_extension(".mtl");
	//TODO IMPLEMENT
	//m_faces = LoadFacesFromObjMtl(objFilename, mtlFilename, materialToTexture);
	//m_faceDoubleSided = std::vector<bool>(m_faces.size(), false); // TODO IMPLEMENT
	m_hasScale = false;
	m_hasOrigin = false;
}

bool InstanceModelHeader::LoadGLTF(const std::filesystem::path& gltfPath, std::unordered_map<std::string, Texture>& materialToTexture)
{
	std::vector<ModelAnimation> animations;
	bool isAnimated = false;
	if (LoadGLTFHeaderData(gltfPath, materialToTexture, animations, isAnimated))
	{
		m_animations = std::move(animations);
		m_isAnimated = isAnimated;
		return true;
	}
	return false;
}

void InstanceModelHeader::ExportOBJ(const std::filesystem::path& modelDir, std::string baseFileName, std::unordered_map<std::string, Texture>& materialToTexture)
{
	std::unordered_map<std::string, std::vector<size_t>> materialToTris; //material name -> list of triangle index
	for (size_t i = 0; i < m_animations[0].frames[0].size(); i++)
		materialToTris[m_animations[0].frames[0][i].tri.texture].push_back(i);

	// --- .mtl ---
	std::ofstream mtl(modelDir / (baseFileName + ".mtl"));
	if (mtl)
	{
		for (const auto& [matName, indices] : materialToTris)
		{
			if (materialToTexture[matName].IsEmpty()) continue;
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
			const Tri& tri = m_animations[0].frames[0][triIdx].tri;

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


namespace
{
	// Accumulates glTF binary buffer content + JSON accessors/bufferViews.
	// Reuses AppendValue/AppendPadding from SerializeInto's helpers.
	struct GltfBuilder
	{
		std::vector<uint8_t> bin;
		nlohmann::json bufferViews = nlohmann::json::array();
		nlohmann::json accessors = nlohmann::json::array();

		int AddVec3Accessor(const std::vector<Vec3>& data, bool withBounds)
		{
			AppendPadding(bin, 4);
			size_t off = bin.size();
			Vec3 mn(FLT_MAX, FLT_MAX, FLT_MAX), mx(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			for (const Vec3& v : data)
			{
				AppendValue(bin, v.x); AppendValue(bin, v.y); AppendValue(bin, v.z);
				mn.x = std::min(mn.x, v.x); mn.y = std::min(mn.y, v.y); mn.z = std::min(mn.z, v.z);
				mx.x = std::max(mx.x, v.x); mx.y = std::max(mx.y, v.y); mx.z = std::max(mx.z, v.z);
			}
			int bv = (int)bufferViews.size();
			bufferViews.push_back({ {"buffer",0}, {"byteOffset",off}, {"byteLength",bin.size() - off} });
			nlohmann::json acc = { {"bufferView",bv}, {"componentType",5126}, {"count",data.size()}, {"type","VEC3"} };
			if (withBounds) { acc["min"] = { mn.x,mn.y,mn.z }; acc["max"] = { mx.x,mx.y,mx.z }; }
			int idx = (int)accessors.size(); accessors.push_back(acc); return idx;
		}

		int AddVec2Accessor(const std::vector<Vec2>& data)
		{
			AppendPadding(bin, 4);
			size_t off = bin.size();
			for (const Vec2& v : data) { AppendValue(bin, v.x); AppendValue(bin, v.y); }
			int bv = (int)bufferViews.size();
			bufferViews.push_back({ {"buffer",0}, {"byteOffset",off}, {"byteLength",bin.size() - off} });
			int idx = (int)accessors.size();
			accessors.push_back({ {"bufferView",bv}, {"componentType",5126}, {"count",data.size()}, {"type","VEC2"} });
			return idx;
		}

		int AddScalarAccessor(const std::vector<float>& data, bool withBounds)
		{
			AppendPadding(bin, 4);
			size_t off = bin.size();
			float mn = FLT_MAX, mx = -FLT_MAX;
			for (float v : data) { AppendValue(bin, v); mn = std::min(mn, v); mx = std::max(mx, v); }
			int bv = (int)bufferViews.size();
			bufferViews.push_back({ {"buffer",0}, {"byteOffset",off}, {"byteLength",bin.size() - off} });
			nlohmann::json acc = { {"bufferView",bv}, {"componentType",5126}, {"count",data.size()}, {"type","SCALAR"} };
			if (withBounds) { acc["min"] = { mn }; acc["max"] = { mx }; }
			int idx = (int)accessors.size(); accessors.push_back(acc); return idx;
		}
	};

	struct GroupKey
	{
		std::string texture; bool doubleSided;
		bool operator==(const GroupKey& o) const { return texture == o.texture && doubleSided == o.doubleSided; }
	};
	struct GroupKeyHash
	{
		size_t operator()(const GroupKey& k) const { return std::hash<std::string>{}(k.texture) ^ (k.doubleSided ? 1u : 0u); }
	};
}

void InstanceModelHeader::ExportGLTF(const std::filesystem::path& modelDir, const std::string& baseFileName,
	std::unordered_map<std::string, Texture>& materialToTexture) const
{
	const std::vector<AnimatedFace>& baseFaces = m_animations[0].frames[0];

	std::unordered_map<GroupKey, std::vector<size_t>, GroupKeyHash> groups;
	for (size_t i = 0; i < baseFaces.size(); i++)
		groups[{baseFaces[i].tri.texture, baseFaces[i].doubleSided}].push_back(i);

	// Every frame after frame 0, across every animation, becomes one morph
	// target, in a flat global order. Track which slice of that order
	// belongs to which named animation.
	struct AnimRange { size_t firstTarget; size_t frameCount; };
	std::vector<AnimRange> animRanges(m_animations.size());
	std::vector<const std::vector<AnimatedFace>*> targetFrames;
	for (size_t a = 0; a < m_animations.size(); a++)
	{
		animRanges[a] = { targetFrames.size(), m_animations[a].frames.size() };
		for (size_t f = 1; f < m_animations[a].frames.size(); f++)
			targetFrames.push_back(&m_animations[a].frames[f]);
	}
	const size_t numTargets = targetFrames.size();

	GltfBuilder gb;
	nlohmann::json primitives = nlohmann::json::array();
	nlohmann::json materials = nlohmann::json::array();
	nlohmann::json textures = nlohmann::json::array();
	nlohmann::json images = nlohmann::json::array();
	std::unordered_map<std::string, int> textureFileToImageIdx;

	for (auto& [key, faceIndices] : groups)
	{
		std::vector<Vec3> positions, colors; std::vector<Vec2> uvs;
		positions.reserve(faceIndices.size() * 3); colors.reserve(faceIndices.size() * 3); uvs.reserve(faceIndices.size() * 3);
		for (size_t fi : faceIndices)
			for (int c = 0; c < 3; c++)
			{
				positions.push_back(baseFaces[fi].tri.p[c].pos);
				uvs.push_back(baseFaces[fi].tri.p[c].uv);
				const Color& col = baseFaces[fi].tri.p[c].color;
				colors.push_back(Vec3(col.r / 255.0f, col.g / 255.0f, col.b / 255.0f));
			}

		int posAcc = gb.AddVec3Accessor(positions, true);
		int uvAcc = gb.AddVec2Accessor(uvs);
		int colAcc = gb.AddVec3Accessor(colors, false);

		nlohmann::json targetsJson = nlohmann::json::array();
		for (size_t t = 0; t < numTargets; t++)
		{
			std::vector<Vec3> deltas;
			deltas.reserve(faceIndices.size() * 3);
			for (size_t fi : faceIndices)
				for (int c = 0; c < 3; c++)
					deltas.push_back((*targetFrames[t])[fi].tri.p[c].pos - baseFaces[fi].tri.p[c].pos);
			targetsJson.push_back({ {"POSITION", gb.AddVec3Accessor(deltas, true)} });
		}

		int matIdx = (int)materials.size();
		nlohmann::json mat = {
			{"name", key.texture.empty() ? std::string("notex") : key.texture},
			{"doubleSided", key.doubleSided},
			{"pbrMetallicRoughness", { {"baseColorFactor", {1.0,1.0,1.0,1.0}}, {"metallicFactor",0.0}, {"roughnessFactor",1.0} }},
			{"extensions", { {"KHR_materials_unlit", nlohmann::json::object()} }}
		};
		//auto texIt = materialToTexture.find(key.texture);
		//if (!key.texture.empty() && texIt != materialToTexture.end())
		if (!key.texture.empty() && materialToTexture.contains(key.texture) && !materialToTexture[key.texture].IsEmpty())
		{
			//std::filesystem::path src = texIt->second.GetPath();
			std::filesystem::path src = materialToTexture[key.texture].GetPath();
			std::filesystem::copy_file(src, modelDir / src.filename(), std::filesystem::copy_options::overwrite_existing);
			std::string filename = src.filename().string();
			int imgIdx;
			auto it = textureFileToImageIdx.find(filename);
			if (it != textureFileToImageIdx.end()) { imgIdx = it->second; }
			else { imgIdx = (int)images.size(); images.push_back({ {"uri", filename} }); textureFileToImageIdx[filename] = imgIdx; }
			int texIdx = (int)textures.size();
			textures.push_back({ {"source", imgIdx} });
			mat["pbrMetallicRoughness"]["baseColorTexture"] = { {"index", texIdx} };
		}
		materials.push_back(mat);

		nlohmann::json prim = { {"attributes", {{"POSITION",posAcc},{"TEXCOORD_0",uvAcc},{"COLOR_0",colAcc}}}, {"material", matIdx} };
		if (!targetsJson.empty()) { prim["targets"] = targetsJson; }
		primitives.push_back(prim);
	}

	nlohmann::json mesh = { {"name", baseFileName}, {"primitives", primitives} };
	if (numTargets > 0) { mesh["weights"] = std::vector<float>(numTargets, 0.0f); }

	// --- Animations: one per ModelAnimation with real motion. Each channel
	// drives the same shared node.weights array; unrelated animations' slices
	// stay zero outside their own contiguous target range. ---
	nlohmann::json animationsJson = nlohmann::json::array();
	constexpr float ASSUMED_SECONDS_PER_FRAME = 1.0f / 30.0f; // UNCONFIRMED fps -- tune if playback speed matters
	for (size_t a = 0; a < m_animations.size(); a++)
	{
		if (m_animations[a].frames.size() <= 1) { continue; }
		const AnimRange& range = animRanges[a];

		std::vector<float> times, weightsFlat;
		for (size_t f = 0; f < range.frameCount; f++)
		{
			times.push_back(f * ASSUMED_SECONDS_PER_FRAME);
			std::vector<float> w(numTargets, 0.0f);
			if (f > 0) { w[range.firstTarget + (f - 1)] = 1.0f; }
			weightsFlat.insert(weightsFlat.end(), w.begin(), w.end());
		}

		int timeAcc = gb.AddScalarAccessor(times, true);
		int wAcc = gb.AddScalarAccessor(weightsFlat, false);

		animationsJson.push_back({
			{"name", m_animations[a].name.empty() ? ("anim_" + std::to_string(a)) : m_animations[a].name},
			{"samplers", { {{"input",timeAcc},{"output",wAcc},{"interpolation", m_animations[a].interpolated ? "LINEAR" : "STEP"}} }},
			{"channels", { {{"sampler",0},{"target",{{"node",0},{"path","weights"}}}} }}
			});
	}

	nlohmann::json gltf;
	gltf["asset"] = { {"version","2.0"}, {"generator","CTR Instance Model Exporter"} };
	if (!materials.empty()) { gltf["extensionsUsed"] = nlohmann::json::array({ "KHR_materials_unlit" }); }
	gltf["scene"] = 0;
	gltf["scenes"] = nlohmann::json::array({ {{"nodes", {0}}} });
	gltf["nodes"] = nlohmann::json::array({ {{"name", baseFileName}, {"mesh", 0}} });
	gltf["meshes"] = nlohmann::json::array({ mesh });
	if (!materials.empty()) { gltf["materials"] = materials; }
	if (!textures.empty()) { gltf["textures"] = textures; }
	if (!images.empty()) { gltf["images"] = images; }
	if (!animationsJson.empty()) { gltf["animations"] = animationsJson; }
	gltf["accessors"] = gb.accessors;
	gltf["bufferViews"] = gb.bufferViews;
	gltf["buffers"] = nlohmann::json::array({ {{"uri", baseFileName + ".bin"}, {"byteLength", gb.bin.size()}} });

	std::ofstream binFile(modelDir / (baseFileName + ".bin"), std::ios::binary);
	binFile.write(reinterpret_cast<const char*>(gb.bin.data()), gb.bin.size());

	std::ofstream gltfFile(modelDir / (baseFileName + ".gltf"));
	gltfFile << std::setw(2) << gltf << std::endl;
}

nlohmann::json InstanceModelHeader::WriteMetadataJson(const std::string& objFile, const std::string& mtlFile, const std::string& gltfFile) const
{
	nlohmann::json json;
	json["name"] = m_name;
	json["animated"] = m_isAnimated;
	json["objFile"] = objFile;
	json["mtlFile"] = mtlFile;
	json["gltfFile"] = gltfFile;
	json["triangleCount"] = m_animations[0].frames[0].size();
	json["maxDistanceLOD"] = m_maxDistLOD;
	json["flags"] = m_flags;
	if (m_hasScale)
		json["scale"] = { {"x", m_scale.x}, {"y", m_scale.y}, {"z", m_scale.z} };
	json["scaleOrPad"] = m_scaleOrPad;
	if (m_hasOrigin)
		json["origin"] = { {"x", m_origin.x}, {"y", m_origin.y}, {"z", m_origin.z} };
	json["originOrPad"] = m_originOrPad;
	json["unk1"] = m_unk1;
	json["colorCount"] = m_colorCount;
	return json;
}

void InstanceModelHeader::SerializeInto(std::vector<uint8_t>& output, uint32_t modelOffset, size_t headerStructOffset,
	std::unordered_map<std::string, Texture>& materialToTexture,
	std::vector<uint32_t>& outPointerLocations) const
{
	PSX::ModelHeader header{};
	std::memset(header.name, 0, sizeof(header.name));
	std::memcpy(header.name, m_name.data(), std::min(m_name.size(), sizeof(header.name)));
	header.unk1 = m_unk1;
	header.maxDistanceLOD = ConvertFloat(m_maxDistLOD, FP_ONE_GEO);
	header.flags = m_flags;
	header.maybeScaleMaybePadding = m_scaleOrPad;
	header.offStaticDeltaArray = 0; // compressed static vertices unsupported by this encoder -- intentional

	const std::vector<AnimatedFace>& baseFaces = m_animations[0].frames[0];
	auto PreFlip = [](const Vec3& pos) { return Vec3(-pos.x, pos.y, -pos.z); };

	// --- Keep only animations whose every frame still matches base topology
	// (face count). m_animations can, in principle, be edited/imported
	// independently per-entry, so a stale one is possible -- drop it with a
	// warning rather than encode misaligned data. m_animations[0] is always
	// kept as a guaranteed fallback (it defines "base topology" itself). ---
	std::vector<const ModelAnimation*> validAnims;
	for (const ModelAnimation& anim : m_animations)
	{
		bool ok = !anim.frames.empty();
		for (const auto& frame : anim.frames)
		{
			if (frame.size() != baseFaces.size()) { ok = false; break; }
		}
		if (!ok)
		{
			printf("WARNING: header '%s' animation '%s' topology mismatch (expected %zu faces) -- dropped\n",
				m_name.c_str(), anim.name.c_str(), baseFaces.size());
			continue;
		}
		validAnims.push_back(&anim);
	}
	if (validAnims.empty()) { validAnims.push_back(&m_animations[0]); }

	// Real motion iff more than one clip survived, or the single surviving
	// clip has more than one frame -- a single-frame single-clip case is
	// indistinguishable from "static" and is encoded that way.
	const bool effectivelyAnimated = m_isAnimated &&
		(validAnims.size() > 1 || (validAnims.size() == 1 && validAnims[0]->frames.size() > 1));

	// --- Bounding box refit: union of every pose across every surviving
	// animation, so the single shared `scale` fits all of them. ---
	Vec3 preFlipMin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	Vec3 preFlipMax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
	auto ExpandBox = [&](const Vec3& pos)
		{
			Vec3 pf = PreFlip(pos);
			preFlipMin.x = std::min(preFlipMin.x, pf.x); preFlipMax.x = std::max(preFlipMax.x, pf.x);
			preFlipMin.y = std::min(preFlipMin.y, pf.y); preFlipMax.y = std::max(preFlipMax.y, pf.y);
			preFlipMin.z = std::min(preFlipMin.z, pf.z); preFlipMax.z = std::max(preFlipMax.z, pf.z);
		};
	for (const ModelAnimation* anim : validAnims)
		for (const auto& frame : anim->frames)
			for (const AnimatedFace& af : frame)
				for (int c = 0; c < 3; c++)
					ExpandBox(af.tri.p[c].pos);

	if (baseFaces.empty()) { preFlipMin = Vec3(0, 0, 0); preFlipMax = Vec3(0, 0, 0); }

	constexpr float MIN_BOX_SIZE = 1.0f / (2.0f * FP_ONE); // smallest extent guaranteed nonzero after int16 rounding
	Vec3 boxSize(
		std::max(preFlipMax.x - preFlipMin.x, MIN_BOX_SIZE),
		std::max(preFlipMax.y - preFlipMin.y, MIN_BOX_SIZE),
		std::max(preFlipMax.z - preFlipMin.z, MIN_BOX_SIZE)
	);

	header.scale = m_hasScale ? ConvertVec3(m_scale, FP_ONE) : ConvertVec3(boxSize, FP_ONE);
	// Recompute float scale FROM the rounded int16 (not from boxSize/m_scale
	// directly) so quantization below agrees exactly with what the decoder
	// reconstructs.
	Vec3 effScale = ConvertPSXVec3(header.scale, FP_ONE);

	// Encodes one full pose into a tight-fit ModelFrame + vertex bytes,
	// using the shared effScale (always big enough, since it was sized
	// from the union of every pose we'll ever call this with).
	auto EncodePose = [&](const std::vector<AnimatedFace>& pose) -> std::pair<PSX::ModelFrame, std::vector<uint8_t>>
		{
			Vec3 poseMin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
			for (const AnimatedFace& af : pose)
				for (int c = 0; c < 3; c++)
				{
					Vec3 pf = PreFlip(af.tri.p[c].pos);
					poseMin.x = std::min(poseMin.x, pf.x);
					poseMin.y = std::min(poseMin.y, pf.y);
					poseMin.z = std::min(poseMin.z, pf.z);
				}
			if (pose.empty()) { poseMin = Vec3(0, 0, 0); }

			Vec3 originF = SafeDivide(poseMin, effScale);

			PSX::ModelFrame frame{};
			frame.pos = ConvertVec3(originF, 256);
			frame.maybePosMaybePadding = m_originOrPad;
			std::memset(frame.unk16, 0, sizeof(frame.unk16));
			frame.vertexOffset = sizeof(PSX::ModelFrame);

			Vec3 effOrigin = ConvertPSXVec3(frame.pos, 256);

			std::vector<uint8_t> vertexBytes;
			vertexBytes.reserve(pose.size() * 9);
			for (const AnimatedFace& af : pose)
			{
				for (int pushOrder = 0; pushOrder < 3; pushOrder++)
				{
					int cornerIdx = 2 - pushOrder; // matches command push order below
					uint8_t bytes[3];
					EncodeVertexBytes(af.tri.p[cornerIdx].pos, effScale, effOrigin, bytes);
					vertexBytes.push_back(bytes[0]);
					vertexBytes.push_back(bytes[1]);
					vertexBytes.push_back(bytes[2]);
				}
			}
			return { frame, std::move(vertexBytes) };
		};

	// --- Command list: topology/color/texture/doubleSided from baseFaces
	// only -- identical across every frame of every animation by construction. ---
	std::vector<PSX::InstDrawCommand> commands;
	std::vector<PSX::TextureLayout> layouts;
	std::vector<uint32_t> colorPalette;
	std::unordered_map<uint32_t, uint32_t> colorLookup;
	bool warnedColorOverflow = false;
	bool warnedTexOverflow = false;

	for (const AnimatedFace& af : baseFaces)
	{
		const Tri& tri = af.tri;
		uint32_t colorIdx[3];
		colorIdx[2] = GetOrAddColorIndex(colorPalette, colorLookup, tri.p[0].color, m_name, warnedColorOverflow);
		colorIdx[1] = GetOrAddColorIndex(colorPalette, colorLookup, tri.p[1].color, m_name, warnedColorOverflow);
		colorIdx[0] = GetOrAddColorIndex(colorPalette, colorLookup, tri.p[2].color, m_name, warnedColorOverflow);

		uint32_t texCoordIndex = 0;
		// Todo : Make layout uniques, can be deduped.
		if (materialToTexture.contains(tri.texture) && !materialToTexture[tri.texture].IsEmpty())
		{
			Vec2 centroid(
				(tri.p[0].uv.x + tri.p[1].uv.x + tri.p[2].uv.x) / 3.0f,
				(tri.p[0].uv.y + tri.p[1].uv.y + tri.p[2].uv.y) / 3.0f
			);
			QuadUV quadUV = { tri.p[2].uv, tri.p[1].uv, tri.p[0].uv, centroid };
			layouts.push_back(materialToTexture[tri.texture].Serialize(quadUV));

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
				texCoordIndex = static_cast<uint32_t>(layouts.size());
			}
		}

		for (int cmdSlot = 0; cmdSlot < 3; cmdSlot++)
		{
			PSX::InstDrawCommand cmd{};
			cmd.stackWriteLocationIndex = 87; // safe: never emits readNextVertFromStackIndexFlag=1, so never read back
			cmd.readNextVertFromStackIndexFlag = 0;
			cmd.resetFlag = (cmdSlot == 0) ? 1 : 0;
			cmd.colorCoordIndex = colorIdx[cmdSlot];
			cmd.texCoordIndex = texCoordIndex;
			cmd.colorFromScratchpadOrRamFlag = static_cast<uint32_t>(materialToTexture[tri.texture].IsEmpty()); // TODO : Verify other spot where texture can be default
			cmd.noBackfaceFlag = af.doubleSided ? 0 : 1; 
			commands.push_back(cmd);
		}
	}

	if (m_colorCount > 63)
	{
		while (colorPalette.size() < 64) { colorPalette.push_back(0u); }
	}

	const size_t commandListOffset = output.size();
	AppendValue(output, static_cast<uint32_t>(colorPalette.size()));
	for (const PSX::InstDrawCommand& cmd : commands) { AppendValue(output, cmd); }
	PSX::InstDrawCommand terminator{};
	terminator.command = 0xFFFFFFFF;
	AppendValue(output, terminator);

	// --- Frame data: one static pose, or one ModelAnim block per surviving animation. ---
	if (!effectivelyAnimated)
	{
		auto [frame, vertexBytes] = EncodePose(baseFaces);
		const size_t frameDataOffset = output.size();
		AppendValue(output, frame);
		AppendBytes(output, vertexBytes.data(), vertexBytes.size());
		AppendPadding(output, 4);

		header.offFrameData = static_cast<uint32_t>(modelOffset + frameDataOffset);
		header.numAnimations = 0;
		header.offAnimations = 0;
	}
	else
	{
		header.offFrameData = 0; // per RenderBucket_GetFrame: only read when !animated
		header.numAnimations = static_cast<uint32_t>(validAnims.size());

		std::vector<uint32_t> animBlockOffsets(validAnims.size());
		for (size_t a = 0; a < validAnims.size(); a++)
		{
			const ModelAnimation& anim = *validAnims[a];
			const size_t numStoredFrames = anim.frames.size();

			std::vector<std::pair<PSX::ModelFrame, std::vector<uint8_t>>> encodedFrames;
			encodedFrames.reserve(numStoredFrames);
			for (size_t f = 0; f < numStoredFrames; f++)
			{
				encodedFrames.push_back(EncodePose(anim.frames[f]));
			}

			const size_t payloadBytes = encodedFrames.empty() ? 0 : encodedFrames[0].second.size();
			const size_t frameStride = Align4(sizeof(PSX::ModelFrame) + payloadBytes);
			if (frameStride > 0x7FFF)
			{
				printf("WARNING: header '%s' animation '%s' frameSize 0x%zx exceeds int16_t range\n",
					m_name.c_str(), anim.name.c_str(), frameStride);
			}

			// Prefer the exact original numFrames bit pattern when it still
			// describes what we're about to write (round-trips interpolated
			// parity exactly); recompute only if the data no longer matches
			// (edited frame count/interpolation, or glTF-authored with no
			// raw value to begin with).
			uint16_t numFramesField;
			bool useRaw = anim.hasRawNumFrames;
			if (useRaw)
			{
				uint16_t storedLogical = anim.rawNumFrames & PSX::ANIM_FRAME_COUNT_MASK;
				bool storedInterp = (anim.rawNumFrames & PSX::ANIM_INTERPOLATED_BIT) != 0;
				size_t expectedStoredFrames = storedInterp ? (storedLogical > 0 ? (storedLogical >> 1) + 1 : 0) : storedLogical;
				if (storedInterp != anim.interpolated || expectedStoredFrames != numStoredFrames)
				{
					printf("WARNING: header '%s' animation '%s' no longer matches its original frame metadata -- recomputing numFrames\n",
						m_name.c_str(), anim.name.c_str());
					useRaw = false;
				}
			}
			if (useRaw)
			{
				numFramesField = anim.rawNumFrames;
			}
			else
			{
				uint16_t logicalCount = anim.interpolated
					? static_cast<uint16_t>(numStoredFrames > 0 ? (numStoredFrames - 1) * 2 : 0)
					: static_cast<uint16_t>(numStoredFrames);
				numFramesField = logicalCount | (anim.interpolated ? PSX::ANIM_INTERPOLATED_BIT : 0);
			}

			PSX::ModelAnim animHeader{};
			std::memset(animHeader.name, 0, sizeof(animHeader.name));
			std::memcpy(animHeader.name, anim.name.data(), std::min(anim.name.size(), sizeof(animHeader.name)));
			animHeader.numFrames = numFramesField;
			animHeader.frameSize = static_cast<int16_t>(frameStride);
			animHeader.offDeltaArray = 0; // uncompressed -- always valid, always decodable

			const size_t animBlockOffset = output.size();
			AppendValue(output, animHeader);
			for (const auto& [frame, vertexBytes] : encodedFrames)
			{
				const size_t frameStart = output.size();
				AppendValue(output, frame);
				AppendBytes(output, vertexBytes.data(), vertexBytes.size());
				const size_t written = output.size() - frameStart;
				if (written < frameStride) { output.insert(output.end(), frameStride - written, uint8_t(0)); }
			}
			animBlockOffsets[a] = static_cast<uint32_t>(modelOffset + animBlockOffset);
		}

		const size_t animPtrArrayOffset = output.size();
		for (size_t a = 0; a < animBlockOffsets.size(); a++)
		{
			AppendValue(output, animBlockOffsets[a]);
			outPointerLocations.push_back(static_cast<uint32_t>(modelOffset + animPtrArrayOffset + a * sizeof(uint32_t)));
		}
		header.offAnimations = static_cast<uint32_t>(modelOffset + animPtrArrayOffset);
	}

	// --- Texture layouts + pointer array ---
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
			outPointerLocations.push_back(static_cast<uint32_t>(modelOffset + texLayoutPtrArrayOffset + i * sizeof(uint32_t)));
		}
	}

	// --- Colors ---
	size_t colorsOffset = 0;
	if (!colorPalette.empty())
	{
		colorsOffset = output.size();
		for (uint32_t packed : colorPalette) { AppendValue(output, packed); }
	}

	header.offCommandList = static_cast<uint32_t>(modelOffset + commandListOffset);
	header.offTexLayout = layouts.empty() ? 0 : static_cast<uint32_t>(modelOffset + texLayoutPtrArrayOffset);
	header.offColors = colorPalette.empty() ? 0 : static_cast<uint32_t>(modelOffset + colorsOffset);

	std::memcpy(output.data() + headerStructOffset, &header, sizeof(PSX::ModelHeader));

	// offFrameData and offAnimations are each legitimately 0 depending on
	// effectivelyAnimated -- SaveLEV rebases every registered field
	// unconditionally, so a registered zero would become a bogus non-null
	// pointer. Guarded, as before.
	const uint32_t headerAbsoluteOffset = static_cast<uint32_t>(modelOffset + headerStructOffset);
	outPointerLocations.push_back(CALCULATE_OFFSET(PSX::ModelHeader, offCommandList, headerAbsoluteOffset));
	if (header.offFrameData != 0)
	{
		outPointerLocations.push_back(CALCULATE_OFFSET(PSX::ModelHeader, offFrameData, headerAbsoluteOffset));
	}
	if (header.offAnimations != 0)
	{
		outPointerLocations.push_back(CALCULATE_OFFSET(PSX::ModelHeader, offAnimations, headerAbsoluteOffset));
	}
	if (!layouts.empty())
	{
		outPointerLocations.push_back(CALCULATE_OFFSET(PSX::ModelHeader, offTexLayout, headerAbsoluteOffset));
	}
	if (!colorPalette.empty())
	{
		outPointerLocations.push_back(CALCULATE_OFFSET(PSX::ModelHeader, offColors, headerAbsoluteOffset));
	}
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
	std::vector<std::string> gltfFiles(m_headers.size());

	for (size_t headerID = 0; headerID < m_headers.size(); headerID++)
	{
		InstanceModelHeader& header = m_headers[headerID];
		std::string baseFileName = m_name + "LOD" + std::to_string(headerID);
		header.ExportOBJ(modelDir, baseFileName, materialToTexture);
		header.ExportGLTF(modelDir, baseFileName, materialToTexture);
		objFiles[headerID] = baseFileName + ".obj";
		mtlFiles[headerID] = baseFileName + ".mtl";
		gltfFiles[headerID] = baseFileName + ".gltf";
	}

	nlohmann::json json;
	json["name"] = m_name;
	json["id"] = m_id;
	json["numHeaders"] = m_headers.size();

	nlohmann::json headersArray = nlohmann::json::array();
	for (size_t headerID = 0; headerID < m_headers.size(); headerID++)
	{
		headersArray.push_back(m_headers[headerID].WriteMetadataJson(objFiles[headerID], mtlFiles[headerID], gltfFiles[headerID]));
	}
	json["headers"] = headersArray;

	std::ofstream file(modelDir / "metadata.json");
	file << std::setw(4) << json << std::endl;
	file.close();
}



std::vector<Primitive> InstanceModel::GetGeometry()
{
	/*if (!m_headers.empty())
	{
		std::vector<Tri>& geom = m_headers[0].GetGeometry();
		return std::vector<Primitive>(geom.begin(), geom.end());
	}*/
		 
	return {};
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
	//m_rot.x = -m_rot.x;
	//m_rot.y += 180.0f;
	//m_rot.z = -m_rot.z;
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
	//printf("Instance %s, hE %d, hES %d\n", m_name, hitbox.halfExtent, hitbox.halfExtentSq);
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
	int he = static_cast<int>(hitbox.halfExtent);
	hitbox.halfExtentSq = static_cast<int16_t>(std::min(he * he, 16384));
	hitbox.padding = 0;
	hitbox.offInstDef = insatnceOffset;

	return hitbox;
}