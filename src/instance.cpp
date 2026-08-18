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



std::string MakeUniqueMaterialName(const std::string& baseName, const std::unordered_map<std::string, Texture>& materialToTexture)
{
	if (!materialToTexture.contains(baseName)) { return baseName; }
	int suffix = 1;
	std::string candidate;
	do
	{
		candidate = baseName + "_" + std::to_string(suffix);
		suffix++;
	} while (materialToTexture.contains(candidate));
	return candidate;
}



namespace // Load GLTF
{


	static std::vector<float> ReadFloatAccessorFlat(const tinygltf::Model& model, int accessorIdx, int numComponents)
	{
		const tinygltf::Accessor& acc = model.accessors[accessorIdx];
		std::vector<float> out(acc.count * numComponents, 0.0f);

		if (acc.bufferView >= 0)
		{
			const tinygltf::BufferView& bv = model.bufferViews[acc.bufferView];
			const tinygltf::Buffer& buf = model.buffers[bv.buffer];
			size_t stride = bv.byteStride != 0 ? bv.byteStride : sizeof(float) * numComponents;
			const uint8_t* base = buf.data.data() + bv.byteOffset + acc.byteOffset;
			for (size_t i = 0; i < acc.count; i++)
			{
				const float* f = reinterpret_cast<const float*>(base + i * stride);
				for (int c = 0; c < numComponents; c++) { out[i * numComponents + c] = f[c]; }
			}
		}

		if (acc.sparse.count > 0)
		{
			const auto& sparse = acc.sparse;
			const tinygltf::BufferView& idxBv = model.bufferViews[sparse.indices.bufferView];
			const tinygltf::Buffer& idxBuf = model.buffers[idxBv.buffer];
			const uint8_t* idxBase = idxBuf.data.data() + idxBv.byteOffset + sparse.indices.byteOffset;

			const tinygltf::BufferView& valBv = model.bufferViews[sparse.values.bufferView];
			const tinygltf::Buffer& valBuf = model.buffers[valBv.buffer];
			const uint8_t* valBase = valBuf.data.data() + valBv.byteOffset + sparse.values.byteOffset;

			for (int s = 0; s < sparse.count; s++)
			{
				uint32_t targetIndex;
				switch (sparse.indices.componentType)
				{
				case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:  targetIndex = idxBase[s]; break;
				case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: targetIndex = reinterpret_cast<const uint16_t*>(idxBase)[s]; break;
				case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:   targetIndex = reinterpret_cast<const uint32_t*>(idxBase)[s]; break;
				default: continue;
				}
				const float* v = reinterpret_cast<const float*>(valBase + s * numComponents * sizeof(float));
				for (int c = 0; c < numComponents; c++) { out[targetIndex * numComponents + c] = v[c]; }
			}
		}
		return out;
	}


	static std::vector<Vec3> ReadVec3Accessor(const tinygltf::Model& model, int accessorIdx)
	{
		std::vector<float> flat = ReadFloatAccessorFlat(model, accessorIdx, 3);
		std::vector<Vec3> out(flat.size() / 3);
		for (size_t i = 0; i < out.size(); i++) { out[i] = Vec3(flat[i * 3 + 0], flat[i * 3 + 1], flat[i * 3 + 2]); }
		return out;
	}

	static std::vector<Vec2> ReadVec2Accessor(const tinygltf::Model& model, int accessorIdx)
	{
		std::vector<float> flat = ReadFloatAccessorFlat(model, accessorIdx, 2);
		std::vector<Vec2> out(flat.size() / 2);
		for (size_t i = 0; i < out.size(); i++) { out[i] = Vec2(flat[i * 2 + 0], flat[i * 2 + 1]); }
		return out;
	}

	static std::vector<float> ReadScalarAccessor(const tinygltf::Model& model, int accessorIdx)
	{
		return ReadFloatAccessorFlat(model, accessorIdx, 1);
	}

	static std::vector<Vec3> ReadColorAccessor(const tinygltf::Model& model, int accessorIdx)
	{
		const tinygltf::Accessor& acc = model.accessors[accessorIdx];
		if (acc.bufferView < 0)
		{
			printf("WARNING: COLOR_0 accessor has no bufferView (sparse-only colors unsupported) -- using default gray\n");
			return std::vector<Vec3>(acc.count, Vec3(0.5f, 0.5f, 0.5f));
		}
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
		if (acc.bufferView < 0)
		{
			printf("ERROR: index accessor has no bufferView -- unsupported, treating primitive as empty\n");
			return {};
		}
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



	struct ChannelSampler
	{
		std::vector<float> times;
		std::vector<float> values; // flat, numComponents per sample
		int numComponents = 3;
		std::string interpolation = "LINEAR"; // STEP, LINEAR; CUBICSPLINE unsupported (see below)
		bool valid = false;
	};

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
		return ReadFloatAccessorFlat(model, accessorIdx, 4);
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
			cs.values = ReadFloatAccessorFlat(model, sampler.output, numComponents);
			cs.interpolation = sampler.interpolation;
			cs.valid = true;
			return cs;
		}
		return cs;
	}


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


	std::vector<ModelAnimation> BuildAnimationsFromTRS(const tinygltf::Model& model, const std::vector<PrimData>& prims,
		int meshNodeIdx, const Mat4& ancestorTransform,
		const std::function<std::vector<Tri>(const std::function<Vec3(size_t, size_t)>&)>& buildFaces)
	{
		std::vector<ModelAnimation> out;
		const tinygltf::Node& restNode = model.nodes[meshNodeIdx];

		for (const tinygltf::Animation& anim : model.animations)
		{
			ChannelSampler tCh = ReadChannelSampler(model, anim, "translation", meshNodeIdx, 3);
			ChannelSampler rCh = ReadChannelSampler(model, anim, "rotation", meshNodeIdx, 4);
			ChannelSampler sCh = ReadChannelSampler(model, anim, "scale", meshNodeIdx, 3);
			if (!tCh.valid && !rCh.valid && !sCh.valid) { continue; } // this Animation doesn't touch our node at all

			const ChannelSampler* refCh = nullptr;
			for (const ChannelSampler* c : { &tCh, &rCh, &sCh })
			{
				if (!c->valid || c->times.empty()) { continue; }
				if (refCh == nullptr || c->times.size() > refCh->times.size()) { refCh = c; }
			}
			if (refCh == nullptr) { continue; }
			const std::vector<float>& sampleTimes = refCh->times;

			if (sampleTimes.size() > 4096)
			{
				printf("WARNING: animation '%s' has %zu sampled frames, which is unusually large -- check the export's bake settings\n",
					anim.name.c_str(), sampleTimes.size());
			}

			ModelAnimation animation{};
			animation.name = !anim.name.empty() ? anim.name : ("anim_" + std::to_string(out.size()));
			animation.interpolated = (refCh->interpolation == "LINEAR");
			animation.hasRawNumFrames = false;

			std::vector<double> restT = restNode.translation, restR = restNode.rotation, restS = restNode.scale;

			for (size_t f = 0; f < sampleTimes.size(); f++)
			{
				float t = sampleTimes[f];

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
		Mat4 ancestorTransform{};
		if (!FindMeshNode(model, meshIdx, meshNodeIdx, ancestorTransform))
		{
			if (model.meshes.empty()) { printf("ERROR: no mesh in %s\n", gltfPath.string().c_str()); return false; }
			meshIdx = 0; // no scene graph present -- fall back to the first mesh, identity transform
		}
		Mat4 fullTransform = ancestorTransform;
		if (meshNodeIdx >= 0)
		{
			fullTransform = Mat4Multiply(ancestorTransform, Mat4FromNode(model.nodes[meshNodeIdx]));
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
				pd.basePositions.push_back(Mat4TransformPoint(fullTransform, rawPositions[vi]));

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
						std::string filename;
						if (!tinygltf::URIDecode(model.images[imgIdx].uri, &filename, nullptr))
						{
							filename = model.images[imgIdx].uri;
						}
						std::filesystem::path pngPath = gltfDir / filename;
						std::string baseName = !mat.name.empty() ? mat.name : std::filesystem::path(filename).stem().string();
						std::string globalName = MakeUniqueMaterialName(baseName, materialToTexture);
						if (!std::filesystem::exists(pngPath))
						{
							printf("WARNING: texture file not found: %s\n", pngPath.string().c_str());
						}
						else
						{
							materialToTexture.emplace(globalName, Texture(pngPath));
							pd.materialName = globalName;
						}
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
				for (Vec3& d : deltas) { d = Mat4TransformVector(fullTransform, d); }
				pd.targetDeltasByVertex.push_back(std::move(deltas));
			}

			prims.push_back(std::move(pd));
		}

		if (prims.empty()) { printf("ERROR: no usable triangle primitives in %s\n", gltfPath.string().c_str()); return false; }
		if (globalNumTargets == SIZE_MAX) { globalNumTargets = 0; }

		auto BuildFaces = [&](const std::function<Vec3(size_t primIdx, size_t corner)>& getPos) -> std::vector<Tri>
			{
				std::vector<Tri> faces;
				for (size_t p = 0; p < prims.size(); p++)
				{
					const PrimData& pd = prims[p];
					for (size_t c = 0; c + 2 < pd.basePositions.size(); c += 3)
					{
						Tri tri;
						tri.doubleSided = pd.doubleSided;
						tri.texture = pd.materialName;
						for (int k = 0; k < 3; k++)
						{
							tri.p[k].pos = getPos(p, c + k);
							tri.p[k].uv = pd.uvs[c + k];
							const Vec3& col = pd.colors[c + k];
							tri.p[k].color = Color(
								static_cast<unsigned char>(std::clamp(col.x, 0.0f, 1.0f) * 255.0f),
								static_cast<unsigned char>(std::clamp(col.y, 0.0f, 1.0f) * 255.0f),
								static_cast<unsigned char>(std::clamp(col.z, 0.0f, 1.0f) * 255.0f));
						}
						Vec3 e1 = tri.p[1].pos - tri.p[0].pos;
						Vec3 e2 = tri.p[2].pos - tri.p[0].pos;
						Vec3 n = e1.Cross(e2);
						if (n.LengthSquared() > 0.0001f) { n.Normalize(); }
						tri.p[0].normal = tri.p[1].normal = tri.p[2].normal = n;
						faces.push_back(tri);
					}
				}
				return faces;
			};

		std::vector<Tri> baseFaces = BuildFaces([&](size_t p, size_t c) { return prims[p].basePositions[c]; });

		auto BuildBlendedFrame = [&](const float* weights, size_t numWeights) -> std::vector<Tri>
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
				std::vector<ModelAnimation> trsAnims = BuildAnimationsFromTRS(model, prims, meshNodeIdx, ancestorTransform, BuildFaces);
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



InstanceModelHeader::InstanceModelHeader(PSX::ModelHeader& modelHeader, PSX::ModelFrame& baseFrame, uint32_t colorCount,
	std::vector<ModelAnimation> animations, bool isAnimated)
{
	m_name = std::string(modelHeader.name, strnlen(modelHeader.name, sizeof(modelHeader.name)));
	m_maxDistLOD = ConvertFP(modelHeader.maxDistanceLOD, FP_ONE_GEO);
	m_flags = modelHeader.flags;
	m_scale = ConvertPSXVec3(modelHeader.scale, FP_ONE_MODEL_SCALE);
	m_scaleOrPad = modelHeader.maybeScaleMaybePadding;
	m_unk1 = modelHeader.unk1;
	m_bannerWave = colorCount > 63;
	m_hasScale = true;
	m_animations = std::move(animations);
	m_isAnimated = isAnimated;
}


InstanceModelHeader::InstanceModelHeader(const nlohmann::json& headerJson, const std::filesystem::path& modelDir,
	std::unordered_map<std::string, Texture>& materialToTexture)
	: m_name(headerJson.value("name", std::string()))
	, m_maxDistLOD(headerJson.value("maxDistanceLOD", 0.0f))
	, m_flags(headerJson.value("flags", static_cast<uint16_t>(0)))
	, m_scale()
	, m_scaleOrPad(headerJson.value("scaleOrPad", static_cast<int16_t>(0)))
	, m_unk1(headerJson.value("unk1", static_cast<uint32_t>(0)))
	, m_bannerWave(headerJson.value("bannerWave", false))
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
	
	
	std::string gltfFilename = headerJson.value("gltfFile", std::string());
	printf("filename : %s\n", gltfFilename.c_str());
	std::filesystem::path gltfPath = modelDir / gltfFilename;
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



void InstanceModelHeader::Clear()
{
	m_name = "NewLOD";
	m_maxDistLOD = 100.0f;
	m_flags = 0;
	m_hasScale = false;
	m_scaleOrPad = 0;
	m_unk1 = 0;
	m_bannerWave = false;
	m_animations.clear();
}

const std::string& InstanceModelHeader::GetName() const 
{ 
	return m_name; 
}
std::vector<Tri>& InstanceModelHeader::GetGeometry()
{
	return m_animations[0].frames[0];
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



void InstanceModelHeader::ExportGLTF(const std::filesystem::path& modelDir, const std::string& baseFileName,
	std::unordered_map<std::string, Texture>& materialToTexture) const
{
	const std::vector<Tri>& baseFaces = m_animations[0].frames[0];
	std::map<std::pair<std::string, bool>, std::vector<size_t>> groups;
	for (size_t i = 0; i < baseFaces.size(); i++)
		groups[{baseFaces[i].texture, baseFaces[i].doubleSided}].push_back(i);

	struct AnimRange { size_t firstTarget; size_t frameCount; };
	std::vector<AnimRange> animRanges(m_animations.size());
	std::vector<const std::vector<Tri>*> targetFrames;
	for (size_t a = 0; a < m_animations.size(); a++)
	{
		animRanges[a] = { targetFrames.size(), m_animations[a].frames.size() };
		for (size_t f = 1; f < m_animations[a].frames.size(); f++)
			targetFrames.push_back(&m_animations[a].frames[f]);
	}
	const size_t numTargets = targetFrames.size();

	tinygltf::Model model;
	model.asset.version = "1.0";
	model.asset.generator = "CTE Instance Model Exporter";

	std::vector<uint8_t> bin;
	auto AddAccessor = [&](const float* data, size_t elemCount, int elemFloats, int type, bool withBounds) -> int
		{
			while (bin.size() % 4 != 0) { bin.push_back(0); }
			size_t byteOffset = bin.size();
			size_t totalFloats = elemCount * static_cast<size_t>(elemFloats);
			const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
			bin.insert(bin.end(), bytes, bytes + totalFloats * sizeof(float));

			tinygltf::BufferView bv;
			bv.buffer = 0;
			bv.byteOffset = byteOffset;
			bv.byteLength = totalFloats * sizeof(float);
			int bvIdx = static_cast<int>(model.bufferViews.size());
			model.bufferViews.push_back(bv);

			tinygltf::Accessor acc;
			acc.bufferView = bvIdx;
			acc.byteOffset = 0;
			acc.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
			acc.count = elemCount;
			acc.type = type;
			if (withBounds)
			{
				std::vector<double> mn(elemFloats, DBL_MAX), mx(elemFloats, -DBL_MAX);
				for (size_t i = 0; i < elemCount; i++)
					for (int c = 0; c < elemFloats; c++)
					{
						double v = data[i * elemFloats + c];
						mn[c] = std::min(mn[c], v);
						mx[c] = std::max(mx[c], v);
					}
				acc.minValues = mn;
				acc.maxValues = mx;
			}
			int idx = static_cast<int>(model.accessors.size());
			model.accessors.push_back(acc);
			return idx;
		};

	std::unordered_map<std::string, int> textureFileToImageIdx;
	tinygltf::Mesh mesh;
	mesh.name = baseFileName;

	for (auto& [key, faceIndices] : groups)
	{
		const std::string& texName = key.first;
		bool doubleSided = key.second;

		std::vector<Vec3> positions, colors;
		std::vector<Vec2> uvs;
		positions.reserve(faceIndices.size() * 3);
		uvs.reserve(faceIndices.size() * 3);
		colors.reserve(faceIndices.size() * 3);
		for (size_t fi : faceIndices)
			for (int c = 0; c < 3; c++)
			{
				positions.push_back(baseFaces[fi].p[c].pos);
				uvs.push_back(baseFaces[fi].p[c].uv);
				const Color& col = baseFaces[fi].p[c].color;
				colors.push_back(Vec3(col.r / 255.0f, col.g / 255.0f, col.b / 255.0f));
			}

		tinygltf::Primitive prim;
		prim.mode = TINYGLTF_MODE_TRIANGLES;
		prim.attributes["POSITION"] = AddAccessor(&positions[0].x, positions.size(), 3, TINYGLTF_TYPE_VEC3, true);
		prim.attributes["TEXCOORD_0"] = AddAccessor(&uvs[0].x, uvs.size(), 2, TINYGLTF_TYPE_VEC2, false);
		prim.attributes["COLOR_0"] = AddAccessor(&colors[0].x, colors.size(), 3, TINYGLTF_TYPE_VEC3, false);

		for (size_t t = 0; t < numTargets; t++)
		{
			std::vector<Vec3> deltas;
			deltas.reserve(faceIndices.size() * 3);
			for (size_t fi : faceIndices)
				for (int c = 0; c < 3; c++)
					deltas.push_back((*targetFrames[t])[fi].p[c].pos - baseFaces[fi].p[c].pos);
			std::map<std::string, int> target;
			target["POSITION"] = AddAccessor(&deltas[0].x, deltas.size(), 3, TINYGLTF_TYPE_VEC3, true);
			prim.targets.push_back(target);
		}

		tinygltf::Material mat;
		mat.name = texName.empty() ? "notex" : texName;
		mat.doubleSided = doubleSided;
		mat.pbrMetallicRoughness.baseColorFactor = { 1.0, 1.0, 1.0, 1.0 };
		mat.pbrMetallicRoughness.metallicFactor = 0.0;
		mat.pbrMetallicRoughness.roughnessFactor = 1.0;
		mat.extensions["KHR_materials_unlit"] = tinygltf::Value(std::map<std::string, tinygltf::Value>());

		auto texIt = materialToTexture.find(texName);
		if (!texName.empty() && texIt != materialToTexture.end() && !texIt->second.IsEmpty())
		{
			std::filesystem::path src = texIt->second.GetPath();
			std::filesystem::copy_file(src, modelDir / src.filename(), std::filesystem::copy_options::overwrite_existing);
			std::string filename = src.filename().string();

			int imgIdx;
			auto imgIt = textureFileToImageIdx.find(filename);
			if (imgIt != textureFileToImageIdx.end()) { imgIdx = imgIt->second; }
			else
			{
				tinygltf::Image img;
				img.uri = filename;
				imgIdx = static_cast<int>(model.images.size());
				model.images.push_back(img);
				textureFileToImageIdx[filename] = imgIdx;
			}
			tinygltf::Texture tex;
			tex.source = imgIdx;
			int texIdx = static_cast<int>(model.textures.size());
			model.textures.push_back(tex);
			mat.pbrMetallicRoughness.baseColorTexture.index = texIdx;
		}

		prim.material = static_cast<int>(model.materials.size());
		model.materials.push_back(mat);
		mesh.primitives.push_back(prim);
	}

	if (numTargets > 0) { mesh.weights = std::vector<double>(numTargets, 0.0); }
	model.meshes.push_back(mesh);

	// --- Animations: one per ModelAnimation with real motion, each driving
	// the shared node's weights array over its own contiguous target range. ---
	constexpr float ASSUMED_SECONDS_PER_FRAME = 1.0f / 30.0f; 
	for (size_t a = 0; a < m_animations.size(); a++)
	{
		if (m_animations[a].frames.size() <= 1) { continue; }
		const AnimRange& range = animRanges[a];

		std::vector<float> times;
		std::vector<float> weightsFlat;
		for (size_t f = 0; f < range.frameCount; f++)
		{
			times.push_back(f * ASSUMED_SECONDS_PER_FRAME);
			std::vector<float> w(numTargets, 0.0f);
			if (f > 0) { w[range.firstTarget + (f - 1)] = 1.0f; }
			weightsFlat.insert(weightsFlat.end(), w.begin(), w.end());
		}

		tinygltf::AnimationSampler sampler;
		sampler.input = AddAccessor(times.data(), times.size(), 1, TINYGLTF_TYPE_SCALAR, true);
		sampler.output = AddAccessor(weightsFlat.data(), weightsFlat.size(), 1, TINYGLTF_TYPE_SCALAR, false);
		sampler.interpolation = m_animations[a].interpolated ? "LINEAR" : "STEP";

		tinygltf::AnimationChannel channel;
		channel.sampler = 0;
		channel.target_node = 0;
		channel.target_path = "weights";

		tinygltf::Animation anim;
		anim.name = m_animations[a].name.empty() ? ("anim_" + std::to_string(a)) : m_animations[a].name;
		anim.samplers.push_back(sampler);
		anim.channels.push_back(channel);
		model.animations.push_back(anim);
	}

	tinygltf::Node node;
	node.name = baseFileName;
	node.mesh = 0;
	model.nodes.push_back(node);

	tinygltf::Scene scene;
	scene.nodes.push_back(0);
	model.scenes.push_back(scene);
	model.defaultScene = 0;

	if (!model.materials.empty()) { model.extensionsUsed.push_back("KHR_materials_unlit"); }

	tinygltf::Buffer buffer;
	buffer.data = std::move(bin);
	model.buffers.push_back(buffer);

	tinygltf::TinyGLTF writer;
	bool ok = writer.WriteGltfSceneToFile(&model, (modelDir / (baseFileName + ".gltf")).string(),
		/*embedImages=*/false, /*embedBuffers=*/false,
		/*prettyPrint=*/true, /*writeBinary=*/false);
	if (!ok)
	{
		printf("ERROR: tinygltf failed to write %s\n", (modelDir / (baseFileName + ".gltf")).string().c_str());
	}
}

nlohmann::json InstanceModelHeader::WriteMetadataJson(const std::string& gltfFile) const
{
	nlohmann::json json;
	json["name"] = m_name;
	json["animated"] = m_isAnimated;
	json["gltfFile"] = gltfFile;
	json["triangleCount"] = m_animations[0].frames[0].size();
	json["maxDistanceLOD"] = m_maxDistLOD;
	json["flags"] = m_flags;
	if (m_hasScale)
		json["scale"] = { {"x", m_scale.x}, {"y", m_scale.y}, {"z", m_scale.z} };
	json["scaleOrPad"] = m_scaleOrPad;
	json["unk1"] = m_unk1;
	json["bannerWave"] = m_bannerWave;
	return json;
}


namespace // SerializeInto
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

}



void InstanceModelHeader::SerializeInto(std::vector<uint8_t>& output, uint32_t modelOffset, size_t headerStructOffset,
	std::unordered_map<std::string, Texture>& materialToTexture,
	std::vector<uint32_t>& outPointerLocations) const
{
	PSX::ModelHeader header{};
	std::memset(header.name, 0, sizeof(header.name));
	std::memcpy(header.name, m_name.data(), std::min(m_name.size(), sizeof(header.name)));
	header.unk1 = m_unk1;
	if (m_maxDistLOD < 0.0f) header.maxDistanceLOD = 0xFFFF;
	else header.maxDistanceLOD = ConvertFloat(m_maxDistLOD, FP_ONE_GEO);
	header.flags = m_flags;
	header.maybeScaleMaybePadding = m_scaleOrPad;
	header.offStaticDeltaArray = 0; // compressed static vertices unsupported by this encoder -- intentional

	const std::vector<Tri>& baseFaces = m_animations[0].frames[0];
	
	auto PreFlip = [](const Vec3& pos) { return Vec3(pos.x, pos.y, pos.z); };

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
			for (const Tri& tri : frame)
				for (int c = 0; c < 3; c++)
					ExpandBox(tri.p[c].pos);

	if (baseFaces.empty()) { preFlipMin = Vec3(0, 0, 0); preFlipMax = Vec3(0, 0, 0); }

	constexpr float MIN_BOX_SIZE = 1.0f / (2.0f * FP_ONE_MODEL_SCALE); // smallest extent guaranteed nonzero after int16 rounding
	Vec3 boxSize(
		std::max(preFlipMax.x - preFlipMin.x, MIN_BOX_SIZE),
		std::max(preFlipMax.y - preFlipMin.y, MIN_BOX_SIZE),
		std::max(preFlipMax.z - preFlipMin.z, MIN_BOX_SIZE)
	);

	// TODO MAKE SURE m_scale IS NEVER 0
	header.scale = m_hasScale ? ConvertVec3(m_scale, FP_ONE_MODEL_SCALE) : ConvertVec3(boxSize, FP_ONE_MODEL_SCALE);
	// Recompute float scale FROM the rounded int16 (not from boxSize/m_scale
	// directly) so quantization below agrees exactly with what the decoder
	// reconstructs.
	Vec3 effScale = ConvertPSXVec3(header.scale, FP_ONE_MODEL_SCALE);

	// Encodes one full pose into a tight-fit ModelFrame + vertex bytes,
	// using the shared effScale (always big enough, since it was sized
	// from the union of every pose we'll ever call this with).
	auto EncodePose = [&](const std::vector<Tri>& pose) -> std::vector<uint8_t>
		{
			Vec3 poseMin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
			for (const Tri& tri : pose)
				for (int c = 0; c < 3; c++)
				{
					Vec3 pf = PreFlip(tri.p[c].pos);
					poseMin.x = std::min(poseMin.x, pf.x);
					poseMin.y = std::min(poseMin.y, pf.y);
					poseMin.z = std::min(poseMin.z, pf.z);
				}
			if (pose.empty()) { poseMin = Vec3(0, 0, 0); }

			Vec3 originF;
			originF.x = std::fabs(effScale.x) < 0.0001f ? 0.0f : poseMin.x / effScale.x;
			originF.y = std::fabs(effScale.y) < 0.0001f ? 0.0f : poseMin.y / effScale.y;
			originF.z = std::fabs(effScale.z) < 0.0001f ? 0.0f : poseMin.z / effScale.z;

			std::vector<uint8_t> res;
			PSX::ModelFrame frame{};
			frame.pos = ConvertVec3(originF, FP_ONE_MODEL_ORIGIN);
			frame.maybePosMaybePadding = 0;
			std::memset(frame.unk16, 0, sizeof(frame.unk16));
			frame.vertexOffset = sizeof(PSX::ModelFrame);
			AppendValue(res, frame);

			Vec3 effOrigin = ConvertPSXVec3(frame.pos, FP_ONE_MODEL_ORIGIN);
			for (const Tri& tri : pose)
			{
				for (int pushOrder = 0; pushOrder < 3; pushOrder++)
				{
					int cornerIdx = 2 - pushOrder; // matches command push order below
					PSX::Vec3b vert = ConvertVec3b((tri.p[cornerIdx].pos / effScale) - effOrigin, 255) ;
					AppendValue(res, vert);
				}
			}
			AppendPadding(res, 4);

			return res;
		};

	// Command list: topology/color/texture/doubleSided
	std::vector<PSX::InstDrawCommand> commands;
	std::vector<PSX::TextureLayout> layouts;
	std::unordered_map<PSX::TextureLayout, uint32_t> layoutLookup; // TextureLayout -> index into layouts
	std::vector<PSX::Color> colorPalette;
	std::unordered_map<PSX::Color, uint32_t> colorLookup;

	auto GetColorIndex = [&](const Color col) -> uint32_t
		{
			PSX::Color psxCol = ConvertColor(col);
			if (!colorLookup.contains(psxCol))
			{
				if (colorPalette.size() > 63)
				{
					printf("WARNING: header '%s' needs more than 63 unique colors; some colors will be approximated\n", m_name.c_str());
					colorLookup[psxCol] = 63;
				}
				else
				{
					colorLookup[psxCol] = static_cast<uint32_t>(colorPalette.size());
					colorPalette.push_back(psxCol);
				}
			}
			return colorLookup[psxCol];
		};

	for (const Tri& tri : baseFaces)
	{
		uint32_t colorIdx[3];
		colorIdx[2] = GetColorIndex(tri.p[0].color);
		colorIdx[1] = GetColorIndex(tri.p[1].color);
		colorIdx[0] = GetColorIndex(tri.p[2].color);

		uint32_t texCoordIndex = 0;
		bool hasTexture = materialToTexture.contains(tri.texture) && !materialToTexture[tri.texture].IsEmpty();
		if (hasTexture)
		{
			Vec2 centroid(
				(tri.p[0].uv.x + tri.p[1].uv.x + tri.p[2].uv.x) / 3.0f,
				(tri.p[0].uv.y + tri.p[1].uv.y + tri.p[2].uv.y) / 3.0f
			);
			QuadUV quadUV = { tri.p[2].uv, tri.p[1].uv, tri.p[0].uv, centroid };
			PSX::TextureLayout layout = materialToTexture[tri.texture].Serialize(quadUV);

			if (!layoutLookup.contains(layout))
			{
				if (layouts.size() >= 511)
				{
					printf("WARNING: header '%s' needs more than 511 unique texture layouts; reusing last one for the rest\n", m_name.c_str());
					layoutLookup[layout] = static_cast<uint32_t>(511);
				}
				else
				{
					layoutLookup[layout] = static_cast<uint32_t>(layouts.size()) + 1; // 1-index
					layouts.push_back(layout);
				}
			}
			texCoordIndex = layoutLookup[layout];
		}

		for (int cmdSlot = 0; cmdSlot < 3; cmdSlot++)
		{
			PSX::InstDrawCommand cmd{};
			cmd.stackWriteLocationIndex = 87;
			cmd.readNextVertFromStackIndexFlag = 0;
			cmd.resetFlag = (cmdSlot == 0) ? 1 : 0;
			cmd.colorCoordIndex = colorIdx[cmdSlot];
			cmd.texCoordIndex = texCoordIndex;
			cmd.colorFromScratchpadOrRamFlag = static_cast<uint32_t>(!hasTexture);
			cmd.noBackfaceFlag = tri.doubleSided ? 0 : 1;
			commands.push_back(cmd);
		}
	}

	if (m_bannerWave)
	{
		while (colorPalette.size() < 64) { colorPalette.push_back(PSX::Color{}); }
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
		const size_t frameDataOffset = output.size();
		std::vector<uint8_t> encodedFrame = EncodePose(baseFaces);
		AppendBytes(output, encodedFrame.data(), encodedFrame.size());
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

			std::vector<std::vector<uint8_t>> encodedFrames;
			encodedFrames.reserve(numStoredFrames);
			for (size_t f = 0; f < numStoredFrames; f++)
			{
				encodedFrames.push_back(EncodePose(anim.frames[f]));
			}

			const size_t frameStride = encodedFrames.empty() ? 0 : encodedFrames[0].size();
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
			for (const std::vector<uint8_t>& encodedFrame: encodedFrames)
			{
				const size_t frameStart = output.size();
				AppendBytes(output, encodedFrame.data(), encodedFrame.size());
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
		for (PSX::Color psxCol : colorPalette) { AppendValue(output, psxCol); }
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
	m_id = static_cast<ModelId>(json.value("id", static_cast<int16_t>(0)));
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
	m_id = static_cast<ModelId>(model.id);
	m_valid = true;
	m_headers.clear();
}


void InstanceModel::Export(const std::filesystem::path& exportDir, std::unordered_map<std::string, Texture>& materialToTexture)
{
	std::filesystem::path modelDir = exportDir / m_name;
	std::filesystem::create_directories(modelDir);

	std::vector<std::string> gltfFiles(m_headers.size());

	for (size_t headerID = 0; headerID < m_headers.size(); headerID++)
	{
		InstanceModelHeader& header = m_headers[headerID];
		std::string baseFileName = m_name + "LOD" + std::to_string(headerID);
		header.ExportGLTF(modelDir, baseFileName, materialToTexture);
		gltfFiles[headerID] = baseFileName + ".gltf";
	}

	nlohmann::json json;
	json["name"] = m_name;
	json["id"] = static_cast<int16_t>(m_id);
	json["numHeaders"] = m_headers.size();

	nlohmann::json headersArray = nlohmann::json::array();
	for (size_t headerID = 0; headerID < m_headers.size(); headerID++)
	{
		headersArray.push_back(m_headers[headerID].WriteMetadataJson(gltfFiles[headerID]));
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
	model.id = static_cast<int16_t>(m_id);
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
	m_modelID = ModelId::NOFUNC;
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
	m_modelID = static_cast<ModelId>(static_cast<int16_t>(inst.modelID));
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

BoundingBox Instance::ComputeBBox() const 
{
	Vec3 center = Center();
	Vec3 half_ext = Vec3(m_hitbox.halfExtent, m_hitbox.halfExtent, m_hitbox.halfExtent);
	BoundingBox bbox{};
	bbox.min = center - half_ext;
	bbox.max = center + half_ext;
	return bbox;
}

Vec3 Instance::Center() const
{
	return m_pos + Vec3(0.0f, m_hitbox.yOffset, 0.0f);
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