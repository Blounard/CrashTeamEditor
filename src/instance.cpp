#include "instance.h"


InstanceModelHeader::InstanceModelHeader(PSX::ModelHeader modelHeader)
{
	m_name = std::string(modelHeader.name, strnlen(modelHeader.name, sizeof(modelHeader.name)));
	m_maxDistLOD = ConvertFP(modelHeader.maxDistanceLOD, FP_ONE_GEO);
	m_flags = modelHeader.flags;
	m_scale = ConvertPSXVec3(modelHeader.scale, FP_ONE);
	m_texNames.clear();
	m_uvs.clear();
}

InstanceModel::InstanceModel(std::string name, std::vector<uint8_t> rawData)
	: m_name(std::move(name))
	, m_rawData(std::move(rawData))
{
}

void InstanceModel::LoadPSXData(const PSX::Model&, const std::vector<PSX::ModelHeader>&)
{
	m_hasPSXData = true;

}

std::vector<uint8_t> InstanceModel::Serialize(std::unordered_map<std::string, size_t>& modelOffsets, std::vector<ModelTextureForVRM>& modelTexturesInVRAM, std::vector<PSX::TextureLayout> layouts) const
{
	const std::vector<uint8_t>& ctrmodelData = m_rawData;
	size_t modelBaseOffset = modelOffsets[m_name];

	// Parse .ctrmodel to get model data and patch table
	const SH::CtrModel* ctrHeader = reinterpret_cast<const SH::CtrModel*>(ctrmodelData.data());
	const uint8_t* modelDataSrc = ctrmodelData.data() + ctrHeader->modelOffset;
	size_t modelDataSize = ctrHeader->modelPatchTableOffset - ctrHeader->modelOffset;

	const uint32_t* patchTablePtr = reinterpret_cast<const uint32_t*>(ctrmodelData.data() + ctrHeader->modelPatchTableOffset);
	const uint32_t patchCount = *patchTablePtr;
	const uint32_t* patchOffsets = patchTablePtr + 1;

	// Copy the model data
	std::vector<uint8_t> modelData(modelDataSrc, modelDataSrc + modelDataSize);

	// Patch TextureLayouts with new VRAM coordinates
	// Parse Model and ModelHeaders to find TextureLayout arrays
	const PSX::Model* model = reinterpret_cast<const PSX::Model*>(modelData.data());
	const uint32_t modelHeadersOffset = model->offHeaders - ctrHeader->modelOffset;
	const PSX::ModelHeader* modelHeaders = reinterpret_cast<const PSX::ModelHeader*>(modelData.data() + modelHeadersOffset);

	for (uint8_t h = 0; h < model->numHeaders; h++)
	{
		const PSX::ModelHeader& modelHdr = modelHeaders[h];
		if (modelHdr.offTexLayout == 0) { continue; }

		// offTexLayout points to a pointer array, each entry points to a TextureLayout
		uint32_t ptrArrayOffset = modelHdr.offTexLayout - ctrHeader->modelOffset;
		const uint32_t* texLayoutPtrs = reinterpret_cast<const uint32_t*>(modelData.data() + ptrArrayOffset);

		// Count TextureLayouts by finding the first null or out-of-range pointer
		size_t numLayouts = 0;
		while (texLayoutPtrs[numLayouts] != 0 &&
			texLayoutPtrs[numLayouts] >= ctrHeader->modelOffset &&
			texLayoutPtrs[numLayouts] < ctrHeader->modelPatchTableOffset)
		{
			numLayouts++;
		}

		for (size_t i = 0; i < numLayouts; i++)
		{
			uint32_t layoutOffset = texLayoutPtrs[i] - ctrHeader->modelOffset;
			PSX::TextureLayout* layout = reinterpret_cast<PSX::TextureLayout*>(modelData.data() + layoutOffset);
			if (!layouts.empty() && !m_headers.empty() && !m_headers[h].m_textureLayoutID.empty())
			{
				PSX::TextureLayout editedLayout = layouts[m_headers[h].m_textureLayoutID[i]];
				*layout = editedLayout;
				continue;
			}

			// Extract original texpage/clut from layout
			uint8_t origPageX = layout->texPage.x;
			uint8_t origPageY = layout->texPage.y;
			uint8_t origPalX = layout->clut.x;
			uint16_t origPalY = layout->clut.y;

			// Find matching ModelTextureForVRM
			for (const ModelTextureForVRM& tex : modelTexturesInVRAM)
			{
				if (tex.modelName != m_name) { continue; }
				if (!tex.placed) { continue; }
				if (tex.origPageX != origPageX) { continue; }
				if (tex.origPageY != origPageY) { continue; }
				if (tex.origPalX != origPalX) { continue; }
				if (tex.origPalY != origPalY) { continue; }

				// Found matching texture! Update TextureLayout with new coordinates
				// Internal buffer position -> VRAM position: add 512 to X (VRM is placed at VRAM X=512)
				size_t vramX = 512 + tex.imageX;
				size_t vramY = tex.imageY;

				// Calculate new texpage (64x256 pages)
				layout->texPage.x = static_cast<uint16_t>(vramX / 64);
				layout->texPage.y = static_cast<uint16_t>(vramY / 256);
				layout->texPage.blendMode = tex.blendMode;
				layout->texPage.texpageColors = tex.bpp;

				// Calculate new CLUT coords (if indexed)
				if (tex.bpp < 2)
				{
					size_t clutVramX = 512 + tex.clutX;
					size_t clutVramY = tex.clutY;
					layout->clut.x = static_cast<uint16_t>(clutVramX / 16);
					layout->clut.y = static_cast<uint16_t>(clutVramY);
				}

				// Calculate UV adjustment
				// The texture was extracted starting at UV (originU, originV)
				// Now it's placed at position (vramX % 64, vramY % 256) within the new texpage
				// UV coordinates are scaled by BPP: 4bpp=4x, 8bpp=2x, 16bpp=1x
				int uvStretch = (tex.bpp == 0) ? 4 : (tex.bpp == 1) ? 2 : 1;
				int newOriginU = static_cast<int>((vramX % 64) * uvStretch);
				int newOriginV = static_cast<int>(vramY % 256);
				int deltaU = newOriginU - tex.originU;
				int deltaV = newOriginV - tex.originV;



				// Adjust all UV coordinates
				layout->u0 = static_cast<uint8_t>(layout->u0 + deltaU);
				layout->v0 = static_cast<uint8_t>(layout->v0 + deltaV);
				layout->u1 = static_cast<uint8_t>(layout->u1 + deltaU);
				layout->v1 = static_cast<uint8_t>(layout->v1 + deltaV);
				layout->u2 = static_cast<uint8_t>(layout->u2 + deltaU);
				layout->v2 = static_cast<uint8_t>(layout->v2 + deltaV);
				layout->u3 = static_cast<uint8_t>(layout->u3 + deltaU);
				layout->v3 = static_cast<uint8_t>(layout->v3 + deltaV);

				break; // Found and patched
			}
		}
	}

	// Convert pointers from .ctrmodel format to .lev format
	// .ctrmodel: absolute offsets pointing directly to targets
	// .lev: stored offsets where (stored + 4) = actual file position
	// Since modelBaseOffset is already a stored offset (actual - 4), we just compute:
	// new_stored_offset = modelBaseOffset + relative_offset_within_model
	for (uint32_t i = 0; i < patchCount; i++)
	{
		uint32_t ctrPatchOffset = patchOffsets[i]; // Absolute offset in .ctrmodel where pointer field is
		uint32_t relativeOffset = ctrPatchOffset - ctrHeader->modelOffset; // Relative to model data

		if (relativeOffset + sizeof(uint32_t) <= modelData.size())
		{
			uint32_t* ptrLocation = reinterpret_cast<uint32_t*>(&modelData[relativeOffset]);
			uint32_t ctrPointerValue = *ptrLocation; // Absolute in .ctrmodel, points directly to target

			// Transform to .lev stored offset format
			// Target's relative position within model = ctrPointerValue - ctrModelOffset
			// Target's stored offset in .lev = modelBaseOffset + relative_position
			uint32_t levPointerValue = static_cast<uint32_t>(
				modelBaseOffset + (ctrPointerValue - ctrHeader->modelOffset)
				);
			*ptrLocation = levPointerValue;
		}
	}
	return modelData;
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

std::vector<uint8_t> Instance::Serialize() const
{
	PSX::InstDef inst = {};
	std::memset(inst.name, 0, sizeof(inst.name));
	std::memcpy(inst.name, m_name.data(), std::min(m_name.size(), sizeof(inst.name)));
	inst.offModel = 0; // Set later during SaveLEV
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