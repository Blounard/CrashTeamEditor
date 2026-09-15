#pragma once

#include "psx_types.h"
#include "quadblock.h"

#include <cstdint>
#include <vector>
#include <filesystem>
#include <unordered_set>
#include <functional>

typedef std::unordered_set<size_t> Shape;

struct RawUV
{
	uint8_t u0, v0, u1, v1, u2, v2, u3, v3;
	RawUV() = default;
	RawUV(const PSX::TextureLayout& layout);
};

struct PixelBounds
{
	uint8_t minU = 255, minV = 255;
	uint8_t maxU = 0, maxV = 0;
	void Update(const RawUV& uvs);
};

struct LayoutKey // 2 PSX::TextureLayout have the same LayoutKey if they use the same vram page and colors. Roughly correspond to materials
{
	uint16_t pageX;
	uint16_t pageY;
	uint16_t bpp;
	uint16_t clutX;
	uint16_t clutY;
	uint16_t blendMode;
	LayoutKey() = default;
	LayoutKey(const PSX::TextureLayout& layout);
	PSX::TextureLayout Serialize(QuadUV uvs, PixelBounds bounds) const;
	bool operator==(const LayoutKey& other) const;
};

namespace std
{
	template<>
	struct hash<LayoutKey>
	{
		size_t operator()(const LayoutKey& key) const;
	};
}

class Texture
{
public:
	enum class BPP
	{
		BPP_4, BPP_8, BPP_16
	};
	Texture() : m_width(0), m_height(0), m_imageX(0), m_imageY(0), m_clutX(0), m_clutY(0), m_blendMode(0), m_semiTransparent(false) {};
	Texture(const std::filesystem::path& path);
	void UpdateTexture(const std::filesystem::path& path);
	Texture::BPP GetBPP() const;
	int GetWidth() const;
	int GetVRAMWidth() const;
	int GetHeight() const;
	uint16_t GetBlendMode() const;
	const std::filesystem::path& GetPath() const;
	bool IsEmpty() const;
	const std::vector<uint16_t>& GetImage() const;
	const std::vector<uint16_t>& GetClut() const;
	size_t GetImageX() const;
	size_t GetImageY() const;
	size_t GetCLUTX() const;
	size_t GetCLUTY() const;
	bool IsSemiTransparent() const;
	void SetImageCoords(size_t x, size_t y);
	void SetCLUTCoords(size_t x, size_t y);
	void SetBlendMode(uint16_t mode);
	PSX::TextureLayout Serialize(const QuadUV& uvs) const;
	bool CompareEquivalency(const Texture& tex);
	void CopyVRAMAttributes(const Texture& tex);
	bool operator==(const Texture& tex) const;
	bool operator!=(const Texture& tex) const;
	void RenderUI(const std::vector<std::pair<size_t, size_t>>& quadFaces, std::vector<Quadblock>& quadblocks, std::function<void(void)> refreshTextureStores);
	void RenderUI();

private:
	void FillShapes(const std::vector<size_t>& colorIndexes);
	void ClearTexture();
	bool CreateTexture(bool updateBlendMode =  false);
	void ConvertPixels(const std::vector<size_t>& colorIndexes, unsigned indexesPerPixel);

private:
	int m_width, m_height;
	uint16_t m_blendMode;
	size_t m_imageX, m_imageY;
	size_t m_clutX, m_clutY;
	bool m_semiTransparent;
	std::vector<uint16_t> m_image;
	std::vector<uint16_t> m_clut;
	std::vector<Shape> m_shapes;
	std::filesystem::path m_path;
};

std::vector<uint8_t> PackVRM(std::vector<Texture*>& textures);
QuadUV ConvertUV(const PixelBounds& bounds, const RawUV rawUV);
RawUV ConvertUV(const QuadUV uvs, int texWidth, int texHeight);
uint16_t ConvertVRAMColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a, uint16_t blendMode);
void ConvertVRAMColor(uint16_t vramColor, uint8_t * rgba, uint16_t blendMode);