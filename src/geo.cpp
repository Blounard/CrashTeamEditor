#include "geo.h"

#include <iostream>
#include <fstream>
#include <unordered_set>
#include <unordered_map>


Tri::Tri(const Point& p0, const Point& p1, const Point& p2)
	: Primitive(PrimitiveType::TRI, 3)
{
	p[0] = p0; p[1] = p1, p[2] = p2;
}

Quad::Quad(const Point& p0, const Point& p1, const Point& p2, const Point& p3)
	: Primitive(PrimitiveType::QUAD, 4)
{
	p[0] = p0; p[1] = p1; p[2] = p2; p[3] = p3;
}

Line::Line(const Point& p0, const Point& p1)
	: Primitive(PrimitiveType::LINE, 2)
{
	p[0] = p0; p[1] = p1;
}

void BoundingBox::Expand(const Vec3& pos)
{
	min.x = std::min(min.x, pos.x); max.x = std::max(max.x, pos.x);
	min.y = std::min(min.y, pos.y); max.y = std::max(max.y, pos.y);
	min.z = std::min(min.z, pos.z); max.z = std::max(max.z, pos.z);
}

float BoundingBox::Area() const
{
	Vec3 dist = max - min;
	return dist.x * dist.y * dist.z;
}

float BoundingBox::SemiPerimeter() const
{
	Vec3 dist = max - min;
	return dist.x + dist.y + dist.z;
}

float BoundingBox::NormL(int power) const
{
	Vec3 dist = max - min;
	return static_cast<float>(std::pow(std::pow(dist.x, power) + std::pow(dist.y, power) + std::pow(dist.z, power), 1.0f/static_cast<float>(power)));
}

float BoundingBox::MaxAxisLength() const
{
	Vec3 dist = max - min;
	return std::max(std::max(dist.x, dist.y), dist.z);
}

Vec3 BoundingBox::AxisLength() const
{
	return max - min;
}

Vec3 BoundingBox::Midpoint() const
{
	return (max + min) / 2;
}

float BoundingBox::Distance(const Vec3& point) const
{
	const Vec3 closest(
		std::max(min.x, std::min(point.x, max.x)),
		std::max(min.y, std::min(point.y, max.y)),
		std::max(min.z, std::min(point.z, max.z))
	);
	return (point - closest).Length();
}

float BoundingBox::Distance(const BoundingBox& other) const
{
	const float dx = std::max({ 0.0f, min.x - other.max.x, other.min.x - max.x });
	const float dy = std::max({ 0.0f, min.y - other.max.y, other.min.y - max.y });
	const float dz = std::max({ 0.0f, min.z - other.max.z, other.min.z - max.z });
	return Vec3(dx, dy, dz).Length();
}

BoundingBox BoundingBox::Union(const BoundingBox& other) const 
{
	BoundingBox box{};
	box.min.x = std::min(min.x, other.min.x); box.max.x = std::max(max.x, other.max.x);
	box.min.y = std::min(min.y, other.min.y); box.max.y = std::max(max.y, other.max.y);
	box.min.z = std::min(min.z, other.min.z); box.max.z = std::max(max.z, other.max.z);
	return box;
}

BoundingBox BoundingBox::Intersect(const BoundingBox& other) const 
{
	BoundingBox result{};
	result.min.x = std::max(min.x, other.min.x);
	result.min.y = std::max(min.y, other.min.y);
	result.min.z = std::max(min.z, other.min.z);
	result.max.x = std::min(max.x, other.max.x);
	result.max.y = std::min(max.y, other.max.y);
	result.max.z = std::min(max.z, other.max.z);
	// If min > max on an axis, there's no overlap there — treat as zero extent, not negative.
	result.max.x = std::max(result.max.x, result.min.x);
	result.max.y = std::max(result.max.y, result.min.y);
	result.max.z = std::max(result.max.z, result.min.z);
	return result;
}

BoundingBox BoundingBox::Empty()
{
	BoundingBox box{};
	box.min.x = std::numeric_limits<float>::max(); box.max.x = std::numeric_limits<float>::lowest();
	box.min.y = std::numeric_limits<float>::max(); box.max.y = std::numeric_limits<float>::lowest();
	box.min.z = std::numeric_limits<float>::max(); box.max.z = std::numeric_limits<float>::lowest();
	return box;
}

std::vector<Primitive> BoundingBox::ToGeometry() const
{
	constexpr size_t numCorners = 8;
	constexpr size_t numEdges = 12;
	constexpr float sqrtThree = 1.44224957031f;
	const Vec3 corners[numCorners] =
	{
		Vec3(min.x, min.y, min.z),
		Vec3(min.x, min.y, max.z),
		Vec3(min.x, max.y, min.z),
		Vec3(max.x, min.y, min.z),
		Vec3(max.x, max.y, min.z),
		Vec3(min.x, max.y, max.z),
		Vec3(max.x, min.y, max.z),
		Vec3(max.x, max.y, max.z),
	};
	const Vec3 cornerNormals[numCorners] =
	{
		Vec3(-1.f / sqrtThree, -1.f / sqrtThree, -1.f / sqrtThree),
		Vec3(-1.f / sqrtThree, -1.f / sqrtThree, 1.f / sqrtThree),
		Vec3(-1.f / sqrtThree, 1.f / sqrtThree, -1.f / sqrtThree),
		Vec3(1.f / sqrtThree, -1.f / sqrtThree, -1.f / sqrtThree),
		Vec3(1.f / sqrtThree, 1.f / sqrtThree, -1.f / sqrtThree),
		Vec3(-1.f / sqrtThree, 1.f / sqrtThree, 1.f / sqrtThree),
		Vec3(1.f / sqrtThree, -1.f / sqrtThree, 1.f / sqrtThree),
		Vec3(1.f / sqrtThree, 1.f / sqrtThree, 1.f / sqrtThree),
	};
	const int edgeIndices[numEdges][2] =
	{
		{0, 1}, {2, 5}, {3, 6}, {4, 7},
		{0, 2}, {1, 5}, {3, 4}, {6, 7},
		{0, 3}, {1, 6}, {2, 4}, {5, 7},
	};

	std::vector<Primitive> primitives;
	primitives.reserve(numEdges);
	for (int edgeIndex = 0; edgeIndex < 12; edgeIndex++)
	{
		const int a = edgeIndices[edgeIndex][0];
		const int b = edgeIndices[edgeIndex][1];
		Line line;
		line.texture = std::string();
		line.p[0].pos = corners[a];
		line.p[0].normal = cornerNormals[a];
		line.p[0].color = Color();
		line.p[0].uv = Vec2();
		line.p[1].pos = corners[b];
		line.p[1].normal = cornerNormals[b];
		line.p[1].color = Color();
		line.p[1].uv = Vec2();
		primitives.push_back(line);
	}

	return primitives;
}

Quaternion::Quaternion(const Vec3& axis, float angleRad)
{
	const float half = angleRad * 0.5f;
	const float s = std::sin(half);
	x = axis.x * s;
	y = axis.y * s;
	z = axis.z * s;
	w = std::cos(half);
}

Quaternion::Quaternion(const Vec3& eulerDeg)
{
	float yaw = eulerDeg.y * MATH_PI / 180.0f * 0.5f;
	float picth = eulerDeg.x * MATH_PI / 180.0f * 0.5f;
	float roll = eulerDeg.z * MATH_PI / 180.0f * 0.5f;
	Quaternion qy{ 0, std::sin(yaw), 0, std::cos(yaw) };
	Quaternion qx{ std::sin(picth), 0, 0, std::cos(picth) };
	Quaternion qz{ 0, 0, std::sin(roll), std::cos(roll) };
	Quaternion res = (qy * (qx * qz)).Normalized();
	x = res.x; y = res.y; z = res.z; w = res.w;
}

Quaternion Quaternion::operator*(const Quaternion& q) const
{
	return {
		w * q.x + x * q.w + y * q.z - z * q.y,
		w * q.y - x * q.z + y * q.w + z * q.x,
		w * q.z + x * q.y - y * q.x + z * q.w,
		w * q.w - x * q.x - y * q.y - z * q.z
	};
}

Vec3 Quaternion::operator*(const Vec3& v) const
{
	const Vec3 qv(x, y, z);
	const Vec3 t = qv.Cross(v) * 2.0f;
	return v + (t * w) + qv.Cross(t);
}

Vec3 Quaternion::ToEulerYXZ() const  
{
	float r00 = 1 - 2 * (y * y + z * z);
	float r01 = 2 * (x * y - w * z); 
	float r02 = 2 * (x * z + w * y);
	float r10 = 2 * (x * y + w * z);
	float r11 = 1 - 2 * (x * x + z * z);
	float r12 = 2 * (y * z - w * x);
	float r22 = 1 - 2 * (x * x + y * y);

	float pitch = std::asin(Clamp(-r12, -1.0f, 1.0f));
	float yaw, roll;
	if (1.0f - std::fabs(r12) > EPSILON)
	{
		yaw = std::atan2(r02, r22);
		roll = std::atan2(r10, r11);
	}
	else if (r12 < 0) // x = +90 deg: only (yaw - roll) is recoverable
	{
		yaw = std::atan2(r01, r00);
		roll = 0.0f;
	}
	else // x = -90 deg: only (yaw + roll) is recoverable
	{
		yaw = std::atan2(-r01, r00);
		roll = 0.0f;
	}
	constexpr float RAD2DEG = 180.0f / MATH_PI;
	return Vec3(pitch * RAD2DEG, yaw * RAD2DEG, roll * RAD2DEG);
}

Color::Color(double hue, double sat, double value)
{
	a = 255u;

	if (sat == 0)
	{
		r = g = b = 0;
		return;
	}

	long i;
	double hh, p, q, t, ff;

	hh = hue;
	if (hh >= 360.0) hh = 0.0;
	hh /= 60.0;
	i = static_cast<long>(hh);
	ff = hh - i;
	p = value * (1.0 - sat);
	q = value * (1.0 - (sat * ff));
	t = value * (1.0 - (sat * (1.0 - ff)));

	t = Clamp(t, 0.0, 1.0);
	p = Clamp(p, 0.0, 1.0);
	q = Clamp(q, 0.0, 1.0);
	value = Clamp(value, 0.0, 1.0);

	switch (i) {
	case 0:
		r = static_cast<unsigned char>(value * 255.0);
		g = static_cast<unsigned char>(t * 255.0);
		b = static_cast<unsigned char>(p * 255.0);
		break;
	case 1:
		r = static_cast<unsigned char>(q * 255.0);
		g = static_cast<unsigned char>(value * 255.0);
		b = static_cast<unsigned char>(p * 255.0);
		break;
	case 2:
		r = static_cast<unsigned char>(p * 255.0);
		g = static_cast<unsigned char>(value * 255.0);
		b = static_cast<unsigned char>(t * 255.0);
		break;
	case 3:
		r = static_cast<unsigned char>(p * 255.0);
		g = static_cast<unsigned char>(q * 255.0);
		b = static_cast<unsigned char>(value * 255.0);
		break;
	case 4:
		r = static_cast<unsigned char>(t * 255.0);
		g = static_cast<unsigned char>(p * 255.0);
		b = static_cast<unsigned char>(value * 255.0);
		break;
	case 5:
	default:
		r = static_cast<unsigned char>(value * 255.0);
		g = static_cast<unsigned char>(p * 255.0);
		b = static_cast<unsigned char>(q * 255.0);
		break;
	}
}

bool TestBarycentric(
	const Vec3& A, const Vec3& B, const Vec3& C,
	const Vec3& point, const Vec3& projectDir,
	float& outdist, Vec3& outnormal,
	float barycentricTolerance)
{
	//moller-trumbore intersection test
	//https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

	if (std::abs(projectDir.LengthSquared() - 1.0f) > EPSILON)
	{
		printf("Warning : must only call TestBarycentric with normalized projectDir");
		return false;
	}

	Vec3 edge1 = B - A;
	Vec3 edge2 = C - A;

	const Vec3 pvec = projectDir.Cross(edge2);
	const float det = edge1.Dot(pvec);

	if (std::abs(det) < EPSILON)
		return false; //Ray is parrallel to plane

	const float invDet = 1.0f / det;
	const Vec3 tvec = point - A;
	const Vec3 qvec = tvec.Cross(edge1);

	// Barycentric U coordinate check
	const float u = tvec.Dot(pvec) * invDet;
	if (u < -barycentricTolerance || u > 1.0f + barycentricTolerance)
		return false;

	// Barycentric V coordinate check
	const float v = projectDir.Dot(qvec) * invDet;
	if (v < -barycentricTolerance || v > 1.0f + barycentricTolerance)
		return false;
	if (u + v > 1.0f + barycentricTolerance)
		return false;

	outdist = edge2.Dot(qvec) * invDet;
	outnormal = edge2.Cross(edge1);
	outnormal.Normalize();

	return true;
}

bool SnapTriangle(const Vec3& A, const Vec3& B, const Vec3& C,
	Vec3& pos, Vec3& rot,
	const Vec3& projectDir, float barycentricTolerance)
{
	float dist;
	Vec3 normal;
	if (!TestBarycentric(A, B, C, pos, projectDir, dist, normal, barycentricTolerance))
	{
		return false;
	}

	pos = pos + projectDir * dist;

	Quaternion q(rot);
	Vec3 currentUp = q * Vec3(0.0f, 1.0f, 0.0f);

	Vec3 axis = currentUp.Cross(normal);
	float axisLenSq = axis.LengthSquared();
	float dot = Clamp(currentUp.Dot(normal), -1.0f, 1.0f);

	Quaternion qAlign;
	if (axisLenSq > EPSILON)
	{
		axis.Normalize();
		float angle = std::acos(dot);
		qAlign = Quaternion::FromAxisAngle(axis, angle);
	}
	else
	{
		if (dot > 0.0f)
		{
			qAlign = Quaternion::Identity(); // already aligned
		}
		else
		{
			Vec3 arbitrary = (std::fabs(currentUp.x) < 0.9f) ? Vec3(1, 0, 0) : Vec3(0, 0, 1);
			Vec3 perp = currentUp.Cross(arbitrary);
			perp.Normalize();
			qAlign = Quaternion::FromAxisAngle(perp, MATH_PI);
		}
	}

	rot = (qAlign * q).Normalized().ToEulerYXZ();
	return true;
}

// Load an .obj file that contain a path. Read and return the list of Vec3 it contains.
std::vector<Vec3> LoadPath(const std::filesystem::path& path)
{
	std::ifstream file(path);
	if (!file.is_open())
		return {};

	std::vector<Vec3> rawVertices;
	std::unordered_map<int, int> adjacency; // 1 based
	bool inFirstObject = false;

	std::string line;
	while (std::getline(file, line))
	{
		if (line.empty() || line[0] == '#')
			continue;
		std::istringstream ss(line);
		std::string token;
		ss >> token;

		if (token == "o")
		{
			if (!inFirstObject)
				inFirstObject = true;
			else
				break;
		}
		else if (token == "v" && inFirstObject)
		{
			float x, y, z;
			ss >> x >> y >> z;
			rawVertices.emplace_back(x, y, z);
		}
		else if (token == "l" && inFirstObject)
		{
			int a, b;
			if (ss >> a >> b)
				adjacency[a] = b;
		}
	}

	if (rawVertices.empty() || adjacency.empty())
		return {};

	// Find the start of the chain: a vertex that appears as a source but never as a destination
	std::unordered_set<int> destinations;
	for (auto& [source, target] : adjacency)
		destinations.insert(target);

	int start = 1; // Default value is just the first vertices (1 indexed)
	for (auto& [source, target] : adjacency)
	{
		if (!destinations.contains(source))
		{
			start = source;
			break;
		}
	}

	std::vector<Vec3> ordered;
	int current = start;
	while (adjacency.contains(current))
	{
		ordered.push_back(rawVertices[current - 1]);
		int next = adjacency[current];
		adjacency.erase(current);  // prevent infinite loops on malformed data
		current = next;
	}

	if (current >= 1 && current <= static_cast<int>(rawVertices.size()))
		ordered.push_back(rawVertices[current - 1]);

	return ordered;
}

std::vector<Vec3> LoadGhostPath(const std::filesystem::path& path, float startTime, float endTime)
{
	std::ifstream file(path, std::ios::binary);
	if (!file.is_open())
		return {};

	int startTimeMs = static_cast<int>(startTime * 960.0f);
	int endTimeMs = static_cast<int>(endTime * 960.0f);

	uint8_t header[0x28];
	file.read(reinterpret_cast<char*>(header), sizeof(header));
	if (!file)
		return {};

	auto ReadS16LE = [](const uint8_t* p) -> int16_t { return static_cast<int16_t>(p[0] | (p[1] << 8)); };

	const int16_t version = ReadS16LE(header + 0x00);
	const int16_t dataSize = ReadS16LE(header + 0x02);

	if (version != -4) // GHOST_TAPE_VERSION_RETAIL (bytes FC FF)
		return {};
	if (dataSize <= 0)
		return {};

	std::vector<uint8_t> stream(static_cast<size_t>(dataSize));
	file.read(reinterpret_cast<char*>(stream.data()), stream.size());
	const std::streamsize got = file.gcount();
	if (got <= 0)
		return {};
	stream.resize(static_cast<size_t>(got));

	auto ReadS16BE = [](const uint8_t* p) -> int16_t { return static_cast<int16_t>((p[0] << 8) | p[1]); };
	auto ReadU16BE = [](const uint8_t* p) -> uint16_t { return static_cast<uint16_t>((p[0] << 8) | p[1]); };

	std::vector<Vec3> result;
	Vec3 curPos;
	int32_t segmentStartMs = 0;
	std::vector<Vec3> pending;

	auto emitIfInRange = [&](const Vec3& v, float timeMs)
		{
			if (timeMs >= startTimeMs && timeMs <= endTimeMs)
				result.push_back(v);
		};

	auto flushPending = [&](int32_t timeDeltaMs)
		{
			const float intervals = static_cast<float>(pending.size() + 1);
			for (size_t i = 0; i < pending.size(); ++i)
			{
				const float t = segmentStartMs + (static_cast<float>(i + 1) / intervals) * timeDeltaMs;
				emitIfInRange(pending[i], t);
			}
			pending.clear();
		};

	size_t offset = 0;
	while (offset < stream.size())
	{
		const uint8_t firstByte = stream[offset];
		const bool isOpcode = (firstByte >= 0x80 && firstByte <= 0x84);

		if (isOpcode)
		{
			switch (firstByte)
			{
			case 0x80: // position keyframe (11 bytes)
			{
				if (offset + 11 > stream.size())
					return result; // truncated mid-packet, stop cleanly

				const uint8_t* p = &stream[offset];
				curPos.x = static_cast<float>(ReadS16BE(p + 1)) / 8.0f;
				curPos.y = static_cast<float>(ReadS16BE(p + 3)) / 8.0f;
				curPos.z = static_cast<float>(ReadS16BE(p + 5)) / 8.0f;
				const uint16_t timeDelta = ReadU16BE(p + 7);
				flushPending(timeDelta);

				const float t = segmentStartMs + timeDelta;
				emitIfInRange(curPos, t);
				segmentStartMs = static_cast<int32_t>(t);
				offset += 11;
				break;
			}
			case 0x81: offset += 3; break; // animation, no position
			case 0x82: offset += 6; break; // boost, no position
			case 0x83: offset += 2; break; // instance flags, no position
			case 0x84: // idle: reuses last position, still a timed sample
				pending.push_back(curPos);
				offset += 1;
				break;
			}
		}
		else
		{
			// opcode-less velocity packet (5 bytes: dx, dy, dz, rot_y, rot_z)
			if (offset + 5 > stream.size())
				break; 
			const uint8_t* p = &stream[offset];
			const int8_t dx = static_cast<int8_t>(firstByte);
			const int8_t dy = static_cast<int8_t>(p[1]);
			const int8_t dz = static_cast<int8_t>(p[2]);

			curPos.x += static_cast<float>(dx) / 8.0f;
			curPos.y += static_cast<float>(dy) / 8.0f;
			curPos.z += static_cast<float>(dz) / 8.0f;
			pending.push_back(curPos);
			offset += 5;
		}
	}
	return result;
}

std::vector<Vec3> ComputeYaw(const std::vector<Vec3>& pos, bool loop)
{
	const Vec3 up(0.0f, 1.0f, 0.0f);
	float curYaw = 0.0f;
	int nodeCount = static_cast<int>(pos.size());
	std::vector<Vec3> rots(nodeCount);
	for (size_t i = 0; i < nodeCount; i++)
	{
		const Vec3& curr = pos[i];
		const Vec3& next = pos[(i + 1) % nodeCount];
		Vec3 delta = next - curr;
		if ((i + 1) == nodeCount && !loop)
			delta = pos[i] - pos[i - 1];
		
		Vec3 forwardVec = delta - up * (up.Dot(delta));
		if (forwardVec.LengthSquared() > EPSILON)
		{
			forwardVec.Normalize();
			curYaw = std::atan2(forwardVec.x, forwardVec.z) * (180.0f / MATH_PI);
		}
		rots[i].y = curYaw;
		rots[i].x = 0.0f;
		rots[i].z = 0.0f;
	}
	return rots;
}

std::vector<Vec3> NormalizePos(const std::vector<Vec3>& pos, const float dist, bool loop) 
{

	int numPoint = static_cast<int>(pos.size());
	if (numPoint < 2 || dist <= 0.0f) return pos;

	auto catmullRomAlpha = [](const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t, float alpha = 0.5f) -> Vec3 {
		auto getT = [alpha](float t, const Vec3& p0, const Vec3& p1) -> float {
			float d = (p1 - p0).Length();
			return t + std::pow(std::max(d, EPSILON), alpha);
			};

		const float t0 = 0.0f;
		const float t1 = getT(t0, p0, p1);
		const float t2 = getT(t1, p1, p2);
		const float t3 = getT(t2, p2, p3);

		const float s = t1 + t * (t2 - t1);

		const Vec3 A1 = p0 * ((t1 - s) / (t1 - t0)) + p1 * ((s - t0) / (t1 - t0));
		const Vec3 A2 = p1 * ((t2 - s) / (t2 - t1)) + p2 * ((s - t1) / (t2 - t1));
		const Vec3 A3 = p2 * ((t3 - s) / (t3 - t2)) + p3 * ((s - t2) / (t3 - t2));

		const Vec3 B1 = A1 * ((t2 - s) / (t2 - t0)) + A2 * ((s - t0) / (t2 - t0));
		const Vec3 B2 = A2 * ((t3 - s) / (t3 - t1)) + A3 * ((s - t1) / (t3 - t1));

		return B1 * ((t2 - s) / (t2 - t1)) + B2 * ((s - t1) / (t2 - t1));
		};

	auto getPoint = [&](int i) -> const Vec3& {
		if (loop) {
			return pos[((i % numPoint) + numPoint) % numPoint];
		}
		else {
			int clamped = std::clamp(i, 0, numPoint - 1);
			return pos[clamped];
		}
		};

	// 1. Generate Dense Samples
	const int stepsPerSegment = 64;
	std::vector<Vec3> denseSamples;
	denseSamples.reserve(numPoint * stepsPerSegment);

	const int segmentCount = loop ? numPoint : (numPoint - 1);
	for (int i = 0; i < segmentCount; i++) {
		const Vec3& p0 = getPoint(i - 1);
		const Vec3& p1 = getPoint(i);
		const Vec3& p2 = getPoint(i + 1);
		const Vec3& p3 = getPoint(i + 2);
		for (int step = 0; step < stepsPerSegment; step++) {
			float t = (float)step / (float)stepsPerSegment;
			denseSamples.push_back(catmullRomAlpha(p0, p1, p2, p3, t));
		}
	}

	
	if (loop)
		denseSamples.push_back(denseSamples.front());
	else
		denseSamples.push_back(getPoint(numPoint - 1));

	// 2. Distribute points by 'dist'
	std::vector<Vec3> result;
	float accumulated = 0.0f;

	// We start by adding the first point
	result.push_back(denseSamples.front());

	for (size_t i = 1; i < denseSamples.size(); i++) {
		Vec3 segment = denseSamples[i] - denseSamples[i - 1];
		float segLen = segment.Length();
		if (segLen <= 0.00001f) continue;

		accumulated += segLen;

		while (accumulated >= dist) {
			float overshot = accumulated - dist;
			float ratio = (segLen - overshot) / segLen;
			Vec3 newPoint = denseSamples[i - 1] + segment * ratio;

			result.push_back(newPoint);

			// Prepare for next potential point in same segment
			accumulated = overshot;
			// In a loop, we usually don't want the last point to overlap the first.
			// If the last point is extremely close to the first, you might want to break.
		}
	}

	return result;
}