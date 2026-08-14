#include "geo.h"

#include <iostream>
#include <fstream>
#include <unordered_set>


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

BoundingBox BoundingBox::Union(const BoundingBox other) const 
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

	outdist = edge2.Dot(qvec) * invDet;
	outnormal = edge1.Cross(edge2);
	outnormal.Normalize();

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


std::vector<Vec3> LoadPath(const std::filesystem::path& path)
{
	// AI MADE, TODO : RECODE / VERIFY
	std::ifstream file(path);
	if (!file.is_open())
		return {};

	std::vector<Vec3>                        rawVertices;
	std::unordered_map<int, int>             adjacency;   // edge map: from -> to (1-based)
	bool                                     inFirstObject = false;

	std::string line;
	while (std::getline(file, line))
	{
		if (line.empty() || line[0] == '#')
			continue;

		std::istringstream ss(line);
		std::string        token;
		ss >> token;

		if (token == "o")
		{
			// Only parse the first object
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
				adjacency[a] = b;  // directed edge a -> b (OBJ indices are 1-based)
		}
	}

	if (rawVertices.empty() || adjacency.empty())
		return rawVertices;

	// Find the start of the chain: a vertex that appears as a source but never as a destination
	std::unordered_set<int> destinations;
	for (auto& [from, to] : adjacency)
		destinations.insert(to);

	int start = -1;
	for (auto& [from, to] : adjacency)
	{
		if (destinations.find(from) == destinations.end())
		{
			start = from;
			break;
		}
	}

	// Fallback: if it's a closed loop, just pick any start
	if (start == -1 && !adjacency.empty())
		start = adjacency.begin()->first;

	// Walk the chain in edge order
	std::vector<Vec3> ordered;
	ordered.reserve(rawVertices.size());

	int current = start;
	while (adjacency.count(current))
	{
		// OBJ indices are 1-based
		ordered.push_back(rawVertices[current - 1]);
		int next = adjacency[current];
		adjacency.erase(current);  // prevent infinite loops on malformed data
		current = next;
	}
	// Push the final vertex (the chain end that has no outgoing edge)
	if (current >= 1 && current <= static_cast<int>(rawVertices.size()))
		ordered.push_back(rawVertices[current - 1]);

	return ordered;
}