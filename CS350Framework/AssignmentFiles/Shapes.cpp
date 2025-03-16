/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: Shape.cpp
Purpose: Set shape
Language: C++
Platform: Visual studio 2022
Project: jungyeon.lee_CS350_1
Author: jungyeon.lee
Creation date: 01.28.2024
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"
#include <cmath>
#include <limits>

//-----------------------------------------------------------------------------LineSegment
LineSegment::LineSegment()
{
  mStart = mEnd = Vector3::cZero;
}

LineSegment::LineSegment(Math::Vec3Param start, Math::Vec3Param end)
{
  mStart = start;
  mEnd = end;
}

DebugShape& LineSegment::DebugDraw() const
{
  return gDebugDrawer->DrawLine(*this);
}

//-----------------------------------------------------------------------------Ray
Ray::Ray()
{
  mStart = mDirection = Vector3::cZero;
}

Ray::Ray(Math::Vec3Param start, Math::Vec3Param dir)
{
  mStart = start;
  mDirection = dir;
}

Ray Ray::Transform(const Math::Matrix4& transform) const
{
  Ray transformedRay;
  transformedRay.mStart = Math::TransformPoint(transform, mStart);
  transformedRay.mDirection = Math::TransformNormal(transform, mDirection);
  return transformedRay;
}

Vector3 Ray::GetPoint(float t) const
{
  return mStart + mDirection * t;
}

DebugShape& Ray::DebugDraw(float t) const
{
  return gDebugDrawer->DrawRay(*this, t);
}

//-----------------------------------------------------------------------------Sphere
Sphere::Sphere()
{
  mCenter = Vector3::cZero;
  mRadius = 0;
}

Sphere::Sphere(const Vector3& center, float radius)
{
  mCenter = center;
  mRadius = radius;
}

void Sphere::ComputeCentroid(const std::vector<Vector3>& points)
{
  Vector3 max = Vector3(std::numeric_limits<float>::min()), min = Vector3(std::numeric_limits<float>::max());
  for (const auto& point : points)
  {
    if (max.x < point.x)
    {
      max.x = point.x;
    }
    if (max.y < point.y)
    {
      max.y = point.y;
    }
    if (max.z < point.z)
    {
      max.z = point.z;
    }
    if (min.x > point.x)
    {
      min.x = point.x;
    }
    if (min.y > point.y)
    {
      min.y = point.y;
    }
    if (min.z > point.z)
    {
      min.z = point.z;
    }

  }
  Vector3 centroid = Vector3((max.x + min.x) * 0.5f, (max.y + min.y) * 0.5f, (max.z + min.z) * 0.5f);

  float maxDistanceSquared = 0.0f;
  for (const auto& point : points)
  {
    float distanceSquared = (centroid.x - point.x) * (centroid.x - point.x) + (centroid.y - point.y) * (centroid.y - point.y) + (centroid.z - point.z) * (centroid.z - point.z);
    if (distanceSquared > maxDistanceSquared)
    {
      maxDistanceSquared = distanceSquared;
    }
  }
  mCenter = centroid;
  mRadius = sqrtf(maxDistanceSquared);
}

void Sphere::ComputeRitter(const std::vector<Vector3>& points)
{
  Vector3 xMin = points[0], xMax = points[0], yMin = points[0], yMax = points[0], zMin = points[0], zMax = points[0];

  for (const auto& point : points)
  {
    if (point.x < xMin.x) xMin = point;
    if (point.x > xMax.x) xMax = point;
    if (point.y < yMin.y) yMin = point;
    if (point.y > yMax.y) yMax = point;
    if (point.z < zMin.z) zMin = point;
    if (point.z > zMax.z) zMax = point;
  }

  float distX = std::sqrt((xMin.x - xMax.x) * (xMin.x - xMax.x) + (xMin.y - xMax.y) * (xMin.y - xMax.y) + (xMin.z - xMax.z) * (xMin.z - xMax.z));
  float distY = std::sqrt((yMin.x - yMax.x) * (yMin.x - yMax.x) + (yMin.y - yMax.y) * (yMin.y - yMax.y) + (yMin.z - yMax.z) * (yMin.z - yMax.z));
  float distZ = std::sqrt((zMin.x - zMax.x) * (zMin.x - zMax.x) + (zMin.y - zMax.y) * (zMin.y - zMax.y) + (zMin.z - zMax.z) * (zMin.z - zMax.z));

  Vector3 p1 = xMin, p2 = xMax;
  if (distY > distX && distY > distZ)
  {
    p1 = yMin;
    p2 = yMax;
  }
  else if (distZ > distX && distZ > distY)
  {
    p1 = zMin;
    p2 = zMax;
  }

  mCenter = (p1 + p2) * 0.5f;
  mRadius = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z)) * 0.5f;

  for (const auto& point : points)
  {
    Vector3 toPoint = point - mCenter;
    float distToPoint = toPoint.Length();
    if (distToPoint > mRadius)
    {
      float newRadius = (mRadius + distToPoint) * 0.5f;
      float radiusDiff = newRadius - mRadius;
      mCenter = mCenter + toPoint * (radiusDiff / distToPoint);
      mRadius = newRadius;
    }
  }
}

bool Sphere::ContainsPoint(const Vector3& point)
{
  return PointSphere(point, mCenter, mRadius);
}

Vector3 Sphere::GetCenter() const
{
  return mCenter;
}

float Sphere::GetRadius() const
{
  return mRadius;
}

bool Sphere::Compare(const Sphere& rhs, float epsilon) const
{
  float posDiff = Math::Length(mCenter - rhs.mCenter);
  float radiusDiff = Math::Abs(mRadius - rhs.mRadius);

  return posDiff < epsilon && radiusDiff < epsilon;
}

DebugShape& Sphere::DebugDraw() const
{
  return gDebugDrawer->DrawSphere(*this);
}

//-----------------------------------------------------------------------------Aabb
Aabb::Aabb()
{
  //set the aabb to an initial bad value (where the min is smaller than the max)
  mMin.Splat(Math::PositiveMax());
  mMax.Splat(-Math::PositiveMax());
}

Aabb::Aabb(const Vector3& min, const Vector3& max)
{
  mMin = min;
  mMax = max;
}

Aabb Aabb::BuildFromCenterAndHalfExtents(const Vector3& center, const Vector3& halfExtents)
{
  return Aabb(center - halfExtents, center + halfExtents);
}

float Aabb::GetVolume() const
{
  float length = mMax.x - mMin.x;
  float width = mMax.y - mMin.y;
  float height = mMax.z - mMin.z;

  return length * width * height;
}

float Aabb::GetSurfaceArea() const
{
  float length = mMax.x - mMin.x;
  float width = mMax.y - mMin.y;
  float height = mMax.z - mMin.z;

  return 2.0f * (length * width + width * height + height * length);
}

bool Aabb::Contains(const Aabb& aabb) const
{
  bool containsMin = mMin.x <= aabb.mMin.x && mMin.y <= aabb.mMin.y && mMin.z <= aabb.mMin.z;

  bool containsMax = mMax.x >= aabb.mMax.x && mMax.y >= aabb.mMax.y && mMax.z >= aabb.mMax.z;

  return containsMin && containsMax;
}

void Aabb::Expand(const Vector3& point)
{
  for (size_t i = 0; i < 3; ++i)
  {
    mMin[i] = Math::Min(mMin[i], point[i]);
    mMax[i] = Math::Max(mMax[i], point[i]);
  }
}

Aabb Aabb::Combine(const Aabb& lhs, const Aabb& rhs)
{
  Aabb result;
  for (size_t i = 0; i < 3; ++i)
  {
    result.mMin[i] = Math::Min(lhs.mMin[i], rhs.mMin[i]);
    result.mMax[i] = Math::Max(lhs.mMax[i], rhs.mMax[i]);
  }
  return result;
}

bool Aabb::Compare(const Aabb& rhs, float epsilon) const
{
  float pos1Diff = Math::Length(mMin - rhs.mMin);
  float pos2Diff = Math::Length(mMax - rhs.mMax);

  return pos1Diff < epsilon && pos2Diff < epsilon;
}

void Aabb::Transform(const Vector3& scale, const Matrix3& rotation, const Vector3& translation)
{
  Vector3 sr;
  sr.x = scale.x * GetHalfSize().x;
  sr.y = scale.y * GetHalfSize().y;
  sr.z = scale.z * GetHalfSize().z;
  Vector3 sc;
  sc.x = scale.x * GetCenter().x;
  sc.y = scale.y * GetCenter().y;
  sc.z = scale.z * GetCenter().z;

  Vector3 newExtents, newCenter;

  newExtents.x = std::abs(rotation[0][0]) * sr.x + std::abs(rotation[0][1]) * sr.y + std::abs(rotation[0][2]) * sr.z;
  newExtents.y = std::abs(rotation[1][0]) * sr.x + std::abs(rotation[1][1]) * sr.y + std::abs(rotation[1][2]) * sr.z;
  newExtents.z = std::abs(rotation[2][0]) * sr.x + std::abs(rotation[2][1]) * sr.y + std::abs(rotation[2][2]) * sr.z;

  newCenter.x = translation.x + rotation[0][0] * sc.x + rotation[0][1] * sc.y + rotation[0][2] * sc.z;
  newCenter.y = translation.y + rotation[1][0] * sc.x + rotation[1][1] * sc.y + rotation[1][2] * sc.z;
  newCenter.z = translation.z + rotation[2][0] * sc.x + rotation[2][1] * sc.y + rotation[2][2] * sc.z;

  mMin = newCenter - newExtents;
  mMax = newCenter + newExtents;
}

Vector3 Aabb::GetMin() const
{
  return mMin;
}

Vector3 Aabb::GetMax() const
{
  return mMax;
}

Vector3 Aabb::GetCenter() const
{
  return (mMin + mMax) * 0.5f;
}

Vector3 Aabb::GetHalfSize() const
{
  return (mMax - mMin) * 0.5f;
}

DebugShape& Aabb::DebugDraw() const
{
  return gDebugDrawer->DrawAabb(*this);
}

//-----------------------------------------------------------------------------Triangle
Triangle::Triangle()
{
  mPoints[0] = mPoints[1] = mPoints[2] = Vector3::cZero;
}

Triangle::Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  mPoints[0] = p0;
  mPoints[1] = p1;
  mPoints[2] = p2;
}

DebugShape& Triangle::DebugDraw() const
{
  return gDebugDrawer->DrawTriangle(*this);
}

//-----------------------------------------------------------------------------Plane
Plane::Plane()
{
  mData = Vector4::cZero;
}

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  Set(p0, p1, p2);
}

Plane::Plane(const Vector3& normal, const Vector3& point)
{
  Set(normal, point);
}

void Plane::Set(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  Vector3 v1 = p1 - p0;
  Vector3 v2 = p2 - p0;
  Vector3 normal = v1.Cross(v2);
  normal.Normalize();
  mData.x = normal.x;
  mData.y = normal.y;
  mData.z = normal.z;

  mData.w = normal.Dot(p0);
}

void Plane::Set(const Vector3& normal, const Vector3& point)
{
  Vector3 normalCopy = normal;
  normalCopy.Normalize();
  mData.x = normalCopy.x;
  mData.y = normalCopy.y;
  mData.z = normalCopy.z;

  mData.w = normalCopy.Dot(point);
}

Vector3 Plane::GetNormal() const
{
  return Vector3(mData.x, mData.y, mData.z);
}

float Plane::GetDistance() const
{
  return mData.w;
}

DebugShape& Plane::DebugDraw(float size) const
{
  return DebugDraw(size, size);
}

DebugShape& Plane::DebugDraw(float sizeX, float sizeY) const
{
  return gDebugDrawer->DrawPlane(*this, sizeX, sizeY);
}

//-----------------------------------------------------------------------------Frustum
void Frustum::Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn, const Vector3& ltn,
  const Vector3& lbf, const Vector3& rbf, const Vector3& rtf, const Vector3& ltf)
{
  mPoints[0] = lbn;
  mPoints[1] = rbn;
  mPoints[2] = rtn;
  mPoints[3] = ltn;
  mPoints[4] = lbf;
  mPoints[5] = rbf;
  mPoints[6] = rtf;
  mPoints[7] = ltf;

  //left
  mPlanes[0].Set(lbf, ltf, lbn);
  //right
  mPlanes[1].Set(rbn, rtf, rbf);
  //top
  mPlanes[2].Set(ltn, ltf, rtn);
  //bot
  mPlanes[3].Set(rbn, lbf, lbn);
  //near
  mPlanes[4].Set(lbn, ltn, rbn);
  //far
  mPlanes[5].Set(rbf, rtf, lbf);
}

Math::Vector4* Frustum::GetPlanes() const
{
  return (Vector4*)mPlanes;
}

DebugShape& Frustum::DebugDraw() const
{
  return gDebugDrawer->DrawFrustum(*this);
}
