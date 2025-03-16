/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: Geometry.cpp
Purpose: Geometry libraries
Language: C++
Platform: Visual studio 2022
Project: jungyeon.lee_CS350_1
Author: jungyeon.lee
Creation date: 01.28.2024
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"


Vector3 ProjectPointOnPlane(const Vector3& point, const Vector3& normal, float planeDistance)
{
  float dist = normal.Dot(point) - planeDistance;
  Vector3 proj = point - (dist * normal);
  return proj;
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b,
  float& u, float& v, float epsilon)
{
  Vector3 ab = b - a;
  if (ab.Length() < 1e-6)
  {
    return false;
  }

  Vector3 ap = point - a;
  Vector3 bp = point - b;

  float ab_ab = ab.Dot(ab);
  float ab_ap = ab.Dot(ap);
  float ab_bp = ab.Dot(bp);

  u = ab_bp / -ab_ab;
  v = ab_ap / ab_ab;

  bool isOutside = u < -epsilon || v < -epsilon || u > 1.0f + epsilon || v > 1.0f + epsilon;
  return !isOutside;
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b, const Vector3& c,
  float& u, float& v, float& w, float epsilon)
{
  Vector3 v0 = b - a, v1 = c - a, v2 = point - a;
  float d00 = v0.Dot(v0);
  float d01 = v0.Dot(v1);
  float d11 = v1.Dot(v1);
  float d20 = v2.Dot(v0);
  float d21 = v2.Dot(v1);
  float denom = d00 * d11 - d01 * d01;

  if (std::abs(denom) < 1e-6) {
    return false;
  }

  v = (d11 * d20 - d01 * d21) / denom;
  w = (d00 * d21 - d01 * d20) / denom;
  u = 1.0f - v - w;

  bool isOutside = u < -epsilon || v < -epsilon || w < -epsilon || u > 1.0f + epsilon || v > 1.0f + epsilon || w > 1.0f + epsilon;
  return !isOutside;
}

IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane, float epsilon)
{
  float result = point.x * plane.x + point.y * plane.y + point.z * plane.z - plane.w;

  if (std::abs(result) <= epsilon)
  {
    return IntersectionType::Coplanar;
  }
  else if (result < 0)
  {
    return IntersectionType::Outside;
  }
  else
  {
    return IntersectionType::Inside;
  }
}

bool PointSphere(const Vector3& point, const Vector3& sphereCenter, float sphereRadius)
{
  float squaredDist = (point.x - sphereCenter.x) * (point.x - sphereCenter.x) +
    (point.y - sphereCenter.y) * (point.y - sphereCenter.y) +
    (point.z - sphereCenter.z) * (point.z - sphereCenter.z);

  float squaredRadius = sphereRadius * sphereRadius;

  if (squaredDist <= squaredRadius)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax)
{
  if (point.x < aabbMin.x || point.x > aabbMax.x) return false;
  if (point.y < aabbMin.y || point.y > aabbMax.y) return false;
  if (point.z < aabbMin.z || point.z > aabbMax.z) return false;
  return true;
}

bool RayPlane(const Vector3& rayStart, const Vector3& rayDir,
  const Vector4& plane, float& t, float epsilon)
{
  ++Application::mStatistics.mRayPlaneTests;
  if (abs(plane.x * rayDir.x + plane.y * rayDir.y + plane.z * rayDir.z) > epsilon)
  {
    t = -(plane.x * rayStart.x + plane.y * rayStart.y + plane.z * rayStart.z - plane.w) / (plane.x * rayDir.x + plane.y * rayDir.y + plane.z * rayDir.z);
    if (t < 0)
    {
      return false;
    }
    return true;
  }
  return false;
}

bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
  const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
  float& t, float triExpansionEpsilon)
{
  ++Application::mStatistics.mRayTriangleTests;
  Plane plane(triP0, triP1, triP2);
  if (!RayPlane(rayStart, rayDir, plane.mData, t))
  {
    return false;
  }

  Vector3 point = rayStart + t * rayDir;
  float u, v, w;
  return BarycentricCoordinates(point, triP0, triP1, triP2, u, v, w, triExpansionEpsilon);
}

bool RaySphere(const Vector3& rayStart, const Vector3& rayDir,
  const Vector3& sphereCenter, float sphereRadius,
  float& t)
{
  ++Application::mStatistics.mRaySphereTests;
  const float EPSILON = 1e-8;
  Vector3 m = rayStart - sphereCenter;

  float b = m.Dot(rayDir);
  float c = m.Dot(m) - sphereRadius * sphereRadius;

  if (c > EPSILON && b > EPSILON)
  {
    return false;
  }

  float discriminant = b * b - c;

  if (discriminant < EPSILON)
  {
    return false;
  }

  t = -b - static_cast<float>(sqrt(discriminant));

  if (t < EPSILON)
  {
    t = EPSILON;
  }

  return true;
}

bool RayAabb(const Vector3& rayStart, const Vector3& rayDir,
  const Vector3& aabbMin, const Vector3& aabbMax, float& t)
{
  ++Application::mStatistics.mRayAabbTests;
  const float EPSILON = 1e-8;
  float tMin = 0.0f;
  float tMax = std::numeric_limits<float>::max();

  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(rayDir[i]) < EPSILON)
    {
      if (rayStart[i] < aabbMin[i] || rayStart[i] > aabbMax[i])
      {
        return false;
      }
    }
    else
    {
      float ood = 1.0f / rayDir[i];
      float t1 = (aabbMin[i] - rayStart[i]) * ood;
      float t2 = (aabbMax[i] - rayStart[i]) * ood;

      if (t1 > t2) std::swap(t1, t2);

      tMin = std::max(tMin, t1);
      tMax = std::min(tMax, t2);

      if (tMin > tMax)
      {
        return false;
      }
    }
  }

  t = tMin;
  return true;
}

IntersectionType::Type PlaneTriangle(const Vector4& plane,
  const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
  float epsilon)
{
  ++Application::mStatistics.mPlaneTriangleTests;
  float dist0 = triP0.x * plane.x + triP0.y * plane.y + triP0.z * plane.z - plane.w;
  float dist1 = triP1.x * plane.x + triP1.y * plane.y + triP1.z * plane.z - plane.w;
  float dist2 = triP2.x * plane.x + triP2.y * plane.y + triP2.z * plane.z - plane.w;

  if (std::abs(dist0) <= epsilon && std::abs(dist1) <= epsilon && std::abs(dist2) <= epsilon)
  {
    return IntersectionType::Coplanar;
  }
  else if (dist0 < epsilon && dist1 < epsilon && dist2 < epsilon)
  {
    return IntersectionType::Outside;
  }
  else if (dist0 > -epsilon && dist1 > -epsilon && dist2 > -epsilon)
  {
    return IntersectionType::Inside;
  }

  return IntersectionType::Overlaps;
}

IntersectionType::Type PlaneSphere(const Vector4& plane,
  const Vector3& sphereCenter, float sphereRadius)
{
  ++Application::mStatistics.mPlaneSphereTests;

  float dist = sphereCenter.x * plane.x + sphereCenter.y * plane.y + sphereCenter.z * plane.z - plane.w;

  if (std::abs(dist) <= sphereRadius)
  {
    return IntersectionType::Overlaps;
  }
  else if (dist < 0.0f)
  {
    return IntersectionType::Outside;
  }
  else
  {
    return IntersectionType::Inside;
  }
}

IntersectionType::Type PlaneAabb(const Vector4& plane,
  const Vector3& aabbMin, const Vector3& aabbMax)
{
  ++Application::mStatistics.mPlaneAabbTests;

  Vector3 aabbCenter = (aabbMin + aabbMax) * 0.5f;
  Vector3 aabbHalf = (aabbMax - aabbMin) * 0.5f;

  float dist = aabbCenter.x * plane.x + aabbCenter.y * plane.y + aabbCenter.z * plane.z - plane.w;
  float radius = std::abs(plane.x) * aabbHalf.x + std::abs(plane.y) * aabbHalf.y + std::abs(plane.z) * aabbHalf.z;

  if (std::abs(dist) <= radius)
  {
    return IntersectionType::Overlaps;
  }
  else if (dist < 0)
  {
    return IntersectionType::Outside;
  }
  else
  {
    return IntersectionType::Inside;
  }
}

IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
  const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
  float epsilon)
{
  ++Application::mStatistics.mFrustumTriangleTests;
  bool insideChecker = true;
  for (int i = 0; i < 6; ++i)
  {
    IntersectionType::Type type = PlaneTriangle(planes[i], triP0, triP1, triP2, epsilon);
    if (type == IntersectionType::Outside)
    {
      return IntersectionType::Outside;
    }
    if (type == IntersectionType::Overlaps)
    {
      insideChecker = false;
    }
  }

  if (!insideChecker)
  {
    return IntersectionType::Overlaps;
  }
  else
  {
    return IntersectionType::Inside;
  }
}

IntersectionType::Type FrustumSphere(const Vector4 planes[6],
  const Vector3& sphereCenter, float sphereRadius, size_t& lastAxis)
{
  ++Application::mStatistics.mFrustumSphereTests;
  bool insideChecker = true;
  for (int i = 0; i < 6; ++i)
  {
    IntersectionType::Type type = PlaneSphere(planes[i], sphereCenter, sphereRadius);
    if (type == IntersectionType::Outside)
    {
      return IntersectionType::Outside;
    }
    if (type == IntersectionType::Overlaps)
    {
      insideChecker = false;
    }
  }

  if (!insideChecker)
  {
    return IntersectionType::Overlaps;
  }
  else
  {
    return IntersectionType::Inside;
  }

}

IntersectionType::Type FrustumAabb(const Vector4 planes[6],
  const Vector3& aabbMin, const Vector3& aabbMax, size_t& lastAxis)
{
  ++Application::mStatistics.mFrustumAabbTests;
  bool insideChecker = true;

  std::vector<IntersectionType::Type> type(6);
  type[lastAxis] = PlaneAabb(planes[lastAxis], aabbMin, aabbMax);

  if (type[lastAxis] == IntersectionType::Outside)
  {
    return IntersectionType::Outside;
  }
  else if (type[lastAxis] == IntersectionType::Overlaps)
  {
    insideChecker = false;
  }

  for (int i = 0; i < 6; ++i)
  {
    if (i == static_cast<int>(lastAxis))
    {
      continue;
    }
    IntersectionType::Type type = PlaneAabb(planes[i], aabbMin, aabbMax);
    if (type == IntersectionType::Outside)
    {
      lastAxis = static_cast<size_t>(i);
      return IntersectionType::Outside;
    }
    if (type == IntersectionType::Overlaps)
    {
      insideChecker = false;
    }
  }

  if (!insideChecker)
  {
    return IntersectionType::Overlaps;
  }
  else
  {
    return IntersectionType::Inside;
  }
}

bool SphereSphere(const Vector3& sphereCenter0, float sphereRadius0,
  const Vector3& sphereCenter1, float sphereRadius1)
{
  ++Application::mStatistics.mSphereSphereTests;
  if ((sphereCenter0 - sphereCenter1).Length() < sphereRadius0 + sphereRadius1)
  {
    return true;
  }
  else
  {
    return false;
  }

}

bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0,
  const Vector3& aabbMin1, const Vector3& aabbMax1)
{
  ++Application::mStatistics.mAabbAabbTests;
  for (int i = 0; i < 3; ++i)
  {
    if (aabbMax0[i] < aabbMin1[i] || aabbMin0[i] > aabbMax1[i])
    {
      return false;
    }
  }
  return true;
}
