/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: Gjk.cpp
Purpose: Gjk algorithm
Language: C++
Platform: Visual studio 2022
Project: jungyeon.lee_CS350_5
Author: jungyeon.lee
Creation date: 04.25.2024
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

float u, v, w, t;

//-----------------------------------------------------------------------------SupportShape
Vector3 SupportShape::GetCenter(const std::vector<Vector3>& localPoints, const Matrix4& transform) const
{
  Vector3 center = Vector3::cZero;
  const size_t size = localPoints.size();
  for (size_t i = 0; i < size; ++i)
  {
    center += localPoints[i];
  }
  center /= static_cast<float>(size);
  center = Math::TransformPoint(transform, center);
  return center;
}

Vector3 SupportShape::Support(const Vector3& worldDirection, const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform) const
{
  Vector3 result = Vector3::cZero;
  const size_t size = localPoints.size();
  float maxDot = -std::numeric_limits<float>::max();

  for (size_t i = 0; i < size; ++i)
  {
    float dot = Math::Dot(Math::TransformPoint(localToWorldTransform, localPoints[i]), worldDirection);
    if (maxDot < dot)
    {
      maxDot = dot;
      result = Math::TransformPoint(localToWorldTransform, localPoints[i]);
    }
  }
  return result;
}

void SupportShape::DebugDraw(const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform, const Vector4& color) const
{
  const size_t size = localPoints.size();
  for (size_t i = 0; i < size; ++i)
  {
    DebugShape& point = gDebugDrawer->DrawPoint(Math::TransformPoint(localToWorldTransform, localPoints[i]));
    point.Color(color);
  }
}

//-----------------------------------------------------------------------------ModelSupportShape
Vector3 ModelSupportShape::GetCenter() const
{
  return SupportShape::GetCenter(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

Vector3 ModelSupportShape::Support(const Vector3& worldDirection) const
{
  return SupportShape::Support(worldDirection, mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

void ModelSupportShape::DebugDraw(const Vector4& color) const
{
  SupportShape::DebugDraw(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

//-----------------------------------------------------------------------------PointsSupportShape
PointsSupportShape::PointsSupportShape()
{
  mScale = Vector3(1);
  mRotation = Matrix3::cIdentity;
  mTranslation = Vector3::cZero;
}

Vector3 PointsSupportShape::GetCenter() const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  return SupportShape::GetCenter(mLocalSpacePoints, transform);
}

Vector3 PointsSupportShape::Support(const Vector3& worldDirection) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  return SupportShape::Support(worldDirection, mLocalSpacePoints, transform);
}

void PointsSupportShape::DebugDraw(const Vector4& color) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  SupportShape::DebugDraw(mLocalSpacePoints, transform, color);
}

//-----------------------------------------------------------------------------SphereSupportShape
Vector3 SphereSupportShape::GetCenter() const
{
  return mSphere.mCenter;
}

Vector3 SphereSupportShape::Support(const Vector3& worldDirection) const
{
  return mSphere.mCenter + (worldDirection.Normalized() * mSphere.mRadius);
}

void SphereSupportShape::DebugDraw(const Vector4& color) const
{
  DebugShape& shape = gDebugDrawer->DrawSphere(mSphere);
  shape.Color(color);
}

//-----------------------------------------------------------------------------ObbSupportShape
Vector3 ObbSupportShape::GetCenter() const
{
  return mTranslation;
}

Vector3 ObbSupportShape::Support(const Vector3& worldDirection) const
{
  Vector3 result = GetCenter();
  Vector3 localDirection = Math::Transform(mRotation.Inverted(), worldDirection);
  for (int i = 0; i < 3; ++i)
  {
    result += Math::GetSign(localDirection[i]) * mScale[i] * 0.5f * mRotation.Basis(i);
  }
  return result;
}

void ObbSupportShape::DebugDraw(const Vector4& color) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  DebugShape& shape = gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
  shape.Color(color);
  shape.SetTransform(transform);
}


//------------------------------------------------------------ Voronoi Region Tests
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  newSize = 1;
  closestPoint = p0;
  searchDirection = q - p0;
  return VoronoiRegion::Point0;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  BarycentricCoordinates(q, p0, p1, u, v);
  if (v <= 0.0f)
  {
    newSize = 1;
    newIndices[0] = 0;
    closestPoint = p0;
    searchDirection = q - p0;

    return VoronoiRegion::Point0;
  }
  if (u <= 0.0f)
  {
    newSize = 1;
    newIndices[0] = 1;
    closestPoint = p1;
    searchDirection = q - p1;

    return VoronoiRegion::Point1;
  }

  newSize = 2;
  newIndices[0] = 0;
  newIndices[1] = 1;
  closestPoint = (p0 * u) + (p1 * v);
  searchDirection = q - closestPoint;

  return VoronoiRegion::Edge01;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  float u01, v01, u12, v12, u02, v02;
  BarycentricCoordinates(q, p0, p1, u01, v01);
  BarycentricCoordinates(q, p0, p2, u02, v02);
  BarycentricCoordinates(q, p1, p2, u12, v12);

  if (v01 <= 0.0f && v02 <= 0.0f)
  {
    newSize = 1;
    newIndices[0] = 0;
    closestPoint = p0;
    searchDirection = q - p0;

    return VoronoiRegion::Point0;
  }
  if (u01 <= 0.0f && v12 <= 0.0f)
  {
    newSize = 1;
    newIndices[0] = 1;
    closestPoint = p1;
    searchDirection = q - p1;

    return VoronoiRegion::Point1;
  }
  if (u02 <= 0.0f && u12 <= 0.0f)
  {
    newSize = 1;
    newIndices[0] = 2;
    closestPoint = p2;
    searchDirection = q - p2;

    return VoronoiRegion::Point2;
  }

  BarycentricCoordinates(q, p0, p1, p2, u, v, w);
  if (u01 > 0.0f && v01 > 0.0f && w <= 0.0f)
  {
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 1;
    closestPoint = (p0 * u01) + (p1 * v01);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Edge01;
  }
  if (u02 > 0.0f && v02 > 0.0f && v <= 0.0f)
  {
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 2;
    closestPoint = (p0 * u02) + (p2 * v02);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Edge02;
  }
  if (u12 > 0.0f && v12 > 0.0f && u <= 0.0f)
  {
    newSize = 2;
    newIndices[0] = 1;
    newIndices[1] = 2;
    closestPoint = p1 * u12 + p2 * v12;
    searchDirection = q - closestPoint;

    return VoronoiRegion::Edge12;
  }

  newSize = 3;
  newIndices[0] = 0;
  newIndices[1] = 1;
  newIndices[2] = 2;
  closestPoint = p0 * u + p1 * v + p2 * w;
  searchDirection = q - closestPoint;
  return VoronoiRegion::Triangle012;
}

float DistanceToPlane(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  return Math::Dot(Math::Cross(p1 - p0, p2 - p0).Normalized(), q - p0);
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  float u01, v01, u02, v02, u03, v03, u12, v12, u13, v13, u23, v23;
  BarycentricCoordinates(q, p0, p1, u01, v01);
  BarycentricCoordinates(q, p0, p2, u02, v02);
  BarycentricCoordinates(q, p0, p3, u03, v03);
  BarycentricCoordinates(q, p1, p2, u12, v12);
  BarycentricCoordinates(q, p1, p3, u13, v13);
  BarycentricCoordinates(q, p2, p3, u23, v23);

  if (v01 <= 0.0f && v02 <= 0.0f && v03 <= 0.0f)
  {
    newSize = 1;
    newIndices[0] = 0;
    closestPoint = p0;
    searchDirection = q - p0;

    return VoronoiRegion::Point0;
  }
  if (u01 <= 0.0f && v12 <= 0.0f && v13 <= 0.0f)
  {
    newSize = 1;
    newIndices[0] = 1;
    closestPoint = p1;
    searchDirection = q - p1;

    return VoronoiRegion::Point1;
  }
  if (u02 <= 0.0f && u12 <= 0.0f && v23 <= 0.0f)
  {
    newSize = 1;
    newIndices[0] = 2;
    closestPoint = p2;
    searchDirection = q - p2;

    return VoronoiRegion::Point2;
  }
  if (u03 <= 0.0f && u13 <= 0.0f && u23 <= 0.0f)
  {
    newSize = 1;
    newIndices[0] = 3;
    closestPoint = p3;
    searchDirection = q - p3;

    return VoronoiRegion::Point3;
  }

  float u012, v012, w012, u013, v013, w013, u023, v023, w023, u123, v123, w123;
  BarycentricCoordinates(q, p0, p1, p2, u012, v012, w012);
  BarycentricCoordinates(q, p0, p1, p3, u013, v013, w013);
  BarycentricCoordinates(q, p0, p2, p3, u023, v023, w023);
  BarycentricCoordinates(q, p1, p2, p3, u123, v123, w123);

  if (u01 > 0.0f && v01 > 0.0f && w012 <= 0.0f && w013 <= 0.0f)
  {
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 1;
    closestPoint = (p0 * u01) + (p1 * v01);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Edge01;
  }
  if (u02 > 0.0f && v02 > 0.0f && v012 <= 0.0f && w023 <= 0.0f)
  {
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 2;
    closestPoint = (p0 * u02) + (p2 * v02);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Edge02;
  }
  if (u03 > 0.0f && v03 > 0.0f && v013 <= 0.0f && v023 <= 0.0f)
  {
    newSize = 2;
    newIndices[0] = 0;
    newIndices[1] = 3;
    closestPoint = (p0 * u03) + (p3 * v03);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Edge03;
  }
  if (u12 > 0.0f && v12 > 0.0f && u012 <= 0.0f && w123 <= 0.0f)
  {
    newSize = 2;
    newIndices[0] = 1;
    newIndices[1] = 2;
    closestPoint = (p1 * u12) + (p2 * v12);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Edge12;
  }
  if (u13 > 0.0f && v13 > 0.0f && u013 <= 0.0f && v123 <= 0.0f)
  {
    newSize = 2;
    newIndices[0] = 1;
    newIndices[1] = 3;
    closestPoint = (p1 * u13) + (p3 * v13);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Edge13;
  }
  if (u23 > 0.0f && v23 > 0.0f && u023 <= 0.0f && u123 <= 0.0f)
  {
    newSize = 2;
    newIndices[0] = 2;
    newIndices[1] = 3;
    closestPoint = (p2 * u23) + (p3 * v23);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Edge23;
  }

  u = DistanceToPlane(q, p1, p2, p3) / DistanceToPlane(p0, p1, p2, p3);
  v = DistanceToPlane(q, p0, p2, p3) / DistanceToPlane(p1, p0, p2, p3);
  w = DistanceToPlane(q, p0, p1, p3) / DistanceToPlane(p2, p0, p1, p3);
  t = 1.0f - u - w - v;

  if (u012 > 0.0f && v012 > 0.0f && w012 > 0.0f && t <= 0.0f)
  {
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 1;
    newIndices[2] = 2;
    closestPoint = (p0 * u012) + (p1 * v012) + (p2 * w012);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Triangle012;
  }
  if (u013 > 0.0f && v013 > 0.0f && w013 > 0.0f && w <= 0.0f)
  {
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 1;
    newIndices[2] = 3;
    closestPoint = (p0 * u013) + (p1 * v013) + (p3 * w013);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Triangle013;
  }
  if (u023 > 0.0f && v023 > 0.0f && w023 > 0.0f && v <= 0.0f)
  {
    newSize = 3;
    newIndices[0] = 0;
    newIndices[1] = 2;
    newIndices[2] = 3;
    closestPoint = (p0 * u023) + (p2 * v023) + (p3 * w023);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Triangle023;
  }
  if (u123 > 0.0f && v123 > 0.0f && w123 > 0.0f && u <= 0.0f)
  {
    newSize = 3;
    newIndices[0] = 1;
    newIndices[1] = 2;
    newIndices[2] = 3;
    closestPoint = (p1 * u123) + (p2 * v123) + (p3 * w123);
    searchDirection = q - closestPoint;

    return VoronoiRegion::Triangle123;
  }

  newSize = 4;
  newIndices[0] = 0;
  newIndices[1] = 1;
  newIndices[2] = 2;
  newIndices[3] = 3;
  closestPoint = q;
  searchDirection = Vector3{ 0.0f };

  return VoronoiRegion::Tetrahedra0123;
}

Gjk::Gjk()
{
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB, unsigned int maxIterations, CsoPoint& closestPoint, float epsilon, int debuggingIndex, bool debugDraw)
{
  Vector3 searchDirection = shapeA->GetCenter() - shapeB->GetCenter();

  if (searchDirection == Vector3::cZero)
  {
    searchDirection = -Vector3::cXAxis;
  }
  searchDirection.Normalize();
  CsoPoint simplex[4];
  simplex[0] = ComputeSupport(shapeA, shapeB, searchDirection);
  searchDirection = -simplex[0].mCsoPoint;

  size_t newSize = 1;
  int newIndices[4]{ 0,0,0,0 };
  VoronoiRegion::Type vr;
  Vector3 P = Vector3::cZero;

  for (size_t i = 0; i < maxIterations; ++i)
  {
    Vector3 Q = Vector3::cZero;

    switch (newSize)
    {
    case 1:
      vr = IdentifyVoronoiRegion(Q, simplex[0].mCsoPoint, newSize, newIndices, P, searchDirection);
      break;
    case 2:
      vr = IdentifyVoronoiRegion(Q, simplex[0].mCsoPoint, simplex[1].mCsoPoint, newSize, newIndices, P, searchDirection);
      break;
    case 3:
      vr = IdentifyVoronoiRegion(Q, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[2].mCsoPoint, newSize, newIndices, P, searchDirection);
      break;
    case 4:
      vr = IdentifyVoronoiRegion(Q, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[2].mCsoPoint, simplex[3].mCsoPoint, newSize, newIndices, P, searchDirection);
      break;
    }

    if (P == Vector3::cZero)
    {
      return true;
    }

    switch (vr)
    {
    case VoronoiRegion::Point0:
    case VoronoiRegion::Point1:
    case VoronoiRegion::Point2:
    case VoronoiRegion::Point3:
      u = 0.0f;
      v = 0.0f;
      w = 0.0f;
      t = 0.0f;
      break;
    case VoronoiRegion::Edge01:
      BarycentricCoordinates(Q, simplex[0].mCsoPoint, simplex[1].mCsoPoint, u, v);
      break;
    case VoronoiRegion::Edge02:
      BarycentricCoordinates(Q, simplex[0].mCsoPoint, simplex[2].mCsoPoint, u, v);
      break;
    case VoronoiRegion::Edge03:
      BarycentricCoordinates(Q, simplex[0].mCsoPoint, simplex[3].mCsoPoint, u, v);
      break;
    case VoronoiRegion::Edge12:
      BarycentricCoordinates(Q, simplex[1].mCsoPoint, simplex[2].mCsoPoint, u, v);
      break;
    case VoronoiRegion::Edge13:
      BarycentricCoordinates(Q, simplex[1].mCsoPoint, simplex[3].mCsoPoint, u, v);
      break;
    case VoronoiRegion::Edge23:
      BarycentricCoordinates(Q, simplex[2].mCsoPoint, simplex[3].mCsoPoint, u, v);
      break;

    case VoronoiRegion::Triangle012:
      BarycentricCoordinates(Q, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[2].mCsoPoint, u, v, w);
      break;
    case VoronoiRegion::Triangle013:
      BarycentricCoordinates(Q, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[3].mCsoPoint, u, v, w);
      break;
    case VoronoiRegion::Triangle023:
      BarycentricCoordinates(Q, simplex[0].mCsoPoint, simplex[2].mCsoPoint, simplex[3].mCsoPoint, u, v, w);
      break;
    case VoronoiRegion::Triangle123:
      BarycentricCoordinates(Q, simplex[1].mCsoPoint, simplex[2].mCsoPoint, simplex[3].mCsoPoint, u, v, w);
      break;

    case VoronoiRegion::Tetrahedra0123:
      u = DistanceToPlane(Q, simplex[1].mCsoPoint, simplex[2].mCsoPoint, simplex[3].mCsoPoint) / DistanceToPlane(simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[2].mCsoPoint, simplex[3].mCsoPoint);
      v = DistanceToPlane(Q, simplex[0].mCsoPoint, simplex[2].mCsoPoint, simplex[3].mCsoPoint) / DistanceToPlane(simplex[1].mCsoPoint, simplex[0].mCsoPoint, simplex[2].mCsoPoint, simplex[3].mCsoPoint);
      w = DistanceToPlane(Q, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[3].mCsoPoint) / DistanceToPlane(simplex[2].mCsoPoint, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[3].mCsoPoint);
      t = 1.0f - u - v - w;
      break;
    }

    for (size_t j = 0; j < newSize; j++)
    {
      simplex[j] = simplex[newIndices[j]];
    }

    CsoPoint newPoint = ComputeSupport(shapeA, shapeB, searchDirection);

    if ((newPoint.mCsoPoint - P).Dot(-P.Normalized()) <= epsilon)
    {
      closestPoint.mCsoPoint = P;
      switch (vr)
      {
      case VoronoiRegion::Point0:
      case VoronoiRegion::Point1:
      case VoronoiRegion::Point2:
      case VoronoiRegion::Point3:
        closestPoint.mPointA = simplex[0].mPointA;
        closestPoint.mPointB = simplex[0].mPointB;
        break;
      case VoronoiRegion::Edge01:
      case VoronoiRegion::Edge02:
      case VoronoiRegion::Edge03:
      case VoronoiRegion::Edge12:
      case VoronoiRegion::Edge13:
      case VoronoiRegion::Edge23:
        closestPoint.mPointA = u * simplex[0].mPointA + v * simplex[1].mPointA;
        closestPoint.mPointB = u * simplex[0].mPointB + v * simplex[1].mPointB;
        break;
      case VoronoiRegion::Triangle012:
      case VoronoiRegion::Triangle013:
      case VoronoiRegion::Triangle023:
      case VoronoiRegion::Triangle123:
        closestPoint.mPointA = u * simplex[0].mPointA + v * simplex[1].mPointA + w * simplex[2].mPointA;
        closestPoint.mPointB = u * simplex[0].mPointB + v * simplex[1].mPointB + w * simplex[2].mPointB;
        break;
      case VoronoiRegion::Tetrahedra0123:
        closestPoint.mPointA = u * simplex[0].mPointA + v * simplex[1].mPointA + w * simplex[2].mPointA + t * simplex[3].mPointA;
        closestPoint.mPointB = u * simplex[0].mPointB + v * simplex[1].mPointB + w * simplex[2].mPointB + t * simplex[3].mPointB;
        break;
      }
      return false;
    }

    simplex[newSize++] = newPoint;
  }
  return false;
}

Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Vector3& direction)
{
  CsoPoint result;
  result.mPointA = shapeA->Support(direction);
  result.mPointB = shapeB->Support(-direction);
  result.mCsoPoint = result.mPointA - result.mPointB;
  return result;
}
