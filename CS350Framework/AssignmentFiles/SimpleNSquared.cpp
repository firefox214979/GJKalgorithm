/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: SimpleNSquared.cpp
Purpose: SimpleNSquared
Language: C++
Platform: Visual studio 2022
Project: jungyeon.lee_CS350_3
Author: jungyeon.lee
Creation date: 02.16.2024
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------NSquaredSpatialPartition
NSquaredSpatialPartition::NSquaredSpatialPartition()
{
  mType = SpatialPartitionTypes::NSquared;
}

void NSquaredSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  // Doing this lazily (and bad, but it's n-squared...).
  // Just store as the key what the client data is so we can look it up later.
  key.mVoidKey = data.mClientData;
  mData.push_back(data.mClientData);
}

void NSquaredSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  for (size_t i = 0; i < mData.size(); ++i)
  {
    if (mData[i] == key.mVoidKey)
    {
      mData[i] = data.mClientData;
      break;
    }
  }
}

void NSquaredSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
  // Find the key data and remove it
  for (size_t i = 0; i < mData.size(); ++i)
  {
    if (mData[i] == key.mVoidKey)
    {
      mData[i] = mData.back();
      mData.pop_back();
      break;
    }
  }
}

void NSquaredSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  // Nothing to debug draw
}

void NSquaredSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
  // Add everything
  for (size_t i = 0; i < mData.size(); ++i)
  {
    CastResult result;
    result.mClientData = mData[i];
    results.AddResult(result);
  }
}

void NSquaredSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
  // Add everything
  for (size_t i = 0; i < mData.size(); ++i)
  {
    CastResult result;
    result.mClientData = mData[i];
    results.AddResult(result);
  }
}

void NSquaredSpatialPartition::SelfQuery(QueryResults& results)
{
  // Add everything
  for (size_t i = 0; i < mData.size(); ++i)
  {
    for (size_t j = i + 1; j < mData.size(); ++j)
    {
      results.AddResult(QueryResult(mData[i], mData[j]));
    }
  }
}

void NSquaredSpatialPartition::GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const
{
  data.mClientData = key.mVoidKey;
}

void NSquaredSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  for (size_t i = 0; i < mData.size(); ++i)
  {
    SpatialPartitionQueryData data;
    data.mClientData = mData[i];
    results.push_back(data);
  }
}

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
BoundingSphereSpatialPartition::BoundingSphereSpatialPartition()
{
  mType = SpatialPartitionTypes::NSquaredSphere;
}

void BoundingSphereSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  key.mVoidKey = data.mClientData;
  mData.push_back(data.mClientData);
  mSpheres.push_back(data.mBoundingSphere);
}

void BoundingSphereSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  for (size_t i = 0; i < mData.size(); ++i)
  {
    if (mData[i] == key.mVoidKey)
    {
      mData[i] = data.mClientData;
      mSpheres[i] = data.mBoundingSphere;
      break;
    }
  }
}

void BoundingSphereSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
  for (size_t i = 0; i < mData.size(); ++i)
  {
    if (mData[i] == key.mVoidKey)
    {
      mData[i] = mData.back();
      mSpheres[i] = mSpheres.back();
      mData.pop_back();
      mSpheres.pop_back();
      break;
    }
  }
}

void BoundingSphereSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  Warn("Assignment2: Required function un-implemented");
}

void BoundingSphereSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
  for (size_t i = 0; i < mData.size(); ++i)
  {
    float t = 0.0f;
    if (RaySphere(ray.mStart, ray.mDirection, mSpheres[i].GetCenter(), mSpheres[i].GetRadius(), t))
    {
      CastResult result;
      result.mTime = t;
      result.mClientData = mData[i];
      results.AddResult(result);
    }
  }
}

void BoundingSphereSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
  for (size_t i = 0; i < mData.size(); ++i)
  {
    size_t t = 0;
    IntersectionType::Type type = FrustumSphere(frustum.GetPlanes(), mSpheres[i].GetCenter(), mSpheres[i].GetRadius(), t);
    if (type == IntersectionType::Overlaps || type == IntersectionType::Inside)
    {
      CastResult result;
      result.mClientData = mData[i];
      results.AddResult(result);
    }
  }
}

void BoundingSphereSpatialPartition::SelfQuery(QueryResults& results)
{
  for (size_t i = 0; i < mData.size(); ++i)
  {
    for (size_t j = i + 1; j < mData.size(); ++j)
    {
      if (SphereSphere(mSpheres[i].GetCenter(), mSpheres[i].GetRadius(), mSpheres[j].GetCenter(), mSpheres[j].GetRadius()))
      {
        results.AddResult(QueryResult(mData[i], mData[j]));
      }
    }
  }
}

void BoundingSphereSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  for (size_t i = 0; i < mData.size(); ++i)
  {
    SpatialPartitionQueryData data;
    data.mClientData = mData[i];
    data.mBoundingSphere = mSpheres[i];
    results.push_back(data);
  }
}
