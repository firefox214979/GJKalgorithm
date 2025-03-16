/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: DynamicAabbTree.hpp
Purpose: DynamicAabbTree
Language: C++
Platform: Visual studio 2022
Project: jungyeon.lee_CS350_4
Author: jungyeon.lee
Creation date: 04.08.2024
End Header -------------------------------------------------------*/
#pragma once

#include "SpatialPartition.hpp"
#include "Shapes.hpp"

class Node
{
public:

  Node()
  {
    mAabb = Aabb();
    mClientData = nullptr;
    mLeft = nullptr;
    mRight = nullptr;
    mParent = nullptr;
    mHeight = 0;
    mLastAxis = 0;
  }

  Node(SpatialPartitionData& data)
  {
    mAabb = data.mAabb;
    mClientData = data.mClientData;
    mLeft = nullptr;
    mRight = nullptr;
    mParent = nullptr;
    mHeight = 0;
    mLastAxis = 0;
  }

  Aabb mAabb;
  void* mClientData;
  Node* mLeft = nullptr;
  Node* mRight = nullptr;
  Node* mParent = nullptr;
  size_t mHeight = 0;
  size_t mLastAxis = 0;

  bool isLeaf() const { return mLeft == nullptr && mRight == nullptr; }
};

/******Student:Assignment3******/
/// You must implement a dynamic aabb tree as we discussed in class.
class DynamicAabbTree : public SpatialPartition
{
public:
  DynamicAabbTree();
  ~DynamicAabbTree();

  // Spatial Partition Interface
  void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

  Node* FindLeaf(Node* node, SpatialPartitionKey& key);

  void BalanceTree(Node* startNode);
  void UpdateTree();

  void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;
  void FilloutDataNode(Node* node, std::vector<SpatialPartitionQueryData>& results, int depth) const;
  static const float mFatteningFactor;

  Node* mRoot = nullptr;

};
