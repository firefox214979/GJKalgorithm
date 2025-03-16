/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: DynamicAabbTree.cpp
Purpose: DynamicAabbTree
Language: C++
Platform: Visual studio 2022
Project: jungyeon.lee_CS350_4
Author: jungyeon.lee
Creation date: 04.08.2024
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

const float DynamicAabbTree::mFatteningFactor = 1.1f;



DynamicAabbTree::DynamicAabbTree()
{
  mType = SpatialPartitionTypes::AabbTree;
}

DynamicAabbTree::~DynamicAabbTree()
{
}


void DynamicAabbTree::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  key.mVoidKey = data.mClientData;

  Aabb fattenedAabb = data.mAabb;
  Vector3 fattenedValue = Vector3((fattenedAabb.GetMax() - fattenedAabb.GetMin()) * (mFatteningFactor - 1.f) * 0.5f);
  fattenedAabb.Expand(fattenedAabb.GetMin() - fattenedValue);
  fattenedAabb.Expand(fattenedAabb.GetMax() + fattenedValue);

  Node* newNode = new Node();
  newNode->mAabb = fattenedAabb;
  newNode->mClientData = data.mClientData;
  newNode->mLeft = nullptr;
  newNode->mRight = nullptr;
  newNode->mHeight = 0;

  if (mRoot == nullptr)
  {
    mRoot = newNode;
    return;
  }

  Node* currentNode = mRoot;
  while (!currentNode->isLeaf())
  {
    Node* left = currentNode->mLeft;
    Node* right = currentNode->mRight;

    Aabb combinedAabb0 = combinedAabb0.Combine(left->mAabb, newNode->mAabb);
    Aabb combinedAabb1 = combinedAabb1.Combine(right->mAabb, newNode->mAabb);

    float cost0 = combinedAabb0.GetSurfaceArea() - left->mAabb.GetSurfaceArea();
    float cost1 = combinedAabb1.GetSurfaceArea() - right->mAabb.GetSurfaceArea();

    if (cost0 < cost1)
    {
      currentNode = left;
    }
    else
    {
      currentNode = right;
    }

  }

  Node* newParent = new Node();
  newParent->mLeft = currentNode;
  newParent->mRight = newNode;
  newParent->mAabb = newParent->mAabb.Combine(currentNode->mAabb, newNode->mAabb);
  newParent->mParent = currentNode->mParent;

  Node* parentNode = newParent->mParent;
  while (parentNode)
  {
    parentNode = parentNode->mParent;
  }

  if (currentNode == mRoot)
  {
    mRoot = newParent;
  }
  else
  {
    if (currentNode->mParent->mLeft == currentNode)
    {
      currentNode->mParent->mLeft = newParent;
    }
    else
    {
      currentNode->mParent->mRight = newParent;
    }
  }

  currentNode->mParent = newParent;
  newNode->mParent = newParent;

  UpdateTree();
  BalanceTree(newParent);

  key.mVoidKey = newNode->mClientData;
}

Node* DynamicAabbTree::FindLeaf(Node* node, SpatialPartitionKey& key)
{
  if (node == nullptr)
  {
    return nullptr;
  }

  if (node->mClientData == key.mVoidKey && node->mHeight == 0)
  {
    return node;
  }

  Node* left = FindLeaf(node->mLeft, key);
  if (left != nullptr)
  {
    return left;
  }
  Node* right = FindLeaf(node->mRight, key);
  if (right != nullptr)
  {
    return right;
  }

  return nullptr;
}

void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  Node* mCurrent = mRoot;
  key.mVoidKey = data.mClientData;

  Node* target = FindLeaf(mCurrent, key);

  if (target == nullptr)
  {
    return;
  }

  Aabb fattenedAabb = data.mAabb;
  Vector3 fattenedValue = Vector3((fattenedAabb.GetMax() - fattenedAabb.GetMin()) * (mFatteningFactor - 1.f) * 0.5f);
  fattenedAabb.Expand(fattenedAabb.GetMin() - fattenedValue);
  fattenedAabb.Expand(fattenedAabb.GetMax() + fattenedValue);

  if (!target->mAabb.Contains(fattenedAabb))
  {
    RemoveData(key);
    InsertData(key, data);
  }
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
  Node* node = FindLeaf(mRoot, key);
  Node* remainNode;
  if (node->mClientData == key.mVoidKey && node->mHeight == 0)
  {
    if (node == mRoot)
    {
      delete node;
      mRoot = nullptr;

    }
    else if (node->mParent == mRoot)
    {
      if (node->mParent->mLeft->mClientData == key.mVoidKey)
      {
        Node* parentNode = node->mParent;
        delete parentNode->mLeft;
        parentNode->mLeft = nullptr;
        mRoot = parentNode->mRight;
        delete parentNode;
        parentNode = nullptr;

      }
      else
      {
        Node* parentNode = node->mParent;
        delete parentNode->mRight;
        parentNode->mRight = nullptr;
        mRoot = parentNode->mLeft;
        delete parentNode;
        parentNode = nullptr;

      }
    }
    else if (node->mParent->mLeft->mClientData == key.mVoidKey)
    {
      Node* parentNode = node->mParent;
      delete parentNode->mLeft;
      parentNode->mLeft = nullptr;
      parentNode->mRight->mParent = parentNode->mParent;
      if (parentNode->mParent->mLeft == parentNode)
      {
        parentNode->mParent->mLeft = parentNode->mRight;
      }
      else
      {
        parentNode->mParent->mRight = parentNode->mRight;
      }
      remainNode = parentNode->mRight;
      delete parentNode;
      parentNode = nullptr;

      UpdateTree();
      BalanceTree(remainNode->mParent);
    }
    else
    {
      Node* parentNode = node->mParent;

      delete parentNode->mRight;
      parentNode->mRight = nullptr;
      parentNode->mLeft->mParent = parentNode->mParent;
      if (parentNode->mParent->mLeft == parentNode)
      {
        parentNode->mParent->mLeft = parentNode->mLeft;
      }
      else
      {
        parentNode->mParent->mRight = parentNode->mLeft;
      }
      remainNode = parentNode->mLeft;
      delete parentNode;
      parentNode = nullptr;

      UpdateTree();
      BalanceTree(remainNode->mParent);
    }
  }
}

void DynamicAabbTree::BalanceTree(Node* startNode)
{
  if (abs(static_cast<int>(startNode->mLeft->mHeight) - static_cast<int>(startNode->mRight->mHeight)) >= 2)
  {
    Node* grandParentNode = startNode->mParent;
    Node* parentNode = startNode;
    Node* pivotNode;
    bool isLeftPivot;
    if (startNode->mLeft->mHeight < startNode->mRight->mHeight)
    {
      pivotNode = startNode->mRight;
      isLeftPivot = false;
    }
    else
    {
      pivotNode = startNode->mLeft;
      isLeftPivot = true;
    }
    Node* smallNode;
    bool isLeftSmall;
    if (pivotNode->mLeft->mHeight < pivotNode->mRight->mHeight)
    {
      smallNode = pivotNode->mLeft;
      isLeftSmall = true;
    }
    else
    {
      smallNode = pivotNode->mRight;
      isLeftSmall = false;
    }

    if (grandParentNode)
    {
      if (grandParentNode->mLeft == parentNode)
      {
        grandParentNode->mLeft = pivotNode;
      }
      else
      {
        grandParentNode->mRight = pivotNode;
      }
      pivotNode->mParent = grandParentNode;
    }
    else
    {
      mRoot = pivotNode;
      pivotNode->mParent = nullptr;
    }

    if (isLeftSmall)
    {
      pivotNode->mLeft = parentNode;
    }
    else
    {
      pivotNode->mRight = parentNode;
    }
    parentNode->mParent = pivotNode;

    if (isLeftPivot)
    {
      parentNode->mLeft = smallNode;
    }
    else
    {
      parentNode->mRight = smallNode;
    }
    smallNode->mParent = parentNode;

    UpdateTree();

  }
  else if (startNode == mRoot)
  {
    UpdateTree();
    return;
  }

  BalanceTree(startNode->mParent);

}

void UpdateAabb(Node* node)
{
  if (node == nullptr) return;

  UpdateAabb(node->mLeft);
  UpdateAabb(node->mRight);

  if (!node->isLeaf())
  {
    node->mAabb = node->mAabb.Combine(node->mLeft->mAabb, node->mRight->mAabb);
  }
}

int UpdateHeight(Node* node)
{
  if (node == nullptr) return -1;

  int leftHeight = UpdateHeight(node->mLeft);
  int rightHeight = UpdateHeight(node->mRight);
  node->mHeight = 1 + std::max(leftHeight, rightHeight);

  return node->mHeight;
}

void DynamicAabbTree::UpdateTree()
{
  UpdateAabb(mRoot);
  UpdateHeight(mRoot);
}

void DebugDrawNode(Node* node, int level, const Math::Matrix4& transform, const Vector4& color, int bitMask, int depth)
{
  if (level == -1)
  {
    DebugShape& aabbDrawer = gDebugDrawer->DrawAabb(node->mAabb);
    aabbDrawer.SetTransform(transform);
    aabbDrawer.Color(color);
    aabbDrawer.SetMaskBit(bitMask);
    if (node->mHeight == 0)
    {
      return;
    }
    DebugDrawNode(node->mLeft, level, transform, color, bitMask, depth + 1);
    DebugDrawNode(node->mRight, level, transform, color, bitMask, depth + 1);
  }
  else if (depth < level + 1)
  {
    DebugShape& aabbDrawer = gDebugDrawer->DrawAabb(node->mAabb);
    aabbDrawer.SetTransform(transform);
    aabbDrawer.Color(color);
    aabbDrawer.SetMaskBit(bitMask);
    if (node->mHeight == 0)
    {
      return;
    }
    DebugDrawNode(node->mLeft, level, transform, color, bitMask, depth + 1);
    DebugDrawNode(node->mRight, level, transform, color, bitMask, depth + 1);
  }
  else
  {
    return;
  }
}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  if (mRoot == nullptr)
  {
    return;
  }
  DebugDrawNode(mRoot, level, transform, color, bitMask, 0);
}

void CastRayNode(Node* node, const Ray& ray, CastResults& results)
{
  if (node == nullptr)
  {
    return;
  }
  float t = 0.0f;

  if (RayAabb(ray.mStart, ray.mDirection, node->mAabb.GetMin(), node->mAabb.GetMax(), t))
  {
    if (node->mHeight == 0)
    {
      CastResult result;
      result.mClientData = node->mClientData;
      result.mTime = t;
      results.AddResult(result);
    }
    else
    {
      CastRayNode(node->mLeft, ray, results);
      CastRayNode(node->mRight, ray, results);
    }
  }
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
  if (mRoot == nullptr)
  {
    return;
  }
  CastRayNode(mRoot, ray, results);
}

void FrustumCastNode_Aux(Node* node, CastResults& results)
{
  if (node == nullptr)
  {
    return;
  }
  if (node->mHeight == 0)
  {
    CastResult result;
    result.mClientData = node->mClientData;
    results.AddResult(result);
  }
  else
  {
    FrustumCastNode_Aux(node->mLeft, results);
    FrustumCastNode_Aux(node->mRight, results);
  }
}

void FrustumCastNode(Node* node, const Frustum& frustum, CastResults& results)
{
  if (node == nullptr)
  {
    return;
  }
  IntersectionType::Type type = FrustumAabb(frustum.GetPlanes(), node->mAabb.GetMin(), node->mAabb.GetMax(), node->mLastAxis);

  if (type == IntersectionType::Inside)
  {
    FrustumCastNode_Aux(node, results);
  }
  else if (type == IntersectionType::Overlaps)
  {
    if (node->mHeight == 0)
    {
      CastResult result;
      result.mClientData = node->mClientData;
      results.AddResult(result);
    }
    else
    {
      if (node->mHeight == 0)
      {
        results.AddResult(CastResult(node->mClientData));
        return;
      }
      FrustumCastNode(node->mLeft, frustum, results);
      FrustumCastNode(node->mRight, frustum, results);
    }
  }
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
  if (mRoot == nullptr)
  {
    return;
  }
  FrustumCastNode(mRoot, frustum, results);
}

void QueryNode(Node* left, Node* right, QueryResults& results);

void SplitNode(Node* left, Node* right, QueryResults& results)
{
  if (left->mHeight == 0)
  {
    QueryNode(left, right->mLeft, results);
    QueryNode(left, right->mRight, results);
  }
  else if (right->mHeight == 0)
  {
    QueryNode(left->mLeft, right, results);
    QueryNode(left->mRight, right, results);
  }
  else
  {
    if (left->mAabb.GetVolume() < right->mAabb.GetVolume())
    {
      QueryNode(left, right->mLeft, results);
      QueryNode(left, right->mRight, results);
    }
    else
    {
      QueryNode(left->mLeft, right, results);
      QueryNode(left->mRight, right, results);
    }
  }
}

void QueryNode(Node* left, Node* right, QueryResults& results)
{
  if (!AabbAabb(left->mAabb.GetMin(), left->mAabb.GetMax(), right->mAabb.GetMin(), right->mAabb.GetMax()))
  {
    return;
  }

  if (left->mHeight == 0 && right->mHeight == 0)
  {
    results.AddResult(QueryResult(left->mClientData, right->mClientData));
    return;
  }

  SplitNode(left, right, results);
}

void QueryNode(Node* node, QueryResults& results)
{
  if (node == nullptr)
  {
    return;
  }
  if (node->mHeight == 0)
  {
    return;
  }
  QueryNode(node->mLeft, results);
  QueryNode(node->mRight, results);
  QueryNode(node->mLeft, node->mRight, results);
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
  if (mRoot == nullptr)
  {
    return;
  }
  QueryNode(mRoot, results);
}

void DynamicAabbTree::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  Node* mCurrent = mRoot;
  FilloutDataNode(mCurrent, results, 0);
}

void DynamicAabbTree::FilloutDataNode(Node* node, std::vector<SpatialPartitionQueryData>& results, int depth) const
{
  if (node == nullptr)
  {
    return;
  }
  SpatialPartitionQueryData data;
  data.mAabb = node->mAabb;
  data.mClientData = node->mClientData;
  data.mDepth = depth;
  results.push_back(data);
  FilloutDataNode(node->mLeft, results, depth + 1);
  FilloutDataNode(node->mRight, results, depth + 1);
}

