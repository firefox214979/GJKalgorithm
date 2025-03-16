/* Start Header ------------------------------------------------------
Copyright (C) 2024 DigiPen Institute of Technology.
File Name: BspTree.cpp
Purpose: BspTree
Language: C++
Platform: Visual studio 2022
Project: jungyeon.lee_CS350_4
Author: jungyeon.lee
Creation date: 04.08.2024
End Header -------------------------------------------------------*/

#include <vector>
#include <algorithm>
#include "Precompiled.hpp"
#include <stack>

BspTreeQueryData::BspTreeQueryData()
{
  mDepth = 0;
}

BspTree::~BspTree()
{
  if (mRoot != nullptr)
  {
    std::stack<BspNode*> nodeStack;
    nodeStack.push(mRoot);
    while (!nodeStack.empty())
    {
      BspNode* currentNode = nodeStack.top();
      nodeStack.pop();
      if (currentNode->mLeft != nullptr)
        nodeStack.push(currentNode->mLeft);
      if (currentNode->mRight != nullptr)
        nodeStack.push(currentNode->mRight);
      delete currentNode;
    }
  }
}

BspNode::BspNode()
{
  mSplitPlane = Plane();
  mParent = nullptr;
  mLeft = nullptr;
  mRight = nullptr;
  mData = BspTreeQueryData();
}

BspNode::~BspNode()
{
}

void BspTree::SplitTriangle(const Plane& plane, const Triangle& tri, TriangleList& coplanarFront, TriangleList& coplanarBack, TriangleList& front, TriangleList& back, float epsilon)
{
  Vector4 planeData = plane.mData;
  std::vector<Vector3> triVert = { tri.mPoints[0], tri.mPoints[1], tri.mPoints[2] };
  IntersectionType::Type type = PlaneTriangle(planeData, triVert[0], triVert[1], triVert[2], epsilon);

  switch (type)
  {
  case IntersectionType::Coplanar:
  {
    Plane triPlane(triVert[0], triVert[1], triVert[2]);
    float result = Math::Dot(triPlane.GetNormal(), plane.GetNormal());
    if (result >= 1 - epsilon && result <= 1 + epsilon)
    {
      coplanarFront.push_back(tri);
    }
    if (result >= -1 - epsilon && result <= -1 + epsilon)
    {
      coplanarBack.push_back(tri);
    }
    break;
  }
  case IntersectionType::Outside:
    back.push_back(tri);
    break;
  case IntersectionType::Inside:
    front.push_back(tri);
    break;
  case IntersectionType::Overlaps:
    std::vector<Vector3> frontPointList, backPointList;
    for (int i = 0; i < 3; i++)
    {
      Vector3 A = tri.mPoints[i % 3];
      Vector3 B = tri.mPoints[(i + 1) % 3];

      IntersectionType::Type vertA = PointPlane(A, plane.mData, epsilon);
      IntersectionType::Type vertB = PointPlane(B, plane.mData, epsilon);

      if (vertA == IntersectionType::Inside && vertB == IntersectionType::Inside)
      {
        frontPointList.push_back(B);
      }
      else if (vertA == IntersectionType::Coplanar && vertB == IntersectionType::Inside)
      {
        frontPointList.push_back(B);
      }
      else if (vertA == IntersectionType::Outside && vertB == IntersectionType::Inside)
      {
        float t;
        if (RayPlane(A, (B - A).Normalized(), plane.mData, t, epsilon))
        {
          Vector3 I = A + t * (B - A).Normalized();
          if (Math::Distance(A, I) <= Math::Distance(A, B))
          {
            frontPointList.push_back(I);
            frontPointList.push_back(B);
            backPointList.push_back(I);
          }
        }
      }
      else if (vertA == IntersectionType::Inside && vertB == IntersectionType::Coplanar)
      {
        frontPointList.push_back(B);
      }
      else if (vertA == IntersectionType::Coplanar && vertB == IntersectionType::Coplanar)
      {
        frontPointList.push_back(B);
      }
      else if (vertA == IntersectionType::Outside && vertB == IntersectionType::Coplanar)
      {
        frontPointList.push_back(B);
        backPointList.push_back(B);
      }
      else if (vertA == IntersectionType::Inside && vertB == IntersectionType::Outside)
      {
        float t;
        if (RayPlane(A, (B - A).Normalized(), plane.mData, t, epsilon))
        {
          Vector3 I = A + t * (B - A).Normalized();
          if (Math::Distance(A, I) <= Math::Distance(A, B))
          {
            frontPointList.push_back(I);
            backPointList.push_back(I);
            backPointList.push_back(B);
          }
        }
      }
      else if (vertA == IntersectionType::Coplanar && vertB == IntersectionType::Outside)
      {
        backPointList.push_back(A);
        backPointList.push_back(B);
      }
      else if (vertA == IntersectionType::Outside && vertB == IntersectionType::Outside)
      {
        backPointList.push_back(B);
      }
    }

    for (int i = 1; i + 1 < (int)frontPointList.size(); i++)
    {
      front.push_back(Triangle(frontPointList[0], frontPointList[i], frontPointList[i + 1]));
    }
    for (int i = 1; i + 1 < (int)backPointList.size(); i++)
    {
      back.push_back(Triangle(backPointList[0], backPointList[i], backPointList[i + 1]));
    }
    break;
  }
}

float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex, float k, float epsilon)
{
  float abDistance = Math::Distance(triangles[testIndex].mPoints[0], triangles[testIndex].mPoints[1]);
  float bcDistance = Math::Distance(triangles[testIndex].mPoints[1], triangles[testIndex].mPoints[2]);
  float caDistance = Math::Distance(triangles[testIndex].mPoints[2], triangles[testIndex].mPoints[0]);

  if (abDistance + bcDistance <= caDistance || abDistance + caDistance <= bcDistance || bcDistance + caDistance <= abDistance)
  {
    return Math::PositiveMax();
  }

  Plane plane(triangles[testIndex].mPoints[0], triangles[testIndex].mPoints[1], triangles[testIndex].mPoints[2]);
  int Ns = 0, Nf = 0, Nb = 0;

  for (int i = 0; i < (int)triangles.size(); i++)
  {
    if (i == testIndex)
    {
      continue;
    }

    IntersectionType::Type type = PlaneTriangle(plane.mData, triangles[i].mPoints[0], triangles[i].mPoints[1], triangles[i].mPoints[2], epsilon);

    switch (type)
    {
    case IntersectionType::Inside:
      Nf++;
      break;
    case IntersectionType::Outside:
      Nb++;
      break;
    case IntersectionType::Overlaps:
      Ns++;
      break;
    }
  }

  return k * Ns + (1 - k) * Math::Abs(Nf - Nb);
}

size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k, float epsilon)
{
  float bestScore = Math::PositiveMax();
  size_t bestIndex = 0;
  for (size_t i = 0; i < triangles.size(); i++)
  {
    float score = CalculateScore(triangles, i, k, epsilon);
    if (score < bestScore)
    {
      bestScore = score;
      bestIndex = i;
    }
  }
  return bestIndex;
}

void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
  mRoot = new BspNode();
  Construct_Aux(triangles, k, epsilon, mRoot, 0);
}

void BspTree::Construct_Aux(const TriangleList& triangles, float k, float epsilon, BspNode* node, int depth)
{
  if (triangles.size() == 0)
  {
    return;
  }
  size_t splitIndex = PickSplitPlane(triangles, k, epsilon);
  node->mSplitPlane = Plane(triangles[splitIndex].mPoints[0], triangles[splitIndex].mPoints[1], triangles[splitIndex].mPoints[2]);

  TriangleList coplanarTriList, frontTriList, backTriList;

  for (size_t i = 0; i < triangles.size(); i++)
  {
    SplitTriangle(node->mSplitPlane, triangles[i], coplanarTriList, coplanarTriList, frontTriList, backTriList, epsilon);
  }

  node->mData.mTriangles = coplanarTriList;

  if (frontTriList.size() > 0)
  {
    node->mLeft = new BspNode();
    node->mLeft->mParent = node;
    node->mLeft->mSplitPlane = Plane();
    node->mLeft->mData.mTriangles = frontTriList;
  }
  if (backTriList.size() > 0)
  {
    node->mRight = new BspNode();
    node->mRight->mParent = node;
    node->mRight->mSplitPlane = Plane();
    node->mRight->mData.mTriangles = backTriList;
  }
  if (node->mLeft != nullptr)
  {
    node->mLeft->mData.mDepth = depth + 1;
    Construct_Aux(node->mLeft->mData.mTriangles, k, epsilon, node->mLeft, depth + 1);
  }
  if (node->mRight != nullptr)
  {
    node->mRight->mData.mDepth = depth + 1;
    Construct_Aux(node->mRight->mData.mTriangles, k, epsilon, node->mRight, depth + 1);
  }
}

bool BspTree::RayCast(const Ray& ray, float& t, float planeEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
  return RayCast_Aux(mRoot, ray, 0, Math::PositiveMax(), t, planeEpsilon, triExpansionEpsilon, debuggingIndex);
}

bool BspTree::RayCast_Aux(BspNode* node, const Ray& ray, float tMin, float tMax, float& t, float planeEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
  if (node == nullptr)
  {
    return false;
  }

  IntersectionType::Type type = PointPlane(ray.mStart, node->mSplitPlane.mData, planeEpsilon);

  if (type == IntersectionType::Coplanar)
  {
    bool isFront = RayCast_Aux(node->mLeft, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, debuggingIndex);
    bool isBack = RayCast_Aux(node->mRight, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, debuggingIndex);

    bool isCoPlanar = false;
    float coplanarT;
    for (size_t i = 0; i < node->mData.mTriangles.size(); i++)
    {
      const Triangle& tri = node->mData.mTriangles[i];
      if (RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], coplanarT, triExpansionEpsilon))
      {
        if (coplanarT < t)
        {
          isCoPlanar = true;
          t = coplanarT;
        }
      }
    }

    return isBack || isFront || isCoPlanar;
  }

  BspNode* nearNode;
  BspNode* farNode;
  nearNode = (type != IntersectionType::Outside ? node->mLeft : node->mRight);
  farNode = (nearNode == node->mLeft ? node->mRight : node->mLeft);

  float planeT = Math::PositiveMax();
  if (RayPlane(ray.mStart, ray.mDirection, node->mSplitPlane.mData, planeT))
  {
    float te = planeEpsilon / Math::Abs(Math::Dot(node->mSplitPlane.GetNormal(), ray.mDirection) / (node->mSplitPlane.GetNormal().Length() * ray.mDirection.Length()));

    if (tMin - te <= planeT && planeT <= tMax + te)
    {
      bool isFront = RayCast_Aux(nearNode, ray, tMin, planeT + te, t, planeEpsilon, triExpansionEpsilon, debuggingIndex);
      bool isBack = RayCast_Aux(farNode, ray, planeT - te, tMax, t, planeEpsilon, triExpansionEpsilon, debuggingIndex);

      bool isCoPlanar = false;
      float coplanarT;
      for (size_t i = 0; i < node->mData.mTriangles.size(); i++)
      {
        const Triangle& tri = node->mData.mTriangles[i];
        if (RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], coplanarT, triExpansionEpsilon))
        {
          if (coplanarT < t)
          {
            isCoPlanar = true;
            t = coplanarT;
          }
        }
      }

      return isFront || isCoPlanar || isBack;
    }
    else if (planeT < 0.0f)
    {
      return RayCast_Aux(nearNode, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, debuggingIndex);
    }
    else if (tMax - te < planeT)
    {
      return RayCast_Aux(nearNode, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, debuggingIndex);
    }
    else if (0.0f < planeT && planeT < tMin + te)
    {
      return RayCast_Aux(farNode, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, debuggingIndex);
    }
  }
  else
  {
    return RayCast_Aux(nearNode, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, debuggingIndex);
  }

  return false;
}

void BspTree::AllTriangles(TriangleList& triangles) const
{
  AllTriangles_Aux(mRoot, triangles);
}

void BspTree::AllTriangles_Aux(BspNode* node, TriangleList& triangles) const
{
  if (node == nullptr)
  {
    return;
  }
  const int size = node->mData.mTriangles.size();
  for (int i = 0; i < size; i++)
  {
    triangles.push_back(node->mData.mTriangles[i]);
  }

  if (node->mLeft != nullptr)
  {
    AllTriangles_Aux(node->mLeft, triangles);
  }

  if (node->mRight != nullptr)
  {
    AllTriangles_Aux(node->mRight, triangles);
  }

}

void BspTree::Invert()
{
  Invert_Aux(mRoot);
}

void BspTree::Invert_Aux(BspNode* node)
{
  if (node == nullptr)
  {
    return;
  }
  BspNode* temp = node->mLeft;
  node->mLeft = node->mRight;
  node->mRight = temp;

  node->mSplitPlane.mData = -node->mSplitPlane.mData;

  size_t size = node->mData.mTriangles.size();
  for (size_t i = 0; i < size; i++)
  {
    std::swap(node->mData.mTriangles[i].mPoints[0], node->mData.mTriangles[i].mPoints[1]);
  }

  Invert_Aux(node->mLeft);
  Invert_Aux(node->mRight);
}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
  std::stack<BspNode*> nodeStack;
  nodeStack.push(mRoot);
  while (!nodeStack.empty())
  {
    BspNode* currentNode = nodeStack.top();
    nodeStack.pop();

    TriangleList result;
    clipTo_Aux(tree->mRoot, epsilon, currentNode->mData.mTriangles, result);
    currentNode->mData.mTriangles = result;

    if (currentNode->mLeft)
    {
      nodeStack.push(currentNode->mLeft);
    }
    if (currentNode->mRight)
    {
      nodeStack.push(currentNode->mRight);
    }
  }
}

void BspTree::clipTo_Aux(BspNode* node, float epsilon, TriangleList clipTriangle, TriangleList& outTriangle)
{
  if (node == nullptr)
  {
    return;
  }
  TriangleList frontTriList, backTriList;
  for (size_t i = 0; i < clipTriangle.size(); i++)
  {
    SplitTriangle(node->mSplitPlane, clipTriangle[i], frontTriList, backTriList, frontTriList, backTriList, epsilon);
  }

  if (node->mLeft == nullptr)
  {
    for (size_t i = 0; i < frontTriList.size(); i++)
    {
      outTriangle.push_back(frontTriList[i]);
    }
  }

  clipTo_Aux(node->mLeft, epsilon, frontTriList, outTriangle);
  clipTo_Aux(node->mRight, epsilon, backTriList, outTriangle);
}

void BspTree::Union(BspTree* tree, float k, float epsilon)
{
  this->ClipTo(tree, epsilon);
  tree->ClipTo(this, epsilon);
  tree->Invert();
  tree->ClipTo(this, epsilon);
  tree->Invert();
  TriangleList tris;
  this->AllTriangles(tris);
  tree->AllTriangles(tris);
  this->Construct(tris, k, epsilon);
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
  Invert();
  tree->Invert();
  Union(tree, k, epsilon);
  Invert();
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
  tree->Invert();
  this->Intersection(tree, k, epsilon);
}

void BspTree::FilloutData(std::vector<BspTreeQueryData>& results) const
{
  FillOutData_Aux(mRoot, results);
}

void BspTree::FillOutData_Aux(BspNode* node, std::vector<BspTreeQueryData>& results) const
{
  if (node == nullptr)
  {
    return;
  }
  results.push_back(node->mData);

  if (node->mLeft != nullptr)
  {
    FillOutData_Aux(node->mLeft, results);
  }

  if (node->mRight != nullptr)
  {
    FillOutData_Aux(node->mRight, results);
  }
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
  if (level >= -1)
  {
    DebugDraw_Aux(mRoot, level, color, bitMask, level);
  }
}

void BspTree::DebugDraw_Aux(BspNode* node, int level, const Vector4& color, int bitMask, int currentLevel)
{
  if (node == nullptr)
  {
    return;
  }

  if (level == -1 || currentLevel <= level)
  {
    for (size_t i = 0; i < node->mData.mTriangles.size(); i++)
    {
      DebugShape& triangle = gDebugDrawer->DrawTriangle(node->mData.mTriangles[i]);
      triangle.Color(color);
      triangle.SetMaskBit(bitMask);
    }
  }

  if (node->mLeft != nullptr)
  {
    DebugDraw_Aux(node->mLeft, level, color, bitMask, currentLevel + 1);
  }

  if (node->mRight != nullptr)
  {
    DebugDraw_Aux(node->mRight, level, color, bitMask, currentLevel + 1);
  }
}

void BspTree::FillOutNode(BspNode* node, std::vector<BspNode*> results) const
{
  if (node == nullptr)
  {
    return;
  }

  results.push_back(node);

  if (node->mLeft != nullptr)
  {
    FillOutNode(node->mLeft, results);
  }

  if (node->mRight != nullptr)
  {
    FillOutNode(node->mRight, results);
  }
}


