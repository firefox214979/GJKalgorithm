///////////////////////////////////////////////////////////////////////////////
///
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

#define ShowDebugDrawWarnings true

DebugDrawer* gDebugDrawer = new DebugDrawer();

//-----------------------------------------------------------------------------DebugShape
DebugShape::DebugShape()
{
	mColor = Vector4(.6f);
	mMask = (unsigned int)-1;
	mTimer = 0;
	mOnTop = false;
	mTransform.SetIdentity();
}

DebugShape& DebugShape::Color(const Vector4& color)
{
	mColor = color;
	return *this;
}

DebugShape& DebugShape::OnTop(bool state)
{
	mOnTop = state;
	return *this;
}

DebugShape& DebugShape::Time(float time)
{
	mTimer = time;
	return *this;
}

DebugShape& DebugShape::SetMaskBit(int bitIndex)
{
	mMask = 1 << bitIndex;
	return *this;
}

DebugShape& DebugShape::SetTransform(const Matrix4& transform)
{
	mTransform = transform;
	return *this;
}

//-----------------------------------------------------------------------------DebugDrawer
DebugDrawer::DebugDrawer()
{
	mActiveMask = (unsigned int)-1;
	mApplication = NULL;
}

void DebugDrawer::Update(float dt)
{
	std::vector<DebugShape> newShapes;
	for (size_t i = 0; i < mShapes.size(); ++i)
	{
		DebugShape& shape = mShapes[i];
		shape.mTimer -= dt;

		// If the shape still has time left then add it to the list of shapes to keep drawing,
		// anything that has a timer that ran out will not be in the new list
		if (shape.mTimer >= 0)
			newShapes.push_back(shape);
	}

	mShapes.swap(newShapes);
}

void DebugDrawer::Draw()
{
	for (size_t i = 0; i < mShapes.size(); ++i)
	{
		DebugShape& shape = mShapes[i];

		// If the shape doesn't have one of the active mask bits set then don't draw it
		if ((shape.mMask & mActiveMask) == 0)
			continue;

		// If this shape always draws on top then disable depth testing
		if (shape.mOnTop)
			glDisable(GL_DEPTH_TEST);


		// Decompose the matrix to set the gl transform (too lazy to properly transform the matrix between formats)
		float radians;
		Vector3 scale, translation, axis;
		Matrix3 rotationMat;
		shape.mTransform.Decompose(&scale, &rotationMat, &translation);
		Math::ToAxisAngle(Math::ToQuaternion(rotationMat), &axis, &radians);
		glPushMatrix();
		// Set the transform
		glTranslatef(translation.x, translation.y, translation.z);
		glRotatef(Math::RadToDeg(radians), axis.x, axis.y, axis.z);
		glScalef(scale.x, scale.y, scale.z);

		glBegin(GL_LINES);
		glColor3fv(shape.mColor.array);

		// Draw all of the line segments of this shape
		for (size_t j = 0; j < shape.mSegments.size(); ++j)
		{
			LineSegment& segment = shape.mSegments[j];

			glVertex3fv(segment.mStart.array);
			glVertex3fv(segment.mEnd.array);
		}

		glEnd();
		glPopMatrix();

		// Make sure to re-enable depth testing
		if (shape.mOnTop)
			glEnable(GL_DEPTH_TEST);
	}
}

DebugShape& DebugDrawer::GetNewShape()
{
	mShapes.push_back(DebugShape());
	return mShapes.back();
}

DebugShape& DebugDrawer::DrawPoint(const Vector3& point)
{
	return DrawSphere(Sphere(point, 0.1f));
}

DebugShape& DebugDrawer::DrawLine(const LineSegment& line)
{
	DebugShape& shape = GetNewShape();
	shape.mSegments.push_back(line);
	return shape;
}

void DebugDrawer::DrawDisc(DebugShape& shape, const Plane& plane, const Vector3& point, float radius)
{
	const int numSegments = 32;
	const Vector3& normal = plane.GetNormal();
	Vector3 vectorV = Vector3(-normal.y, normal.x, 0.f);

	if (vectorV.Length() <= std::numeric_limits<float>::epsilon() || (std::abs(normal.x) <= std::numeric_limits<float>::epsilon() && std::abs(normal.y) <= std::numeric_limits<float>::epsilon()))
	{
		vectorV = Vector3(0.f, -normal.z, normal.y);
	}
    vectorV = vectorV.Normalized();

	Vector3 vectorW = Cross(normal, vectorV).Normalized();

	float angle = (M_PI * 2) / numSegments;
	Vector3 startPoint = point + vectorV * radius;
	Vector3 endPoint;

	for (int i = 1; i <= numSegments; ++i) 
	{
		float theta = angle * i;
		endPoint = point + vectorV * radius * Math::Cos(theta) + vectorW * radius * Math::Sin(theta);
		shape.mSegments.push_back(LineSegment(startPoint, endPoint));
		startPoint = endPoint;
	}
}

DebugShape& DebugDrawer::DrawRay(const Ray& ray, float t)
{
	DebugShape& shape = GetNewShape();

	Vector3 endPoint = ray.mStart + ray.mDirection * t;
	shape.mSegments.push_back(LineSegment(ray.mStart, endPoint));

	DrawDisc(shape, Plane(ray.mDirection.Normalized(), ray.mStart + ray.mDirection * (t * 0.9f)), ray.mStart + ray.mDirection * (t * 0.9f), t / 32.f);

	int numSegments = shape.mSegments.size() - 1;
	for (int index = 1; index <= numSegments; ++index)
	{
		const LineSegment& segment = shape.mSegments[index];
		shape.mSegments.push_back(LineSegment(endPoint, segment.mEnd));
	}
	return shape;
}

DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere)
{
	DebugShape& shape = GetNewShape();

	DrawDisc(shape, Plane(Vector3(1, 0, 0), sphere.mCenter), sphere.mCenter, sphere.mRadius);
	DrawDisc(shape, Plane(Vector3(0, 1, 0), sphere.mCenter), sphere.mCenter, sphere.mRadius);
	DrawDisc(shape, Plane(Vector3(0, 0, 1), sphere.mCenter), sphere.mCenter, sphere.mRadius);

	Vector3 cameraPos = Vector3(1, 1, 1);
	if (mApplication)
	{
		cameraPos = mApplication->mCamera.mTranslation;
	}

	Vector3 cameraVector = sphere.mCenter - cameraPos;
	float dist = cameraVector.Normalize();

	Vector3 centerPrime = sphere.mCenter - cameraVector * sphere.mRadius * sphere.mRadius / dist;

	float radiusPrime = Math::Sqrt(dist * dist - sphere.mRadius * sphere.mRadius) * sphere.mRadius / dist;

	DrawDisc(shape, Plane(cameraVector, centerPrime), sphere.mCenter, radiusPrime);

	return shape;
}

DebugShape& DebugDrawer::DrawAabb(const Aabb& aabb)
{
	DebugShape& shape = GetNewShape();
	const Vector3& min = aabb.mMin;
	const Vector3& max = aabb.mMax;
	std::vector<Vector3> vertices;
	vertices.push_back(Vector3(min.x, min.y, min.z));
	vertices.push_back(Vector3(max.x, min.y, min.z));
	vertices.push_back(Vector3(max.x, min.y, max.z));
	vertices.push_back(Vector3(min.x, min.y, max.z));
	vertices.push_back(Vector3(min.x, max.y, min.z));
	vertices.push_back(Vector3(max.x, max.y, min.z));
	vertices.push_back(Vector3(max.x, max.y, max.z));
	vertices.push_back(Vector3(min.x, max.y, max.z));
	for (int i = 0; i < 4; ++i)
	{
		shape.mSegments.push_back(LineSegment(vertices[i], vertices[(i + 1) % 4]));
		shape.mSegments.push_back(LineSegment(vertices[i + 4], vertices[((i + 1) % 4) + 4]));
		shape.mSegments.push_back(LineSegment(vertices[i], vertices[i + 4]));
	}
	return shape;
}

DebugShape& DebugDrawer::DrawTriangle(const Triangle& triangle)
{
	DebugShape& shape = GetNewShape();
	shape.mSegments.push_back(LineSegment(triangle.mPoints[0], triangle.mPoints[1]));
	shape.mSegments.push_back(LineSegment(triangle.mPoints[1], triangle.mPoints[2]));
	shape.mSegments.push_back(LineSegment(triangle.mPoints[2], triangle.mPoints[0]));
	return shape;
}

DebugShape& DebugDrawer::DrawPlane(const Plane& plane, float sizeX, float sizeY)
{
	const Vector3 point = plane.GetNormal() * plane.GetDistance();
	Ray normalRay = Ray(point, plane.GetNormal());
	DebugShape& shape = DrawRay(normalRay, 1.0f);

	const Vector3& normal = plane.GetNormal();
	Vector3 vectorV = Vector3(-normal.y, normal.x, 0);

	if (vectorV.Length() < std::numeric_limits<float>::epsilon() || (std::abs(normal.x) <= std::numeric_limits<float>::epsilon() && std::abs(normal.y) <= std::numeric_limits<float>::epsilon()))
	{
		vectorV = Vector3(0, -normal.z, normal.y);
	}
	vectorV = vectorV.Normalized();
	Vector3 vectorW = Math::Cross(normal, vectorV).Normalized();

	Vector3 pointV = vectorV * sizeX;
	Vector3 pointW = vectorW * sizeY;

	shape.mSegments.push_back(LineSegment(point + pointV + pointW, point + pointV - pointW));
	shape.mSegments.push_back(LineSegment(point + pointV - pointW, point - pointV - pointW));
	shape.mSegments.push_back(LineSegment(point - pointV - pointW, point - pointV + pointW));
	shape.mSegments.push_back(LineSegment(point - pointV + pointW, point + pointV + pointW));
	return shape;
}

DebugShape& DebugDrawer::DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
	DebugShape& shape = GetNewShape();
	shape.mSegments.push_back(LineSegment(p0, p1));
	shape.mSegments.push_back(LineSegment(p1, p2));
	shape.mSegments.push_back(LineSegment(p2, p3));
	shape.mSegments.push_back(LineSegment(p3, p0));
	return shape;
}

DebugShape& DebugDrawer::DrawFrustum(const Frustum& frustum)
{
	DebugShape& shape = GetNewShape();

	for (int i = 0; i < 4; ++i)
	{
		shape.mSegments.push_back(LineSegment(frustum.mPoints[i], frustum.mPoints[(i + 1) % 4]));
		shape.mSegments.push_back(LineSegment(frustum.mPoints[i + 4], frustum.mPoints[((i + 1) % 4) + 4]));
		shape.mSegments.push_back(LineSegment(frustum.mPoints[i], frustum.mPoints[i + 4]));
	}
	return shape;
}
