#pragma once

#include "../Foundation/BpVec.h"

namespace bPhysics
{

struct BpPlane
{
	BpVec3 normal;
	float constant;

	BpPlane()
	{
		normal.Zero();
		constant = 0;
	}

	BpPlane(const BpVec3& normal, const BpVec3& point)
	{
		this->normal = normal;
		this->constant = normal.Dot(point);
	}

	BpPlane(const BpVec3& normal, float constant)
	{
		this->normal = normal;
		this->constant = constant;
	}

	void Normalize()
	{
		const float inverseLength = 1.0f / normal.Length();
		normal *= inverseLength;
		constant *= inverseLength;
	}

	void Clip(BpVec3& point, float distance = 0.0f)
	{
		const float d = (point.Dot(normal) - constant) - distance;

		if (d < 0) {
			point -= normal * d;
		}
	}
};

}