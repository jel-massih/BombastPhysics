#pragma once

#include "../Foundation/BpVec.h"

namespace bPhysics
{
		
	class BpParticle
	{
	public:
		void SetMass(f32 mass);
		void SetInverseMass(f32 inverseMass);

		void Simulate(f32 timestep);

		void AddForce(const BpVec3& force);

	private:
		BpVec3 m_position;
		BpVec3 m_velocity;
		BpVec3 m_acceleration;

		//Holds force to be applied next step. Zeroed at each step
		BpVec3 m_accumulatedForce;

		f32 m_damping;
		f32 m_inverseMass;
	};

}