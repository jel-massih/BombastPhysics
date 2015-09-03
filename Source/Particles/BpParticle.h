#pragma once

#include "../Foundation/BpVec.h"
#include "../Foundation/BpMat.h"
#include <vector>
#include "../Geometry/BpPlane.h"

namespace bPhysics
{
		
	class BpParticle
	{
		struct IntegrationDerivative
		{
			BpVec3 velocity;
			BpVec3 force;
		};

	public:
		struct IntegrationState
		{
			//Primary State
			BpVec3 position;
			BpVec3 velocity;
			BpVec3 acceleration;

			f32 size;                    
			f32 mass;
			f32 inverseMass;

			void Recalculate()
			{
			}
		};

	public:
		BpParticle() : m_damping(0.001f) { SetMass(1.0f); m_currentState.size = 1; }

		void SetMass(f32 mass);
		void SetInverseMass(f32 inverseMass);

		void Simulate(f32 timestep);

		void AddForce(const BpVec3& force);

		BpVec3 GetPosition();
		void SetPosition(f32 x, f32 y, f32 z);
		
		BpVec3 GetVelocity();
		void SetVelocity(f32 x, f32 y, f32 z);

		BpVec3 GetAcceleration();
		void SetAcceleration(f32 x, f32 y, f32 z);

		void SetDamping(f32 damping);

		void ClearAccumulator();

		const IntegrationState& GetState() const;

	private:
		void Integrate(IntegrationState& initialState, f32 dt);

	private:
		IntegrationState m_currentState;

		BpVec3 m_gravity;

		//Holds force to be applied next step. Zeroed at each step
		BpVec3 m_accumulatedForce;

		f32 m_damping;
	};

}