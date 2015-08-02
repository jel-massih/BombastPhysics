#pragma once

#include "../Foundation/BpVec.h"

namespace bPhysics
{
		
	class BpParticle
	{
		struct IntegrationState
		{
			//Primary State
			BpVec3 position;
			BpVec3 momentum;

			//Secondary State
			BpVec3 velocity; //Calculated from momentum

			float mass;
			float inverseMass;

			void Recalculate()
			{
				velocity = momentum * inverseMass;
			}
		};

		struct IntegrationDerivative
		{
			BpVec3 velocity;
			BpVec3 force;
		};

	public:
		void SetMass(f32 mass);
		void SetInverseMass(f32 inverseMass);

		void Simulate(f32 currentTime, f32 timestep);

		void AddForce(const BpVec3& force);

		BpVec3 GetPosition();
		void SetPosition(float x, float y, float z);
		
		BpVec3 GetVelocity();
		void SetVelocity(float x, float y, float z);

		BpVec3 GetGravity();
		void SetGravity(float x, float y, float z);

		void SetDamping(float damping);

		void ClearAccumulator();

	private:
		void Integrate(IntegrationState& initialState, float t, float dt);

		IntegrationDerivative Evaluate(const IntegrationState& state, float t);
		IntegrationDerivative Evaluate(IntegrationState state, float t, float dt, const IntegrationDerivative& derivative);

		void Forces(const IntegrationState& state, BpVec3& force);

		IntegrationState m_currentState;

		BpVec3 m_gravity;

		//Holds force to be applied next step. Zeroed at each step
		BpVec3 m_accumulatedForce;

		f32 m_damping;
	};

}