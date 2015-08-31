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
			BpVec3 momentum;

			//Secondary State
			BpVec3 velocity; //Calculated from momentum

			BpMat4x4 bodyToWorld;

			float size;                    
			float mass;
			float inverseMass;

			void Recalculate()
			{
				velocity = momentum * inverseMass;

				BpMat4x4 translation;
				translation.BuildTranslation(position);
				bodyToWorld = translation;
			}
		};

	public:
		BpParticle() : m_damping(0.001f) { SetMass(1.0f); m_currentState.size = 1; }

		void SetMass(f32 mass);
		void SetInverseMass(f32 inverseMass);

		void Simulate(const std::vector<BpPlane>& planes, f32 timestep);

		void AddForce(const BpVec3& force);

		BpVec3 GetPosition();
		void SetPosition(float x, float y, float z);
		
		BpVec3 GetVelocity();
		void SetVelocity(float x, float y, float z);

		BpVec3 GetGravity();
		void SetGravity(float x, float y, float z);

		void SetDamping(float damping);

		void ClearAccumulator();

		void CollisionForPoint(const IntegrationState & state, BpVec3 & force, const BpVec3 & point, const BpPlane & plane);

		const IntegrationState& GetState() const;

	private:
		void Integrate(const std::vector<BpPlane>& planes, IntegrationState& initialState, float dt);

		IntegrationDerivative Evaluate(const std::vector<BpPlane>& planes, const IntegrationState& state, float dt);
		IntegrationDerivative Evaluate(const std::vector<BpPlane>& planes, IntegrationState state, float dt, const IntegrationDerivative& derivative);

		void Forces(const std::vector<BpPlane>& planes, const IntegrationState& state, BpVec3& force);
		static void Collision(const std::vector<BpPlane>& planes, const IntegrationState & state, BpVec3 & force);

	private:
		IntegrationState m_currentState;

		BpVec3 m_gravity;

		//Holds force to be applied next step. Zeroed at each step
		BpVec3 m_accumulatedForce;

		f32 m_damping;
	};

}