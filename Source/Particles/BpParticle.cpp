#include "BpParticle.h"

#include <stdio.h>
#include <assert.h>

using namespace bPhysics;

void BpParticle::SetMass(f32 mass)
{
	assert(mass != 0);
	m_currentState.inverseMass = (f32)1.0 / mass;
	m_currentState.Recalculate();
}

void BpParticle::SetInverseMass(f32 inverseMass)
{
	m_currentState.inverseMass = inverseMass;
	m_currentState.Recalculate();
}

void BpParticle::Simulate(const std::vector<BpPlane>& planes, f32 timestep)
{
	assert(timestep > 0);

	Integrate(planes, m_currentState, timestep);

	//zero accumulated force
	m_accumulatedForce.Zero();
}

void BpParticle::AddForce(const BpVec3& force)
{
	m_accumulatedForce += force;
}

BpVec3 BpParticle::GetPosition()
{
	return m_currentState.position;
}

void BpParticle::SetPosition(float x, float y, float z)
{
	m_currentState.position.x = x;
	m_currentState.position.y = y;
	m_currentState.position.z = z;
}

BpVec3 BpParticle::GetVelocity()
{
	return m_currentState.velocity;
}

void BpParticle::SetVelocity(float x, float y, float z)
{
	m_currentState.momentum.x = x;
	m_currentState.momentum.y = y;
	m_currentState.momentum.z = z;

	m_currentState.Recalculate();
}

BpVec3 BpParticle::GetGravity()
{
	return m_gravity;
}

void BpParticle::SetGravity(float x, float y, float z)
{
	m_gravity.x = x;
	m_gravity.y = y;
	m_gravity.z = z;
}

void BpParticle::SetDamping(float damping)
{
	m_damping = damping;
}

void BpParticle::ClearAccumulator()
{
	m_accumulatedForce.Zero();
}

void BpParticle::Integrate(const std::vector<BpPlane>& planes, IntegrationState& initialState, float dt)
{
	IntegrationDerivative a = Evaluate(planes, initialState, dt);
	IntegrationDerivative b = Evaluate(planes, initialState, dt * 0.5f, a);
	IntegrationDerivative c = Evaluate(planes, initialState, dt * 0.5f, b);
	IntegrationDerivative d = Evaluate(planes, initialState, dt, c);

	initialState.position += 1.0f / 6.0f * (a.velocity + 2.0f*(b.velocity + c.velocity) + d.velocity) * dt;
	initialState.momentum += 1.0f / 6.0f * (a.force + 2.0f*(b.force + c.force) + d.force) * dt;

	initialState.Recalculate();
}

BpParticle::IntegrationDerivative BpParticle::Evaluate(const std::vector<BpPlane>& planes, const IntegrationState& state, float dt)
{
	IntegrationDerivative output;
	output.velocity = state.velocity;
	Forces(planes, state, output.force);

	return output;
}

BpParticle::IntegrationDerivative BpParticle::Evaluate(const std::vector<BpPlane>& planes, IntegrationState state, float dt, const IntegrationDerivative& derivative)
{
	state.position += derivative.velocity * dt;
	state.momentum +=  derivative.force * dt;
	state.Recalculate();

	IntegrationDerivative output;
	output.velocity = state.velocity;
	Forces(planes, state, output.force);

	return output;
}

void BpParticle::Forces(const std::vector<BpPlane>& planes, const IntegrationState& state, BpVec3& force)
{
	force.Zero();

	//Apply Gravity
	force += m_gravity;

	//Apply Damping
	force -= m_damping * state.velocity;

	//Apply Collision
	Collision(planes, state, force);

	//Apply forces
	force += m_accumulatedForce;
}

void bPhysics::BpParticle::Collision(const std::vector<BpPlane>& planes, const IntegrationState & state, BpVec3 & force)
{
	BpVec3 a = state.bodyToWorld * (BpVec3(-1, -1, -1) * state.size * 0.5);
	BpVec3 b = state.bodyToWorld * (BpVec3(+1, -1, -1) * state.size * 0.5);
	BpVec3 c = state.bodyToWorld * (BpVec3(+1, +1, -1) * state.size * 0.5);
	BpVec3 d = state.bodyToWorld * (BpVec3(-1, +1, -1) * state.size * 0.5);
	BpVec3 e = state.bodyToWorld * (BpVec3(-1, -1, +1) * state.size * 0.5);
	BpVec3 f = state.bodyToWorld * (BpVec3(+1, -1, +1) * state.size * 0.5);
	BpVec3 g = state.bodyToWorld * (BpVec3(+1, +1, +1) * state.size * 0.5);
	BpVec3 h = state.bodyToWorld * (BpVec3(-1, +1, +1) * state.size * 0.5);

	for (unsigned int i = 0; i < planes.size(); i++)
	{
		CollisionForPoint(state, force, a, planes[i]);
		collisionForPoint(state, force, b, planes[i]);
		collisionForPoint(state, force, c, planes[i]);
		collisionForPoint(state, force, d, planes[i]);
		collisionForPoint(state, force, e, planes[i]);
		collisionForPoint(state, force, f, planes[i]);
		collisionForPoint(state, force, g, planes[i]);
		collisionForPoint(state, force, h, planes[i]);
	}
}

void BpParticle::CollisionForPoint(const IntegrationState &state, BpVec3 &force, const BpVec3 &point, const BpPlane &plane)
{
	const float c = 10;
	const float k = 100;
	const float b = 5;
	const float f = 3;

	const float penetration = plane.constant - point.dot(plane.normal);

	if (penetration > 0)
	{
		Vector velocity = state.velocity.cross(point - state.position) + state.velocity;
		assert(velocity == velocity);

		const float relativeSpeed = -plane.normal.dot(velocity);
		assert(relativeSpeed == relativeSpeed);

		if (relativeSpeed > 0)
		{
			Vector collisionForce = plane.normal * (relativeSpeed * c);
			assert(collisionForce == collisionForce);
			force += collisionForce;
			torque += (point - state.position).cross(collisionForce);
		}

		Vector tangentialVelocity = velocity + (plane.normal * relativeSpeed);
		Vector frictionForce = -tangentialVelocity * f;
		assert(frictionForce == frictionForce);
		force += frictionForce;
		torque += (point - state.position).cross(frictionForce);

		Vector penaltyForce = plane.normal * (penetration * k);
		assert(penaltyForce == penaltyForce);
		force += penaltyForce;
		torque += (point - state.position).cross(penaltyForce);

		Vector dampingForce = plane.normal * (relativeSpeed * penetration * b);
		assert(dampingForce == dampingForce);
		force += dampingForce;
		torque += (point - state.position).cross(dampingForce);
	}
}

const BpParticle::IntegrationState& BpParticle::GetState() const
{
	return m_currentState;
}