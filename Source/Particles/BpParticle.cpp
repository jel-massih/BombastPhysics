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

void BpParticle::Simulate(f32 currentTime, f32 timestep)
{
	assert(timestep > 0);

	Integrate(m_currentState, currentTime, timestep);

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

void BpParticle::Integrate(IntegrationState& initialState, float t, float dt)
{
	IntegrationDerivative a = Evaluate(initialState, t);
	IntegrationDerivative b = Evaluate(initialState, t, dt * 0.5f, a);
	IntegrationDerivative c = Evaluate(initialState, t, dt * 0.5f, b);
	IntegrationDerivative d = Evaluate(initialState, t, dt, c);

	initialState.position += 1.0f / 6.0f * (a.velocity + 2.0f*(b.velocity + c.velocity) + d.velocity) * dt;
	initialState.momentum += 1.0f / 6.0f * (a.force + 2.0f*(b.force + c.force) + d.force) * dt;

	initialState.Recalculate();
}

BpParticle::IntegrationDerivative BpParticle::Evaluate(const IntegrationState& state, float t)
{
	IntegrationDerivative output;
	output.velocity = state.velocity;
	Forces(state, output.force);

	return output;
}

BpParticle::IntegrationDerivative BpParticle::Evaluate(IntegrationState state, float t, float dt, const IntegrationDerivative& derivative)
{
	state.position += derivative.velocity * dt;
	state.momentum +=  derivative.force * dt;
	state.Recalculate();

	IntegrationDerivative output;
	output.velocity = state.velocity;
	Forces(state, output.force);

	return output;
}

void BpParticle::Forces(const IntegrationState& state, BpVec3& force)
{
	force.Zero();

	//Apply Gravity
	force += m_gravity;

	//Apply Damping
	force -= m_damping * state.velocity;

	//Apply forces
	force += m_accumulatedForce;
}