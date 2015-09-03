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

void BpParticle::Simulate(f32 timestep)
{
	assert(timestep > 0);

	Integrate(m_currentState, timestep);

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

void BpParticle::SetPosition(f32 x, f32 y, f32 z)
{
	m_currentState.position.x = x;
	m_currentState.position.y = y;
	m_currentState.position.z = z;
}

BpVec3 BpParticle::GetVelocity()
{
	return m_currentState.velocity;
}

void BpParticle::SetVelocity(f32 x, f32 y, f32 z)
{
	m_currentState.velocity.x = x;
	m_currentState.velocity.y = y;
	m_currentState.velocity.z = z;

	m_currentState.Recalculate();
}

BpVec3 BpParticle::GetAcceleration()
{
	return m_currentState.acceleration;
}

void BpParticle::SetAcceleration(f32 x, f32 y, f32 z)
{
	m_currentState.acceleration.x = x;
	m_currentState.acceleration.y = y;
	m_currentState.acceleration.z = z;
}

void BpParticle::SetDamping(f32 damping)
{
	m_damping = damping;
}

void BpParticle::ClearAccumulator()
{
	m_accumulatedForce.Zero();
}

void BpParticle::Integrate(IntegrationState& initialState, f32 dt)
{
	if (m_currentState.inverseMass <= 0.0f) return;

	assert(dt > 0.0);

	initialState.position += initialState.velocity * dt;

	BpVec3 resultingAcceleration = initialState.acceleration;
	resultingAcceleration += m_accumulatedForce * initialState.inverseMass;

	initialState.velocity += resultingAcceleration * dt;

	initialState.velocity += pow(m_damping, dt);

	ClearAccumulator();

	initialState.Recalculate();
}

const BpParticle::IntegrationState& BpParticle::GetState() const
{
	return m_currentState;
}