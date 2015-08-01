#include "BpParticle.h"

#include <assert.h>

using namespace bPhysics;

void BpParticle::SetMass(f32 mass)
{
	assert(mass != 0);
	m_inverseMass = (f32)1.0 / mass;
}

void BpParticle::SetInverseMass(f32 inverseMass)
{
	m_inverseMass = inverseMass;
}

void BpParticle::Simulate(f32 timestep)
{
	assert(timestep > 0);

	m_position += m_velocity * timestep;

	BpVec3 accelVec = m_acceleration;
	accelVec += m_accumulatedForce * m_inverseMass;

	m_velocity += accelVec * timestep;

	//Drag
	m_velocity *= BpPow(m_damping, timestep);

	//zero accumulated force
	m_accumulatedForce.Zero();
}

void BpParticle::AddForce(const BpVec3& force)
{
	m_accumulatedForce += force;
}

BpVec3 bPhysics::BpParticle::GetPosition()
{
	return m_position;
}

void bPhysics::BpParticle::SetPosition(float x, float y, float z)
{
	m_position.x = x;
	m_position.y = y;
	m_position.z = z;
}

BpVec3 bPhysics::BpParticle::GetVelocity()
{
	return m_velocity;
}

void bPhysics::BpParticle::SetVelocity(float x, float y, float z)
{
	m_velocity.x = x;
	m_velocity.y = y;
	m_velocity.z = z;
}

BpVec3 bPhysics::BpParticle::GetAcceleration()
{
	return m_acceleration;
}

void bPhysics::BpParticle::SetAcceleration(float x, float y, float z)
{
	m_acceleration.x = x;
	m_acceleration.y = y;
	m_acceleration.z = z;
}

void bPhysics::BpParticle::SetDamping(float damping)
{
	m_damping = daping;
}

void bPhysics::BpParticle::ClearAccumulator()
{
	m_accumulatedForce.Zero();
}
