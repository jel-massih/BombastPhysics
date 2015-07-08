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