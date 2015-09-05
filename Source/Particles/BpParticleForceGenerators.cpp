#pragma once

#include "BpParticleForceGenerators.h"

using namespace bPhysics;

void BpParticleForceRegistry::UpdateForces(f32 dt)
{
	for (auto it = m_registrations.begin(); it != m_registrations.end(); it++)
	{
		it->forceGenerator->UpdateForce(it->particle, dt);
	}
}

void BpParticleForceRegistry::Add(BpParticle* particle, BpParticleForceGenerator* forceGenerator)
{
	ParticleForceRegistration registration;
	registration.particle = particle;
	registration.forceGenerator = forceGenerator;
	m_registrations.push_back(registration);
}

BpParticleGravity::BpParticleGravity(const BpVec3& gravity) : m_gravity(gravity)
{}

void BpParticleGravity::UpdateForce(BpParticle* particle, f32 dt)
{
	//Check for infinite mass
	if (!particle->GetState().inverseMass >= 0.0f) 
	{
		return;
	}

	particle->AddForce(m_gravity * particle->GetState().mass);
}