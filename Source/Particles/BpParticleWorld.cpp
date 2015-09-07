#include "BpParticleWorld.h"
#include "../BpPhysicsStd.h"

using namespace bPhysics;

BpParticleWorld::BpParticleWorld(unsigned maxContacts, unsigned iterations)
	:m_resolver(iterations), m_maxContacts(maxContacts)
{
	m_contacts = BP_NEW BpParticleContact[m_maxContacts];
	m_calculateIterations = (iterations == 0);
}

BpParticleWorld::~BpParticleWorld()
{
	delete[] m_contacts;
}

void BpParticleWorld::StartFrame()
{
	for (auto it = m_particles.begin(); it != m_particles.end(); it++)
	{
		(*it)->ClearAccumulator();
	}
}

unsigned BpParticleWorld::GenerateContacts()
{
	unsigned limit = m_maxContacts;
	BpParticleContact* nextContact = m_contacts;

	for (auto it = m_contactGenerators.begin(); it != m_contactGenerators.end(); it++)
	{
		unsigned used = (*it)->AddContact(nextContact, limit);
		limit -= used;
		nextContact += used;

		if (limit <= 0) break;
	}

	return m_maxContacts - limit;
}

void BpParticleWorld::Integrate(f32 dt)
{
	for (auto it = m_particles.begin(); it != m_particles.end(); it++)
	{
		(*it)->Simulate(dt);
	}
}

void BpParticleWorld::RunPhysics(f32 dt)
{
	m_registry.UpdateForces(dt);

	Integrate(dt);

	unsigned usedContacts = GenerateContacts();
		
	if (usedContacts)
	{
		if (m_calculateIterations)
		{
			m_resolver.SetIterations(usedContacts * 2);
		}

		m_resolver.ResolveContacts(m_contacts, usedContacts, dt);
	}
}

BpParticleWorld::Particles& BpParticleWorld::GetParticles()
{
	return m_particles;
}

BpParticleWorld::ContactGenerators& BpParticleWorld::GetContactGenerators()
{
	return m_contactGenerators;
}

BpParticleForceRegistry& BpParticleWorld::GetForceRegistry()
{
	return m_registry;
}

void BpParticleGroundContacts::Init(BpParticleWorld::Particles* particles)
{
	m_particles = particles;
}

unsigned BpParticleGroundContacts::AddContact(BpParticleContact* contact, unsigned limit) const
{
	unsigned count = 0;
	for (auto it = m_particles->begin(); it != m_particles->end(); it++)
	{
		f32 y = (*it)->GetPosition().y;
		if (y < 0.0f)
		{
			contact->contactNormal = BpVec3(0, 1, 0);
			contact->particle[0] = *it;
			contact->particle[1] = NULL;
			contact->penetrationDepth = -y;
			contact->restitution = 0.2f;
			contact++;
			count++;
		}

		if (count >= limit) 
			return count;
	}

	return count;
}