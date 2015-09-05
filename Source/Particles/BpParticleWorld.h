#pragma once
#include "BpParticleContacts.h"
#include "BpParticleForceGenerators.h"

namespace bPhysics
{
	class BpParticleWorld
	{
	public:
		typedef std::vector<BpParticle*> Particles;
		typedef std::vector<BpParicleContactGenerator*> ContactGenerators;

	protected:
		Particles m_particles;
		
		bool m_calculateIterations;

		BpParticleForceRegistry m_registry;

		BpParticleContactResolver m_resolver;

		ContactGenerators m_contactGenerators;

		BpParticleContact* m_contacts;
		unsigned int m_maxContacts;

	public:
		BpParticleWorld(unsigned maxContacts, unsigned iterations = 0);
		~BpParticleWorld();

		//Calls each generator to report contacts, returns number of generated
		unsigned GenerateContacts();

		void Integrate(f32 dt);

		void RunPhysics(f32 dt);

		void StartFrame();

		Particles& GetParticles();

		ContactGenerators& GetContactGenerators();

		BpParticleForceRegistry& GetForceRegistry();
	};

	class GroundContacts : public BpParicleContactGenerator
	{
		BpParticleWorld::Particles* m_particles;

	public:
		void Init(BpParticleWorld::Particles* particles);

		virtual unsigned AddContact(BpParticleContact* contact, unsigned limit) const;
	};
}