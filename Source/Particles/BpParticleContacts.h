#pragma once

#include "BpParticle.h"

namespace bPhysics
{
	//Contact represents two objects in contact (2 particles).
	class BpParticleContact
	{
		friend class BpParticleContactResolver;

	public:
		//Particles in contact
		BpParticle* particle[2];

		//normal restitution coefficient
		f32 restitution;

		BpVec3 contactNormal;

		//Holds depth of penetration of contact
		f32 penetrationDepth;

		//Hold amount particle moved by from interpenetration resolution
		BpVec3 particleMovement[2];

	protected:
		//Resolves this contact for velocity and interpenetration
		void Resolve(f32 duration);

		f32 CalculateSeparatingVelocity() const;

	private:
		void ResolveVelocity(f32 duration);
		void ResolveIntepenetration(f32 duration);
	};

	class BpParticleContactResolver
	{
	protected:
		unsigned iterations;
		unsigned iterationsUsed;

	public:
		BpParticleContactResolver(unsigned iterations);

		void SetIterations(unsigned iterations);

		void ResolveContacts(BpParticleContact* contactArray, unsigned numContacts, f32 duration);
	};

	class BpParicleContactGenerator
	{
	public:
		//Fill structure with generated contact.
		virtual unsigned AddContact(BpParticleContact* contact, unsigned limit) const = 0;
	};

}