#pragma once

#include "BpParticle.h"

namespace bPhysics
{
	class BpParticleForceGenerator
	{
	public:
		virtual void UpdateForce(BpParticle* particle, f32 dt) = 0;
	};

	class BpParticleGravity : public BpParticleForceGenerator
	{
		BpVec3 m_gravity;

	public:
		BpParticleGravity(const BpVec3& gravity);

		virtual void UpdateForce(BpParticle* particle, f32 dt);
	};

	class BpParticleForceRegistry
	{
	protected:
		struct ParticleForceRegistration
		{
			BpParticle* particle;
			BpParticleForceGenerator* forceGenerator;
		};

		typedef std::vector<ParticleForceRegistration> Registry;
		Registry m_registrations;

	public:
		void Add(BpParticle* particle, BpParticleForceGenerator* forceGenerator);

		void Remove(BpParticle* particle, BpParticleForceGenerator* forceGenerator);

		void Clear();

		void UpdateForces(f32 dt);
	};
}