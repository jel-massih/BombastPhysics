#pragma once

#include "BpParticleContacts.h"

namespace bPhysics
{
	class BpParticleLink : public BpParticleContactGenerator
	{
	public:
		BpParticle* particle[2];

	protected:
		f32 CurrentLength() const;

	public:
		virtual unsigned AddContact(BpParticleContact* contact, unsigned limit) const = 0;
	};

	class BpParticleCable : public BpParticleLink
	{
	public:
		f32 maxLength;
		f32 restitution;

	public:
		virtual unsigned AddContact(BpParticleContact* contact, unsigned limit) const;
	};

	class BpParticleRod : public BpParticleLink
	{
	public:
		f32 length;

	public:
		virtual unsigned AddContact(BpParticleContact* contact, unsigned limit) const;
	};

	class BpParticleConstraint : public BpParticleContactGenerator
	{
	public:
		BpParticle* particle;
		BpVec3 anchor;

	protected:
		f32 CurrentLength() const;

	public:
		virtual unsigned AddContact(BpParticleContact* contact, unsigned limit) const = 0;
	};

	class BpParticleCableConstraint : public BpParticleConstraint
	{
	public:
		f32 maxLength;
		f32 restitution;

	public:
		virtual unsigned AddContact(BpParticleContact* contact, unsigned limit) const;
	};

	class BpParticleRodConstraint : public BpParticleConstraint
	{
	public:
		f32 length;

	public:
		virtual unsigned AddContact(BpParticleContact* contact, unsigned limit) const;
	};
}