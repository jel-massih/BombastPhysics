#include "BpParticleLinks.h"

using namespace bPhysics;

f32 BpParticleLink::CurrentLength() const 
{
	BpVec3 relativePos = particle[0]->GetPosition() - particle[1]->GetPosition();
	return relativePos.Length();
}

unsigned BpParticleCable::AddContact(BpParticleContact* contact, unsigned limit) const 
{
	f32 length = CurrentLength();

	if (length < maxLength)
	{
		return 0;
	}

	contact->particle[0] = particle[0];
	contact->particle[1] = particle[1];

	BpVec3 normal = particle[1]->GetPosition() - particle[0]->GetPosition();
	normal.Normalise();
	contact->contactNormal = normal;

	contact->penetrationDepth = length - maxLength;
	contact->restitution = restitution;

	return 1;
}

unsigned BpParticleRod::AddContact(BpParticleContact* contact, unsigned limit) const
{
	f32 curLen = CurrentLength();

	if (curLen == length)
	{
		return 0;
	}

	contact->particle[0] = particle[0];
	contact->particle[1] = particle[1];

	BpVec3 normal = particle[1]->GetPosition() - particle[0]->GetPosition();
	normal.Normalise();

	if (curLen > length)
	{
		contact->contactNormal = normal;
		contact->penetrationDepth = curLen - length;
	}
	else
	{
		contact->contactNormal = normal * -1;
		contact->penetrationDepth = length - curLen;
	}

	contact->restitution = 0;

	return 1;
}

f32 BpParticleConstraint::CurrentLength() const
{
	BpVec3 relativePos = particle->GetPosition() - anchor;
	return relativePos.Length();
}

unsigned BpParticleCableConstraint::AddContact(BpParticleContact* contact, unsigned limit) const
{
	f32 length = CurrentLength();

	if (length < maxLength)
	{
		return 0;
	}

	contact->particle[0] = particle;
	contact->particle[1] = nullptr;

	BpVec3 normal = anchor - particle->GetPosition();
	normal.Normalise();
	contact->contactNormal = normal;

	contact->penetrationDepth = length - maxLength;
	contact->restitution = restitution;

	return 1;
}

unsigned BpParticleRodConstraint::AddContact(BpParticleContact* contact, unsigned limit) const
{
	f32 currentLength = CurrentLength();

	if (currentLength == length)
	{
		return 0;
	}

	contact->particle[0] = particle;
	contact->particle[1] = nullptr;

	BpVec3 normal = anchor - particle->GetPosition();
	normal.Normalise();

	if (currentLength > length)
	{
		contact->contactNormal = normal;
		contact->penetrationDepth = currentLength - length;
	}
	else
	{
		contact->contactNormal = normal * -1;
		contact->penetrationDepth = length - currentLength;
	}

	contact->restitution = 0;

	return 1;
}

