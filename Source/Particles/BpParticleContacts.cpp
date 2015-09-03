#include "BpParticleContacts.h"

using namespace bPhysics;

void BpParticleContact::Resolve(f32 duration)
{
	ResolveVelocity(duration);
	ResolveInterpenetration(duration);
}

f32 BpParticleContact::CalculateSeparatingVelocity() const
{
	BpVec3 relativeVelocity = particle[0]->GetVelocity();
	
	if (particle[1] != nullptr)
	{
		relativeVelocity -= particle[1]->GetVelocity();
	}

	return relativeVelocity.Dot(contactNormal);
}

void BpParticleContact::ResolveVelocity(f32 duration)
{
	f32 separatingVelocity = CalculateSeparatingVelocity();

	//If > 0, contact is separating or stationary and does not need to be resolved
	if (separatingVelocity > 0)
	{
		return;
	}

	f32 newSeparatingVelocity = -separatingVelocity * restitution;

	BpVec3 accelerationCausedVelocity = particle[0]->GetAcceleration();
	if (particle[1] != nullptr)
	{
		accelerationCausedVelocity -= particle[1]->GetAcceleration();
	}
	f32 accelerationCausedSepVelocity = accelerationCausedVelocity.Dot(contactNormal) * duration;

	//If closing velocity from acceleration buildup, remove it from separating velocity
	if (accelerationCausedSepVelocity < 0)
	{
		newSeparatingVelocity += restitution * accelerationCausedSepVelocity;
		if (newSeparatingVelocity < 0)
			newSeparatingVelocity = 0;
	}

	f32 deltaVelocity = newSeparatingVelocity - separatingVelocity;

	f32 totalInverseMass = particle[0]->GetState().inverseMass;
	if (particle[1] != nullptr) totalInverseMass += particle[1]->GetState().inverseMass;

	if (totalInverseMass <= 0)
		return;

	f32 impules = deltaVelocity / totalInverseMass;

	BpVec3 impulsePerIMass = contactNormal.Dot(impules);

	particle[0]->SetVelocity(particle[0]->GetVelocity() + impulsePerIMass * particle[0]->GetState().inverseMass);
}

void BpParticleContact::ResolveInterpenetration(f32 duration)
{
	throw std::logic_error("The method or operation is not implemented.");
}
