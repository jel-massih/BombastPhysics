#include "BpParticleContacts.h"
#include "../Foundation/BpTypes.h"

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

void BpParticleContact::ResolveVelocity(f32 dt)
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
	f32 accelerationCausedSepVelocity = accelerationCausedVelocity.Dot(contactNormal) * dt;

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

	BpVec3 impulsePerIMass = contactNormal * impules;

	particle[0]->SetVelocity(particle[0]->GetVelocity() + impulsePerIMass * particle[0]->GetState().inverseMass);
	if (particle[1] != nullptr)
	{
		particle[1]->SetVelocity(particle[1]->GetVelocity() + impulsePerIMass * -particle[1]->GetState().inverseMass);
	}
}

void BpParticleContact::ResolveInterpenetration(f32 dt)
{
	//Skip if no penetration
	if (penetrationDepth <= 0)
		return;

	f32 totalInverseMass = particle[0]->GetState().inverseMass;
	if (particle[1] != nullptr) totalInverseMass += particle[1]->GetState().inverseMass;
	if (totalInverseMass <= 0)
		return;

	BpVec3 movePerIMass = contactNormal * (penetrationDepth / totalInverseMass);

	particleMovement[0] = movePerIMass * particle[0]->GetState().inverseMass;
	if (particle[1] != nullptr)
	{
		particleMovement[1] = movePerIMass * -particle[1]->GetState().inverseMass;
	}
	else
	{
		particleMovement[1].Zero();
	}

	//Apply Resolution
	particle[0]->SetPosition(particle[0]->GetPosition() + particleMovement[0]);
	if (particle[1] != nullptr)
	{
		particle[1]->SetPosition(particle[1]->GetPosition() + particleMovement[1]);
	}
}

BpParticleContactResolver::BpParticleContactResolver(unsigned iterations) 
	: iterations(iterations)
{}

void BpParticleContactResolver::SetIterations(unsigned newIterations)
{
	iterations = newIterations;
}

void BpParticleContactResolver::ResolveContacts(BpParticleContact* contactArray, unsigned numContacts, f32 dt)
{
	unsigned i;

	iterationsUsed = 0;
	while (iterationsUsed < iterations)
	{
		//Find Contact with largest closing velocity
		f32 max = BP_MAX_REAL;
		unsigned maxIndex = numContacts;
		for (i = 0; i < numContacts; i++)
		{
			f32 sepVel = contactArray[i].CalculateSeparatingVelocity();
			if (sepVel < max && (sepVel < 0 || contactArray[i].penetrationDepth > 0))
			{
				max = sepVel;
				maxIndex = i;
			}
		}

		if(maxIndex == numContacts) break;

		contactArray[maxIndex].Resolve(dt);

		BpVec3* move = contactArray[maxIndex].particleMovement;
		for (i = 0; i < numContacts; i++)
		{
			if (contactArray[i].particle[0] == contactArray[maxIndex].particle[0])
			{
				contactArray[i].penetrationDepth -= move[0].Dot(contactArray[i].contactNormal);
			}
			else if (contactArray[i].particle[0] == contactArray[maxIndex].particle[1])
			{
				contactArray[i].penetrationDepth -= move[1].Dot(contactArray[i].contactNormal);
			}

			if (contactArray[i].particle[1] != nullptr)
			{
				if (contactArray[i].particle[1] == contactArray[maxIndex].particle[0])
				{
					contactArray[i].penetrationDepth += move[0].Dot(contactArray[i].contactNormal);
				}
				else if (contactArray[i].particle[1] == contactArray[maxIndex].particle[1])
				{
					contactArray[i].penetrationDepth += move[1].Dot(contactArray[i].contactNormal);
				}
			}
		}

		iterationsUsed++;
	}
}