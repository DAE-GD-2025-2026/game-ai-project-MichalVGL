#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	Target.Position = pFlock->GetAverageNeighborPos();
	
	return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)

SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	
	const auto& Neighbours = pFlock->GetNeighbors();
	
	for (int i = 0; i < pFlock->GetNrOfNeighbors(); ++i)
	{
		if (Neighbours[i])
		{
			FVector2D ToAgent = pAgent.GetPosition() - Neighbours[i]->GetPosition();

			if (const float Distance = ToAgent.Size()
				; Distance > 0.f)
			{
				Steering.LinearVelocity += (ToAgent / Distance) * (1.f / Distance);	//todo maybe delete /distance
			}
		}
	}
	
	return Steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = pFlock->GetAverageNeighborVelocity();
	
	return Steering;
}