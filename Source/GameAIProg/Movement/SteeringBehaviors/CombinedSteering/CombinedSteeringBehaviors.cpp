
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

// HELPERS

namespace CSHelper
{
	void DrawSteeringDebug(const ASteeringAgent& Agent, const SteeringOutput& Steering, float LineLength = 100.f)
	{
		DrawDebugLine(
			Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0.f),
			FVector(Agent.GetPosition(), 0.f) + Agent.GetActorForwardVector() * LineLength,
			FColor::Purple,
			false,
			0.f,
			0,
			3.f
		);
	}
}

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};
	SteeringOutput SingleSteering{};
	for (WeightedBehavior& Behaviour : WeightedBehaviors)
	{
		SingleSteering = Behaviour.pBehavior->CalculateSteering(DeltaT, Agent);
		
		SingleSteering.LinearVelocity.Normalize();
		SingleSteering *= Behaviour.Weight;
		Output = Output + SingleSteering;
	}
	
	if (Agent.GetDebugRenderingEnabled())
		CSHelper::DrawSteeringDebug(Agent, Output, 200.f);

	Output.LinearVelocity.Normalize();
	return Output;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}