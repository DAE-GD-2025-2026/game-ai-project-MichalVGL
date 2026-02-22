#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

// HELPERS

namespace
{
	void DrawSteeringDebug(const ASteeringAgent& Agent, const SteeringOutput& Steering, float LineLength = 100.f)
	{
		DrawDebugLine(
			Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0.f),
			FVector(Agent.GetPosition(), 0.f) + Agent.GetActorForwardVector() * LineLength,
			FColor::Red,
			false,
			0.f,
			0,
			3.f
		);
	}
}

// SEEK

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity.Normalize();
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawSteeringDebug(Agent, Steering);
	}

	return Steering;
}

// FLEE

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = Agent.GetPosition() - Target.Position;

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawSteeringDebug(Agent, Steering);
	}

	return Steering;
}

// ARRIVE

Arrive::Arrive(const float TargetRad, const float SlowRad)
	: m_TargetRadiusSq(TargetRad* TargetRad),
	m_SlowRadiusSq(SlowRad* SlowRad)
{
}

Arrive::~Arrive()
{
	if (m_pCachedAgent != nullptr)
		m_pCachedAgent->SetMaxLinearSpeed(m_CachedMaxSpeed);
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	if (m_pCachedAgent != &Agent)
	{
		m_CachedMaxSpeed = Agent.GetMaxLinearSpeed();
		m_pCachedAgent = &Agent;
	}

	SteeringOutput Steering{};

	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity = ToTarget;

	float DistanceSq = ToTarget.SizeSquared();
	DistanceSq = DistanceSq - m_TargetRadiusSq;

	const float SlowPercent = std::clamp(DistanceSq / m_SlowRadiusSq, 0.f, 1.f);
	m_pCachedAgent->SetMaxLinearSpeed(m_CachedMaxSpeed * SlowPercent);

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawSteeringDebug(Agent, Steering, 100.f * SlowPercent);

		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Target.Position.X, Target.Position.Y, 0.f),
			FMath::Sqrt(m_SlowRadiusSq),
			32,
			FColor::Blue,
			false,
			0.f,
			0,
			3.f,
			FVector(1, 0, 0),
			FVector(0, 1, 0),
			false
		);

		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Target.Position.X, Target.Position.Y, 0.f),
			FMath::Sqrt(m_TargetRadiusSq),
			32,
			FColor::Black,
			false,
			0.f,
			0,
			3.f,
			FVector(1, 0, 0),
			FVector(0, 1, 0),
			false
		);
	}
	
	return Steering;
}

// FACE
 
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	 
	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	ToTarget.Normalize();
	
	const float TargetAngle = FMath::RadiansToDegrees(FMath::Atan2(ToTarget.Y, ToTarget.X));
	const float Angle = FMath::FindDeltaAngleDegrees(Agent.GetRotation(), TargetAngle);

	Steering.AngularVelocity = Angle;
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawSteeringDebug(Agent, Steering);
	}
	
	return Steering;
}

// PURSUIT

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	const FVector2D ToTarget = Target.Position - Agent.GetPosition();
	//FVector TargetForward =
	const FVector2D ForwardTarget = FVector2D(1.f, 0.f).GetRotated(Target.Orientation);
	const FVector2D FuturePosition = Target.Position + ForwardTarget * ToTarget.Size();

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawSteeringDebug(Agent, Steering);
		
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(FuturePosition.X, FuturePosition.Y, 0.f),
			30.f,
			32,
			FColor::Black,
			false,
			0.f,
			0,
			3.f,
			FVector(1, 0, 0),
			FVector(0, 1, 0),
			false
		);
	}
	
	Steering.LinearVelocity = FuturePosition - Agent.GetPosition();

	return Steering;
}

// EVADE

void Evade::SetEvadeRadius(float EvadeRadius)
{
	if (EvadeRadius > 0.f)
	{
		m_EvadeRadiusSq = EvadeRadius * EvadeRadius;
	}
	else
	{
		m_EvadeRadiusSq = -1.f;
	}
}

float Evade::GetEvadeRadius() const
{
	if (m_EvadeRadiusSq > 0.f)
	{
		return FMath::Sqrt(m_EvadeRadiusSq);
	}
	else
	{
		return m_EvadeRadiusSq;
	}
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	const FVector2D ToTarget = Target.Position - Agent.GetPosition();
	
	if (Agent.GetDebugRenderingEnabled() && m_EvadeRadiusSq > 0.f)	//draw debug evade radius
	{
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0.f),
			FMath::Sqrt(m_EvadeRadiusSq),
			32,
			FColor::Black,
			false,
			0.f,
			0,
			3.f,
			FVector(1, 0, 0),
			FVector(0, 1, 0),
			false
		);
	}
	
	if (m_EvadeRadiusSq > 0.f //treat negative values as no limited range
		&& ToTarget.SquaredLength() > m_EvadeRadiusSq)
	{
		Steering.IsValid = false;
		return Steering;
	}
	
	const FVector2D ForwardTarget = FVector2D(1.f, 0.f).GetRotated(Target.Orientation);
	const FVector2D FuturePosition = Target.Position + ForwardTarget * ToTarget.Size();

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawSteeringDebug(Agent, Steering);
		
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(FuturePosition.X, FuturePosition.Y, 0.f),
			30.f,
			32,
			FColor::Black,
			false,
			0.f,
			0,
			3.f,
			FVector(1, 0, 0),
			FVector(0, 1, 0),
			false
		);
	}
	
	Steering.LinearVelocity = -(FuturePosition - Agent.GetPosition()); // -() to flee from future position instead of seek towards it

	return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	//update angle
	float DeltaAngle = FMath::RandRange(-m_MaxAngleChange, m_MaxAngleChange);
	m_WanderAngle += DeltaAngle;
	
	//get unit Vector2d offset
	FVector2D WanderDir = FVector2D(1.f, 0.f).GetRotated(m_WanderAngle);
	
	//Update target pos
	Target.Position = Agent.GetPosition();
	Target.Position += FVector2D{Agent.GetActorForwardVector()} * m_Offset;
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Target.Position, 0.f),
			m_Radius,
			32,
			FColor::Blue,
			false,
			0.f,
			0,
			3.f,
			FVector(1, 0, 0),
			FVector(0, 1, 0),
			false
		);
	}
	Target.Position += WanderDir * m_Radius;
	
	SteeringOutput Steering = Seek::CalculateSteering(DeltaT, Agent);
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawSteeringDebug(Agent, Steering);
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Target.Position, 0.f),
			10.f,
			32,
			FColor::Black,
			false,
			0.f,
			0,
			3.f,
			FVector(1, 0, 0),
			FVector(0, 1, 0),
			false
		);
	}
	
	return Steering;
}

