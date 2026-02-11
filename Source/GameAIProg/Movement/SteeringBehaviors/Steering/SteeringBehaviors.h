#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }

	template <class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{
		return static_cast<T*>(this);
	}

protected:
	FTargetData Target;
};

// Your own SteeringBehaviors should follow here...

class Seek : public ISteeringBehavior
{
public:
	virtual ~Seek() override = default;

protected:
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class Flee : public ISteeringBehavior
{
public:
	virtual ~Flee() override = default;

private:
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class Arrive : public ISteeringBehavior
{
public:
	Arrive(float TargetRad, float SlowRad);
	virtual ~Arrive() override;

private:
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	float m_TargetRadiusSq;
	float m_SlowRadiusSq; //radius on top of target radius

	float m_CachedMaxSpeed{-1.f};
	ASteeringAgent* m_pCachedAgent{nullptr};
};

//class Face : public ISteeringBehavior
//{
//public:
//
//	virtual ~Face() = default;
//
//private:
//
//	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
//};

class Face : public ISteeringBehavior
{
public:
	virtual ~Face() override = default;

private:
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class Pursuit : public ISteeringBehavior
{
public:
	virtual ~Pursuit() override = default;

private:
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class Evade : public ISteeringBehavior
{
public:
	Evade(float CircleOffset = 70.f, float CircleRadius = 50.f)
		: m_CircleOffset(CircleOffset), m_CircleRadius(CircleRadius)
	{
	}

	virtual ~Evade() override = default;

private:
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	float m_CircleOffset;
	float m_CircleRadius;

	float m_CachedAngle{0.f};
};

class Wander : public Seek
{
public:
	Wander(float Offset, float Radius, float MaxAngleChange)
		: m_Offset(Offset), m_Radius(Radius), m_MaxAngleChange(MaxAngleChange) {}
	virtual ~Wander() override = default;

private:
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	float m_Offset;
	float m_Radius;
	float m_MaxAngleChange;
	float m_WanderAngle{ 0.f };
};