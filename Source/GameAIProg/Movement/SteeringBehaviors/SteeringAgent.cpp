// Fill out your copyright notice in the Description page of Project Settings.

#include "SteeringAgent.h"


// Sets default values
ASteeringAgent::ASteeringAgent()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ASteeringAgent::BeginPlay()
{
	Super::BeginPlay();
}

void ASteeringAgent::BeginDestroy()
{
	Super::BeginDestroy();
}
// Called every frame
void ASteeringAgent::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (SteeringBehavior)
	{
		const SteeringOutput Output = SteeringBehavior->CalculateSteering(DeltaTime, *this);
		
		/* 
		 * Based on Reynolds Steering Behavior (https://www.red3d.com/cwr/steer/gdc99/), 
		 * not used due to the non-use of the angularvelocity and the behavior of movementcomponent with missing variables like max_force
		 */
		//FVector2D DesiredVelocity = Output.LinearVelocity * GetMaxLinearSpeed();
		//FVector2D Steering = DesiredVelocity - FVector2D{GetVelocity()};
		//const float MaxAngle{GetMaxAngularSpeed()};
		//const float RotationStrength = FMath::Clamp(Output.AngularVelocity, -MaxAngle, MaxAngle) * DeltaTime;
		//AddMovementInput(FVector{Steering, 0.f}, RotationStrength);
		
		AddMovementInput(FVector{Output.LinearVelocity, 0.f});
		
		if (std::abs(Output.AngularVelocity) > KINDA_SMALL_NUMBER)
		{
			//const float TargetAngle = GetRotation() + Output.AngularVelocity;
			//float DeltaAngle = GetMaxAngularSpeed() * DeltaTime;
			const float MaxStep = GetMaxAngularSpeed() * DeltaTime;

			// Clamp step to remaining angular difference
			const float DeltaAngle = FMath::Clamp(
				Output.AngularVelocity,
				-MaxStep,
				MaxStep
			);
			
			SetActorRotation(FRotator{ 0.f
				, GetRotation() + DeltaAngle 
				, 0.f });
		}
	}
}

// Called to bind functionality to input
void ASteeringAgent::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void ASteeringAgent::SetSteeringBehavior(ISteeringBehavior* NewSteeringBehavior)
{
	SteeringBehavior = NewSteeringBehavior;
}

