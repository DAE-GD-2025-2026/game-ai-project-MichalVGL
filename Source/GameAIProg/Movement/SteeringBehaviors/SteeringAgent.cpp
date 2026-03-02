// Fill out your copyright notice in the Description page of Project Settings.

#include "SteeringAgent.h"

#include "AIController.h"


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

		SetIsAutoOrienting(FMath::IsNearlyEqual(Output.AngularVelocity, 0.f));

		if (!IsAutoOrienting())
		{
			if (AAIController* AIController = Cast<AAIController>(GetController()))
			{
				const float DeltaYaw{
					FMath::Clamp(Output.AngularVelocity * DeltaTime, -1.f, 1.f) * GetMaxAngularSpeed() * DeltaTime
				};

				const FRotator CurrentRotation{GetActorForwardVector().ToOrientationRotator()};
				const FRotator DeltaRotation{0.f, DeltaYaw, 0.f};
				const FRotator DesiredRotation{CurrentRotation + DeltaRotation};

				//only yaw
				if (!FMath::IsNearlyEqual(CurrentRotation.Yaw, DesiredRotation.Yaw))
				{
					AIController->SetControlRotation((DesiredRotation));
					FaceRotation(DesiredRotation);
				}
			}
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
