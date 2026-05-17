// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "UObject/Object.h"
#include <memory>

#include "BehaviorTree/BlackboardComponent.h"
#include "BlackBoardDataWrappers.generated.h"

UCLASS()
class GAMEAIPROG_API UPatrolPathData : public UObject
{
	GENERATED_BODY()
public:
	TArray<FVector2D> Nodes;
};

UCLASS()
class GAMEAIPROG_API UAgentSteeringBehavior : public UObject
{
	GENERATED_BODY()
	public:
	
	std::unique_ptr<ISteeringBehavior> SteeringBehavior;
};

void PrintBlackBoardData(UBlackboardComponent* blackboard);
