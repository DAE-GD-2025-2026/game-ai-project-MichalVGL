// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTService.h"
#include "PatrolServices.generated.h"

// ====================================================
// CHASE 
// ====================================================

// Chase Target
UCLASS()
class UBTService_ChaseTarget : public UBTService
{
	GENERATED_BODY()

public:
	UBTService_ChaseTarget()
	{
		NodeName = "Chase";
	}

protected:
	virtual void TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
};

// ====================================================
// SEARCH
// ====================================================

// Search timer
UCLASS()
class UBTService_SearchTimer : public UBTService
{
	GENERATED_BODY()

public:
	UBTService_SearchTimer()
	{
		NodeName = "Search Timer";
	}

protected:
	virtual void TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
};

// ====================================================
// PATROL 
// ====================================================

UCLASS()
class UBTService_UpdateLastPatrolPos : public UBTService
{
	GENERATED_BODY()

public:
	UBTService_UpdateLastPatrolPos()
	{
		NodeName = "Update Last Patrol Pos";
	}

protected:
	virtual void TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
};
