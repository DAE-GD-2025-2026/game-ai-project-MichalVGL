// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "PatrolTasks.generated.h"

// ====================================================
// CHASE 
// ====================================================

UCLASS()
class UBTTask_ChaseWait : public UBTTaskNode
{
	GENERATED_BODY()
	
protected:
	
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
};

// ====================================================
// SEARCH 
// ====================================================

// Move To Last Known Loc

UCLASS()
class UBTTask_CheckSearchTime : public UBTTaskNode
{
	GENERATED_BODY()
	
public: 
	UBTTask_CheckSearchTime()
	{
		NodeName = "Check Search Time";
	}
	
protected:
	
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
};

// Move To Last Known Loc

UCLASS()
class UBTTask_MoveToLastKnownLocation : public UBTTaskNode
{
	GENERATED_BODY()
	
public: 
	UBTTask_MoveToLastKnownLocation()
	{
		NodeName = "Move To Last Known Location";
	}
	
protected:
	
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void OnMessage(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, FName Message, int32 RequestID,
		bool bSuccess) override;
};

// Wander

UCLASS()
class UBTTask_Wander : public UBTTaskNode
{
	GENERATED_BODY()
	
public: 
	UBTTask_Wander()
	{
		NodeName = "Wander";
	}
	
protected:
	
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void OnMessage(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, FName Message, int32 RequestID,
		bool bSuccess) override;
};

// ====================================================
// PATROL 
// ====================================================

// Move To Last Patrol Pos

UCLASS()
class UBTTask_MoveToLastPatrolPos : public UBTTaskNode
{
	GENERATED_BODY()
	
protected:
	
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void OnMessage(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, FName Message, int32 RequestID,
		bool bSuccess) override;
};

// Move To Waypoint

UCLASS()
class UBTTask_MoveToWaypoint : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_MoveToWaypoint() = default;
	virtual ~UBTTask_MoveToWaypoint() override = default;
	
protected:
	
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void OnMessage(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, FName Message, int32 RequestID,
		bool bSuccess) override;
};
