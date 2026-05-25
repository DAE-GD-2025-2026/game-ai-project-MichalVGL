// Fill out your copyright notice in the Description page of Project Settings.


#include "PatrolTasks.h"

#include "NavigationSystem.h"
#include "DecisionMaking/PatrolStates.h" //using the helper class and names, should be in another file ideally
#include "Navigation/PathFollowingComponent.h"
#include "Shared/BlackBoardDataWrappers.h"

// ====================================================
// CHASE 
// ====================================================

// Move To Target

class UNavigationSystemV1;

EBTNodeResult::Type UBTTask_ChaseWait::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	Super::ExecuteTask(OwnerComp, NodeMemory);

	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC) return EBTNodeResult::Failed;

	GameAI::FSM::PatrolBlackboard bb{OwnerComp.GetBlackboardComponent()};

	bb.SetSearchTime(0.f);

	// the chase service handles the actual chasing
	return EBTNodeResult::InProgress;
}

// ====================================================
// SEARCH
// ====================================================

// Check Search Time

EBTNodeResult::Type UBTTask_CheckSearchTime::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	Super::ExecuteTask(OwnerComp, NodeMemory);

	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC) return EBTNodeResult::Failed;

	GameAI::FSM::PatrolBlackboard bb{OwnerComp.GetBlackboardComponent()};

	if (bb.GetSearchTime() > bb.GetMaximumSearchTime())
	{
		return EBTNodeResult::Failed;
	}

	return EBTNodeResult::Succeeded;
}

// MoveToLastKnownLocation

EBTNodeResult::Type UBTTask_MoveToLastKnownLocation::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	Super::ExecuteTask(OwnerComp, NodeMemory);

	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC) return EBTNodeResult::Failed;

	GameAI::FSM::PatrolBlackboard bb{OwnerComp.GetBlackboardComponent()};

	FAIMoveRequest moveRequest;
	moveRequest.SetGoalLocation(bb.GetLastKnownLocation());
	moveRequest.SetAcceptanceRadius(50.f);

	if (!bb.GetHasLastKnownLocation())
	{
		return EBTNodeResult::Failed;
	}

	FPathFollowingRequestResult Result = AIC->MoveTo(moveRequest);
	switch (Result.Code)
	{
	case EPathFollowingRequestResult::Type::Failed:
		return EBTNodeResult::Failed;

	case EPathFollowingRequestResult::Type::AlreadyAtGoal:
		bb.SetHasLastKnownLocation(false);
		return EBTNodeResult::Succeeded;

	case EPathFollowingRequestResult::Type::RequestSuccessful:
		WaitForMessage(OwnerComp, UBrainComponent::AIMessage_MoveFinished);
		return EBTNodeResult::InProgress;
	}

	return EBTNodeResult::Failed;
}

void UBTTask_MoveToLastKnownLocation::OnMessage(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, FName Message,
                                                int32 RequestID, bool bSuccess)
{
	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
		return;
	}

	GameAI::FSM::PatrolBlackboard bb{OwnerComp.GetBlackboardComponent()};

	bb.SetHasLastKnownLocation(false);

	FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
}

EBTNodeResult::Type UBTTask_Wander::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	Super::ExecuteTask(OwnerComp, NodeMemory);

	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC || !AIC->GetPawn()) return EBTNodeResult::Failed;

	UNavigationSystemV1* navSys = UNavigationSystemV1::GetCurrent(AIC->GetWorld());
	if (!navSys) return EBTNodeResult::Failed;

	FNavLocation randomLocation;
	bool bFound = false;

	for (int32 i = 0; i < 10; i++)
	{
		bFound = navSys->GetRandomReachablePointInRadius(
			AIC->GetPawn()->GetActorLocation(), 1000.f, randomLocation);

		if (bFound && FVector::Dist(randomLocation.Location,
		                            AIC->GetPawn()->GetActorLocation()) > 200.f)
			break;

		bFound = false;
	}

	if (!bFound) return EBTNodeResult::Failed;

	FAIMoveRequest moveRequest;
	moveRequest.SetGoalLocation(randomLocation.Location);
	moveRequest.SetAcceptanceRadius(50.f);

	FPathFollowingRequestResult r = AIC->MoveTo(moveRequest);
	switch (r.Code)
	{
	case EPathFollowingRequestResult::Type::Failed:
		return EBTNodeResult::Failed;

	case EPathFollowingRequestResult::Type::AlreadyAtGoal:
		return EBTNodeResult::Succeeded;

	case EPathFollowingRequestResult::Type::RequestSuccessful:
		WaitForMessage(OwnerComp, UBrainComponent::AIMessage_MoveFinished);
		return EBTNodeResult::InProgress;
	}

	return EBTNodeResult::Failed;
}

void UBTTask_Wander::OnMessage(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, FName Message, int32 RequestID,
                               bool bSuccess)
{
	FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
}

// ====================================================
// PATROL 
// ====================================================

// Move To Last Patrol Pos

EBTNodeResult::Type UBTTask_MoveToLastPatrolPos::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	Super::ExecuteTask(OwnerComp, NodeMemory);

	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC)
		return EBTNodeResult::Failed;

	GameAI::FSM::PatrolBlackboard bb{OwnerComp.GetBlackboardComponent()};

	if (bb.GetHasLastPatrolPos())
	{
		FAIMoveRequest moveRequest;
		moveRequest.SetGoalLocation(bb.GetLastPatrolPosition());
		moveRequest.SetAcceptanceRadius(50.f);

		FPathFollowingRequestResult result = AIC->MoveTo(moveRequest);
		switch (result.Code)
		{
		case EPathFollowingRequestResult::Type::Failed:
			return EBTNodeResult::Failed;

		case EPathFollowingRequestResult::Type::AlreadyAtGoal:
			return EBTNodeResult::Succeeded;

		case EPathFollowingRequestResult::Type::RequestSuccessful:
			int32 nodeCount = bb.GetPatrolPath()->Nodes.Num();
			bb.SetCurrentPatrolNodeIndex((bb.GetCurrentPatrolNodeIndex() - 1 + nodeCount) % nodeCount);
			WaitForMessage(OwnerComp, UBrainComponent::AIMessage_MoveFinished);
			return EBTNodeResult::InProgress;
		}

		return EBTNodeResult::Failed;
	}

	return EBTNodeResult::Succeeded;
}

void UBTTask_MoveToLastPatrolPos::OnMessage(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, FName Message,
                                            int32 RequestID, bool bSuccess)
{
	FinishLatentTask(OwnerComp, bSuccess ? EBTNodeResult::Succeeded : EBTNodeResult::Failed);
}


// Move To Waypoint

EBTNodeResult::Type UBTTask_MoveToWaypoint::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	Super::ExecuteTask(OwnerComp, NodeMemory);

	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC)
		return EBTNodeResult::Failed;

	GameAI::FSM::PatrolBlackboard bb{OwnerComp.GetBlackboardComponent()};

	UPatrolPathData* patrolPath = bb.GetPatrolPath();
	if (!patrolPath || patrolPath->Nodes.Num() == 0)
		return EBTNodeResult::Failed;

	int32 currentIndex = bb.GetCurrentPatrolNodeIndex();
	currentIndex = (currentIndex + 1) % patrolPath->Nodes.Num();

	bb.SetCurrentPatrolNodeIndex(currentIndex);

	FAIMoveRequest moveRequest;
	moveRequest.SetGoalLocation(FVector(patrolPath->Nodes[currentIndex], 0.f));
	moveRequest.SetAcceptanceRadius(50.f);

	FPathFollowingRequestResult result = AIC->MoveTo(moveRequest);
	switch (result.Code)
	{
	case EPathFollowingRequestResult::Type::Failed:
		return EBTNodeResult::Failed;

	case EPathFollowingRequestResult::Type::AlreadyAtGoal:
		return EBTNodeResult::Succeeded;

	case EPathFollowingRequestResult::Type::RequestSuccessful:
		WaitForMessage(OwnerComp, UBrainComponent::AIMessage_MoveFinished);
		return EBTNodeResult::InProgress;
	}

	return EBTNodeResult::Failed;
}

void UBTTask_MoveToWaypoint::OnMessage(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, FName Message,
                                       int32 RequestID, bool bSuccess)
{
	FinishLatentTask(OwnerComp, bSuccess ? EBTNodeResult::Succeeded : EBTNodeResult::Failed);
}
