// Fill out your copyright notice in the Description page of Project Settings.


#include "PatrolServices.h"

#include "DecisionMaking/PatrolStates.h"
#include "Navigation/PathFollowingComponent.h"
#include "Perception/AIPerceptionComponent.h"
#include "Perception/AISense_Sight.h"

// ====================================================
// CHASE 
// ====================================================

// Chase target

void UBTService_ChaseTarget::TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC) return;

	GameAI::FSM::PatrolBlackboard bb{OwnerComp.GetBlackboardComponent()};

	AActor* target = bb.GetTargetActor();
	if (!target) return;
	FVector targetLocation = target->GetActorLocation();

	FAIMoveRequest MoveRequest;
	MoveRequest.SetGoalLocation(targetLocation);
	MoveRequest.SetAcceptanceRadius(50.f);

	AIC->MoveTo(MoveRequest);
	bb.SetLastKnownLocation(targetLocation);
	bb.SetHasLastKnownLocation(true);
}

// ====================================================
// SEARCH
// ====================================================

// Search timer

void UBTService_SearchTimer::TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC || !AIC->GetPawn()) return;

	GameAI::FSM::PatrolBlackboard bb{OwnerComp.GetBlackboardComponent()};
	
	bb.SetSearchTime(bb.GetSearchTime() + DeltaSeconds);
}

// ====================================================
// PATROL 
// ====================================================

void UBTService_UpdateLastPatrolPos::TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	AAIController* AIC = OwnerComp.GetAIOwner();
	if (!AIC || !AIC->GetPawn()) return;

	GameAI::FSM::PatrolBlackboard bb{OwnerComp.GetBlackboardComponent()};
	
	if (bb.GetIsReturningToPatrol()) 
		return; // don't overwrite while returning
	
	bb.SetLastPatrolPosition(AIC->GetPawn()->GetActorLocation());
	bb.SetHasLastPatrolPos(true);
}
