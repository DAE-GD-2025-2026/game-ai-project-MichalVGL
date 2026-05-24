#include "PatrolStates.h"

#include <filesystem>

#include "NavigationSystem.h"
#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "Navigation/PathFollowingComponent.h"

// Patrol

void GameAI::FSM::PatrolState::OnEnter(State* previousState, UBlackboardComponent* pBlackboard)
{
	PatrolBlackboard bb{pBlackboard};

	APawn* pawn = Cast<APawn>(bb.GetActor());
	AAIController* aiController = Cast<AAIController>(pawn->GetController());

	if (bb.GetHasLastPatrolPos()) //go back to the last position, if not Update() will set the next node
	{
		bb.SetIsReturningToPatrol(true);
		FAIMoveRequest MoveRequest;
		MoveRequest.SetGoalLocation(bb.GetLastPatrolPosition());
		MoveRequest.SetAcceptanceRadius(50.f);
		aiController->MoveTo(MoveRequest);
	}
	else
	{
		if (aiController)
			aiController->StopMovement();
	}
}

void GameAI::FSM::PatrolState::Update(UBlackboardComponent* pBlackboard, float deltaTime)
{
	PatrolBlackboard bb{pBlackboard};

	APawn* pawn = Cast<APawn>(bb.GetActor());
	AAIController* aiController = Cast<AAIController>(pawn->GetController());
	UPathFollowingComponent* pathFollowComp = aiController->GetPathFollowingComponent();

	if (pathFollowComp->GetStatus() == EPathFollowingStatus::Idle)
	{
		UPatrolPathData* patrolPath = bb.GetPatrolPath();
		int currentIndex = bb.GetCurrentPatrolNodeIndex();

		if (bb.GetIsReturningToPatrol()) //finish the current node
		{
			bb.SetIsReturningToPatrol(false);
		}
		else //go to next node
		{
			++currentIndex;
			currentIndex %= patrolPath->Nodes.Num();
			bb.SetCurrentPatrolNodeIndex(currentIndex);
		}

		FAIMoveRequest MoveRequest;
		MoveRequest.SetGoalLocation(FVector(patrolPath->Nodes[currentIndex], 0.f));
		MoveRequest.SetAcceptanceRadius(50.f);

		aiController->MoveTo(MoveRequest);
	}
	
	bb.SetLastPatrolPosition(pawn->GetActorLocation());
	bb.SetHasLastPatrolPos(true);
}

void GameAI::FSM::PatrolState::OnExit(State* nextState, UBlackboardComponent* pBlackboard)
{
	PatrolBlackboard bb{pBlackboard};

	APawn* pawn = Cast<APawn>(bb.GetActor());

	if (AAIController* aiController = Cast<AAIController>(pawn->GetController()))
		aiController->StopMovement();
}

// Chase

void GameAI::FSM::ChaseState::OnEnter(State* previousState, UBlackboardComponent* pBlackboard)
{
	PatrolBlackboard bb{pBlackboard};

	bb.SetSearchTime(0.f);
}

void GameAI::FSM::ChaseState::Update(UBlackboardComponent* pBlackboard, float deltaTime)
{
	PatrolBlackboard bb{pBlackboard};

	APawn* pawn = Cast<APawn>(bb.GetActor());
	AAIController* aiController = Cast<AAIController>(pawn->GetController());

	AActor* target = bb.GetTargetActor();
	FVector targetLocation = target->GetActorLocation();

	FAIMoveRequest MoveRequest;
	MoveRequest.SetGoalLocation(targetLocation);
	MoveRequest.SetAcceptanceRadius(50.f);

	aiController->MoveTo(MoveRequest);

	bb.SetLastKnownLocation(targetLocation);
}

void GameAI::FSM::ChaseState::OnExit(State* nextState, UBlackboardComponent* pBlackboard)
{
	PatrolBlackboard bb{pBlackboard};

	bb.SetSearchTime(0.f);
}

// Search

void GameAI::FSM::SearchState::OnEnter(State* previousState, UBlackboardComponent* pBlackboard)
{
	PatrolBlackboard bb{pBlackboard};
	
	APawn* pawn = Cast<APawn>(bb.GetActor());
	AAIController* aiController = Cast<AAIController>(pawn->GetController());
	
	FAIMoveRequest MoveRequest;
	MoveRequest.SetGoalLocation(bb.GetLastKnownLocation());
	MoveRequest.SetAcceptanceRadius(50.f);

	aiController->MoveTo(MoveRequest);
}

void GameAI::FSM::SearchState::Update(UBlackboardComponent* pBlackboard, float deltaTime)
{
	PatrolBlackboard bb{pBlackboard};
	
	bb.SetSearchTime(bb.GetSearchTime() + deltaTime);
	
	APawn* pawn = Cast<APawn>(bb.GetActor());
	AAIController* aiController = Cast<AAIController>(pawn->GetController());
	UPathFollowingComponent* pathFollowComp = aiController->GetPathFollowingComponent();

	if (pathFollowComp->GetStatus() == EPathFollowingStatus::Idle)
	{
		UNavigationSystemV1* navSys = UNavigationSystemV1::GetCurrent(pawn->GetWorld());
		if (!navSys)
			return;
		
		FNavLocation randLocation;
		bool found = navSys->GetRandomReachablePointInRadius(pawn->GetActorLocation(), 1000.f, randLocation);
		
		if (found)
		{
			FAIMoveRequest MoveRequest;
			MoveRequest.SetGoalLocation(randLocation.Location);
			MoveRequest.SetAcceptanceRadius(50.f);

			aiController->MoveTo(MoveRequest);
		}
	}
}

void GameAI::FSM::SearchState::OnExit(State* nextState, UBlackboardComponent* pBlackboard)
{
	PatrolBlackboard bb{pBlackboard};
	
	bb.SetSearchTime(0.f);
}
