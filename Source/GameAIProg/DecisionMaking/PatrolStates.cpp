#include "PatrolStates.h"

#include <filesystem>

#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"

// Patrol

void GameAI::FSM::PatrolState::OnEnter(State* previousState, UBlackboardComponent* pBlackboard)
{
	PatrolBlackboard bb{pBlackboard};

	ASteeringAgent* steeringAgent = bb.GetSteeringAgent();
	if (!steeringAgent)
	{
		UE_LOG(LogTemp, Error, TEXT("[PatrolStates] No steering agent set to the blackboard"));
		return; 
	}

	PathFollow* pathFollow{};
	{
		std::unique_ptr<PathFollow> pathFollowUPtr = std::make_unique<PathFollow>();
		pathFollow = pathFollowUPtr.get();
		bb.SetCurrentSteeringBehavior(std::move(pathFollowUPtr));
		steeringAgent->SetSteeringBehavior(pathFollow);
	}
	//todo, remove and use the graph

	//get path from blackboard
	UPatrolPathData* patrolPath = bb.GetPatrolPath();
	using VectorSizeType = std::vector<FVector2D>::size_type;
	std::vector<FVector2D> path{static_cast<VectorSizeType>(patrolPath->Nodes.Num())};
	for (int i{}; i < patrolPath->Nodes.Num(); i++)
	{
		path.push_back(patrolPath->Nodes[i]);
	}

	pathFollow->SetPath(path);
}

void GameAI::FSM::PatrolState::Update(UBlackboardComponent* pBlackboard, float deltaTime)
{
	PatrolBlackboard bb{pBlackboard};

	ASteeringAgent* steeringAgent = bb.GetSteeringAgent();
	PathFollow* pathFollow = static_cast<PathFollow*>(bb.GetCurrentSteeringBehavior());


	//auto actor = bb.GetActor();

	//static int counter{};

	//counter++;

	//UE_LOG(LogTemp, Log, TEXT("%d"), counter);
	//todo, add patrol nodes movement here
}

void GameAI::FSM::PatrolState::OnExit(State* nextState, UBlackboardComponent* pBlackboard)
{
}

// Chase

void GameAI::FSM::ChaseState::OnEnter(State* previousState, UBlackboardComponent* pBlackboard)
{
}

void GameAI::FSM::ChaseState::Update(UBlackboardComponent* pBlackboard, float deltaTime)
{
	PatrolBlackboard bb{pBlackboard};

	//bb.g
}

void GameAI::FSM::ChaseState::OnExit(State* nextState, UBlackboardComponent* pBlackboard)
{
}

// Search

void GameAI::FSM::SearchState::OnEnter(State* previousState, UBlackboardComponent* pBlackboard)
{
}

void GameAI::FSM::SearchState::Update(UBlackboardComponent* pBlackboard, float deltaTime)
{
	//todo, wander for some time around the last player seen location, then return to the last patrol spot
}

void GameAI::FSM::SearchState::OnExit(State* nextState, UBlackboardComponent* pBlackboard)
{
}
