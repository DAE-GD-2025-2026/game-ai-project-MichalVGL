// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "FSMComponent.h"
#include "InteractiveToolManager.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Object.h"
#include "DecisionMaking/GameAIController.h"
#include "DecisionMaking/PatrolStates.h"
#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"


// Sets default values
ALevel_FSM::ALevel_FSM()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();

	//Set world parameters
	TrimWorld->SetTrimWorldSize(1700.f);
	GraphRenderer = GameAI::GraphRenderer{GetWorld()};

	//Create patrol nodes
	const float size = 1100.f;
	int id0 = Graph.AddNode(NodeFactory.CreateNode({-size, -size}));
	int id1 = Graph.AddNode(NodeFactory.CreateNode({-size, size}));
	int id2 = Graph.AddNode(NodeFactory.CreateNode({size, size}));
	int id3 = Graph.AddNode(NodeFactory.CreateNode({size, -size}));
	Graph.AddConnection(id0, id1);
	Graph.AddConnection(id1, id2);
	Graph.AddConnection(id2, id3);
	Graph.AddConnection(id3, id0);

	// Spawn AI Steering agent (custom steering behaviours) no longer used but still works for the setup (also has visuals)
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
	                                               FVector{-size, -size, 90}, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);
	
	AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController());
	if (!AIController)
	{
		UE_LOG(LogTemp, Error, TEXT("Agent does not have a AGameAIController"));
		return;
	}
	
	// Spawn mouse controlled agent
	MouseAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
												   FVector{0, 0, 90}, FRotator::ZeroRotator);
	
	MouseAgent->GetCharacterMovement()->MaxWalkSpeed = 700.f; //make it be able to outrun the patrolpawn
	
	
	//init state machine and start state
	AIController->InitFiniteStateMachine(std::make_unique<GameAI::FSM::PatrolState>()
		,  std::function<void(UBlackboardComponent*)>(std::bind(&ALevel_FSM::InitBlackboardData, this, std::placeholders::_1)));
	
	UFSMComponent* FSM = Cast<UFSMComponent>(AIController->GetBrainComponent());
	if (!FSM)
	{
		UE_LOG(LogTemp, Error, TEXT("AIController does not have a UFSMComponent"));
		return;
	}
	
	//set the blackboard init data
	GameAI::FSM::PatrolBlackboard blackBoard{FSM->GetBlackboard()};
	if (blackBoard.Blackboard == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("Blackboard does not exist"));
		return;
	}
	
	SetupFSMStates(FSM);
	
	//FSM->AddState(std::make_unique<GameAI::FSM::TestState>());
	AIController->RunFiniteStateMachine();
}

void ALevel_FSM::BindLevelInputActions()
{
	Super::BindLevelInputActions();
	
	PlayerEnhancedInputComponent->BindAction(SetTargetAction, ETriggerEvent::Started, this, &ALevel_FSM::OnClick);
}

void ALevel_FSM::InitBlackboardData(UBlackboardComponent* blackboard)
{
	if (blackboard == nullptr)
	{
		return;
	}
	
	GameAI::FSM::PatrolBlackboard bb{blackboard};
	
	//set the agent itself
	bb.SetActor(Agent);
	
	//set the patrol path
	UPatrolPathData* patrolPath = NewObject<UPatrolPathData>(this);
	
	const float size = 1100.f;
	patrolPath->Nodes.Add(FVector2D{-size, -size});
	patrolPath->Nodes.Add(FVector2D{-size,  size});
	patrolPath->Nodes.Add(FVector2D{ size,  size});
	patrolPath->Nodes.Add(FVector2D{ size, -size});

	bb.SetPatrolPath(patrolPath);
	
	//set target
	bb.SetTargetActor(MouseAgent);
	
	//set search time
	bb.SetMaximumSearchTime(5.f);
	
	//test code
	//PrintBlackBoardData(blackboard);
}

void ALevel_FSM::SetupFSMStates(UFSMComponent* fsm)
{
	auto* patrolState = fsm->GetStartState();
	auto* chaseState = fsm->AddState(std::make_unique<GameAI::FSM::ChaseState>());
	auto* searchState = fsm->AddState(std::make_unique<GameAI::FSM::SearchState>());
	
	fsm->AddTransition(patrolState, chaseState, [](UBlackboardComponent* bb) { return GameAI::FSM::IsTargetVisible(bb); });
	fsm->AddTransition(chaseState, searchState, [](UBlackboardComponent* bb) { return !GameAI::FSM::IsTargetVisible(bb); });
	fsm->AddTransition(searchState, chaseState, [](UBlackboardComponent* bb) { return GameAI::FSM::IsTargetVisible(bb); });
	fsm->AddTransition(searchState, patrolState, [](UBlackboardComponent* bb) { return GameAI::FSM::IsSearchingTooLong(bb); });
}

void ALevel_FSM::OnClick()
{
	AAIController* Con = Cast<AAIController>(MouseAgent->GetController());
	if (!Con) return;

	Con->MoveToLocation(LatestMouseWorldPos);
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	GraphRenderer.RenderGraph(Graph);
}
