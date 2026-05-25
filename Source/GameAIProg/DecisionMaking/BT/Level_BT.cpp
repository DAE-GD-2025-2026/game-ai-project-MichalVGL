// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_BT.h"

#include "InteractiveToolManager.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Object.h"
#include "DecisionMaking/GameAIController.h"
#include "DecisionMaking/PatrolStates.h"
#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"


// Sets default values
ALevel_BT::ALevel_BT()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_BT::BeginPlay()
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
	
	//disable behaviour tree of the mouseagent
	if (AController* MouseController = MouseAgent->GetController())
	{
		MouseController->UnPossess();
	}
	
	AAIController* SimpleController = GetWorld()->SpawnActor<AAIController>();
	SimpleController->Possess(MouseAgent);
	
	//init state machine and start state
	//AIController->InitFiniteStateMachine(std::make_unique<GameAI::FSM::PatrolState>()
	//	,  std::function<void(UBlackboardComponent*)>(std::bind(&ALevel_BT::InitBlackboardData, this, std::placeholders::_1)));
	
	//set the blackboard init data
	GameAI::FSM::PatrolBlackboard blackBoard{AIController->GetBlackboardComponent()};
	if (blackBoard.Blackboard == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("Blackboard does not exist"));
		return;
	}
	
	InitBlackboardData(blackBoard.Blackboard);
	
	//SetupFSMStates(FSM);
	
	//FSM->AddState(std::make_unique<GameAI::FSM::TestState>());
	//AIController->RunFiniteStateMachine();
}

void ALevel_BT::BindLevelInputActions()
{
	Super::BindLevelInputActions();
	
	PlayerEnhancedInputComponent->BindAction(SetTargetAction, ETriggerEvent::Started, this, &ALevel_BT::OnClick);
}

void ALevel_BT::InitBlackboardData(UBlackboardComponent* blackboard)
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

void ALevel_BT::OnClick()
{
	AAIController* Con = Cast<AAIController>(MouseAgent->GetController());
	if (!Con) return;

	Con->MoveToLocation(LatestMouseWorldPos);
}

// Called every frame
void ALevel_BT::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	GraphRenderer.RenderGraph(Graph);
}
