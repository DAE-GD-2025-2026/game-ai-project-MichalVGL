// Fill out your copyright notice in the Description page of Project Settings.


#include "GameAIController.h"

#include "PatrolStates.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "FSM/FSMComponent.h"


// Sets default values
AGameAIController::AGameAIController()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	BrainComponent = CreateDefaultSubobject<UFSMComponent>(TEXT("FSMComponent"));
}

// Called when the game starts or when spawned
void AGameAIController::BeginPlay()
{
	Super::BeginPlay();
	
	// Create Blackboard if need be
	//InitFiniteStateMachine();
}

// Called every frame
void AGameAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AGameAIController::InitFiniteStateMachine(std::unique_ptr<GameAI::FSM::State>&& startState, std::function<void(UBlackboardComponent*)> blackboardInitFunc)
{
	UFSMComponent* FSMComp = FindComponentByClass<UFSMComponent>();
	if (ensure(FSMComp) && FSMBlackboardAsset)
	{
		UBlackboardComponent* BlackboardComp = Blackboard;
		UseBlackboard(FSMBlackboardAsset, BlackboardComp);
		Blackboard = BlackboardComp;
		
		blackboardInitFunc(Blackboard);
		
		FSMComp->Initialize(BlackboardComp, std::move(startState));
	}
}

void AGameAIController::RunFiniteStateMachine()
{
	if (FSMInitialized) return;
	
	FSMInitialized = true;
	
	UFSMComponent* FSMComp = FindComponentByClass<UFSMComponent>();
	if (ensure(FSMComp))
	{
		FSMComp->StartLogic();
	}
}



