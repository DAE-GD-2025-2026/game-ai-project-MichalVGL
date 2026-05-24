// Fill out your copyright notice in the Description page of Project Settings.


#include "GameAIController.h"

#include "PatrolStates.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "FSM/FSMComponent.h"
#include "Perception/AIPerceptionComponent.h"
#include "Perception/AISenseConfig_Sight.h"


// Sets default values
AGameAIController::AGameAIController()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	BrainComponent = CreateDefaultSubobject<UFSMComponent>(TEXT("FSMComponent"));

	PerceptionComponent = CreateDefaultSubobject<UAIPerceptionComponent>(TEXT("AIPerception"));

	SightConfig = CreateDefaultSubobject<UAISenseConfig_Sight>(TEXT("SightConfig"));
	SightConfig->SightRadius = 1500.f;
	SightConfig->LoseSightRadius = 1800.f;
	SightConfig->PeripheralVisionAngleDegrees = 45.f;
	SightConfig->DetectionByAffiliation.bDetectNeutrals = true;

	PerceptionComponent->ConfigureSense(*SightConfig);
	PerceptionComponent->SetDominantSense(SightConfig->GetSenseImplementation());

	PerceptionComponent->OnTargetPerceptionUpdated.AddDynamic(this, &AGameAIController::OnPerceptionUpdated);
}

// Called when the game starts or when spawned
void AGameAIController::BeginPlay()
{
	Super::BeginPlay();

	// Create Blackboard if need be
	//InitFiniteStateMachine();
}

void AGameAIController::OnPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus)
{
	auto blackBoard = GetBlackboardComponent();
	if (!blackBoard)
	{
		return;
	}

	GameAI::FSM::PatrolBlackboard bb{blackBoard};
	
	if (Actor == bb.GetTargetActor())
	{
		bb.SetTargetVisible(Stimulus.WasSuccessfullySensed());
	}
}

// Called every frame
void AGameAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AGameAIController::InitFiniteStateMachine(std::unique_ptr<GameAI::FSM::State>&& startState,
                                               std::function<void(UBlackboardComponent*)> blackboardInitFunc)
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
