// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"

#include <stdexcept>

#include "FSMBaseClasses.h"

// Sets default values for this component's properties
UFSMComponent::UFSMComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSMComponent::Initialize(UBlackboardComponent* blackboardComp, std::unique_ptr<GameAI::FSM::State>&& startState)
{
	if (StartState != nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("[FSMComponent] Initialize called multiple times, make sure to only call it once"));
		return;
	}
	
	StartState = startState.get();
	BlackboardComp = blackboardComp;
	
	FSMInstance = std::make_unique<GameAI::FSM::FSM>(std::move(startState), blackboardComp);
}

GameAI::FSM::State* UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	GameAI::FSM::State* state = NewState.get();
	if (state == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("[FSMComponent] AddState called with nullptr as parameter"));
	}
	FSMInstance->AddState(std::move(NewState));
	return state;
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To,
                                  std::function<bool()> EvalFunc) const
{
	// TODO
	if (!FSMInstance->HasState(From)
		|| !FSMInstance->HasState(To))
	{
		UE_LOG(LogTemp, Error, TEXT("Invalid AddTransition call from UFSMComponent, FSM doesnt contain the state"));
		throw std::runtime_error("Invalid AddTransition call from UFSMComponent");
	}
	
	std::unique_ptr<GameAI::FSM::Transition> transition = std::make_unique<GameAI::FSM::Transition>(From, To, std::move(EvalFunc));
	FSMInstance->AddTransition(std::move(transition));
}

GameAI::FSM::State* UFSMComponent::GetStartState() const
{
	return StartState;
}

// Called when the game starts
void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	// TODO
	
	FSMInstance->Update(BlackboardComp, DeltaTime);
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();

	// TODO
}

void UFSMComponent::StopLogic(const FString& Reason)
{
	// TODO
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}
