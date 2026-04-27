// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"
#include "FSMBaseClasses.h"

// Sets default values for this component's properties
UFSMComponent::UFSMComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// TODO Setup FSM
}


void UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	// TODO

	FSMInstance->AddState(std::move(NewState));
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To,
                                  std::function<bool()> EvalFunc) const
{
	// TODO
	std::unique_ptr<GameAI::FSM::Transition> transition = std::make_unique<GameAI::FSM::Transition>(From, To, std::move(EvalFunc));
	FSMInstance->AddTransition(std::move(transition));
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
