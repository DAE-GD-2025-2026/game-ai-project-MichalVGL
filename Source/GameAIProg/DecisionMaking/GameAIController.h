// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "AIController.h"
#include "FSM/FSMBaseClasses.h"
#include "GameAIController.generated.h"

UCLASS()
class GAMEAIPROG_API AGameAIController : public AAIController
{
	GENERATED_BODY()
	
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI|FSM")
	TObjectPtr<UBlackboardData> FSMBlackboardAsset;

	// Sets default values for this actor's properties
	AGameAIController();
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
	void InitFiniteStateMachine(std::unique_ptr<GameAI::FSM::State>&& startState, std::function<void(UBlackboardComponent*)> blackboardInitFunc);
	
	void RunFiniteStateMachine();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	bool FSMInitialized{false};
};
