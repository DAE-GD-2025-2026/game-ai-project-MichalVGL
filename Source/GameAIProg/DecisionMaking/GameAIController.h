// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "AIController.h"
#include "FSM/FSMBaseClasses.h"
#include "Perception/AISenseConfig_Sight.h"
#include "GameAIController.generated.h"

UCLASS()
class GAMEAIPROG_API AGameAIController : public AAIController
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI|FSM_BT")
	bool UseBehaviourTree; 
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI|FSM_BT")
	TObjectPtr<UBlackboardData> FSMBlackboardAsset;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI|FSM_BT")
	TObjectPtr<UBehaviorTree> BehaviorTree;
	
	// Sets default values for this actor's properties
	AGameAIController();
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
	void InitFiniteStateMachine(std::unique_ptr<GameAI::FSM::State>&& startState, std::function<void(UBlackboardComponent*)> blackboardInitFunc);
	
	void RunFiniteStateMachine();
	
protected:
	
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	UFUNCTION()
	void OnPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus);
	
	virtual void OnPossess(APawn* InPawn) override;
	
	UPROPERTY()
	UAISenseConfig_Sight* SightConfig;
	
	bool FSMInitialized{false};
};
