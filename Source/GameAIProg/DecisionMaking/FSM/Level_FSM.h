// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "DecisionMaking/PatrolStates.h"
#include "Shared/Level_Base.h"
#include "Shared/Graph/GraphNodeFactory.h"
#include "Shared/Graph/GraphRenderer.h"
#include "Level_FSM.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_FSM : public ALevel_Base
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ALevel_FSM();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	UPROPERTY()
	ASteeringAgent* Agent{nullptr}; // ref
	
	void InitBlackboardData(UBlackboardComponent* blackboard);
	void SetupFSMStates(UFSMComponent* fsm);
	
	GameAI::GraphRenderer GraphRenderer{nullptr};
	GameAI::Graph Graph{false};
	GameAI::GraphNodeFactory<GameAI::Node> NodeFactory{};
};
