// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "DecisionMaking/PatrolStates.h"
#include "Shared/Level_Base.h"
#include "Shared/Graph/GraphNodeFactory.h"
#include "Shared/Graph/GraphRenderer.h"
#include "Level_BT.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_BT : public ALevel_Base
{
private:
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="FSMLevel|Input")
	UInputAction* SetTargetAction{};
	
	// Sets default values for this actor's properties
	ALevel_BT();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void BindLevelInputActions() override;
	
private:
	UPROPERTY()
	ASteeringAgent* Agent{nullptr}; // ref
	
	UPROPERTY()
	ASteeringAgent* MouseAgent{nullptr};
	
	void InitBlackboardData(UBlackboardComponent* blackboard);
	
	void OnClick();
	
	GameAI::GraphRenderer GraphRenderer{nullptr};
	GameAI::Graph Graph{false};
	GameAI::GraphNodeFactory<GameAI::Node> NodeFactory{};
};
