#pragma once

#include "BehaviorTree/BlackboardComponent.h"
#include "FSM/FSMBaseClasses.h"
#include "FSM/FSMComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Shared/BlackBoardDataWrappers.h"
#include "Shared/BlackboardExtensions.h"
#include "UObject/ObjectRename.h"

namespace GameAI::FSM
{
	// =======================
	// Blackboard FNames 
	// =======================

	namespace PatrolBBItems
	{
		inline const FName Actor{"SelfActor"};
		inline const FName LootLocation{"LootLocation"};
		inline const FName TargetActor{"TargetActor"};
		inline const FName TargetVisible{"TargetVisible"};
		inline const FName LastKnownLocation{"LastKnownLocation"};
		inline const FName LastPatrolPos{"LastPatrolPosition"};
		inline const FName CurrentPatrolNodeIndex{"CurrentPatrolNodeIndex"};
		inline const FName PatrolPath{"PatrolPath"};
		inline const FName NavGraph{"NavGraph"};
		inline const FName SearchTime{"SearchTime"};
		inline const FName SelfSteeringAgent{"SelfSteeringAgent"};
		inline const FName CurrentSteeringBehavior{"CurrentSteeringBehavior"};
	}

	// =======================
	// Blackboard data struct 
	// =======================

	struct PatrolBlackboard
	{
		PatrolBlackboard(UBlackboardComponent* blackboard)
			: Blackboard{blackboard}
		{
		}

		// --- SelfActor ---
		AActor* GetActor() const
		{
			return Cast<AActor>(Blackboard->GetValueAsObject(PatrolBBItems::Actor));
		}

		void SetActor(AActor* Actor) const
		{
			Blackboard->SetValueAsObject(PatrolBBItems::Actor, Actor);
		}

		// --- TargetActor ---
		AActor* GetTargetActor() const
		{
			return Cast<AActor>(Blackboard->GetValueAsObject(PatrolBBItems::TargetActor));
		}

		void SetTargetActor(AActor* Actor) const
		{
			Blackboard->SetValueAsObject(PatrolBBItems::TargetActor, Actor);
		}

		// --- TargetVisible ---
		bool IsTargetVisible() const
		{
			return Blackboard->GetValueAsBool(PatrolBBItems::TargetVisible);
		}

		void SetTargetVisible(bool bVisible) const
		{
			Blackboard->SetValueAsBool(PatrolBBItems::TargetVisible, bVisible);
		}

		// --- LastKnownLocation ---
		FVector GetLastKnownLocation() const
		{
			return Blackboard->GetValueAsVector(PatrolBBItems::LastKnownLocation);
		}

		void SetLastKnownLocation(const FVector& Location) const
		{
			Blackboard->SetValueAsVector(PatrolBBItems::LastKnownLocation, Location);
		}

		// --- LastPatrolPosition ---
		FVector GetLastPatrolPosition() const
		{
			return Blackboard->GetValueAsVector(PatrolBBItems::LastPatrolPos);
		}

		void SetLastPatrolPosition(const FVector& Location) const
		{
			Blackboard->SetValueAsVector(PatrolBBItems::LastPatrolPos, Location);
		}

		// --- CurrentPatrolNodeIndex ---
		int32 GetCurrentPatrolNodeIndex() const
		{
			return Blackboard->GetValueAsInt(PatrolBBItems::CurrentPatrolNodeIndex);
		}

		void SetCurrentPatrolNodeIndex(int32 Index) const
		{
			Blackboard->SetValueAsInt(PatrolBBItems::CurrentPatrolNodeIndex, Index);
		}

		// --- PatrolPath ---
		UPatrolPathData* GetPatrolPath() const
		{
			return Cast<UPatrolPathData>(Blackboard->GetValueAsObject(PatrolBBItems::PatrolPath));
		}

		void SetPatrolPath(UPatrolPathData* Path) const
		{
			Blackboard->SetValueAsObject(PatrolBBItems::PatrolPath, Path);
		}

		// --- NavGraph ---
		//todo, Replace UObject with your actual navgraph class
		UObject* GetNavGraph() const
		{
			return Blackboard->GetValueAsObject(PatrolBBItems::NavGraph);
		}

		void SetNavGraph(UObject* Graph) const
		{
			Blackboard->SetValueAsObject(PatrolBBItems::NavGraph, Graph);
		}

		// --- SearchTime ---
		float GetSearchTime() const
		{
			return Blackboard->GetValueAsFloat(PatrolBBItems::SearchTime);
		}

		void SetSearchTime(float Time) const
		{
			Blackboard->SetValueAsFloat(PatrolBBItems::SearchTime, Time);
		}

		// --- LootLocation ---
		FVector GetLootLocation() const
		{
			return Blackboard->GetValueAsVector(PatrolBBItems::LootLocation);
		}

		void SetLootLocation(const FVector& Location) const
		{
			Blackboard->SetValueAsVector(PatrolBBItems::LootLocation, Location);
		}

		// --- SelfSteeringAgent ---
		ASteeringAgent* GetSteeringAgent() const
		{
			return Cast<ASteeringAgent>(Blackboard->GetValueAsObject(PatrolBBItems::SelfSteeringAgent));
		}

		void SetSelfSteeringAgent(ASteeringAgent* Agent) const
		{
			//Blackboard->SetValue<UBlackboardKeyType_TrackedObject>(PatrolBBItems::SelfSteeringAgent, Agent);
			Blackboard->SetValueAsObject(PatrolBBItems::SelfSteeringAgent, Agent);
		}

		// --- CurrentSteeringBehavior ---
		ISteeringBehavior* GetCurrentSteeringBehavior() const
		{
			return Cast<UAgentSteeringBehavior>(Blackboard->GetValue<UBlackboardKeyType_TrackedObject>(PatrolBBItems::CurrentSteeringBehavior))->
			       SteeringBehavior.get();
		}

		void SetCurrentSteeringBehavior(std::unique_ptr<ISteeringBehavior>&& behavior) const
		{
			UAgentSteeringBehavior* wrapper = Cast<UAgentSteeringBehavior>(
				Blackboard->GetValue<UBlackboardKeyType_TrackedObject>(PatrolBBItems::CurrentSteeringBehavior));
			if (!wrapper) //initialize the first time the steeringbehaviour is set
			{
				if (auto* actor = GetActor(); actor != nullptr)
				{
					wrapper = NewObject<UAgentSteeringBehavior>(actor);
				}
				else
				{
					UE_LOG(LogTemp, Warning,
					       TEXT(
						       "Tried to assign a steeringbehavior while the actor isn't set on the blackboard, make sure to call SetActor before!!"
					       ));
					return;
				}
			}

			wrapper->SteeringBehavior = std::move(behavior);
			Blackboard->SetValue<UBlackboardKeyType_TrackedObject>(PatrolBBItems::CurrentSteeringBehavior, wrapper);
		}

		UBlackboardComponent* Blackboard;
	};

	// =======================
	// Evaluation Functions
	// =======================

	class IsTargetVisible final //todo, delete and use polymorphism
	{
	public:
		bool operator()()
		{
			//logic of visibility using te mem variables


			return true;
		}

	private:
		//target
		//currentpos
	};

	// =======================
	// Patrol State
	// =======================

	class PatrolState final : public State
	{
	public:
		PatrolState()
			: State{"Patrol"}
		{
		}

		virtual ~PatrolState() override = default;

		virtual void OnEnter(State* previousState, UBlackboardComponent* pBlackboard) override;
		virtual void Update(UBlackboardComponent* pBlackboard, float deltaTime) override;
		virtual void OnExit(State* nextState, UBlackboardComponent* pBlackboard) override;
	};

	// =======================
	// Chase State
	// =======================

	class ChaseState final : public State
	{
	public:
		ChaseState()
			: State{"Chase"}
		{
		};

		virtual ~ChaseState() override = default;

		virtual void OnEnter(State* previousState, UBlackboardComponent* pBlackboard) override;
		virtual void Update(UBlackboardComponent* pBlackboard, float deltaTime) override;
		virtual void OnExit(State* nextState, UBlackboardComponent* pBlackboard) override;
	};

	// =======================
	// Search State
	// =======================

	class SearchState final : public State
	{
	public:
		SearchState()
			: State{"Search"}
		{
		};

		virtual ~SearchState() override = default;

		virtual void OnEnter(State* previousState, UBlackboardComponent* pBlackboard) override;
		virtual void Update(UBlackboardComponent* pBlackboard, float deltaTime) override;
		virtual void OnExit(State* nextState, UBlackboardComponent* pBlackboard) override;
	};
}
