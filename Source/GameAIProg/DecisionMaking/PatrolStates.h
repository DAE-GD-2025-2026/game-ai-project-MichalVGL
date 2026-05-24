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
		inline const FName HasLastPatrolPos{"HasLastPatrolPosition"};
		inline const FName IsReturningToPatrol{"IsReturningToPatrol"};
		inline const FName CurrentPatrolNodeIndex{"CurrentPatrolNodeIndex"};
		inline const FName PatrolPath{"PatrolPath"};
		inline const FName SearchTime{"SearchTime"};
		inline const FName MaximumSearchTime{"MaximumSearchTime"};
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
		
		// --- IsReturningToPatrol ---
		bool GetIsReturningToPatrol() const
		{
			return Blackboard->GetValueAsBool(PatrolBBItems::IsReturningToPatrol);
		}
		
		void SetIsReturningToPatrol(bool bIsReturning) const
		{
			Blackboard->SetValueAsBool(PatrolBBItems::IsReturningToPatrol, bIsReturning);
		}
		
		// --- HasLastPatrolPos ---
		bool GetHasLastPatrolPos() const
		{
			return Blackboard->GetValueAsBool(PatrolBBItems::HasLastPatrolPos);
		}
		
		void SetHasLastPatrolPos(bool nHasPatrolPos) const
		{
			Blackboard->SetValueAsBool(PatrolBBItems::HasLastPatrolPos, nHasPatrolPos);
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
			return Cast<UPatrolPathData>(
				Blackboard->GetValue<UBlackboardKeyType_TrackedObject>(PatrolBBItems::PatrolPath));
			//return Cast<UPatrolPathData>(Blackboard->GetValueAsObject(PatrolBBItems::PatrolPath));
		}

		void SetPatrolPath(UPatrolPathData* Path) const
		{
			Blackboard->SetValue<UBlackboardKeyType_TrackedObject>(PatrolBBItems::PatrolPath, Path);
			//Blackboard->SetValueAsObject(PatrolBBItems::PatrolPath, Path);
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
		
		// --- MaximumSearchTime ---
		float GetMaximumSearchTime() const
		{
			return Blackboard->GetValueAsFloat(PatrolBBItems::MaximumSearchTime);
		}
		
		void SetMaximumSearchTime(float Time) const
		{
			Blackboard->SetValueAsFloat(PatrolBBItems::MaximumSearchTime, Time);
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

		UBlackboardComponent* Blackboard;
	};

	// =======================
	// Evaluation Functions
	// =======================

	static bool IsTargetVisible(UBlackboardComponent* pBlackboard)
	{
		PatrolBlackboard bb{pBlackboard};

		return bb.IsTargetVisible();
	}
	
	static bool IsSearchingTooLong(UBlackboardComponent* pBlackboard)
	{
		PatrolBlackboard bb{pBlackboard};
		
		return bb.GetSearchTime() >= bb.GetMaximumSearchTime(); 
	}

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
