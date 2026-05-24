#pragma once
#include <functional>
#include <memory>
#include <string>

#include "AIController.h"

namespace GameAI::FSM
{
	class State
	{
	public:
		State(const std::string& name);
		
		virtual ~State() = default;

		const std::string& GetName() const;
		
		virtual void OnEnter(State* previousState, UBlackboardComponent* pBlackboard) = 0;
		virtual void Update(UBlackboardComponent* pBlackboard, float deltaTime) = 0;
		virtual void OnExit(State* nextState, UBlackboardComponent* pBlackboard) = 0;
		
	private:
		std::string Name;
	};

	class Transition final
	{
	public:
		Transition(State* from, State* to, std::function<bool(UBlackboardComponent*)>&& evalFunc);

		bool Evaluate(UBlackboardComponent* pBlackboard) const;

		State* const GetToState() const;
		State* const GetFromState() const;

	private:
		State* pFromState;
		State* pToState;

		std::function<bool(UBlackboardComponent*)> EvalFunc;
	};

	class FSM
	{
	public:
		FSM(std::unique_ptr<GameAI::FSM::State>&& startState, UBlackboardComponent* blackboard);

		void AddState(std::unique_ptr<GameAI::FSM::State>&& newState);
		void AddTransition(std::unique_ptr<Transition>&& transition);

		void Update(UBlackboardComponent* blackboard, float deltaTime);

		bool HasState(const std::string& name) const;
		bool HasState(const State* const state) const;
		bool HasTransition(const std::string& fromName, const std::string& toName) const;
		bool HasTransition(const State* const fromState, const State* const toState) const;
		
	private:
		void SwitchToState(State* newState, UBlackboardComponent* blackboard);
		void SetValidTransitions();

		std::vector<std::unique_ptr<State>> States{};
		std::vector<std::unique_ptr<Transition>> Transitions{};

		State* pCurrentState{};
		std::vector<Transition*> CurrentTransitions{};
	};
}
