#pragma once
#include <functional>
#include <memory>
#include <string>

namespace GameAI::FSM
{
	class State;
	
	class StateBuilder final
	{
	public:
		
		StateBuilder(const std::string& stateName);

		StateBuilder& SetOnEnter(std::function<void(State* previousState)>&& func);
		StateBuilder& SetOnExit(std::function<void(State* previousState)>&& func);
		StateBuilder& SetUpdate(std::function<bool()>&& func);
		
		std::unique_ptr<State>&& GetState();
		
	private: 
		std::unique_ptr<State> BuildingState;
	};

	class State final
	{
	public:
		State(const std::string& name);

		const std::string& GetName() const;

		std::function<void(State* previousState)> OnEnter{};
		std::function<void()> Update{};
		std::function<void(State* nextState)> OnExit{};

	private:
		std::string Name;
	};

	class Transition final
	{
	public:
		Transition(State* from, State* to, std::function<bool()>&& evalFunc);

		bool Evaluate() const;

		State* const GetToState() const;
		State* const GetFromState() const;

	private:
		State* pFromState;
		State* pToState;

		std::function<bool()> EvalFunc;
	};

	class FSM
	{
	public:
		FSM(std::unique_ptr<GameAI::FSM::State>&& startState);

		void AddState(std::unique_ptr<GameAI::FSM::State>&& newState);
		void AddTransition(std::unique_ptr<Transition>&& transition);

		void Update();

		bool HasState(const std::string& name) const;
		bool HasState(const State* const state) const;
		bool HasTransition(const std::string& fromName, const std::string& toName) const;
		bool HasTransition(const State* const fromState, const State* const toState) const;
		
	private:
		void SwitchToState(State* newState);

		std::vector<std::unique_ptr<State>> States;
		std::vector<std::unique_ptr<Transition>> Transitions;

		State* pCurrentState;
		std::vector<Transition*> CurrentTransitions;
		
		//agent specific, use parameters in functions
		//aicontroller
		//blackboard
	};
}
