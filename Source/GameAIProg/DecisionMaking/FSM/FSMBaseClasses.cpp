#include "FSMBaseClasses.h"

#include <ranges>
#include <stdexcept>
#include <format>

#include "Binding/States/WidgetStateRegistration.h"
#include "Chaos/ClusterUnionManager.h"

// ==========================================================
// STATE Builder
// ==========================================================

GameAI::FSM::StateBuilder::StateBuilder(const std::string& stateName)
{
	BuildingState = std::make_unique<State>(stateName);
}

GameAI::FSM::StateBuilder& GameAI::FSM::StateBuilder::SetOnEnter(std::function<void(State* previousState)>&& func)
{
	BuildingState->OnEnter = std::move(func);
	return *this;
}

GameAI::FSM::StateBuilder& GameAI::FSM::StateBuilder::SetOnExit(std::function<void(State* previousState)>&& func)
{
	BuildingState->OnExit = std::move(func);
	return *this;
}

GameAI::FSM::StateBuilder& GameAI::FSM::StateBuilder::SetUpdate(std::function<bool()>&& func)
{
	BuildingState->Update = std::move(func);
	return *this;
}

std::unique_ptr<GameAI::FSM::State>&& GameAI::FSM::StateBuilder::GetState()
{
	return std::move(BuildingState);
}

// ==========================================================
// STATE
// ==========================================================

GameAI::FSM::State::State(const std::string& name)
	: Name(name)
{
}

const std::string& GameAI::FSM::State::GetName() const
{
	return Name;
}

// ==========================================================
// TRANSITION
// ==========================================================

GameAI::FSM::Transition::Transition(State* from, State* to, std::function<bool()>&& evalFunc)
	: pFromState(from),
	  pToState(to),
	  EvalFunc(std::move(evalFunc))
{
}

bool GameAI::FSM::Transition::Evaluate() const
{
	return EvalFunc();
}

GameAI::FSM::State* const GameAI::FSM::Transition::GetToState() const
{
	return pToState;
}

GameAI::FSM::State* const GameAI::FSM::Transition::GetFromState() const
{
	return pFromState;
}

// ==========================================================
// TRANSITION
// ==========================================================

GameAI::FSM::FSM::FSM(std::unique_ptr<GameAI::FSM::State>&& startState)
{
	AddState(std::move(startState));
	SwitchToState(startState.get());
}

void GameAI::FSM::FSM::AddState(std::unique_ptr<GameAI::FSM::State>&& newState)
{
	//validate state
	if (newState.get() == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("Tried to add a nullptr state to the FSM"));
		throw std::invalid_argument("Tried to add a nullptr transition to the FSM");
	} else if (HasState(newState->GetName()))
	{
		UE_LOG(LogTemp, Warning, TEXT("Transition already exists"));
	}
	
	States.emplace_back(std::move(newState));
}

void GameAI::FSM::FSM::AddTransition(std::unique_ptr<Transition>&& transition)
{
	//validate transition
	if (transition.get() == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("Tried to add a nullptr transition to the FSM"));
		throw std::invalid_argument("Tried to add a nullptr transition to the FSM");
	} 
	else if (transition->GetFromState() == nullptr || transition->GetToState() == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("Tried to add a transition containing a nullptr state to the FSM"));
		throw std::invalid_argument("Tried to add a nullptr transition to the FSM");
	} 
	else if (HasTransition(transition->GetFromState(), transition->GetToState()))
	{
		UE_LOG(LogTemp, Warning, TEXT("Transition already exists"));
		return;
	}
	else if (!HasState(transition->GetFromState()) || !HasState(transition->GetToState()))
	{
		UE_LOG(LogTemp, Warning, TEXT("Tried adding transition that contains an unknows state, add the state first!!"));
		return;
	}
	
	Transitions.emplace_back(std::move(transition));
}

void GameAI::FSM::FSM::Update()
{
	//update state
	pCurrentState->Update();
	
	//check transitions
	auto validTransition = std::ranges::find_if(CurrentTransitions, [this](auto& transition)
	{
		return transition->Evaluate() == true;
	});
	
	if (validTransition != CurrentTransitions.end())
	{
		SwitchToState((*validTransition)->GetToState());
	}
}

bool GameAI::FSM::FSM::HasState(const std::string& name) const
{
	return std::ranges::any_of(States, [&name](const auto& state) 
		{ return state->GetName() == name; });
}

bool GameAI::FSM::FSM::HasState(const State* const state) const
{
	return HasState(state->GetName());
}

bool GameAI::FSM::FSM::HasTransition(const std::string& fromName, const std::string& toName) const
{
	return std::ranges::any_of(Transitions, [&fromName, &toName](const auto& transition)
	{
		return transition->GetFromState()->GetName() == fromName 
		&& transition->GetToState()->GetName() == toName;
	});
}

bool GameAI::FSM::FSM::HasTransition(const State* const fromState, const State* const toState) const
{
	return HasTransition(fromState->GetName(), toState->GetName());
}

void GameAI::FSM::FSM::SwitchToState(State* newState)
{
	pCurrentState->OnExit(newState);	//exit the current
	newState->OnEnter(pCurrentState);	//enter the new
	pCurrentState = newState;			//set the current to new
	
	CurrentTransitions.clear();
	if (CurrentTransitions.size() < Transitions.size())
	{
		CurrentTransitions.resize(Transitions.size());
	}
	
	//copy transition that have the newstate as the fromState
	auto transitionsView = Transitions | std::views::transform([](auto& transition) { return transition.get(); });
	std::ranges::copy_if(transitionsView, std::back_inserter(CurrentTransitions)
		, [&](const Transition* transition) { return transition->GetFromState()->GetName() == newState->GetName(); });
}
