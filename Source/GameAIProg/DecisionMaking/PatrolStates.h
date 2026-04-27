#pragma once

#include "FSM/FSMBaseClasses.h"
#include "FSM/FSMComponent.h"

namespace GameAI
{
	class IsTargetVisible final
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
	
	void CreatePatrolFSM(UFSMComponent& fsm)
	{
		//patrol state
		FSM::StateBuilder stateBuilder{"Patrol"};
		//stateBuilder.SetUpdate(); //add patrol code here
		
		std::unique_ptr<FSM::State> patrolState = stateBuilder.GetState();
		
		//search state
		stateBuilder = FSM::StateBuilder{"Search"};
		
		std::unique_ptr<FSM::State> searchState = stateBuilder.GetState();
		
		//chase state
		stateBuilder = FSM::StateBuilder("Chase");
		
		std::unique_ptr<FSM::State> chaseState = stateBuilder.GetState();
		
		//set transitions
		
	}
}
