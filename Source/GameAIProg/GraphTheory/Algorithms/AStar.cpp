#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	  , HeuristicFunction(hFunction)
{
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};
	std::vector<NodeRecord> openList;
	std::vector<NodeRecord> closedList;
	path.reserve(pGraph->GetNodeCount());
	openList.reserve(pGraph->GetNodeCount());
	closedList.reserve(pGraph->GetNodeCount());

	openList.emplace_back(pStartNode, nullptr, 0.f, GetHeuristicCost(pStartNode, pGoalNode));

	auto listContainsNodeFunc = [&](std::vector<NodeRecord>& list, int nodeId) -> NodeRecord*
	{
		auto it = std::ranges::find_if(list, [&](const NodeRecord& record)-> bool
		{
			return record.pNode->GetId() == nodeId;
		});
		
		return it != list.end() ? &*it : nullptr;
	};

	while (!openList.empty())
	{
		//get the new currentnode based on the cheapest predicted cost
		NodeRecord currentNodeRecord = *std::min_element(openList.rbegin(), openList.rend());
		
		//add all connections to the openlist with the calculated costs
		for (auto* pConnection : pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId()))
		{
			//does the connection lead to a node already in a list?
			NodeRecord* pListedRecord = std::invoke([&]() -> NodeRecord*
			{
				NodeRecord* pFoundRecord = listContainsNodeFunc(openList, pConnection->GetToId());
				if (pFoundRecord) 
					return pFoundRecord;
				else
					return listContainsNodeFunc(closedList, pConnection->GetToId());
			});
			
			if (pListedRecord)//if it leads to a node already in any list, check if the path is cheaper and update
			{
				float costToNewNode = currentNodeRecord.costSoFar + pConnection->GetWeight();
				float newEstimatedTotal = costToNewNode + GetHeuristicCost(pListedRecord->pNode, pGoalNode);
				
				if (pListedRecord->estimatedTotalCost > newEstimatedTotal)
				{
					pListedRecord->pConnection = pConnection;
					pListedRecord->estimatedTotalCost = newEstimatedTotal;
					pListedRecord->costSoFar = costToNewNode;
				}
				continue;
			}

			//is the connection reaching the goal?
			Node* pToNode = pGraph->GetNode(pConnection->GetToId()).get();
			Node* pFromNode = pGraph->GetNode(pConnection->GetFromId()).get();
			if (pToNode == pGoalNode)
			{
				//reconstruct path and return
				path.push_back(pGoalNode);

				while (currentNodeRecord.pConnection != nullptr)
				{
					path.emplace_back(currentNodeRecord.pNode);
					
					auto nodeRecordIt = std::ranges::find_if(closedList, [&](const NodeRecord& record) -> bool
					{
						return record.pNode == pGraph->GetNode(currentNodeRecord.pConnection->GetFromId()).get();
					});
					if (nodeRecordIt != closedList.end())
					{
						currentNodeRecord = *nodeRecordIt;
					}
					else
					{
						UE_LOG(LogTemp, Warning,
						       TEXT(
							       "AStar::FindPath created and invalid find iterator. The closedlist behaviour is faulty."
						       ));
					}
				}
				
				path.emplace_back(currentNodeRecord.pNode); //add the first node

				std::ranges::reverse(path);
				return path;
			}

			//add to the openlist
			float costToNewNode{currentNodeRecord.costSoFar + pConnection->GetWeight()};
			openList.emplace_back(pToNode, pConnection
			                      , costToNewNode
			                      , costToNewNode + GetHeuristicCost(pToNode, pGoalNode));
		}

		//remove the current from the openList and add it to the closed one
		auto eraseIt = std::ranges::find(openList, currentNodeRecord);
		if (eraseIt != openList.end())
		{
			openList.erase(eraseIt);
		}
		else
		{
			UE_LOG(LogTemp, Warning,
			       TEXT(
				       "AStar::FindPath created and invalid erase iterator. The openlist and closedlist behaviour is faulty."
			       ));
		}
		closedList.push_back(currentNodeRecord);
	}

	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->
		GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}
