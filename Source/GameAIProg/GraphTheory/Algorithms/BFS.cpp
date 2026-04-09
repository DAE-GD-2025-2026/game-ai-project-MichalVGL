#include "BFS.h"

#include <map>
#include <queue>
#include <unordered_map>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode, bool useFallback) const
{
	std::vector<Node*> path;
	
	std::queue<Node*> queue{};
	std::unordered_map<int, Node*> parentMap{};
	queue.push(pStartNode);
	parentMap[pStartNode->GetId()] = nullptr;
	
	while (!queue.empty())
	{
		Node* pNode = queue.front();
		queue.pop();
		
		if (pNode == pDestinationNode) //reconstruct path
		{
			Node* current = pDestinationNode;
			while (current != nullptr)
			{
				path.push_back(current);
				current = parentMap[current->GetId()];
			}
			std::ranges::reverse(path);
			return path;
		} 
		
		for (auto connection : pGraph->FindConnectionsFrom(pNode->GetId()))
		{
			int neighbourID = connection->GetToId();
			
			if (!parentMap.contains(neighbourID))
			{
				Node* pNeighbour = pGraph->GetNode(neighbourID).get();
				parentMap[neighbourID] = pNode;
				queue.push(pNeighbour);
			}
		}
	}
	
	if (path.empty() && useFallback) //no path found
	{
		//find closest reachable node to destination
		Node* pClosest = nullptr;
		float closestDistSq = FLT_MAX;

		for (const auto& id : parentMap | std::views::keys)
		{
			Node* pNode = pGraph->GetNode(id).get();
			float dist = FVector2D::DistSquared(pNode->GetPosition(), pDestinationNode->GetPosition());
			if (dist < closestDistSq)
			{
				closestDistSq = dist;
				pClosest = pNode;
			}
		}

		//reconstruct using the closest node
		if (pClosest && pClosest != pStartNode)
		{
			Node* current = pClosest;
			while (current != nullptr)
			{
				path.push_back(current);
				current = parentMap[current->GetId()];
			}
			std::ranges::reverse(path);
		}
	}
	
	return path;
}
