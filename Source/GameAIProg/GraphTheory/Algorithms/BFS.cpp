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

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
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
	
	return path;
}
