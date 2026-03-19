#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//todo, complete findpath
	//Create the path to return
	std::vector<FVector2D> finalPath{};
	
	if (pNavGraph->GetNavPolygon() == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("pNavGraph contains an invalid NavPolygon"));
		return finalPath;
	}
	

	//Get the start and endTriangle
	const auto* startTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(startPos, true);
	const auto* endTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(endPos, true);
	
	// Cache the edges before modifying the graph
	auto startEdges = startTriangle->GetEdges();
	auto endEdges = endTriangle->GetEdges();
	
	if (startTriangle == nullptr 
		|| endTriangle == nullptr 
		|| startTriangle == endTriangle) return finalPath;

	//We have valid start/end triangles and they are not the same
	//=> Start looking for a path
	//Copy the graph
	NavGraph graphCopy = *pNavGraph;
	
	//Create Extra node for the Start Node (Agent's position
	//Create extra node for the endNode
	int startNodeId = graphCopy.AddNode(std::make_unique<Node>(startPos));
	int endNodeId = graphCopy.AddNode(std::make_unique<Node>(endPos));
	
	//add debug node positions
	debugNodePositions.push_back(graphCopy.GetNode(startNodeId)->GetPosition());
	debugNodePositions.push_back(graphCopy.GetNode(endNodeId)->GetPosition());

	//Create the connections
	//for start node
	for (const auto& edge: startEdges)
	{
		if (auto edgeIndex = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge); edgeIndex.has_value())
		{
			if (int nodeId = graphCopy.GetNodeIdFromEdgeIndex(edgeIndex.value()); nodeId != Graphs::InvalidNodeId)
			{
				graphCopy.AddConnection(startNodeId, nodeId);
				graphCopy.AddConnection(nodeId, startNodeId);
			}
		}
	}
	
	//for end node
	for (const auto& edge: endEdges)
	{
		if (auto edgeIndex = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge); edgeIndex.has_value())
		{
			if (int nodeId = graphCopy.GetNodeIdFromEdgeIndex(edgeIndex.value()); nodeId != Graphs::InvalidNodeId)
			{
				graphCopy.AddConnection(endNodeId, nodeId);
				graphCopy.AddConnection(nodeId, endNodeId);
			}
		}
	}

	//Run A star on new graph
	//todo make heuristic function a imgui setting
	AStar pathfinder{&graphCopy, HeuristicFunctions::Manhattan};

	std::vector<Node*> nodePath = pathfinder.FindPath(graphCopy.GetNode(startNodeId).get(), graphCopy.GetNode(endNodeId).get());
	//Debug Visualisation
	
	//todo, make the extra and remove the temp finalpath creation from nodes
	std::ranges::transform(nodePath, std::back_inserter(finalPath), [](const Node* pNode) -> FVector2D
	{
		return pNode->GetPosition();
	});

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	// debugPortals = SSFA::FindPortals(nodes, *pNavGraph->GetNavPolygon());
	// finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}