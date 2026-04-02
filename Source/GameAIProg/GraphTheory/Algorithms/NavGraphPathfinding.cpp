#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals, HeuristicFunctions::Heuristic heuristic) 
{
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
	
	if (startTriangle == nullptr 
		|| endTriangle == nullptr 
		) return finalPath;
	
	if (startTriangle == endTriangle)	//in the same triangle, no use of A* pathfinding
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		
		debugNodePositions = finalPath;
		debugPortals = std::vector<NavLine>();
		return finalPath;
	}
	
	// Cache the edges before modifying the graph
	const auto startEdges = startTriangle->GetEdges();
	const auto endEdges = endTriangle->GetEdges();
	
	//We have valid start/end triangles and they are not the same
	//=> Start looking for a path
	//Copy the graph
	NavGraph graphCopy = *pNavGraph;
	
	//Create Extra node for the Start Node (Agent's position
	//Create extra node for the endNode
	const int startNodeId = graphCopy.AddNode(std::make_unique<Node>(startPos));
	const int endNodeId = graphCopy.AddNode(std::make_unique<Node>(endPos));
	
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
	AStar pathfinder{&graphCopy, heuristic};

	std::vector<Node*> nodePath = pathfinder.FindPath(graphCopy.GetNode(startNodeId).get(), graphCopy.GetNode(endNodeId).get());

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	debugPortals = SSFA::FindPortals(nodePath, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	debugNodePositions = finalPath;
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals, HeuristicFunctions::Manhattan);
}