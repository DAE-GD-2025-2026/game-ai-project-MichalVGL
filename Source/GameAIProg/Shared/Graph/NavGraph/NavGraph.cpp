#include "NavGraph.h"

#include <numeric>
#include <unordered_map>

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon>&& NavPoly)
	: Graph{false}
	  , pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const& OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}

	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const& OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const& pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}

	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	UE_LOG(LogTemp, Log, TEXT("Edges: %d Triangles: %d"), (int)pNavPoly->GetEdges().size(),
	       (int)pNavPoly->GetTriangles().size());
	//TODO create navGraph
	//1. Go over all the edges of the navigation mesh and create nodes
	// Create node here
	//1st version
	//std::ranges::for_each(pNavPoly->GetEdges(), [&](const TriPolygon::Edge& edge)
	//{
	//	const FVector center = (edge.GetP1(*pNavPoly) + edge.GetP2(*pNavPoly)) / 2.f;
	//	//todo, check how many triangles have this edge, it should be 2
	//	auto edgeIndex = pNavPoly->FindEdgeIndex(edge);
	//	if (edgeIndex.has_value())
	//		AddNode(std::make_unique<NavGraphNode>(FVector2d(center), edgeIndex.value()));
	//});

	//2nd version
	{
		using EdgeId = int;
		using EdgeCount = int;
		std::unordered_map<EdgeId, EdgeCount> edgeMap{};
		std::ranges::for_each(pNavPoly->GetTriangles(), [&](const TriPolygon::Triangle& triangle)
		{
			for (auto& edge : triangle.GetEdges())
			{
				int edgeIndex{};
				if (auto edgeI = pNavPoly->FindEdgeIndex(edge); edgeI.has_value())
				{
					edgeIndex = edgeI.value();
				}
				else continue;

				//update map to keep track of the amount of triangles an edge has
				edgeMap[edgeIndex]++;

				//create node only if the edge has 2 triangles
				if (auto edgeIt = edgeMap.find(edgeIndex); edgeIt != edgeMap.end() && edgeIt->second == 2)
				{
					const FVector center = (edge.GetP1(*pNavPoly) + edge.GetP2(*pNavPoly)) / 2.f;
					AddNode(std::make_unique<NavGraphNode>(FVector2d(center), edgeIndex));
				}
			}
		});
	}

	//2. Create connections now that every node is created	
	//2 valid nodes -> 1 connection
	//3 valid nodes -> 3 connections
	std::ranges::for_each(pNavPoly->GetTriangles(), [&](const TriPolygon::Triangle& triangle)
	{
		std::array<std::optional<int>, 3> edgeIndices{};
		//get the edge indices
		for (int i = 0; i < edgeIndices.size(); ++i)
		{
			edgeIndices[i] = pNavPoly->FindEdgeIndex(triangle.GetEdges()[i]);
			if (!edgeIndices[i].has_value())
			{
				//the triangle is invalid
				UE_LOG(LogTemp, Error, TEXT("TriPolygon::FindEdgeIndex() failed"));
				return;
			}
		}

		std::vector<int> nodeIndices{};
		nodeIndices.reserve(edgeIndices.size());
		// get valid nodes
		for (int i = 0; i < edgeIndices.size(); ++i)
		{
			const int nodeIndex = GetNodeIdFromEdgeIndex(edgeIndices[i].value());
			if (nodeIndex != Graphs::InvalidNodeId)
			{
				nodeIndices.push_back(nodeIndex);
			}
		}

		if (nodeIndices.size() < 2)
		{
			//not enough valid nodes
			return;
		}

		const int connectionsAmount = 2 * static_cast<int>(nodeIndices.size()) - 3;
		//1 connection for 2 nodes and 3 connections for 3 nodes

		for (int i = 0; i < connectionsAmount; ++i)
		{
			const int startNodeId = nodeIndices[i];
			const int endNodeId = nodeIndices[(i + 1) % nodeIndices.size()];

			AddConnection(std::make_unique<Connection>(startNodeId, endNodeId));
			AddConnection(std::make_unique<Connection>(endNodeId, startNodeId));
			UE_LOG(LogTemp, Error, TEXT("Node %d has connections"), startNodeId);
		}
	});

	//3. Set the connections cost to the actual distance
	for (auto& pConnection : GetConnections())
	{
		const FVector2D startPos = GetNode(pConnection->GetFromId())->GetPosition();
		const FVector2D endPos = GetNode(pConnection->GetToId())->GetPosition();
		pConnection->SetWeight(FVector2D::Distance(startPos, endPos));
	}
}
