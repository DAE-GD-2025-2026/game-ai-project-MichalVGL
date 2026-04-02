#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

#include <ranges>

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		bool connected = IsConnected();
		if (!connected)
			return Eulerianity::notEulerian;

		const auto& nodes = m_pGraph->GetNodes();
		int oddAmount = std::ranges::count_if(nodes, [&](const std::unique_ptr<Node>& pNode)
		{
			int amountOfConnections = m_pGraph->FindConnectionsFrom(pNode->GetId()).size();
			return amountOfConnections & 1; //return true if odd
		});

		if (oddAmount > 2)
			return Eulerianity::notEulerian;
		else if (oddAmount > 0)
			return Eulerianity::semiEulerian;

		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{Graphs::InvalidNodeId};

		eulerianity = IsEulerian();

		switch (eulerianity)
		{
		case Eulerianity::notEulerian:
			return Path;
		case Eulerianity::semiEulerian:
			{
				auto startNodeIter = std::ranges::find_if(Nodes, [&](const Node* pNode)
				{
					int amountOfConnections = graphCopy.FindConnectionsFrom(pNode->GetId()).size();
					return amountOfConnections & 1; //return true if odd
				});

				if (startNodeIter != Nodes.end())
					currentNodeId = (*startNodeIter)->GetId();
				else
				{
					UE_LOG(LogTemp, Warning,
					       TEXT(
						       "Fault in FindPath logic, the eulerianity wad defined semiEulerian but didnt have any odd connected node"
					       ))
					return Path;
				}
			}
			break;
		case Eulerianity::eulerian:
			currentNodeId = Nodes[0]->GetId();
			break;
		}


		std::stack<int> nodeStack;
		std::vector<Node*> activeNodes = m_pGraph->GetActiveNodes();
		do
		{
			auto connections = graphCopy.FindConnectionsFrom(currentNodeId);
			if (connections.size() != 0)
			{
				nodeStack.push(currentNodeId);
				currentNodeId = connections[0]->GetToId();
				graphCopy.RemoveConnection(nodeStack.top(), currentNodeId);
			}
			else
			{
				Path.emplace_back(m_pGraph->GetNode(currentNodeId).get());
				if (!nodeStack.empty())
				{
					currentNodeId = nodeStack.top();
					nodeStack.pop();
				}
			}
		}
		while (!nodeStack.empty() || graphCopy.FindConnectionsFrom(currentNodeId).size() != 0);

		Path.emplace_back(m_pGraph->GetNode(currentNodeId).get());

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited,
	                                           int startIndex) const
	{
		if (visited.size() != Nodes.size()) //should only be called once by the first time this function is called
			visited.resize(Nodes.size(), false);

		visited[startIndex] = true;
		auto connections = m_pGraph->FindConnectionsFrom(Nodes[startIndex]->GetId());

		std::ranges::for_each(connections, [&](const Connection* pConnection)
		{
			int toId = pConnection->GetToId();
			auto it = std::ranges::find_if(Nodes, [toId](const Node* n) { return n->GetId() == toId; });
			if (it == Nodes.end())
				return;
			int toIndex = std::distance(Nodes.begin(), it);

			if (visited[toIndex])
				return;

			VisitAllNodesDFS(Nodes, visited, toIndex);
		});
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> nodes = m_pGraph->GetActiveNodes();
		if (nodes.size() == 0)
			return false;

		const auto& connections = m_pGraph->GetConnections();

		//find the first node index that has a connection to another node
		int startNodeIndex = std::invoke([&]() -> int
		{
			Node* pNode{};
			for (int i = 0; i < nodes.size(); ++i)
			{
				pNode = nodes[i];
				bool found =
					std::ranges::any_of(connections, [&](const std::unique_ptr<Connection>& pConnection) -> bool
					{
						return pConnection->GetFromId() == pNode->GetId();
					});

				if (found)
					return i;
			}

			return -1;
		});

		if (startNodeIndex < 0)
		{
			return false;
		}

		std::vector<bool> visited{};
		visited.reserve(nodes.size());
		VisitAllNodesDFS(nodes, visited, startNodeIndex);

		bool anyFalse = std::find(visited.begin(), visited.end(), false) != visited.end();
		return !anyFalse;
	}
}
