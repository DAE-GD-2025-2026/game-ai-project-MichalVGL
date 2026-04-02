#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
	{
	public:
		//=== SSFA Functions ===
		//--- References ---
		//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
		//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
		static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
		{
			//Container
			std::vector<NavLine> Portals = {};
			Portals.reserve(Path.size());
			
			if (Path.size() < 2)
			{
				return Portals;
			}

			//For each node received, get it's corresponding line

			//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point

			//Store portal

			//add first node as the start position (degenerate portal)
			Portals.emplace_back(FVector2d{Path[0]->GetPosition()}, FVector2d{Path[0]->GetPosition()});

			using Edge = TriPolygon::Edge;
			const std::vector<Edge>& edges = NavPoly.GetEdges();
			const int portalsAmount = static_cast<int>(Path.size()) - 1;
			for (int i{1}; i < portalsAmount; ++i)
			{
				const int edgeIdx = dynamic_cast<NavGraphNode*>(Path[i])->GetEdgeIdx();
				const Edge& portalEdge = edges[edgeIdx];

				Portals.emplace_back(FVector2d{portalEdge.GetP1(NavPoly)}, FVector2D{portalEdge.GetP2(NavPoly)});

				const FVector2D pathDir = Path[i - 1]->GetPosition() - Path[i]->GetPosition();
				const FVector2D edgeDir = FVector2D{portalEdge.GetP2(NavPoly)} - FVector2D{portalEdge.GetP1(NavPoly)};

				//swap the points if cross is negative
				if (FVector2D::CrossProduct(pathDir, edgeDir) < 0.f)
				{
					std::swap(Portals.back().P1, Portals.back().P2);
				}
			}

			//Add degenerate portal to force end evaluation
			Portals.emplace_back(Path.back()->GetPosition(), Path.back()->GetPosition());

			return Portals;
		}

		static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
		{
			std::vector<FVector2D> Path{};

			if (Portals.size() < 2)
				return Path;

			Path.reserve(Portals.size());
			//P1 == right point of portal, P2 == left point of portal

			//--- RIGHT CHECK ---
			//1. See if moving funnel inwards - RIGHT

			//2. See if new line degenerates a line segment - RIGHT

			//Leftleg becomes new apex point

			//Calculate new legs (if not the end)


			//--- LEFT CHECK ---
			//1. See if moving funnel inwards - LEFT

			//2. See if new line degenerates a line segment - LEFT

			//Rightleg becomes new apex point

			//Calculate new legs (if not the end)


			// Add last path point

			FVector2d currentApex = Portals.at(0).P1;
			FVector2d currentRightLeg = Portals.at(1).P1 - currentApex;
			FVector2d currentLeftLeg = Portals.at(1).P2 - currentApex;
			int rightLegIdx{1};
			int leftLegIdx{1};

			int portalAmount = static_cast<int>(Portals.size());
			for (int portalIdx{1}; portalIdx < portalAmount; ++portalIdx)
			{
				NavLine newPortal = Portals.at(portalIdx);

				//check right leg
				const FVector2d newRightLeg = newPortal.P1 - currentApex;

				if (FVector2D::CrossProduct(currentRightLeg, newRightLeg) < 0.f)
				{
					//right leg is moving inwards

					if (FVector2D::CrossProduct(currentLeftLeg, newRightLeg) > 0.f)
					{
						//legs do not cross over eachother
						currentRightLeg = newRightLeg;
						rightLegIdx = portalIdx;
					}
					else //legs do cross
					{
						Path.push_back(currentApex);
						currentApex += currentLeftLeg;
						portalIdx = leftLegIdx + 1;
						leftLegIdx = portalIdx;
						rightLegIdx = portalIdx;

						if (portalIdx >= portalAmount)
							break;

						currentRightLeg = Portals.at(portalIdx).P1 - currentApex;
						currentLeftLeg = Portals.at(portalIdx).P2 - currentApex;
						continue;
					}
				}

				//check left leg
				const FVector2d newLeftLeg = newPortal.P2 - currentApex;

				if (FVector2D::CrossProduct(currentLeftLeg, newLeftLeg) > 0.f)
				{
					//left leg is moving inwards
					if (FVector2D::CrossProduct(currentRightLeg, newLeftLeg) < 0.f)
					{
						//legs do not cross over eachother
						currentLeftLeg = newLeftLeg;
						leftLegIdx = portalIdx;
					}
					else //legs do cross
					{
						Path.push_back(currentApex);
						currentApex += currentRightLeg;
						portalIdx = rightLegIdx + 1;
						leftLegIdx = portalIdx;
						rightLegIdx = portalIdx;

						if (portalIdx >= portalAmount)
							break;

						currentRightLeg = Portals.at(portalIdx).P1 - currentApex;
						currentLeftLeg = Portals.at(portalIdx).P2 - currentApex;
						continue;
					}
				}
			}

			Path.push_back(currentApex);
			Path.push_back(Portals.back().P1);

			return Path;
		}

	private:
		SSFA()
		{
		};

		~SSFA()
		{
		};
	};
}
