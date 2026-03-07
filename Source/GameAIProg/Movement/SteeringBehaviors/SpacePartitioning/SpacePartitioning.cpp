#include "SpacePartitioning.h"

#include "DebugRenderSceneProxy.h"
#include "DynamicMesh/DynamicMesh3.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = {Left, Bottom};
	BoundingBox.Max = {BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height};
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{left, bottom},
		{left, bottom + height},
		{left + width, bottom + height},
		{left + width, bottom},
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	  , SpaceWidth{Width}
	  , SpaceHeight{Height}
	  , NrOfRows{Rows}
	  , NrOfCols{Cols}
	  , NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);

	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;
	CellOrigin = FVector2D(-Width / 2.f, -Height / 2.f);

	Cells.reserve(Rows * Cols);
	for (int r{}; r < Rows; r++)
	{
		for (int c{}; c < Cols; c++)
		{
			Cells.emplace_back(CellOrigin.X + c * CellWidth, CellOrigin.Y + r * CellHeight, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	const FVector2D relPos = Agent.GetPosition() - CellOrigin;
	const int col = relPos.X / CellWidth;
	const int row = relPos.Y / CellHeight;

	if (!ValidateIndex(row, col))
	{
		return;
	}

	Cells[row * NrOfCols + col].Agents.emplace_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	FVector2D newRelPos = Agent.GetPosition() - CellOrigin;
	FVector2D oldRelPos = OldPos - CellOrigin;

	int newCol = newRelPos.X / CellWidth;
	int newRow = newRelPos.Y / CellHeight;

	int oldCol = oldRelPos.X / CellWidth;
	int oldRow = oldRelPos.Y / CellHeight;

	if (newCol != oldCol || newRow != oldRow)
	{
		if (ValidateIndex(oldRow, oldCol))
		{
			Cells[oldRow * NrOfCols + oldCol].Agents.remove(&Agent);
		}

		if (ValidateIndex(newRow, newCol))
		{
			Cells[newRow * NrOfCols + newCol].Agents.emplace_back(&Agent);
		}
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;
	const float queryRadiusSq = QueryRadius * QueryRadius;

	const FVector2D relPos = Agent.GetPosition() - CellOrigin;

	const FRect queryRect = FRect(
		relPos - QueryRadius,
		relPos + QueryRadius);

	const int lowIndex = PositionToIndex<PosToIndexValidateMethod::Clamp, true>(queryRect.Min);
	const int highIndex = PositionToIndex<PosToIndexValidateMethod::Clamp, true>(queryRect.Max);

	const int lowRow = lowIndex / NrOfCols;
	const int highRow = highIndex / NrOfCols;

	const int lowCol = lowIndex % NrOfCols;
	const int highCol = highIndex % NrOfCols;

	for (int row = lowRow; row <= highRow; row++)
	{
		for (int col = lowCol; col <= highCol; col++)
		{
			for (auto* cellAgent : Cells[row * NrOfCols + col].Agents)
			{
				if (FVector2D::DistSquared(cellAgent->GetPosition(), Agent.GetPosition()) < queryRadiusSq)
				{
					Neighbors[NrOfNeighbors] = cellAgent;
					NrOfNeighbors++;
				}
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	for (const Cell& c : Cells)
	{
		const FRect& rect = c.BoundingBox;

		const FVector2D center2D = (rect.Min + rect.Max) * 0.5f;
		const FVector2D extent2D = (rect.Max - rect.Min) * 0.5f;
		const FVector center(center2D.X, center2D.Y, 0.f);
		const FVector extent(extent2D.X, extent2D.Y, 0.f);

		DrawDebugBox(pWorld, center, extent, FColor::Red, false, -1.f);

		DrawDebugString(
			pWorld,
			center,
			FString::FromInt(c.Agents.size()),
			nullptr,
			FColor::White,
			-1.f,
			false
		);
	}
}

void CellSpace::DrawNeighborhoodCells(ASteeringAgent& Agent, float QueryRadius) const
{
	const FVector2D relPos = Agent.GetPosition() - CellOrigin;

	const FRect queryRect = FRect(
		relPos - QueryRadius,
		relPos + QueryRadius);

	const int lowIndex = PositionToIndex<PosToIndexValidateMethod::Clamp, true>(queryRect.Min);
	const int highIndex = PositionToIndex<PosToIndexValidateMethod::Clamp, true>(queryRect.Max);

	const int lowRow = lowIndex / NrOfCols;
	const int highRow = highIndex / NrOfCols;

	const int lowCol = lowIndex % NrOfCols;
	const int highCol = highIndex % NrOfCols;

	for (int row = lowRow; row <= highRow; row++)
	{
		for (int col = lowCol; col <= highCol; col++)
		{
			const FRect& cellBox = Cells[row * NrOfCols + col].BoundingBox;
			const FVector2D center2D = (cellBox.Min + cellBox.Max) * 0.5f;
			const FVector2D extent2D = (cellBox.Max - cellBox.Min) * 0.5f;
			const FVector center(center2D.X, center2D.Y, 0.f);
			const FVector extent(extent2D.X, extent2D.Y, 0.f);
			
			DrawDebugSolidBox(pWorld, center, extent, FColor::Purple, false, -1.f);
		}
	}
}

bool CellSpace::DoRectsOverlap(FRect const& RectA, FRect const& RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;

	// If they are not separated, they must overlap
	return true;
}

bool CellSpace::ValidateIndex(int Row, int Col) const
{
	if (Row >= NrOfRows || Col >= NrOfCols
		|| Row < 0 || Col < 0)
	{
		return false;
	}
	return true;
}

void CellSpace::ClampIndex(int& Row, int& Col) const
{
	if (Row >= NrOfRows)
	{
		Row = NrOfRows - 1;
	}
	else if (Row < 0)
	{
		Row = 0;
	}

	if (Col >= NrOfCols)
	{
		Col = NrOfCols - 1;
	}
	else if (Col < 0)
	{
		Col = 0;
	}
}
