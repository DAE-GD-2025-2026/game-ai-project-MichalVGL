#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "Shared/Level_Base.h"

#include <format>

Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	  , FlockSize{FlockSize}
	  , pAgentToEvade{pAgentToEvade}
{
	//reserve max space possible for neighbours
	Neighbors.SetNum(FlockSize);

	//reserve for partitions
	pPartitionedSpace = std::make_unique<CellSpace>(pWorld, WorldSize * 2.f, WorldSize * 2.f, NrOfCellsX, NrOfCellsY,
	                                                FlockSize);
	OldPositions.Reserve(FlockSize);

	//setup base behaviours
	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>(150.f, 100.f, 2.f);	
	//wander is currently broken for the flock due to it calculating the behavior x amount of times and adding up the angles
	//there is no easy fix for it, it requires changing core elements of the interface and the way all behaviors and steering work
	pEvadeBehavior = std::make_unique<Evade>(400.f);

	//setup blended behaviour
	pBlendedSteering = std::make_unique<BlendedSteering>(std::vector<BlendedSteering::WeightedBehavior>{
		{pSeparationBehavior.get(), 0.3f},
		{pCohesionBehavior.get(), 0.2f},
		{pVelMatchBehavior.get(), 0.2f},
		{pSeekBehavior.get(), 0.2f},
		{pWanderBehavior.get(), 0.1f}
	});

	pPrioritySteering = std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*>{
		pEvadeBehavior.get(),
		pBlendedSteering.get()
	});

	//spawn flock agents
	Agents.Reserve(FlockSize);
	for (int i = 0; i < FlockSize; i++)
	{
		FActorSpawnParameters SpawnParams{};

		ASteeringAgent* Agent;
		int maxTries{10};
		int tries{0};
		do
		{
			FVector SpawnLocation(
				FMath::RandRange(-WorldSize, WorldSize),
				FMath::RandRange(-WorldSize, WorldSize),
				0.f);

			Agent = pWorld->SpawnActor<ASteeringAgent>(AgentClass
			                                           , SpawnLocation
			                                           , FRotator::ZeroRotator
			                                           , SpawnParams);
			++tries;
		}
		while (Agent == nullptr && tries < maxTries);

		if (Agent)
		{
			Agents.Add(Agent);
			Agent->SetSteeringBehavior(pPrioritySteering.get());
			Agent->SetDebugRenderingEnabled(false);
			Agent->SetActorTickEnabled(false);
			pPartitionedSpace->AddAgent(*Agent);
			OldPositions.Add(Agent->GetPosition());
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Couldn't spawn an agent in the flock"))
		}
	}

	UE_LOG(LogTemp, Log, TEXT("Amount of agents spawned: %d"), Agents.Num());
	UE_LOG(LogTemp, Log, TEXT("Amount desired: %d"), FlockSize);
}

Flock::~Flock()
{
}

void Flock::Tick(float DeltaTime)
{
	//update evade target
	FTargetData Target;
	Target.Position = pAgentToEvade->GetPosition();
	Target.Orientation = pAgentToEvade->GetRotation();
	Target.LinearVelocity = pAgentToEvade->GetLinearVelocity();
	Target.AngularVelocity = pAgentToEvade->GetAngularVelocity();
	pEvadeBehavior->SetTarget(Target);

	//update every flock agent
	for (int i{}; i < Agents.Num(); i++)
	{
		ASteeringAgent* Agent = Agents[i];

		if (!Agent)
			continue;

		RegisterNeighbors(Agent);
		Agent->Tick(DeltaTime);

		if (UsePartitioning)
		{
			pPartitionedSpace->UpdateAgentCell(*Agent, OldPositions[i]);
			OldPositions[i] = Agent->GetPosition();
		}
	}

	if (DebugRenderNeighborhood)
	{
		RenderNeighborhood();
	}

	FlushDebugStrings(pWorld);
	//must always be flushed here, otherwise will leave numbers if you turn off partitioning in IMGUI
	if (DebugRenderPartitions && UsePartitioning)
	{
		pPartitionedSpace->RenderCells();
	}
}

void Flock::RenderDebug()
{
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive,
		             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		//ImGui::Separator();
		ImGui::Text("Partitioning");
		ImGui::Indent();
		ImGui::Spacing();
		ImGui::Checkbox("Use partitioning", &UsePartitioning);
		ImGui::Unindent();

		ImGui::Text("Debug Rendering");
		ImGui::Indent();
		ImGui::Spacing();
		ImGui::Checkbox("Steering", &DebugRenderSteering);
		Agents[0]->SetDebugRenderingEnabled(DebugRenderSteering);
		ImGui::Checkbox("1st Neighbourhood", &DebugRenderNeighborhood);
		if (UsePartitioning) ImGui::Checkbox("Partitions", &DebugRenderPartitions);
		ImGui::Unindent();

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		ImGui::Indent();

		auto WeightGuiSliderFunc = [&](ISteeringBehavior* Behaviour, const char* Label)
		{
			if (float* pWeight = pBlendedSteering->GetWeight(Behaviour))
			{
				ImGuiHelpers::ImGuiSliderFloatWithSetter(Label, float{*pWeight}
				                                         , 0.f, 1.f, [&](float Weight) { *pWeight = Weight; });
			}
		};

		WeightGuiSliderFunc(pSeparationBehavior.get(), "Separation");
		WeightGuiSliderFunc(pCohesionBehavior.get(), "Cohesion");
		WeightGuiSliderFunc(pVelMatchBehavior.get(), "Velocity Match");
		WeightGuiSliderFunc(pSeekBehavior.get(), "Seek");
		WeightGuiSliderFunc(pWanderBehavior.get(), "Wander");

		ImGui::Unindent();

		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
	if (UsePartitioning)
	{
		return pPartitionedSpace->GetNeighbors();
	}
	else
	{
		return Neighbors;
	}
}

void Flock::RenderNeighborhood()
{
	ASteeringAgent* const pAgent = Agents[0];

	//draw neighbourhood
	DrawDebugCircle(pWorld, FVector{pAgent->GetPosition(), 0.f}, NeighborhoodRadius
	                , 32, FColor::Purple, false, -1.f, 0, 3.f,
	                FVector(1, 0, 0),
	                FVector(0, 1, 0),
	                false);

	RegisterNeighbors(pAgent);

	auto& neigbors = GetNeighbors();
	int nrOfNeighbors = GetNrOfNeighbors();

	for (int i = 0; i < nrOfNeighbors; ++i)
	{
		DrawDebugCircle(pWorld, FVector{neigbors[i]->GetPosition(), 0.f}, 20.f
		                , 16, FColor::Emerald, false, -1.f, 1, 10.f,
		                FVector(1, 0, 0),
		                FVector(0, 1, 0),
		                false);
	}

	//draw optional cells of partitions
	if (UsePartitioning)
	{
		pPartitionedSpace->DrawNeighborhoodCells(*pAgent, NeighborhoodRadius);
	}
}

void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (UsePartitioning)
	{
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
		return;
	}

	NrOfNeighbors = 0;

	for (ASteeringAgent* OtherAgent : Agents)
	{
		if (OtherAgent == pAgent)
			continue;

		float Distance = FVector2D::Distance(
			FVector2D(pAgent->GetActorLocation()),
			FVector2D(OtherAgent->GetActorLocation())
		);

		if (Distance < NeighborhoodRadius)
		{
			Neighbors[NrOfNeighbors] = OtherAgent;
			++NrOfNeighbors;
		}
	}
}

int Flock::GetNrOfNeighbors() const
{
	if (UsePartitioning)
	{
		return pPartitionedSpace->GetNrOfNeighbors();
	}
	else
	{
		return NrOfNeighbors;
	}
}

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D accPosition = FVector2D::ZeroVector;

	auto& neighbours = GetNeighbors();
	int nrOfNeighbors = GetNrOfNeighbors();

	if (nrOfNeighbors == 0)
		return accPosition;

	for (int i = 0; i < nrOfNeighbors; ++i)
	{
		accPosition += neighbours[i]->GetPosition();
	}

	return accPosition / nrOfNeighbors;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D accVelocity = FVector2D::ZeroVector;

	auto& neighbours = GetNeighbors();
	int nrOfNeighbors = GetNrOfNeighbors();

	if (nrOfNeighbors == 0)
		return accVelocity;

	for (int i = 0; i < nrOfNeighbors; ++i)
	{
		accVelocity += neighbours[i]->GetLinearVelocity();
	}

	return accVelocity / nrOfNeighbors;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	pSeekBehavior->SetTarget(Target);
}
