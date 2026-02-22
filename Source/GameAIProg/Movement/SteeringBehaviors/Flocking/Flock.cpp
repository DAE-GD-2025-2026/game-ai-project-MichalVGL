#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "Shared/Level_Base.h"


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

	//setup base behaviours
	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>(100.f, 50.f, 10.f);
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
		FVector SpawnLocation(
			FMath::RandRange(-WorldSize / 2.f, WorldSize / 2.f),
			FMath::RandRange(-WorldSize / 2.f, WorldSize / 2.f),
			0.f
		);

		FActorSpawnParameters SpawnParams{};
		ASteeringAgent* Agent = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnLocation, FRotator::ZeroRotator,
		                                                           SpawnParams);
		if (Agent)
		{
			Agents.Add(Agent);
			Agent->SetSteeringBehavior(pPrioritySteering.get());
			Agent->SetDebugRenderingEnabled(false);
		}
	}
}

Flock::~Flock()
{
	// TODO: Cleanup any additional data
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
	for (ASteeringAgent* Agent : Agents)
	{
		if (!Agent)
			continue;

		
		RegisterNeighbors(Agent);
		Agent->Tick(DeltaTime);

		// todo trim to world, (if that is even handled here????)
	}
}

void Flock::RenderDebug()
{
	// TODO: Render all the agents in the flock
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

		// TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// TODO: implement ImGUI sliders for steering behavior weights here
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// TODO: Debugrender the neighbors for the first agent in the flock
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
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
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D AccPosition = FVector2D::ZeroVector;

	if (NrOfNeighbors == 0)
		return AccPosition;

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		AccPosition += FVector2D(Neighbors[i]->GetActorLocation());
	}

	return AccPosition / NrOfNeighbors;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D accVelocity = FVector2D::ZeroVector;

	if (NrOfNeighbors == 0)
		return accVelocity;

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		accVelocity += Neighbors[i]->GetLinearVelocity();
	}

	return accVelocity / NrOfNeighbors;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	pSeekBehavior->SetTarget(Target);
}
