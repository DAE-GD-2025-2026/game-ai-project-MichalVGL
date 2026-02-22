#include "Level_CombinedSteering.h"

#include "imgui.h"


// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();
	
	//setup base behaviours
	pSeekBehaviour = std::make_unique<Seek>();
	pWanderBehaviour = std::make_unique<Wander>(200.f, 100.f, 10.f);
	pEvadeBehaviour = std::make_unique<Evade>(300.f);

	//setup behaviours for blended (drunkagent)
	std::vector<BlendedSteering::WeightedBehavior> WeightedBehaviors{};
	WeightedBehaviors.reserve(2);
	WeightedBehaviors.emplace_back(pSeekBehaviour.get(), 0.5f);
	WeightedBehaviors.emplace_back(pWanderBehaviour.get(), 0.5f);
	pBlendedSteering = std::make_unique<BlendedSteering>(WeightedBehaviors);
	
	//setup behaviour for priority (evading agent)
	std::vector<ISteeringBehavior*> PrioritySteeringBehaviours{};
	PrioritySteeringBehaviours.reserve(2);
	PrioritySteeringBehaviours.emplace_back(pEvadeBehaviour.get());
	PrioritySteeringBehaviours.emplace_back(pWanderBehaviour.get());
	pPrioritySteering = std::make_unique<PrioritySteering>(PrioritySteeringBehaviours);
	
	//spawn the 2 agents
	DrunkAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ -300,0,90 }, FRotator::ZeroRotator);
	EvadingAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ -300,0,90 }, FRotator::ZeroRotator);
	
	DrunkAgent->SetSteeringBehavior(pBlendedSteering.get());
	EvadingAgent->SetSteeringBehavior(pPrioritySteering.get());
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();

}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
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
		ImGui::Spacing();
	
		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
   // TODO: Handle the debug rendering of your agents here :)
			if (CanDebugRender)
			{
				DrunkAgent->SetDebugRenderingEnabled(CanDebugRender);
				EvadingAgent->SetDebugRenderingEnabled(CanDebugRender);
			}
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}
		
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();
	
		ImGui::Text("Drunk Behavior Weights");
		ImGui::Spacing();


		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
			pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
		pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
		[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
	
		ImGui::Text("Evader Behavior Weights");
		ImGui::Spacing();
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("EvadeRadius",
		pEvadeBehaviour->GetEvadeRadius(), -1.f, 700.f,
		[this](float InVal) { pEvadeBehaviour->SetEvadeRadius(InVal); }, "%.2f");
		
		//End
		ImGui::End();
	}
#pragma endregion
	
	// Combined Steering Update
	
	pSeekBehaviour->SetTarget(MouseTarget);
	
	FTargetData Target;
	Target.Position = DrunkAgent->GetPosition();
	Target.Orientation = DrunkAgent->GetRotation();
	Target.LinearVelocity = DrunkAgent->GetLinearVelocity();
	Target.AngularVelocity = DrunkAgent->GetAngularVelocity();
	pEvadeBehaviour->SetTarget(Target);
	
	
}
