#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Object.h"
#include "BlackboardExtensions.generated.h"

UCLASS(EditInlineNew, meta=(DisplayName="TrackedObject"), MinimalAPI)
class UBlackboardKeyType_TrackedObject : public UBlackboardKeyType_Object
{
	GENERATED_UCLASS_BODY()

private:
	UPROPERTY()
	TObjectPtr<UObject> TrackedObject;

public:
	static bool SetValue(UBlackboardKeyType_TrackedObject* KeyOb, uint8* RawData, UObject* Value)
	{
		KeyOb->TrackedObject = Value;
		return UBlackboardKeyType_Object::SetValue(KeyOb, RawData, Value);
	}
	
	virtual void InitializeMemory(UBlackboardComponent& OwnerComp, uint8* MemoryBlock) override
	{
		TrackedObject = nullptr;
		Super::InitializeMemory(OwnerComp, MemoryBlock);
	}
};

