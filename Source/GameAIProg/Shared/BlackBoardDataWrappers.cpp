// Fill out your copyright notice in the Description page of Project Settings.


#include "BlackBoardDataWrappers.h"

#include "BehaviorTree/BlackboardData.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Object.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Class.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Vector.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Rotator.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Bool.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Int.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Float.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_String.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Name.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Enum.h"

void PrintBlackBoardData(UBlackboardComponent* blackboard)
{
	if (!blackboard) return;
	
	const UBlackboardData* BBData = blackboard->GetBlackboardAsset();
	if (!BBData) return;

	for (const FBlackboardEntry& Entry : BBData->Keys)
	{
		const FName& KeyName = Entry.EntryName;
		
		if (Entry.KeyType->IsA<UBlackboardKeyType_Object>())
        {
            UObject* Value = blackboard->GetValueAsObject(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (Object) | Address: %p"), *KeyName.ToString(), Value);
        }
        else if (Entry.KeyType->IsA<UBlackboardKeyType_Class>())
        {
            UClass* Value = blackboard->GetValueAsClass(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (Class) | Address: %p"), *KeyName.ToString(), Value);
        }
        else if (Entry.KeyType->IsA<UBlackboardKeyType_Vector>())
        {
            FVector Value = blackboard->GetValueAsVector(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (Vector) | Value: %s"), *KeyName.ToString(), *Value.ToString());
        }
        else if (Entry.KeyType->IsA<UBlackboardKeyType_Rotator>())
        {
            FRotator Value = blackboard->GetValueAsRotator(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (Rotator) | Value: %s"), *KeyName.ToString(), *Value.ToString());
        }
        else if (Entry.KeyType->IsA<UBlackboardKeyType_Bool>())
        {
            bool Value = blackboard->GetValueAsBool(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (Bool) | Value: %s"), *KeyName.ToString(), Value ? TEXT("true") : TEXT("false"));
        }
        else if (Entry.KeyType->IsA<UBlackboardKeyType_Int>())
        {
            int32 Value = blackboard->GetValueAsInt(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (Int) | Value: %d"), *KeyName.ToString(), Value);
        }
        else if (Entry.KeyType->IsA<UBlackboardKeyType_Float>())
        {
            float Value = blackboard->GetValueAsFloat(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (Float) | Value: %f"), *KeyName.ToString(), Value);
        }
        else if (Entry.KeyType->IsA<UBlackboardKeyType_String>())
        {
            FString Value = blackboard->GetValueAsString(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (String) | Value: %s"), *KeyName.ToString(), *Value);
        }
        else if (Entry.KeyType->IsA<UBlackboardKeyType_Name>())
        {
            FName Value = blackboard->GetValueAsName(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (Name) | Value: %s"), *KeyName.ToString(), *Value.ToString());
        }
        else if (Entry.KeyType->IsA<UBlackboardKeyType_Enum>())
        {
            uint8 Value = blackboard->GetValueAsEnum(KeyName);
            UE_LOG(LogTemp, Log, TEXT("Key: %s (Enum) | Value: %d"), *KeyName.ToString(), Value);
        }
        else
        {
            UE_LOG(LogTemp, Log, TEXT("Key: %s | Unknown type"), *KeyName.ToString());
        }
	}
}
