#include "GAPerceptionComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GAPerceptionSystem.h"

UGAPerceptionComponent::UGAPerceptionComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;

	// Default vision parameters
	VisionParameters.VisionAngle = 90.0f;
	VisionParameters.VisionDistance = 1000.0;
}


void UGAPerceptionComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterPerceptionComponent(this);
	}
}

void UGAPerceptionComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterPerceptionComponent(this);
	}
}


APawn* UGAPerceptionComponent::GetOwnerPawn() const
{
	AActor* Owner = GetOwner();
	if (Owner)
	{
		APawn* Pawn = Cast<APawn>(Owner);
		if (Pawn)
		{
			return Pawn;
		}
		else
		{
			AController* Controller = Cast<AController>(Owner);
			if (Controller)
			{
				return Controller->GetPawn();
			}
		}
	}

	return NULL;
}



// Returns the Target this AI is attending to right now.

UGATargetComponent* UGAPerceptionComponent::GetCurrentTarget() const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);

	if (PerceptionSystem && PerceptionSystem->TargetComponents.Num() > 0)
	{
		UGATargetComponent* TargetComponent = PerceptionSystem->TargetComponents[0];
		if (TargetComponent->IsKnown())
		{
			return PerceptionSystem->TargetComponents[0];
		}
	}

	return NULL;
}

bool UGAPerceptionComponent::HasTarget() const
{
	return GetCurrentTarget() != NULL;
}


bool UGAPerceptionComponent::GetCurrentTargetState(FTargetCache& TargetStateOut, FTargetData& TargetDataOut) const
{
	UGATargetComponent* Target = GetCurrentTarget();
	if (Target)
	{
		const FTargetData* TargetData = TargetMap.Find(Target->TargetGuid);
		if (TargetData)
		{
			TargetStateOut = Target->LastKnownState;
			TargetDataOut = *TargetData;
			return true;
		}

	}
	return false;
}


void UGAPerceptionComponent::GetAllTargetStates(bool OnlyKnown, TArray<FTargetCache>& TargetCachesOut, TArray<FTargetData>& TargetDatasOut) const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			const FTargetData* TargetData = TargetMap.Find(TargetComponent->TargetGuid);
			if (TargetData)
			{
				if (!OnlyKnown || TargetComponent->IsKnown())
				{
					TargetCachesOut.Add(TargetComponent->LastKnownState);
					TargetDatasOut.Add(*TargetData);
				}
			}
		}
	}
}


void UGAPerceptionComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateAllTargetData();
}


void UGAPerceptionComponent::UpdateAllTargetData()
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			UpdateTargetData(TargetComponent);
		}
	}
}

void UGAPerceptionComponent::UpdateTargetData(UGATargetComponent* TargetComponent)
{
    // REMEMBER: the UGAPerceptionComponent is going to be attached to the controller, not the pawn. So we call this special accessor to 
    // get the pawn that our controller is controlling
    APawn* OwnerPawn = GetOwnerPawn();
    if (OwnerPawn == NULL)
    {
        return;
    }

    FTargetData *TargetData = TargetMap.Find(TargetComponent->TargetGuid);
    
    if (TargetData == NULL)     // If we don't already have a target data for the given target component, add it
    {
        FTargetData NewTargetData;
        FGuid TargetGuid = TargetComponent->TargetGuid;
        TargetData = &TargetMap.Add(TargetGuid, NewTargetData);
    }

    if (TargetData)
    {

        AActor* TargetActor = TargetComponent->GetOwner();
        if (!TargetActor)
            return;

    	// Get positions
        FVector OwnerLocation = OwnerPawn->GetActorLocation();
        FVector TargetLocation = TargetActor->GetActorLocation();
        FVector DirectionToTarget = (TargetLocation - OwnerLocation).GetSafeNormal();
        float DistanceToTarget = FVector::Distance(OwnerLocation, TargetLocation);
        bool inVisionCone = false;
        
        if (DistanceToTarget <= VisionParameters.VisionDistance) // is the actual distance less than the vision distance
        {
	        // Check if target is within vision angle and calculate dot product
        	FVector OwnerForward = OwnerPawn->GetActorForwardVector();
        	float DotProduct = FVector::DotProduct(OwnerForward, DirectionToTarget);
        	float AngleToTarget = FMath::RadiansToDegrees(
				FMath::Acos(FMath::Clamp(DotProduct, -1.0f, 1.0f))
			);
        	
        	inVisionCone = (AngleToTarget <= VisionParameters.VisionAngle / 2.0f); // vision cone check
        }
        TargetData->bClearLos = false;
        
        
        if (inVisionCone) //if we are in the vision cone
        {
            // Do the line trace
            FHitResult HitResult;
            FCollisionQueryParams CollisionParams;
            CollisionParams.AddIgnoredActor(OwnerPawn);
            
            bool bHit = GetWorld()->LineTraceSingleByChannel(
                HitResult,
                OwnerLocation,
                TargetLocation,
                ECC_Visibility,
                CollisionParams
            );
            
            // we have clearLos if the line hit nothing/the actor
            TargetData->bClearLos = !bHit || (HitResult.GetActor() == TargetActor);
        }

    	// gain awareness twice as fast
        float awarenessGainRate = 0.1f; 
        float awarenessDecayRate = 0.05f;

        if (TargetData->bClearLos)
        {
            TargetData->Awareness += awarenessGainRate;
        }
        else
        {
            TargetData->Awareness -= awarenessDecayRate;
        }
    	
        TargetData->Awareness = FMath::Clamp(TargetData->Awareness, 0.0f, 1.0f);
    }
}


// A visibility test for a specific perception component in the scene. Helper
bool UGAPerceptionComponent::TestVisibility(const FVector& TestLocation) const
{
	APawn* OwnerPawn = GetOwnerPawn();
	if (!OwnerPawn)
		return false;
	
	FVector OwnerLocation = OwnerPawn->GetActorLocation();
	FVector DirectionToTest = (TestLocation - OwnerLocation).GetSafeNormal();
	float DistanceToTest = FVector::Distance(OwnerLocation, TestLocation);
	if (DistanceToTest > VisionParameters.VisionDistance)
	{
		return false; // outside of the range
	}
	
	// Check if target is within vision angle and calculate dot product
	FVector OwnerForward = OwnerPawn->GetActorForwardVector();
	float DotProduct = FVector::DotProduct(OwnerForward, DirectionToTest);
	float AngleToTest = FMath::RadiansToDegrees(
		FMath::Acos(FMath::Clamp(DotProduct, -1.0f, 1.0f))
	);
    
	if (AngleToTest > VisionParameters.VisionAngle / 2.0f)
		return false; // not in vision cone

	// Do the line trace
	FHitResult HitResult;
	FCollisionQueryParams CollisionParams;
	CollisionParams.AddIgnoredActor(OwnerPawn);
	bool bHit = GetWorld()->LineTraceSingleByChannel(
		HitResult,
		OwnerLocation,
		TestLocation,
		ECC_Visibility,
		CollisionParams
	);
	
	if (!bHit)
		return true;

	return false;
}


const FTargetData* UGAPerceptionComponent::GetTargetData(FGuid TargetGuid) const
{
	return TargetMap.Find(TargetGuid);
}
