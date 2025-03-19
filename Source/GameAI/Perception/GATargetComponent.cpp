#include "GATargetComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GAPerceptionSystem.h"
#include "ProceduralMeshComponent.h"



UGATargetComponent::UGATargetComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;

	SetTickGroup(ETickingGroup::TG_PostUpdateWork);

	// Generate a new guid
	TargetGuid = FGuid::NewGuid();
}


AGAGridActor* UGATargetComponent::GetGridActor() const
{
	AGAGridActor* Result = GridActor.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
		if (GenericResult)
		{
			Result = Cast<AGAGridActor>(GenericResult);
			if (Result)
			{
				// Cache the result
				// Note, GridActor is marked as mutable in the header, which is why this is allowed in a const method
				GridActor = Result;
			}
		}

		return Result;
	}
}


void UGATargetComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterTargetComponent(this);
	}

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		OccupancyMap = FGAGridMap(Grid, 0.0f);
	}
}

void UGATargetComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterTargetComponent(this);
	}
}



void UGATargetComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	bool isImmediate = false;

	// update my perception state FSM
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGAPerceptionComponent>> &PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
		for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
		{
			const FTargetData* TargetData = PerceptionComponent->GetTargetData(TargetGuid);
			if (TargetData && (TargetData->Awareness >= 1.0f))
			{
				isImmediate = true;
				break;
			}
		}
	}

	if (isImmediate)
	{
		AActor* Owner = GetOwner();
		LastKnownState.State = GATS_Immediate;

		// REFRESH MY STATE
		LastKnownState.Set(Owner->GetActorLocation(), Owner->GetVelocity());

		// Tell the omap to clear out and put all the probability in the observed location
		OccupancyMapSetPosition(LastKnownState.Position);
	}
	else if (IsKnown())
	{
		LastKnownState.State = GATS_Hidden;
	}

	if (LastKnownState.State == GATS_Hidden)
	{
		OccupancyMapUpdate();
	}

	// As long as I'm known, whether I'm immediate or not, diffuse the probability in the omap

	if (IsKnown())
	{
		OccupancyMapDiffuse();
	}

	if (bDebugOccupancyMap)
	{
		AGAGridActor* Grid = GetGridActor();
		Grid->DebugGridMap = OccupancyMap;
		GridActor->RefreshDebugTexture();
		GridActor->DebugMeshComponent->SetVisibility(true);
	}
}


void UGATargetComponent::OccupancyMapSetPosition(const FVector& Position)
{
	const AGAGridActor* Grid = GetGridActor();
	if (!Grid)
		return;
    
	// Clear out all probability in the omap (set all cells to 0.0f)
	for (int32 Y = 0; Y < OccupancyMap.YCount; Y++)
	{
		for (int32 X = 0; X < OccupancyMap.XCount; X++)
		{
			FCellRef CellRef(X, Y);
			OccupancyMap.SetValue(CellRef, 0.0f);
		}
	}
    
	// Convert world position to grid cell
	FCellRef PositionCell = Grid->GetCellRef(Position);
    
	// If the cell is valid, set its probability to 1.0
	if (Grid->IsCellRefInBounds(PositionCell))
	{
		OccupancyMap.SetValue(PositionCell, 1.0f);
	}
}


void UGATargetComponent::OccupancyMapUpdate()
{
    const AGAGridActor* Grid = GetGridActor();
    if (!Grid)
        return;
    
    FGAGridMap VisibilityMap(Grid, 0.0f);
	
    UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	
    if (PerceptionSystem)
    {
		//Yoink all the components
    	TArray<TObjectPtr<UGAPerceptionComponent>>& PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
        for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
        {
        	// traverse the visibility map
            for (int32 x = 0; x < VisibilityMap.XCount; x++)
            {
                for (int32 y = 0; y < VisibilityMap.XCount; y++)
                {
                    FCellRef CellRef(x, y);

                	// skip cells that are already visibile
                    float CurrentVisibility = 0.0f;
                    VisibilityMap.GetValue(CellRef, CurrentVisibility);
                    if (CurrentVisibility > 0.0f)
                        continue;
                    
                    // Skip cells that aren't traversable
                    if (!EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable))
                        continue;
                    FVector CellWorldPos = Grid->GetCellPosition(CellRef);
                    
                    //HELPER FUNCTION HERE
                    if (PerceptionComponent->TestVisibility(CellWorldPos))
                    {
                        VisibilityMap.SetValue(CellRef, 1.0f);
                    }
                }
            }
        }
    }
    float totalProbability = 0.0f;
    for (int32 x = 0; x < OccupancyMap.XCount; x++)
    {
        for (int32 y = 0; y < OccupancyMap.YCount; y++)
        {
            FCellRef CellRef(x, y);
            float CellVisibility = 0.0f;
            float CellProbability = 0.0f;
            VisibilityMap.GetValue(CellRef, CellVisibility);
            OccupancyMap.GetValue(CellRef, CellProbability);
            
            // If the cell is visible but we don't see the target there, the probability should be 0
            if (CellVisibility > 0.0f)
            {
                OccupancyMap.SetValue(CellRef, 0.0f);
            }
            else
            {
               // keep track of probability for nonvisible cells
                totalProbability += CellProbability;
            }
        }
    }
    
   //re normalize nonvisible cells
    if (totalProbability > 0.0f)
    {
    	for (int32 x = 0; x < OccupancyMap.XCount; x++)
    	{
    		for (int32 y = 0; y < OccupancyMap.YCount; y++)
    		{
                FCellRef CellRef(x, y);
                float CellVisibility = 0.0f;
                float CellProbability = 0.0f;
                
                VisibilityMap.GetValue(CellRef, CellVisibility);
                if (CellVisibility <= 0.0f)
                {
                    OccupancyMap.GetValue(CellRef, CellProbability);
                    OccupancyMap.SetValue(CellRef, CellProbability / totalProbability);
                }
            }
        }
    }
    
    // Update lastKnownState
    float maxProbability = 0.0f;
    FCellRef maxProbCell = FCellRef::Invalid;
	for (int32 x = 0; x < OccupancyMap.XCount; x++)
	{
		for (int32 y = 0; y < OccupancyMap.YCount; y++)
		{
            FCellRef CellRef(x, y);
            float CellProbability = 0.0f;
            OccupancyMap.GetValue(CellRef, CellProbability);
            
            if (CellProbability > maxProbability)
            {
                maxProbability = CellProbability;
                maxProbCell = CellRef;
            }
        }
    }
	
    if (maxProbCell.IsValid())
    {
        FVector BestGuessPosition = Grid->GetCellPosition(maxProbCell);
        LastKnownState.Position = BestGuessPosition;
    }
}


void UGATargetComponent::OccupancyMapDiffuse()
{
    const AGAGridActor* Grid = GetGridActor();
    if (!Grid)
        return;
    
   // creating a copy
    FGAGridMap OriginalMap = OccupancyMap;
	
    const float ProbabilitySpread = 0.1f;
    
    // For each cell in the grid
    for (int32 Y = 0; Y < OccupancyMap.YCount; Y++)
    {
        for (int32 X = 0; X < OccupancyMap.XCount; X++)
        {
            FCellRef CellRef(X, Y);
        	
            if (!EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable))
                continue; // not traversable
                
            float CurrentValue = 0.0f;
            OriginalMap.GetValue(CellRef, CurrentValue);
        	
            if (CurrentValue <= 0.0f) // no probability
                continue;

        	// how much to diffuse to neighbors:
            float AmountToDiffuse = CurrentValue * ProbabilitySpread;
            float NewCellValue = CurrentValue - AmountToDiffuse;
            OccupancyMap.SetValue(CellRef, NewCellValue);
            
            // Get valid and traversable neighbors
            TArray<FCellRef> Neighbors;
            Grid->GetNeighbors(CellRef, true, Neighbors);
            
            if (Neighbors.Num() > 0)
            {
                float AmountPerNeighbor = AmountToDiffuse / Neighbors.Num();
                for (const FCellRef& Neighbor : Neighbors)
                {
                    float NeighborValue = 0.0f;
                    OccupancyMap.GetValue(Neighbor, NeighborValue);
                    OccupancyMap.SetValue(Neighbor, NeighborValue + AmountPerNeighbor);
                }
            }
            else
            { // keep probability in cell if no neighbors
                OccupancyMap.SetValue(CellRef, CurrentValue);
            }
        }
    }
}
