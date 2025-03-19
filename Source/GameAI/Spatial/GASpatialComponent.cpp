#include "GASpatialComponent.h"
#include "GameAI/Pathfinding/GAPathComponent.h"
#include "GameAI/Grid/GAGridMap.h"
#include "Kismet/GameplayStatics.h"
#include "Math/MathFwd.h"
#include "GASpatialFunction.h"
#include "ProceduralMeshComponent.h"
#include "GameAI/Perception/GAPerceptionComponent.h"

UE_DISABLE_OPTIMIZATION

UGASpatialComponent::UGASpatialComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	SampleDimensions = 8000.0f;		// should cover the bulk of the test map
}


const AGAGridActor* UGASpatialComponent::GetGridActor() const
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

UGAPathComponent* UGASpatialComponent::GetPathComponent() const
{
	UGAPathComponent* Result = PathComponent.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* Owner = GetOwner();
		if (Owner)
		{
			// Note, the UGAPathComponent and the UGASpatialComponent are both on the controller
			Result = Owner->GetComponentByClass<UGAPathComponent>();
			if (Result)
			{
				PathComponent = Result;
			}
		}
		return Result;
	}
}

APawn* UGASpatialComponent::GetOwnerPawn() const
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


bool UGASpatialComponent::ChoosePosition(bool PathfindToPosition, bool Debug)
{
	bool Result = false;
	const APawn* OwnerPawn = GetOwnerPawn();
	if (OwnerPawn == NULL)
	{
		return false;
	}

	const AGAGridActor* Grid = GetGridActor();
	UGAPathComponent* PathComponentPtr = GetPathComponent();

	FCellRef LastCell = BestCell;
	BestCell = FCellRef::Invalid;

	if (Grid == NULL)
	{
		return false;
	}

	if (PathComponentPtr == NULL)
	{
		return false;
	}

	if (SpatialFunctionReference.Get() == NULL)
	{
		UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent has no SpatialFunctionReference assigned."));
		return false;
	}

	if (GridActor == NULL)
	{
		UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent::ChoosePosition can't find a GridActor."));
		return false;
	}

	// Don't worry too much about the Unreal-ism below. Technically our SpatialFunctionReference is not ACTUALLY
	// a spatial function instance, rather it's a class, which happens to have a lot of data in it.
	// Happily, Unreal creates, under the hood, a default object for every class, that lets you access that data
	// as if it were a normal instance
	const UGASpatialFunction* SpatialFunction = SpatialFunctionReference->GetDefaultObject<UGASpatialFunction>();

	// The below is to create a GridMap (which you will fill in) based on a bounding box centered around the OwnerPawn

	FBox2D Box(EForceInit::ForceInit);
	FIntRect CellRect;
	FVector StartLocation = OwnerPawn->GetActorLocation();
	FVector2D PawnLocation(StartLocation);
	Box += PawnLocation;
	Box = Box.ExpandBy(SampleDimensions / 2.0f);
	if (GridActor->GridSpaceBoundsToRect2D(Box, CellRect))
	{
		// Super annoying, by the way, that FIntRect is not blueprint accessible, because it forces us instead
		// to make a separate bp-accessible FStruct that represents _exactly the same thing_.
		FGridBox GridBox(CellRect);

		// This is the grid map I'm going to fill with values
		FGAGridMap GridMap(Grid, GridBox, 0.0f);

		// Fill in this distance map using Dijkstra!
		FGAGridMap DistanceMap(Grid, GridBox, FLT_MAX);


		// ~~~ STEPS TO FILL IN FOR ASSIGNMENT 3 ~~~


		// Step 1: Run Dijkstra's to determine which cells we should even be evaluating (the GATHER phase)
		// (You should add a Dijkstra() function to the UGAPathComponent())
		// I would recommend adding a method to the path component which looks something like
		PathComponentPtr->Dijkstra(StartLocation, DistanceMap);

		// Give the last best cell a bonus
		GridMap.SetValue(LastCell, SpatialFunction->LastCellBonus);

		// Step 2: For each layer in the spatial function, evaluate and accumulate the layer in GridMap
		// Note, only evaluate accessible cells found in step 1
		for (const FFunctionLayer& Layer : SpatialFunction->Layers)
		{
			// figure out how to evaluate each layer type, and accumulate the value in the GridMap
			EvaluateLayer(Layer, DistanceMap, GridMap);
		}

		// Step 3: pick the best cell in GridMap

		{
			float BestScore = -FLT_MAX;

			for (int32 Y = GridMap.GridBounds.MinY; Y <= GridMap.GridBounds.MaxY; Y++)
			{
				for (int32 X = GridMap.GridBounds.MinX; X <= GridMap.GridBounds.MaxX; X++)
				{
					FCellRef CellRef(X, Y);
					float D;

					DistanceMap.GetValue(CellRef, D);

					if (D < FLT_MAX)
					{
						float V;

						GridMap.GetValue(CellRef, V);

						if (V > BestScore)
						{
							BestScore = V;
							BestCell = CellRef;
							Result = true;
						}
					}
				}
			}
		}

		if (PathfindToPosition)
		{
			if (BestCell.IsValid())
			{
				// Step 4: Go there!
				// This will involve reconstructing the path and then getting it into the UGAPathComponent
				// Depending on what your cached Dijkstra data looks like, the path reconstruction might be implemented here
				// or in the UGAPathComponent

				PathComponentPtr->BuidPathFromDistanceMap(StartLocation, BestCell, DistanceMap);
			}
			else
			{
				PathComponentPtr->ClearPath();
			}
		}

		
		if (Debug)
		{
			// Note: this outputs (basically) the results of the position selection
			// However, you can get creative with the debugging here. For example, maybe you want
			// to be able to examine the values of a specific layer in the spatial function
			// You could create a separate debug map above (where you're doing the evaluations) and
			// cache it off for debug rendering. Ideally you'd be able to control what layer you wanted to 
			// see from blueprint
			
			GridActor->DebugGridMap = GridMap;
			GridActor->RefreshDebugTexture();
			GridActor->DebugMeshComponent->SetVisibility(true);		//cheeky!
		}
	}

	return Result;
}


void UGASpatialComponent::EvaluateLayer(const FFunctionLayer& Layer, const FGAGridMap &DistanceMap, FGAGridMap &GridMap) const
{
	UWorld* World = GetWorld();
	AActor* OwnerPawn = GetOwnerPawn();
	const AGAGridActor* Grid = GetGridActor();
	// Get the AI controller
	AController* AIController = Cast<AController>(GetOwner());
	if (!AIController)
		return;
    
	// Get the perception component
	UGAPerceptionComponent* PerceptionComponent = AIController->GetComponentByClass<UGAPerceptionComponent>();
	if (!PerceptionComponent)
		return;
    
	// Get target information from perception component
	FVector TargetPosition;
	FVector TargetVelocity;
	bool bHasTarget = false;
    
	FTargetCache TargetCache;
	FTargetData TargetData;
	if (PerceptionComponent->GetCurrentTargetState(TargetCache, TargetData))
	{
		TargetPosition = TargetCache.Position;
		TargetVelocity = TargetCache.Velocity;
		bHasTarget = true;
	}
	else
	{
		// If no target is known, default to using the player position
		// This is a fallback that you'll eventually want to remove
		APawn *PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);
		if (PlayerPawn)
		{
			TargetPosition = PlayerPawn->GetActorLocation();
		}
	}
    
	FVector Offset(0.0f, 0.0f, 60.0f);;

	TArray<FVector> AllyPositions;
	TArray<float> AllyDistances;

	if (Layer.Input == SI_AllyDistance)
	{
		TArray<AActor *> Actors;
		UGameplayStatics::GetAllActorsOfClass(World, APawn::StaticClass(), Actors);

		for (AActor* Actor : Actors)
		{
			if (Actor == OwnerPawn)
			{
				continue;
			}

			APawn* Pawn = Cast<APawn>(Actor);
			if (Pawn)
			{
				AController* Controller = Pawn->GetController();
				if (Controller)
				{
					UGAPathComponent *OtherPathComponent = Controller->GetComponentByClass<UGAPathComponent>();
					if (OtherPathComponent)
					{
						FVector Position;
						float D;

						// Keep track of where our allies are -- but note that if they are headed towards a
						// destination (according to their path component) we use THAT as the ally position, 
						// rather than their current position.
						// Note, we also keep track of their distance to that destination.

						if (OtherPathComponent->State == GAPS_Active)
						{
							Position = OtherPathComponent->Destination;
							D = OtherPathComponent->GetPathLength();
						}
						else
						{
							Position = Pawn->GetActorLocation();
							D = 0.0f;
						}
						AllyPositions.Add(Position);
						AllyDistances.Add(D);
					}
				}
			}
		}
	}


	for (int32 Y = GridMap.GridBounds.MinY; Y <= GridMap.GridBounds.MaxY; Y++)
	{
		for (int32 X = GridMap.GridBounds.MinX; X <= GridMap.GridBounds.MaxX; X++)
		{
			FCellRef CellRef(X, Y);

			if (EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable))
			{
				float CellDistance;
				if (DistanceMap.GetValue(CellRef, CellDistance) &&
					(CellDistance < FLT_MAX))
				{
					// evaluate me!

					float Value = 0.0f;

					switch (Layer.Input)
					{
					case SI_None:
						break;
					case SI_TargetRange:
					{
						FVector CellPosition = Grid->GetCellPosition(CellRef);
						Value = FVector::Distance(CellPosition, TargetPosition);
					}
					break;
					case SI_PathDistance:
						Value = CellDistance;
						break;
					case SI_LOS:
					{
						FVector CellPosition = Grid->GetCellPosition(CellRef) + Offset;
						FHitResult HitResult;
						FCollisionQueryParams Params;
						FVector Start = CellPosition;
						FVector End = TargetPosition;
						Params.AddIgnoredActor(OwnerPawn);  
						// Get the target actor if known
						if (PerceptionComponent->HasTarget())
						{
							UGATargetComponent* TargetComponent = PerceptionComponent->GetCurrentTarget();
							if (TargetComponent && TargetComponent->GetOwner())
							{
								Params.AddIgnoredActor(TargetComponent->GetOwner());
							}
						
						bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
						Value = bHitSomething ? 0.0f : 1.0f;
						}
						else
						{
							Value = 0.0f; // No LOS if no target
						}
						break;
					}
					case SI_AllyDistance:
					{
						FVector CellPosition = Grid->GetCellPosition(CellRef);
						float MinDistanceToAlly = BIG_NUMBER;
						int32 NumAllies = AllyPositions.Num();

						// find the closest ally to this point
						// HOWEVER ... if we are (path) closer to this cell than THEY are to THEIR destination
						// we are allowed to disregard them, since we would get their first, and they can deal 
						// with us instead.

						for (int32 AllyIndex = 0; AllyIndex < NumAllies; AllyIndex++)
						{
							if (AllyDistances[AllyIndex] < CellDistance)
							{
								float D = FVector::Distance(CellPosition, AllyPositions[AllyIndex]);
								if (D < MinDistanceToAlly)
								{
									MinDistanceToAlly = D;
								}
							}
						}
						Value = MinDistanceToAlly;
						break;
					}
					};


					{
						// Next, run it through the response curve using something like this
						float ModifiedValue = Layer.ResponseCurve.GetRichCurveConst()->Eval(Value, Value);
						float CurrentValue = 0.0f;
						float ResultValue = 0.0f;

						GridMap.GetValue(CellRef, CurrentValue);

						switch (Layer.Op)
						{
						case SO_None:
							ResultValue = CurrentValue;
							break;
						case SO_Add:
							ResultValue = CurrentValue + ModifiedValue;
							break;
						case SO_Multiply:
							ResultValue = CurrentValue * ModifiedValue;
							break;
						}

						GridMap.SetValue(CellRef, ResultValue);
					}


					// HERE ARE SOME ADDITIONAL HINTS

					// Here's how to get the player's pawn

					// Here's how to cast a ray

					// UWorld* World = GetWorld();
					// FHitResult HitResult;
					// FCollisionQueryParams Params;
					// FVector Start = Grid->GetCellPosition(CellRef);		// need a ray start
					// FVector End = PlayerPawn->GetActorLocation();		// need a ray end
					// Start.Z = End.Z;		// Hack: we don't have Z information in the grid actor -- take the player's z value and raycast against that
					// Add any actors that should be ignored by the raycast by calling
					// Params.AddIgnoredActor(PlayerPawn);			// Probably want to ignore the player pawn
					// Params.AddIgnoredActor(OwnerPawn);			// Probably want to ignore the AI themself
					// bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
					// If bHitSomething is false, then we have a clear LOS
				}
			}
		}
	}
}

UE_ENABLE_OPTIMIZATION
