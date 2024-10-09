// Fill out your copyright notice in the Description page of Project Settings.


#include "Floor.h"

#include "PointGenerator.h"

// Sets default values
AFloor::AFloor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
}

// Called when the game starts or when spawned
void AFloor::BeginPlay()
{
	Super::BeginPlay();
	DebugPoints();
}

void AFloor::DebugPoints()
{
	TArray<FVector2d> Points = PointGenerator::GeneratePoints(50, -100, 100);
	if (const UWorld* World = GetWorld())
	{
		for (const FVector2d& Point : Points)
		{
			DrawDebugSphere(World, FVector(Point, 10), 5.f, 10.f, FColor::Magenta, true);
		}
	}
}
