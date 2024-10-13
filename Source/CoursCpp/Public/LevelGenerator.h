// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "LevelGenerator.generated.h"

struct FPriorityEdge;
struct FGraphPath;

UCLASS()
class COURSCPP_API ALevelGenerator : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ALevelGenerator();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	TArray<FVector2d> Points;

	FGraphPath GenerateTriangles();
	TArray<FPriorityEdge> GenerateMST(FGraphPath pGraph);
	FGraphPath CreateGraphFromMST(FGraphPath pGraph);
	
	void DebugPoints(TArray<FVector2d> pPoints);
	void DebugGraph(FGraphPath pGraph);
	void DebugMSTGraph(FGraphPath pGraph);

};
