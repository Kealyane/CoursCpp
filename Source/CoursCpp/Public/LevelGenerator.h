// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "LevelGenerator.generated.h"

namespace UE::Geometry
{
	struct FIndex3i;
}

class AFloor;
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
	UPROPERTY(EditAnywhere)
	TSubclassOf<AFloor> RoomType;
	UPROPERTY(EditAnywhere)
	TSubclassOf<AFloor> CorridorType;

	const int32 CELL_SIZE = 128;

	// Algorithm
	
	/**
	 * use delaunay to generate triangles from points
	 * @param Points 
	 * @return a graph
	 */
	FGraphPath GenerateTriangles(TArray<FVector2d> Points);
	
	FGraphPath GenerateVoronoi(TArray<FVector2d> Points);
	
	/**
	 * Prim's Algorithm
	 * @param pGraph the delaunay graph
	 * @return List of shorter edges 
	 */
	TArray<FPriorityEdge> PrimAlgo(FGraphPath pGraph);

	/**
	 * Minimum Spanning Tree
	 * @param pGraph the delaunay graph
	 * @return MST graph
	 */
	FGraphPath CreateMSTUsingPrim(FGraphPath pGraph);

	// Graphic Representation

	TSet<FVector2d> VisitedCells;

	/**
	 * Bresenham Algorithm
	 * @param StartNodePos 
	 * @param EndNodePos 
	 * @return 
	 */
	TArray<FVector2d> GetAllCoordsOfEdge(FVector2d StartNodePos, FVector2d EndNodePos);
	
	/**
	 * Create Actors for floor following MST graph
	 * @param pGraph 
	 */
	void GenerateLevelFromMST(FGraphPath pGraph);

	/**
	 * Create Room Actors for cells around NodePos
	 * @param NodePos 
	 * @param CellSize 
	 */
	void AddTilesAroundNode(FVector2d NodePos, int32 CellSize);

	/**
	 * Spawn the player in a random node of the mst graph
	 * @param pGraph 
	 */
	void SpawnPlayer(FGraphPath pGraph);

	// Debug
	
	void DebugPoints(TArray<FVector2d> pPoints);
	void DebugGraph(FGraphPath pGraph);
	void DebugMSTGraph(FGraphPath pGraph);

};
