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

	/**
	 * use voronoi to generate cells from points
	 * @param Points 
	 * @return 
	 */
	FGraphPath GenerateVoronoi(TArray<FVector2d> Points);
	
	/**
	 * Prim's Algorithm
	 * @param Graph the delaunay graph
	 * @return List of shorter edges 
	 */
	TArray<FPriorityEdge> PrimAlgo(FGraphPath Graph);

	/**
	 * Minimum Spanning Tree
	 * @param Graph the delaunay graph
	 * @return MST graph
	 */
	FGraphPath CreateMSTUsingPrim(FGraphPath Graph);

	// Graphic Representation

	TSet<FVector2d> VisitedCells;

	/**
	 * Bresenham Algorithm
	 * @param StartNodePos 
	 * @param EndNodePos 
	 * @return 
	 */
	TArray<FVector2d> GetAllCoordsOfEdgeDelaunay(FVector2d StartNodePos, FVector2d EndNodePos);
	TArray<FVector2d> GetAllCoordsOfEdgeVoronoi(TTuple<int32, int32> const& StartNodePos, TTuple<int32, int32> const& EndNodePos);
	
	/**
	 * Create Actors for floor following MST graph
	 * @param Graph 
	 */
	void GenerateLevelFromMSTDelaunay(FGraphPath Graph);
	void GenerateLevelFromMSTVoronoi(FGraphPath Graph);

	/**
	 * Create Room Actors for cells around NodePos
	 * @param NodePos 
	 * @param CellSize 
	 */
	void AddTilesAroundNode(FVector2d NodePos, int32 CellSize);

	/**
	 * Spawn the player in a random node of the mst graph
	 * @param Graph 
	 */
	void SpawnPlayer(FGraphPath Graph);
	
	void SpawnPlayerZero();

	// Debug
	
	void DebugPoints(TArray<FVector2d> Points);
	void DebugGraph(FGraphPath Graph, FColor ColorNode, FColor ColorEdge);

};
