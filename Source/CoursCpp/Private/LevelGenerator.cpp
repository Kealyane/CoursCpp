// Fill out your copyright notice in the Description page of Project Settings.


#include "LevelGenerator.h"
#include "PointGenerator.h"
#include "GraphPath.h"
#include "CompGeom/Delaunay2.h"

// Sets default values
ALevelGenerator::ALevelGenerator()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
}

// Called when the game starts or when spawned
void ALevelGenerator::BeginPlay()
{
	Super::BeginPlay();
	FGraphPath MyGraph = GenerateTriangles();
	DebugPoints(Points);
	DebugGraph(MyGraph);
}

FGraphPath ALevelGenerator::GenerateTriangles()
{
	UE::Geometry::FDelaunay2 DelaunayTriangulation;
	
	Points = PointGenerator::GeneratePoints(50, -100, 100);
	bool bSuccess;
	int32 counter = 0;
	do
	{
		UE_LOG(LogTemp, Warning, TEXT("generate delaunay : %d"), counter);
		bSuccess = DelaunayTriangulation.Triangulate(Points);
		
	} while (!bSuccess && !DelaunayTriangulation.IsDelaunay(Points));
	
	TArray<UE::Geometry::FIndex3i> Triangles = DelaunayTriangulation.GetTriangles();

	FGraphPath lGraph = FGraphPath();

	for (const auto& Triangle : Triangles)
	{
		int32 Vertice1 = Triangle.A;
		int32 Vertice2 = Triangle.B;
		int32 Vertice3 = Triangle.C;

		FVector2d Pos1 = Points[Vertice1];
		FVector2d Pos2 = Points[Vertice2];
		FVector2d Pos3 = Points[Vertice3];

		float Weight1 = FVector2d::Distance(Pos1, Pos2);
		float Weight2 = FVector2d::Distance(Pos2, Pos3);
		float Weight3 = FVector2d::Distance(Pos1, Pos3);

		lGraph.AddNode(Vertice1, Pos1, Vertice2, Pos2, Weight1, Vertice3, Pos3, Weight3);
		lGraph.AddNode(Vertice2, Pos2, Vertice1, Pos1, Weight1, Vertice3, Pos3, Weight2);
		lGraph.AddNode(Vertice3, Pos3, Vertice1, Pos1, Weight3, Vertice2, Pos2, Weight2);		
	}
	
	return lGraph;
}

void ALevelGenerator::DebugPoints(TArray<FVector2d> pPoints)
{
	if (const UWorld* World = GetWorld())
	{
		for (const FVector2d& Point : pPoints)
		{
			DrawDebugSphere(World, FVector(Point, 10), 5.f, 10.f, FColor::Magenta, true);
		}
	}
}

void ALevelGenerator::DebugGraph(FGraphPath pGraph)
{
	const UWorld* World = GetWorld();
	if (World == nullptr) return;
	
	for(FNode& Node : pGraph.Nodes)
	{
		DrawDebugSphere(World, FVector(Node.Position, 20), 2.f, 10.f, FColor::Cyan, true);
		
		for (FGraphEdge& Edge : Node.Edges)
		{
			DrawDebugLine(World, FVector(Node.Position,20), FVector(Edge.TargetNodePos,20), FColor::Blue, true);
		}
	}
}
