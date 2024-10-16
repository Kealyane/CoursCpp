// Fill out your copyright notice in the Description page of Project Settings.


#include "LevelGenerator.h"
#include "PointGenerator.h"
#include "GraphPath.h"
#include "CompGeom/Delaunay2.h"
#include "Floor.h"

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
	TArray<FVector2d> Points = PointGenerator::GeneratePoints(20, 0, 50);
	FGraphPath MyGraph = GenerateTriangles(Points);
	DebugPoints(Points);
	DebugGraph(MyGraph);
	FGraphPath MSTGraph = CreateMSTUsingPrim(MyGraph);
	DebugMSTGraph(MSTGraph);
	GenerateLevelFromMST(MSTGraph);
}

// ALGORITHM

FGraphPath ALevelGenerator::GenerateTriangles(TArray<FVector2d> Points)
{
	UE::Geometry::FDelaunay2 DelaunayTriangulation;
	
	bool bSuccess;

	do
	{
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

TArray<FPriorityEdge> ALevelGenerator::PrimAlgo(FGraphPath pGraph)
{
	TSet<int32> VisitedNodes;
	TArray<FPriorityEdge> StoreMSTEdges;
	
	TArray<FPriorityEdge> EdgesPriorityQueue;

	const int32 StartNode = pGraph.Nodes[0].Node;
	VisitedNodes.Add(StartNode);

	// sort all edges of start node by min weight
	const FNode StartGraphNode = pGraph.GetNode(StartNode);
	for (const FGraphEdge& Edge : StartGraphNode.Edges)
	{
		EdgesPriorityQueue.HeapPush(FPriorityEdge(StartNode, Edge.TargetNode, Edge.Weight));
	}
	
	while (VisitedNodes.Num() < pGraph.Nodes.Num())
	{
		FPriorityEdge CurrentEdge;
		
		// found a node that hasn't been visited
		while (EdgesPriorityQueue.Num() > 0)
		{
			EdgesPriorityQueue.HeapPop(CurrentEdge);
			if (!VisitedNodes.Contains(CurrentEdge.EndNode)) break; 
		}
		VisitedNodes.Add(CurrentEdge.EndNode);
		StoreMSTEdges.Add(CurrentEdge);

		// next node
		const FNode CurrentGraphNode = pGraph.GetNode(CurrentEdge.EndNode);
		for (const FGraphEdge& Edge : CurrentGraphNode.Edges)
		{
			if (!VisitedNodes.Contains(Edge.TargetNode))
			{
				EdgesPriorityQueue.HeapPush(FPriorityEdge(CurrentEdge.EndNode, Edge.TargetNode, Edge.Weight));
			}
		}
	}
	return StoreMSTEdges;
}

FGraphPath ALevelGenerator::CreateMSTUsingPrim(FGraphPath pGraph)
{
	TArray<FPriorityEdge> StoredMSTEdges = PrimAlgo(pGraph);
	FGraphPath MSTPath = FGraphPath();

	for (const FPriorityEdge Edge : StoredMSTEdges)
	{
		const FVector2d StartNodePos = pGraph.GetNode(Edge.StartNode).Position;
		const FVector2d EndNodePos = pGraph.GetNode(Edge.EndNode).Position;

		// Process StartNode
		if (!MSTPath.IsNodeInGraph(Edge.StartNode))
		{
			MSTPath.AddNode(Edge.StartNode, StartNodePos, Edge.EndNode, EndNodePos, Edge.Weight);
		}
		else
		{
			MSTPath.GetNode(Edge.StartNode).AddEdge(Edge.EndNode, EndNodePos, Edge.Weight);
		}

		// Process EndNode
		if (!MSTPath.IsNodeInGraph(Edge.EndNode))
		{
			MSTPath.AddNode(Edge.EndNode, EndNodePos, Edge.StartNode, StartNodePos, Edge.Weight);
		}
		else
		{
			MSTPath.GetNode(Edge.EndNode).AddEdge(Edge.StartNode, StartNodePos, Edge.Weight);
		}
	}
	
	return MSTPath;
}

// GRAPHIC REPRESENTATION

TArray<FVector2d> ALevelGenerator::GetAllCoordsOfEdge(FVector2d StartNodePos, FVector2d EndNodePos)
{
	TArray<FVector2d> EdgePoints;

	const int DistanceX = FMath::Abs(EndNodePos.X - StartNodePos.X);
	const int DistanceY = FMath::Abs(EndNodePos.Y - StartNodePos.Y);

	const int DirectionX = (StartNodePos.X < EndNodePos.X) ? 1 : -1;
	const int DirectionY = (StartNodePos.Y < EndNodePos.Y) ? 1 : -1;

	int Err = DistanceX - DistanceY;

	int CurrentX = StartNodePos.X;
	int CurrentY = StartNodePos.Y;

	while (true)
	{
		if (CurrentX == EndNodePos.X && CurrentY == EndNodePos.Y) break;
		const int Err2 = 2 * Err;
		
		if (Err2 > -DistanceY)
		{
			Err -= DistanceY;
			CurrentX += DirectionX;
		}
		if (Err2 < DistanceX)
		{
			Err += DistanceX;
			CurrentY += DirectionY;
		}
		EdgePoints.Add(FVector2D(CurrentX, CurrentY));
	} 
	
	return EdgePoints;
}

void ALevelGenerator::GenerateLevelFromMST(FGraphPath pGraph)
{
	int32 CellSize = 128;
	if (UWorld* World = GetWorld())
	{
		for (const TPair<int32, FNode>& NodePair : pGraph.Nodes)
		{
			const FNode CurrentNode = NodePair.Value;
			if (!VisitedCells.Contains(CurrentNode.Position))
			{
				VisitedCells.Add(CurrentNode.Position);
				FVector RoomPosition = FVector(CurrentNode.Position.X * CellSize, CurrentNode.Position.Y * CellSize, 0.0f); 
				AFloor* Room = World->SpawnActor<AFloor>(RoomType, RoomPosition, FRotator::ZeroRotator);
				AddTilesAroundNode(CurrentNode.Position, CellSize);
			}
			
			for (const FGraphEdge& Edge : CurrentNode.Edges)
			{
				FVector2D StartPos = CurrentNode.Position;
				FVector2D EndPos = Edge.TargetNodePos;
				
				TArray<FVector2D> EdgePoints = GetAllCoordsOfEdge(StartPos, EndPos);
				
				for (const FVector2D& Point : EdgePoints)
				{
					if (Point == StartPos || Point == EndPos) continue;
					if (!VisitedCells.Contains(Point))
					{
						VisitedCells.Add(Point);
						FVector CorridorPosition = FVector(Point.X*CellSize, Point.Y*CellSize, 0.0f);
						AFloor* Corridor = World->SpawnActor<AFloor>(CorridorType, CorridorPosition, FRotator::ZeroRotator);

						FVector2d Offset = FVector2d(-1, 0);
						FVector2d PerpendicularPos = FVector2d(Point.X + Offset.X, Point.Y + Offset.Y);
						if (!VisitedCells.Contains(PerpendicularPos))
						{
							VisitedCells.Add(PerpendicularPos);
							FVector CorridorPosition2 = FVector(PerpendicularPos.X*CellSize, PerpendicularPos.Y*CellSize, 0.0f);
							AFloor* Corridor2 = World->SpawnActor<AFloor>(CorridorType, CorridorPosition2, FRotator::ZeroRotator);
						}
					}
				}
			}
		}
		
	}
}

void ALevelGenerator::AddTilesAroundNode(FVector2d NodePos, int32 CellSize)
{
	TArray<FVector2D> Offsets = {
		FVector2D(-1, -1), FVector2D(0, -1), FVector2D(1, -1),
		FVector2D(-1, 0),  FVector2D(1, 0),
		FVector2D(-1, 1), FVector2D(0, 1), FVector2D(1, 1)
	};
	
	if (UWorld* World = GetWorld())
	{
		for (FVector2D Offset : Offsets)
		{
			FVector2D NewPos = NodePos + Offset;
			if (VisitedCells.Contains(NewPos)) continue;
			VisitedCells.Add(NewPos);
			FVector TilePosition = FVector(NewPos.X * CellSize, NewPos.Y * CellSize, 0.0f);
			World->SpawnActor<AFloor>(RoomType, TilePosition, FRotator::ZeroRotator);
		}
	}
}

// DEBUG

void ALevelGenerator::DebugPoints(TArray<FVector2d> pPoints)
{
	if (const UWorld* World = GetWorld())
	{
		for (const FVector2d& Point : pPoints)
		{
			DrawDebugSphere(World, FVector(Point, 10), 2.f, 10.f, FColor::Magenta, true);
		}
	}
}

void ALevelGenerator::DebugGraph(FGraphPath pGraph)
{
	const UWorld* World = GetWorld();
	if (World == nullptr) return;
	
	for(TTuple<int32,FNode>& Elt : pGraph.Nodes)
	{
		FNode Node = Elt.Value;
		DrawDebugSphere(World, FVector(Node.Position, 10), 2.f, 10.f, FColor::Cyan, true);
		
		for (FGraphEdge& Edge : Node.Edges)
		{
			DrawDebugLine(World, FVector(Node.Position,10), FVector(Edge.TargetNodePos,10), FColor::Blue, true);
		}
	}
}

void ALevelGenerator::DebugMSTGraph(FGraphPath pGraph)
{
	const UWorld* World = GetWorld();
	if (World == nullptr) return;
	
	for(TTuple<int32,FNode>& Elt : pGraph.Nodes)
	{
		FNode Node = Elt.Value;
		DrawDebugSphere(World, FVector(Node.Position, 10), 2.f, 10.f, FColor::Yellow, true);
		
		for (FGraphEdge& Edge : Node.Edges)
		{
			DrawDebugLine(World, FVector(Node.Position,10), FVector(Edge.TargetNodePos,10), FColor::Orange, true);
		}
	}
}
