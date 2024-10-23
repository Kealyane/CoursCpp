// Fill out your copyright notice in the Description page of Project Settings.


#include "LevelGenerator.h"
#include "PointGenerator.h"
#include "GraphPath.h"
#include "CompGeom/Delaunay2.h"
#include "Floor.h"
#include "Player/MyPawn.h"

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

	TArray<FIntPoint> Points = PointGenerator::GeneratePoints(20, 0, 100);

	FGraphPath MyGraph = GenerateTriangles(Points);
	UE_LOG(LogTemp, Warning, TEXT("Delaunay :"));
	MyGraph.DebugGraph();
	DebugPoints(Points);
	DebugGraph(MyGraph, FColor::Cyan, FColor::Blue);

	FGraphPath VoronoiGraph = GenerateVoronoi(Points);
	DebugGraph(VoronoiGraph, FColor::Cyan, FColor::Blue);
	UE_LOG(LogTemp, Warning, TEXT("Voronoi :"));
	VoronoiGraph.DebugGraph();
	
	FGraphPath MSTGraphVoronoi = CreateMSTUsingPrim(VoronoiGraph);
	UE_LOG(LogTemp, Warning, TEXT("MST :"));
	MSTGraphVoronoi.DebugGraph();
	DebugGraph(MSTGraphVoronoi, FColor::Yellow, FColor::Orange);

	// MST From Delaunay
	
	// FGraphPath MSTGraphDelaunay = CreateMSTUsingPrim(MyGraph);
	// UE_LOG(LogTemp, Warning, TEXT("MST :"));
	// MSTGraphDelaunay.DebugGraph();
	// DebugGraph(MSTGraphDelaunay, FColor::Yellow, FColor::Orange);
	
	GenerateLevelFromMST(MSTGraphVoronoi);
	SpawnPlayer(MSTGraphVoronoi);
	//SpawnPlayerZero();
}

// ALGORITHM

FGraphPath ALevelGenerator::GenerateTriangles(TArray<FIntPoint> Points)
{
	UE::Geometry::FDelaunay2 DelaunayTriangulation;
	
	bool bSuccess;
	TArray<FVector2d> PointsVector = PointGenerator::ConvertPointsToVector2d(Points);
	do
	{
		bSuccess = DelaunayTriangulation.Triangulate(PointsVector);
		
	} while (!bSuccess && !DelaunayTriangulation.IsDelaunay(PointsVector));
	
	TArray<UE::Geometry::FIndex3i> Triangles = DelaunayTriangulation.GetTriangles();

	FGraphPath ResultGraph = FGraphPath();

	for (const auto& Triangle : Triangles)
	{
		int32 Vertice1 = Triangle.A;
		int32 Vertice2 = Triangle.B;
		int32 Vertice3 = Triangle.C;

		FIntPoint Pos1 = Points[Vertice1];
		FIntPoint Pos2 = Points[Vertice2];
		FIntPoint Pos3 = Points[Vertice3];

		float Weight1 = FVector2d::Distance(Pos1, Pos2);
		float Weight2 = FVector2d::Distance(Pos2, Pos3);
		float Weight3 = FVector2d::Distance(Pos1, Pos3);

		ResultGraph.AddNode(Vertice1, Pos1, Vertice2, Pos2, Weight1, Vertice3, Pos3, Weight3);
		ResultGraph.AddNode(Vertice2, Pos2, Vertice1, Pos1, Weight1, Vertice3, Pos3, Weight2);
		ResultGraph.AddNode(Vertice3, Pos3, Vertice1, Pos1, Weight3, Vertice2, Pos2, Weight2);		
	}
	
	return ResultGraph;
}

FGraphPath ALevelGenerator::GenerateVoronoi(TArray<FIntPoint> Points)
{
	UE::Geometry::FDelaunay2 DelaunayTriangulation;

	bool bSuccess;
	TArray<FVector2d> PointsVector = PointGenerator::ConvertPointsToVector2d(Points);

	do
	{
		bSuccess = DelaunayTriangulation.Triangulate(PointsVector);
		
	} while (!bSuccess && !DelaunayTriangulation.IsDelaunay(PointsVector));
	
	TArray<TArray<FVector2d>> Voronoi = DelaunayTriangulation.GetVoronoiCells(PointsVector);

	// DEBUG POINTS
	for (auto Cell : Voronoi)
	{
		for (auto Vect2 : Cell)
		{
			DrawDebugSphere(GetWorld(), FVector(Vect2, 10), 2.f, 10.f, FColor::Red, true);
		}
	}
	
	FGraphPath ResultGraph = FGraphPath();
	int32 Count = 0;
	for (int CellIndex = 0; CellIndex < Voronoi.Num(); CellIndex++)
	{
		TArray<FVector2d> VoronoiCell = Voronoi[CellIndex];
		
		for (int PointIndex = 0; PointIndex < VoronoiCell.Num(); PointIndex++)
		{
			FVector2d CurrentPoint = VoronoiCell[PointIndex];
			int NextIndex = PointIndex == VoronoiCell.Num() - 1 ? 0 : PointIndex + 1;
			FVector2d NextPoint = VoronoiCell[NextIndex];
			
			float Weight = FVector2d::Distance(CurrentPoint, NextPoint);

			// DEBUG EDGE : check if all edge are treated
			FVector2d MidPoint = (CurrentPoint + NextPoint) / 2;
			FVector MidPoint3D(MidPoint.X, MidPoint.Y, 10.0f); 
			DrawDebugSphere(GetWorld(), MidPoint3D, 1.0f, 4, FColor::Green, true);
			//

			FIntPoint CurrentPointInt = FIntPoint(CurrentPoint.X, CurrentPoint.Y);
			int CurrentNodeIndex = ResultGraph.IsNodeInGraph(CurrentPointInt);
			if (CurrentNodeIndex == -1)
			{
				CurrentNodeIndex = Count++;
				ResultGraph.AddNode(FNode(CurrentNodeIndex, CurrentPointInt));
			}
			FIntPoint NextPointInt = FIntPoint(NextPoint.X, NextPoint.Y);
			int NextNodeIndex = ResultGraph.IsNodeInGraph(NextPointInt);
			if (NextNodeIndex == -1)
			{
				NextNodeIndex = Count++;
				ResultGraph.AddNode(FNode(NextNodeIndex, NextPointInt));
			}
			
			if (!ResultGraph.IsEdgeInGraph(CurrentNodeIndex, NextNodeIndex))
			{
				ResultGraph.AddNode(CurrentNodeIndex, CurrentPointInt, NextNodeIndex, NextPointInt, Weight);
			}
		}
	}
	return ResultGraph;
}


TArray<FPriorityEdge> ALevelGenerator::PrimAlgo(FGraphPath Graph)
{
	TSet<int32> VisitedNodes;
	TArray<FPriorityEdge> StoreMSTEdges;
	
	TArray<FPriorityEdge> EdgesPriorityQueue;

	const int32 StartNode = Graph.Nodes[0].Node;
	VisitedNodes.Add(StartNode);

	// sort all edges of start node by min weight
	const FNode StartGraphNode = Graph.GetNode(StartNode);
	for (const FGraphEdge& Edge : StartGraphNode.Edges)
	{
		EdgesPriorityQueue.HeapPush(FPriorityEdge(StartNode, Edge.TargetNode, Edge.Weight));
	}
	
	while (VisitedNodes.Num() < Graph.Nodes.Num())
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
		const FNode CurrentGraphNode = Graph.GetNode(CurrentEdge.EndNode);
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

FGraphPath ALevelGenerator::CreateMSTUsingPrim(FGraphPath Graph)
{
	TArray<FPriorityEdge> StoredMSTEdges = PrimAlgo(Graph);
	FGraphPath MSTPath = FGraphPath();

	for (const FPriorityEdge Edge : StoredMSTEdges)
	{
		const FIntPoint StartNodePos = Graph.GetNode(Edge.StartNode).Position;
		const FIntPoint EndNodePos = Graph.GetNode(Edge.EndNode).Position;

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

TArray<FIntPoint> ALevelGenerator::GetAllCoordsOfEdge(FIntPoint StartNodePos, FIntPoint EndNodePos)
{
	TArray<FIntPoint> EdgePoints;

	const int DistanceX = FMath::Abs(EndNodePos.X - StartNodePos.X);
	const int DistanceY = FMath::Abs(EndNodePos.Y - StartNodePos.Y);

	const int DirectionX = (StartNodePos.X < EndNodePos.X) ? 1 : -1;
	const int DirectionY = (StartNodePos.Y < EndNodePos.Y) ? 1 : -1;

	int Err = DistanceX - DistanceY;

	int CurrentX = StartNodePos.X;
	int CurrentY = StartNodePos.Y;

	bool bHasMovedOnX = false;
	bool bHasMovedOnY = false;

	int EndPosX = EndNodePos.X;
	int EndPosY = EndNodePos.Y;

	while (true)
	{
		
		if (CurrentX == EndPosX && CurrentY == EndPosY) break;
		int Err2 = 2 * Err;

		FIntPoint LastPoint = FIntPoint(CurrentX, CurrentY);
		
		if (Err2 > -DistanceY)
		{
			Err -= DistanceY;
			CurrentX += DirectionX;
			bHasMovedOnX = true;
		}
		if (Err2 < DistanceX)
		{
			Err += DistanceX;
			CurrentY += DirectionY;
			bHasMovedOnY = true;
		}
		EdgePoints.Add(FIntPoint(CurrentX, CurrentY));

		// diagonal mouvement
		if (bHasMovedOnX && bHasMovedOnY)
		{
			if (FMath::RandBool())
			{
				EdgePoints.Add(FIntPoint(CurrentX, LastPoint.Y));
			}
			else
			{
				EdgePoints.Add(FIntPoint(LastPoint.X, CurrentY));
			}
		}
	} 
	
	return EdgePoints;
}

void ALevelGenerator::GenerateLevelFromMST(FGraphPath Graph)
{
	if (UWorld* World = GetWorld())
	{
		for (const TPair<int32, FNode>& NodePair : Graph.Nodes)
		{
			FNode CurrentNode = NodePair.Value;
			if (!VisitedCells.Contains(CurrentNode.Position))
			{
				VisitedCells.Add(CurrentNode.Position);
				FVector RoomPosition = FVector(CurrentNode.Position.X * CELL_SIZE, CurrentNode.Position.Y * CELL_SIZE, 0.0f); 
				World->SpawnActor<AFloor>(RoomType, RoomPosition, FRotator::ZeroRotator);
				AddTilesAroundNode(CurrentNode.Position, CELL_SIZE);
			}
			
			for (const FGraphEdge& Edge : CurrentNode.Edges)
			{
				FIntPoint StartPos = CurrentNode.Position;
				FIntPoint EndPos = Edge.TargetNodePos;
				
				TArray<FIntPoint> EdgePoints = GetAllCoordsOfEdge(StartPos, EndPos);
				
				for (const FIntPoint& Point : EdgePoints)
				{
					if (Point == StartPos || Point == EndPos) continue;
					if (!VisitedCells.Contains(Point))
					{
						VisitedCells.Add(Point);
						FVector CorridorPosition = FVector(Point.X * CELL_SIZE, Point.Y * CELL_SIZE, 0.0f);
						World->SpawnActor<AFloor>(CorridorType, CorridorPosition, FRotator::ZeroRotator);
					}
				}
			}
		}
	}
	VisitedCells.Reset();
}

void ALevelGenerator::AddTilesAroundNode(FIntPoint NodePos, int32 CellSize)
{
	TArray<FIntPoint> Offsets = {
		FIntPoint(-1, -1), FIntPoint(0, -1), FIntPoint(1, -1),
		FIntPoint(-1, 0),  FIntPoint(1, 0),
		FIntPoint(-1, 1), FIntPoint(0, 1), FIntPoint(1, 1)
	};
	
	if (UWorld* World = GetWorld())
	{
		for (FIntPoint Offset : Offsets)
		{
			FIntPoint NewPos = NodePos + Offset;
			if (VisitedCells.Contains(NewPos)) continue;
			VisitedCells.Add(NewPos);
			FVector TilePosition = FVector(NewPos.X * CellSize, NewPos.Y * CellSize, 0.0f);
			World->SpawnActor<AFloor>(RoomType, TilePosition, FRotator::ZeroRotator);
		}
	}
}

void ALevelGenerator::SpawnPlayer(FGraphPath Graph)
{
	int32 randNode = 0;
	do
	{
		randNode = FMath::RandRange(0, Graph.Nodes.Num()-1);
	} while (!Graph.Nodes.Contains(randNode));
	
	FIntPoint NodePosition = Graph.Nodes[randNode].Position;
	if (UWorld* World = GetWorld())
	{
		if (APlayerController* PlayerController = World->GetFirstPlayerController())
		{
			if (AMyPawn* MyPawn = CastChecked<AMyPawn>(PlayerController->GetPawn()))
			{
				MyPawn->SetActorLocation(FVector(NodePosition.X * CELL_SIZE, NodePosition.Y * CELL_SIZE, 100.f));
			}
		}
	}

}

void ALevelGenerator::SpawnPlayerZero()
{
	APlayerController* PlayerController = GetWorld()->GetFirstPlayerController();
	AMyPawn* MyPawn = CastChecked<AMyPawn>(PlayerController->GetPawn());
	MyPawn->SetActorLocation(FVector(0, 0, 100.f));
}

// DEBUG

void ALevelGenerator::DebugPoints(TArray<FIntPoint> Points)
{
	if (const UWorld* World = GetWorld())
	{
		for (const FIntPoint& Point : Points)
		{
			DrawDebugSphere(World, FVector(Point, 10), 2.f, 10.f, FColor::Magenta, true);
		}
	}
}

void ALevelGenerator::DebugGraph(FGraphPath Graph, FColor ColorNode, FColor ColorEdge)
{
	const UWorld* World = GetWorld();
	if (World == nullptr) return;
	
	for(TTuple<int32,FNode>& Elt : Graph.Nodes)
	{
		FNode Node = Elt.Value;
		DrawDebugSphere(World, FVector(Node.Position, 10), 2.f, 10.f, ColorNode, true);
		
		for (FGraphEdge& Edge : Node.Edges)
		{
			DrawDebugLine(World, FVector(Node.Position,10), FVector(Edge.TargetNodePos,10), ColorEdge, true);
		}
	}
}
