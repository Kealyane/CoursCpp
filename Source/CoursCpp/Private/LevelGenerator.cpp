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

	TArray<FVector2d> Points = PointGenerator::GeneratePoints(20, 0, 100);

	FGraphPath MyGraph = GenerateTriangles(Points);
	UE_LOG(LogTemp, Warning, TEXT("Delaunay :"));
	MyGraph.DebugGraph();
	DebugPoints(Points);
	DebugGraph(MyGraph, FColor::Cyan, FColor::Blue);

	FGraphPath VoronoiGraph = GenerateVoronoi(Points);
	DebugGraph(VoronoiGraph, FColor::Cyan, FColor::Blue);
	UE_LOG(LogTemp, Warning, TEXT("Voronoi :"));
	VoronoiGraph.DebugGraph();
	
	FGraphPath MSTGraph = CreateMSTUsingPrim(VoronoiGraph);
	UE_LOG(LogTemp, Warning, TEXT("MST :"));
	MSTGraph.DebugGraph();
	DebugGraph(MSTGraph, FColor::Yellow, FColor::Orange);
	
	FGraphPath MSTGraphDelaunay = CreateMSTUsingPrim(MyGraph);
	UE_LOG(LogTemp, Warning, TEXT("MST :"));
	MSTGraph.DebugGraph();
	DebugGraph(MSTGraph, FColor::Yellow, FColor::Orange);
	
	GenerateLevelFromMSTVoronoi(MSTGraph);
	//GenerateLevelFromMSTDelaunay(MSTGraphDelaunay);
	//SpawnPlayer(MSTGraphDelaunay);
	SpawnPlayerZero();
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

FGraphPath ALevelGenerator::GenerateVoronoi(TArray<FVector2d> Points)
{
	UE::Geometry::FDelaunay2 DelaunayTriangulation;

	bool bSuccess;

	do
	{
		bSuccess = DelaunayTriangulation.Triangulate(Points);
		
	} while (!bSuccess && !DelaunayTriangulation.IsDelaunay(Points));
	
	TArray<TArray<FVector2d>> Voronoi = DelaunayTriangulation.GetVoronoiCells(Points);

	// DEBUG POINTS
	for (auto Cell : Voronoi)
	{
		for (auto Vect2 : Cell)
		{
			DrawDebugSphere(GetWorld(), FVector(Vect2, 10), 2.f, 10.f, FColor::Red, true);
		}
	}
	
	FGraphPath lGraph = FGraphPath();
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
			
			int CurrentNodeIndex = lGraph.IsNodeInGraph(CurrentPoint);
			if (CurrentNodeIndex == -1)
			{
				CurrentNodeIndex = Count++;
				lGraph.AddNode(FNode(CurrentNodeIndex, CurrentPoint));
			}

			int NextNodeIndex = lGraph.IsNodeInGraph(NextPoint);
			if (NextNodeIndex == -1)
			{
				NextNodeIndex = Count++;
				lGraph.AddNode(FNode(NextNodeIndex, NextPoint));
			}
			
			if (!lGraph.IsEdgeInGraph(CurrentNodeIndex, NextNodeIndex))
			{
				lGraph.AddNode(CurrentNodeIndex, CurrentPoint, NextNodeIndex, NextPoint, Weight);
			}
		}
	}
	return lGraph;
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
		const FVector2d StartNodePos = Graph.GetNode(Edge.StartNode).Position;
		const FVector2d EndNodePos = Graph.GetNode(Edge.EndNode).Position;

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

TArray<FVector2d> ALevelGenerator::GetAllCoordsOfEdgeDelaunay(FVector2d StartNodePos, FVector2d EndNodePos)
{
	TArray<FVector2d> EdgePoints;

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

		FVector2d LastPoint = FVector2d(CurrentX, CurrentY);
		
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
		EdgePoints.Add(FVector2d(CurrentX, CurrentY));

		// diagonal mouvement
		if (bHasMovedOnX && bHasMovedOnY)
		{
			if (FMath::RandBool())
			{
				EdgePoints.Add(FVector2d(CurrentX, LastPoint.Y));
			}
			else
			{
				EdgePoints.Add(FVector2d(LastPoint.X, CurrentY));
			}
		}
	} 
	
	return EdgePoints;
}

TArray<FVector2d> ALevelGenerator::GetAllCoordsOfEdgeVoronoi(TTuple<int32, int32> const& StartNodePos, TTuple<int32, int32> const& EndNodePos)
{
	TArray<FVector2d> EdgePoints;

	const int DistanceX = FMath::Abs(EndNodePos.Key - StartNodePos.Key);
	const int DistanceY = FMath::Abs(EndNodePos.Value - StartNodePos.Value);

	const int DirectionX = (StartNodePos.Key < EndNodePos.Key) ? 1 : -1;
	const int DirectionY = (StartNodePos.Value < EndNodePos.Value) ? 1 : -1;

	int Err = DistanceX - DistanceY;

	int CurrentX = StartNodePos.Key;
	int CurrentY = StartNodePos.Value;

	bool bHasMovedOnX = false;
	bool bHasMovedOnY = false;

	int EndPosX = EndNodePos.Key;
	int EndPosY = EndNodePos.Value;

	while (true)
	{
		
		if (CurrentX == EndPosX && CurrentY == EndPosY) break;
		int Err2 = 2 * Err;

		FVector2d LastPoint = FVector2d(CurrentX, CurrentY);
		
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
		EdgePoints.Add(FVector2d(CurrentX, CurrentY));

		//diagonal mouvement
		if (bHasMovedOnX && bHasMovedOnY)
		{
			if (FMath::RandBool())
			{
				EdgePoints.Add(FVector2d(CurrentX, LastPoint.Y));
			}
			else
			{
				EdgePoints.Add(FVector2d(LastPoint.X, CurrentY));
			}
		}
	} 
	
	return EdgePoints;
}

void ALevelGenerator::GenerateLevelFromMSTVoronoi(FGraphPath Graph)
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
			
			for (int i = 0; i < CurrentNode.Edges.Num(); i++)	
			{
				FVector2d StartPos = CurrentNode.Position;
				TTuple<int32, int32> StartPosInt = TTuple<int32, int32>(CurrentNode.Position.X, CurrentNode.Position.Y);
				FVector2d EndPos = CurrentNode.Edges[i].TargetNodePos;
				TTuple<int32, int32> EndPosInt = TTuple<int32, int32>(EndPos.X, EndPos.Y);
				
				TArray<FVector2d> EdgePoints = GetAllCoordsOfEdgeVoronoi(StartPosInt, EndPosInt);
				
				for (const FVector2d& Point : EdgePoints)
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

void ALevelGenerator::GenerateLevelFromMSTDelaunay(FGraphPath Graph)
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
				FVector2d StartPos = CurrentNode.Position;
				FVector2d EndPos = Edge.TargetNodePos;
				
				TArray<FVector2d> EdgePoints = GetAllCoordsOfEdgeDelaunay(StartPos, EndPos);
				
				for (const FVector2d& Point : EdgePoints)
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

void ALevelGenerator::AddTilesAroundNode(FVector2d NodePos, int32 CellSize)
{
	TArray<FVector2d> Offsets = {
		FVector2d(-1, -1), FVector2d(0, -1), FVector2d(1, -1),
		FVector2d(-1, 0),  FVector2d(1, 0),
		FVector2d(-1, 1), FVector2d(0, 1), FVector2d(1, 1)
	};
	
	if (UWorld* World = GetWorld())
	{
		for (FVector2d Offset : Offsets)
		{
			FVector2d NewPos = NodePos + Offset;
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
	
	FVector2d NodePosition = Graph.Nodes[randNode].Position;
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

void ALevelGenerator::DebugPoints(TArray<FVector2d> Points)
{
	if (const UWorld* World = GetWorld())
	{
		for (const FVector2d& Point : Points)
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
