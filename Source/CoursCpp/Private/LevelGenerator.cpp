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
	FGraphPath MSTGraph = CreateGraphFromMST(MyGraph);
	DebugMSTGraph(MSTGraph); 
}

FGraphPath ALevelGenerator::GenerateTriangles()
{
	UE::Geometry::FDelaunay2 DelaunayTriangulation;
	
	Points = PointGenerator::GeneratePoints(50, -100, 100);
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

TArray<FPriorityEdge> ALevelGenerator::GenerateMST(FGraphPath pGraph)
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

FGraphPath ALevelGenerator::CreateGraphFromMST(FGraphPath pGraph)
{
	TArray<FPriorityEdge> StoredMSTEdges = GenerateMST(pGraph);
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
