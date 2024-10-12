#pragma once

#include "CoreMinimal.h"
#include "GraphPath.generated.h"

USTRUCT()
struct FGraphEdge
{
	GENERATED_BODY()

	int32 StartNode;
	int32 TargetNode;
	float Weight;

	FGraphEdge()
		: StartNode(0), TargetNode(0), Weight(1.0f)
	{
	}

	FGraphEdge(int32 pStartNode, int32 pTargetNode, float pWeight)
		: StartNode(pStartNode), TargetNode(pTargetNode), Weight(pWeight)
	{
	}
};

USTRUCT()
struct FNode
{
	GENERATED_BODY()

	int32 Node;
	FVector2d Position;
	TArray<FGraphEdge> Edges;

	FNode()
		: Node(0), Position(FVector2d::Zero())
	{
	}
	
	FNode(int32 pNode, FVector2d pPosition)
		: Node(pNode), Position(pPosition)
	{
	}
	
};

USTRUCT()
struct FGraphPath
{
	GENERATED_BODY();

	FNode StartNode;
	TArray<FNode> Nodes;

	FGraphPath()
	{
	}
};
