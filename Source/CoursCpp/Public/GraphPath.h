#pragma once

#include "CoreMinimal.h"
#include "GraphPath.generated.h"

USTRUCT()
struct FGraphEdge
{
	GENERATED_BODY()
	
	int32 TargetNode;
	FVector2d TargetNodePos;
	float Weight;

	FGraphEdge()
		: TargetNode(0), TargetNodePos(FVector2d::Zero()), Weight(1.0f)
	{
	}

	FGraphEdge(int32 pTargetNode, FVector2d pTargetPos, float pWeight)
		: TargetNode(pTargetNode), TargetNodePos(pTargetPos), Weight(pWeight)
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
		: Node(-1), Position(FVector2d::Zero())
	{
	}
	
	FNode(int32 pNode, FVector2d pPosition)
		: Node(pNode), Position(pPosition)
	{
	}

	void AddEdge(int32 pTargetNode, FVector2d pTargetPos, float pWeight)
	{
		Edges.Add(FGraphEdge(pTargetNode, pTargetPos, pWeight));
	}
};

USTRUCT()
struct FGraphPath
{
	GENERATED_BODY();
	
	TMap<int32, FNode> Nodes;

	FGraphPath()
	{
	}

	bool IsNodeInGraph(int32 pNode)
	{
		return Nodes.Contains(pNode);
	}
	FNode GetNode(int32 pNode)
	{
		return Nodes[pNode];
	}
	// Used for Prim (add node from triangle)
	void AddNode(int32 currentNode, FVector2d posNode,
	 			int32 Node1, FVector2d posNode1, int32 Weight1,
	 			int32 Node2, FVector2d posNode2, int32 Weight2)
	{
		if (Nodes.Contains(currentNode))
		{
			Nodes[currentNode].AddEdge(Node1, posNode1, Weight1);
			Nodes[currentNode].AddEdge(Node2, posNode2, Weight2);
		}
		else
		{
			FNode node = FNode(currentNode, posNode);
			node.AddEdge(Node1, posNode1, Weight1);
			node.AddEdge(Node2, posNode2, Weight2);
			Nodes.Add(currentNode,node);
		}
	}
	// Used for MST
	void AddNode(int32 StartNode, FVector2d PosStart, int32 EndNode, FVector2d PosEnd, float Weight)
	{
		if (Nodes.Contains(StartNode))
		{
			Nodes[StartNode].AddEdge(EndNode, PosEnd, Weight);
		}
		else
		{
			FNode node = FNode(StartNode, PosStart);
			node.AddEdge(EndNode, PosEnd, Weight);
			Nodes.Add(StartNode, node);			
		}
	}
};

USTRUCT()
struct FPriorityEdge
{
	GENERATED_BODY()

	int32 StartNode;
	int32 EndNode;
	float Weight;

	FPriorityEdge()
		: StartNode(0), EndNode(0), Weight(0.f)
	{
	}
	FPriorityEdge(int32 pStartNode, int32 pEndNode, float pWeight)
		: StartNode(pStartNode), EndNode(pEndNode), Weight(pWeight)
	{
	}

	bool operator < (const FPriorityEdge& Other) const
	{
		return Weight < Other.Weight;
	}
};
