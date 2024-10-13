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
	
	TArray<FNode> Nodes;

	FGraphPath()
	{
	}

	bool IsNodeInGraph(int32 pNode)
	{
		for (const FNode lNode : Nodes)
		{
			if (lNode.Node == pNode) return true;
		}
		return false;
	}

	bool IsEdgeInGraph(int32 pNode1, int32 pNode2)
	{
		if (!IsNodeInGraph(pNode1)) return false;
		if (!IsNodeInGraph(pNode2)) return false;
		return true;
	}
	
	FNode GetNode(int32 pNode)
	{
		FNode resultNode;
		for (const FNode lNode : Nodes)
		{
			if (lNode.Node == pNode)
			{
				return lNode;
			}
		}
		return resultNode;
	}

	void AddNode(int32 currentNode, FVector2d posNode,
				int32 Node1, FVector2d posNode1, int32 Weight1,
				int32 Node2, FVector2d posNode2, int32 Weight2)
	{
		FNode node = FNode(currentNode, posNode);
		if (!IsEdgeInGraph(currentNode, Node1))
		{
			node.AddEdge(Node1, posNode1, Weight1);
		}
		if (!IsEdgeInGraph(currentNode, Node2))
		{
			node.AddEdge(Node2, posNode2, Weight2);
		}
		Nodes.Add(node);
	}
};
