// Fill out your copyright notice in the Description page of Project Settings.


#include "PointGenerator.h"

PointGenerator::PointGenerator()
{
}

PointGenerator::~PointGenerator()
{
}

TArray<FVector2d> PointGenerator::GeneratePoints(int32 NumberOfPoints, int32 MinPos, int32 MaxPos)
{
	TArray<FVector2d> Points;

	for (int i = 0; i < NumberOfPoints; i++)
	{
		Points.Add(FVector2d(FMath::RandRange(MinPos, MaxPos), FMath::RandRange(MinPos, MaxPos)));
	}

	return Points;
}

