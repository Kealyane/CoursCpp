// Fill out your copyright notice in the Description page of Project Settings.


#include "PointGenerator.h"

PointGenerator::PointGenerator()
{
}

PointGenerator::~PointGenerator()
{
}

TArray<FIntPoint> PointGenerator::GeneratePoints(int32 NumberOfPoints, int32 MinPos, int32 MaxPos)
{
	TArray<FIntPoint> Points;

	for (int i = 0; i < NumberOfPoints; i++)
	{
		Points.Add(FIntPoint(FMath::RandRange(MinPos, MaxPos), FMath::RandRange(MinPos, MaxPos)));
	}

	return Points;
}

TArray<FVector2d> PointGenerator::ConvertPointsToVector2d(TArray<FIntPoint> Points)
{
	TArray<FVector2d> Result;
	for (FIntPoint Element : Points)
	{
		Result.Add(FVector2d(Element.X, Element.Y));
	}
	return Result;
}

