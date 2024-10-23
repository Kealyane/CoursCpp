// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"


class COURSCPP_API PointGenerator
{
public:
	PointGenerator();
	~PointGenerator();

	static TArray<FIntPoint> GeneratePoints(int32 NumberOfPoints, int32 MinPos, int32 MaxPos);
	static TArray<FVector2d> ConvertPointsToVector2d(TArray<FIntPoint> Points);
};
