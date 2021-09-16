/************************************************************************/
/* Author: YWT20                                                        */
/* Expected release year : 2020                                         */
/************************************************************************/

#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "SimilarityRecognitionBPLibrary.generated.h"

UCLASS()
class USimilarityRecognitionBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_UCLASS_BODY()
public:

	UFUNCTION(BlueprintCallable, Category = "SimilarityRecognition")
		static float Recognize(TArray<FVector2D> mainVec, TArray<FVector2D> TestVec,bool bIsAutoRotation = true, bool bIsAutoScale = true);

	UFUNCTION(BlueprintCallable, Category = "")
		static void CopyMessageToClipboard(FString text);

	UFUNCTION(BlueprintCallable, Category = "")
		static FString PasteMessageFromClipboard();
};

class DTWRecognizer
{
private:
	double Phi = 0.5 * (-1 + FMath::Sqrt(5)); // Golden Ratio
	double DX = 250.0;
	FVector2D ResampleScale = FVector2D(DX, DX);
	double Diagonal = FMath::Sqrt(DX * DX + DX * DX);
	double HalfDiagonal = 0.5 * Diagonal;

	// translates the points so that their centroid lies at 'toPt'
	TArray<FVector2D> TranslateCentroidTo(TArray<FVector2D>& points, FVector2D toPt);

	// scales the points so that they form the size given. does not restore the 
	// origin of the box.
	TArray<FVector2D> ScaleTo(const TArray<FVector2D>& points, FVector2D sz);

	// determines the angle, in degrees, between two points. the angle is defined 
	// by the circle centered on the start point with a radius to the end point, 
	// where 0 degrees is straight right from start (+x-axis) and 90 degrees is
	// straight down (+y-axis).
	double AngleInDegrees(FVector2D start, FVector2D end, bool positiveOnly);

	// determines the angle, in radians, between two points. the angle is defined 
	// by the circle centered on the start point with a radius to the end point, 
	// where 0 radians is straight right from start (+x-axis) and PI/2 radians is
	// straight down (+y-axis).
	double AngleInRadians(FVector2D start, FVector2D end, bool positiveOnly);

	// rotate the points by the given radians about their centroid
	TArray<FVector2D> RotateByRadians(TArray<FVector2D>& points, double radians);

	// compute the centroid of the points given
	FVector2D Centroid(TArray<FVector2D>& points);

public:
	// candidate points
	float Recognize(TArray<FVector2D>& mainVec, TArray<FVector2D>& TestVec, bool bIsAutoRotation = true, bool bIsAutoScale = true);

	// Golden Section Search
	TArray<double> GoldenSectionSearch(TArray<FVector2D> pts1, TArray<FVector2D> pts2, double a, double b, double threshold);

	// DTW
	double DTWPathDistance(TArray<FVector2D>& v, TArray<FVector2D>& w);

};