/************************************************************************/
/* Author: YWT20                                                        */
/* Expected release year : 2020                                         */
/************************************************************************/

#include "SimilarityRecognitionBPLibrary.h"
#include "SimilarityRecognition.h"

#if PLATFORM_MAC
#include "Mac/MacPlatformApplicationMisc.h"
#elif PLATFORM_WINDOWS
#include "Windows/WindowsPlatformApplicationMisc.h"
#elif PLATFORM_LINUX
#include "Linux/LinuxPlatformApplicationMisc.h"
#elif PLATFORM_ANDROID
#include "Android/AndroidPlatformApplicationMisc.h"
#elif PLATFORM_IOS
#include "IOS/IOSPlatformApplicationMisc.h"
#endif 



USimilarityRecognitionBPLibrary::USimilarityRecognitionBPLibrary(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{

}

float USimilarityRecognitionBPLibrary::Recognize(TArray<FVector2D> mainVec, TArray<FVector2D> TestVec, bool bIsAutoRotation, bool bIsAutoScale)
{
	DTWRecognizer dtwr;
	return dtwr.Recognize(mainVec, TestVec, bIsAutoRotation, bIsAutoScale);
}

void USimilarityRecognitionBPLibrary::CopyMessageToClipboard(FString text)
{
	FPlatformApplicationMisc::ClipboardCopy(*text);
}

FString USimilarityRecognitionBPLibrary::PasteMessageFromClipboard()
{
	FString ClipboardContent;
	FPlatformApplicationMisc::ClipboardPaste(ClipboardContent);
	return ClipboardContent;
}

TArray<FVector2D> DTWRecognizer::TranslateCentroidTo(TArray<FVector2D>& points, FVector2D toPt)
{
	TArray<FVector2D> newPoints;
	FVector2D centroid = Centroid(points);
	for (int i = 0; i < points.Num(); i++)
	{
		FVector2D p = points[i];
		p.X += (toPt.X - centroid.X);
		p.Y += (toPt.Y - centroid.Y);
		newPoints.Add(p);
	}
	return newPoints;
}

TArray<FVector2D> DTWRecognizer::ScaleTo(const TArray<FVector2D>& points, FVector2D sz)
{
	TArray<FVector2D> newPoints;
	FBox2D r = FBox2D(points);
	for (int i = 0; i < points.Num(); i++)
	{
		FVector2D p = points[i];
		if (r.GetSize().X != 0.0f)
			p.X *= (sz.X / r.GetSize().X);
		if (r.GetSize().Y != 0.0f)
			p.Y *= (sz.Y / r.GetSize().Y);
		newPoints.Add(p);
	}
	return newPoints;
}

double DTWRecognizer::AngleInDegrees(FVector2D start, FVector2D end, bool positiveOnly)
{
	double radians = AngleInRadians(start, end, positiveOnly);
	return FMath::RadiansToDegrees(radians);
}

double DTWRecognizer::AngleInRadians(FVector2D start, FVector2D end, bool positiveOnly)
{
	double radians = 0.0;
	if (start.X != end.X)
	{
		radians = FMath::Atan2(end.Y - start.Y, end.X - start.X);
	}
	else // pure vertical movement
	{
		if (end.Y < start.Y)
			radians = -PI / 2.0; // -90 degrees is straight up
		else if (end.Y > start.Y)
			radians = PI / 2.0; // 90 degrees is straight down
	}
	if (positiveOnly && radians < 0.0)
	{
		radians += PI * 2.0;
	}
	return radians;
}

TArray<FVector2D> DTWRecognizer::RotateByRadians(TArray<FVector2D>& points, double radians)
{
	TArray<FVector2D> newPoints;

	FVector2D c = Centroid(points);

	float cos = FMath::Cos(radians);
	float sin = FMath::Sin(radians);

	float cx = c.X;
	float cy = c.Y;

	for (int i = 0; i < points.Num(); i++)
	{
		FVector2D p = points[i];
		double dx = p.X - cx;
		double dy = p.Y - cy;

		FVector2D q = FVector2D();
		q.X = dx * cos - dy * sin + cx;
		q.Y = dx * sin + dy * cos + cy;
		newPoints.Add(q);
	}
	return newPoints;
}

FVector2D DTWRecognizer::Centroid(TArray<FVector2D>& points)
{
	float xsum = 0.0;
	float ysum = 0.0;

	for (FVector2D& p : points)
	{
		xsum += p.X;
		ysum += p.Y;
	}
	return FVector2D(xsum / points.Num(), ysum / points.Num());
}

float DTWRecognizer::Recognize(TArray<FVector2D>& mainVec, TArray<FVector2D>& TestVec, bool bIsAutoRotation, bool bIsAutoScale)
{
	double radians = 0.0;
	// rotate so that the centroid-to-1st-point is at zero degrees
	if (bIsAutoRotation)
	{
		radians = AngleInRadians(Centroid(mainVec), (FVector2D)mainVec[0], false); // indicative angle
		mainVec = RotateByRadians(mainVec, -radians); // undo angle
	}
	// scale to a common (square) dimension
	if (bIsAutoScale)
	{
		mainVec = ScaleTo(mainVec, ResampleScale);
	}
	// translate to a common origin
	mainVec = TranslateCentroidTo(mainVec, FVector2D::ZeroVector);


	if (bIsAutoRotation)
	{
		radians = AngleInRadians(Centroid(TestVec), (FVector2D)TestVec[0], false); // indicative angle
		TestVec = RotateByRadians(TestVec, -radians); // undo angle
	}
	// scale to a common (square) dimension
	if (bIsAutoScale)
	{
		TestVec = ScaleTo(TestVec, ResampleScale);
	}
	// translate to a common origin
	TestVec = TranslateCentroidTo(TestVec, FVector2D::ZeroVector);

	TArray<double> best = GoldenSectionSearch(
		mainVec,                 // to rotate
		TestVec,               // to match
		FMath::DegreesToRadians(-45.0),   // lbound
		FMath::DegreesToRadians(+45.0),   // ubound
		bIsAutoRotation ? FMath::DegreesToRadians(2.0) : DBL_MAX);    // threshold

	double score = 1.0f - best[0] / HalfDiagonal;
	return score;
}

TArray<double> DTWRecognizer::GoldenSectionSearch(TArray<FVector2D> pts1, TArray<FVector2D> pts2, double a, double b, double threshold)
{
	double x1 = Phi * a + (1 - Phi) * b;
	TArray<FVector2D> newPoints = RotateByRadians(pts1, x1);
	double fx1 = DTWPathDistance(newPoints, pts2) / newPoints.Num();

	double x2 = (1 - Phi) * a + Phi * b;
	newPoints = RotateByRadians(pts1, x2);
	double fx2 = DTWPathDistance(newPoints, pts2) / newPoints.Num();

	double i = 2.0; // calls
	while (FMath::Abs(b - a) > threshold)
	{
		if (fx1 < fx2)
		{
			b = x2;
			x2 = x1;
			fx2 = fx1;
			x1 = Phi * a + (1 - Phi) * b;
			newPoints = RotateByRadians(pts1, x1);
			fx1 = DTWPathDistance(newPoints, pts2) / newPoints.Num();
		}
		else
		{
			a = x1;
			x1 = x2;
			fx1 = fx2;
			x2 = (1 - Phi) * a + Phi * b;
			newPoints = RotateByRadians(pts1, x2);
			fx2 = DTWPathDistance(newPoints, pts2) / newPoints.Num();
		}
		i++;
	}
	TArray<double> retList;
	retList.Add(FMath::Min(fx1, fx2));
	retList.Add(FMath::RadiansToDegrees((b + a) / 2.0));
	retList.Add(i);

	return retList; // distance, angle, calls to pathdist
}

double DTWRecognizer::DTWPathDistance(TArray<FVector2D>& v, TArray<FVector2D>& w)
{
	int n = v.Num();
	int m = w.Num();

	TArray<TArray<double>> _DTW;
	for (int32 i = 0; i < n; i++)
	{
		TArray<double> tmpDoubleArray;
		for (int32 x = 0; x < m; x++)
		{
			tmpDoubleArray.Add(0);
		}
		_DTW.Add(tmpDoubleArray);
	}

	for (int i = 1; i < m; i++)
	{
		_DTW[0][i] = FLT_MAX;
	}
	for (int i = 1; i < n; i++)
	{
		_DTW[i][0] = FLT_MAX;
	}
	_DTW[0][0] = 0;

	for (int i = 1; i < n; i++)
	{
		for (int j = 1; j < m; j++)
		{
			FVector2D p1 = v[i];
			FVector2D p2 = w[j];
			double cost = FVector2D::Distance(p1, p2);

			_DTW[i][j] = FMath::Min(FMath::Min(      // min3
				_DTW[i - 1][j] + cost,           // insertion
				_DTW[i][j - 1] + cost),          // deletion
				_DTW[i - 1][j - 1] + cost);      // match
		}
	}

	return _DTW[n - 1][m - 1];
}
