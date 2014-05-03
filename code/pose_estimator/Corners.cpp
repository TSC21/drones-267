#include "Corners.h"
#include <cstdio>

#define APPROX_POLY_PARAMETER 0.05
#define PI 3.1416

using namespace std;

inline float sqr(float x)
{
    return x*x;
}

inline float calcDistanceBet2Points(Point2f P1, Point2f P2)
{
    return sqrt(sqr(P1.x - P2.x) + sqr(P1.y - P2.y));
}

inline float maxFloat(float & A, float & B)
{
	return ((A > B) ? A : B);
}

void labelPolygons(_Polygon (&Polygons)[6], int (&orderOfPolygons)[6])
{
    float distanceFromAToD = 0, dist;
    for (int k=0; k<6; k++)
    {
        if (k != orderOfPolygons[0])
        {
            dist = calcDistanceBet2Points(Polygons[orderOfPolygons[0]].Center, Polygons[k].Center);
            if (dist > distanceFromAToD)
            {
                distanceFromAToD = dist;
                orderOfPolygons[3] = k;
            }
        }
    }

    Vec3f DA = Vec3f(Polygons[orderOfPolygons[0]].Center.x - Polygons[orderOfPolygons[3]].Center.x,
                    Polygons[orderOfPolygons[0]].Center.y - Polygons[orderOfPolygons[3]].Center.y,
                    0);

    float distFromD, distBCFromD[2][2] = {0}, distEFFromD[2][2] = {0};
	int indexBC = 0, indexEF = 0;
    for (int k=0; k<6; k++)
    {
        if (k == orderOfPolygons[3] || k == orderOfPolygons[0])
            continue;
        Vec3f DV = Vec3f(Polygons[k].Center.x - Polygons[orderOfPolygons[3]].Center.x,
                    Polygons[k].Center.y - Polygons[orderOfPolygons[3]].Center.y,
                    0);

        distFromD = calcDistanceBet2Points(Polygons[orderOfPolygons[3]].Center, Polygons[k].Center);

        Vec3f cP = DA.cross(DV);
        if (cP.val[2] > 0)
        {
			distEFFromD[indexEF][0] = distFromD;
			distEFFromD[indexEF][1] = k;
			indexEF++;
        }
        else
        {
			distBCFromD[indexBC][0] = distFromD;
			distBCFromD[indexBC][1] = k;
			indexBC++;
        }
		if (distBCFromD[0][0] > distBCFromD[1][0])
		{
			orderOfPolygons[1] = distBCFromD[0][1]; // Polygon B
			orderOfPolygons[2] = distBCFromD[1][1]; // Polygon C
		}
		else
		{
			orderOfPolygons[1] = distBCFromD[1][1]; // Polygon B
			orderOfPolygons[2] = distBCFromD[0][1]; // Polygon C
		}
		
		if (distEFFromD[0][0] > distEFFromD[1][0])
		{
			orderOfPolygons[5] = distEFFromD[0][1]; // Polygon F
			orderOfPolygons[4] = distEFFromD[1][1]; // Polygon E
		}
		else
		{
			orderOfPolygons[5] = distEFFromD[1][1]; // Polygon F
			orderOfPolygons[4] = distEFFromD[0][1]; // Polygon E
		}		
    }
}

void drawPolygonLabels(Mat & contourImg, _Polygon (&Polygons)[6], int (&orderOfPolygons)[6])
{
	char label = 'A';
	for (int i = 0; i < 6; i++)
	{
		Polygons[orderOfPolygons[i]].Label[0] = label;
		putText(contourImg, Polygons[orderOfPolygons[i]].Label, Polygons[orderOfPolygons[i]].Center, FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
		label++;
	}
}

void labelCorners(_Polygon (&Polygons)[6], int (&orderOfPolygons)[6], Point2f (&Corners)[24])
{
    Vec2f DA = Vec2f(Polygons[orderOfPolygons[0]].Center.x - Polygons[orderOfPolygons[3]].Center.x,
                        Polygons[orderOfPolygons[0]].Center.y - Polygons[orderOfPolygons[3]].Center.y);
    DA = normalize(DA);

    Vec3f DA3;
    DA3.val[0] = DA.val[0];
    DA3.val[1] = DA.val[1];
    DA3.val[2] = 0;

    _Diagonal Diagonal;
    float angle, dist, dist2;
    int startingPoint[6], cornerLabel = 1, cornerIndex;
    for (int polygonIndex = 0; polygonIndex < 6; polygonIndex++)
    {
        Diagonal.Diag = Vec2f(Polygons[polygonIndex].Corners[0].x - Polygons[polygonIndex].Corners[2].x,
                            Polygons[polygonIndex].Corners[0].y - Polygons[polygonIndex].Corners[2].y);
        Diagonal.indexOfCorner1 = 0;
        Diagonal.indexOfCorner2 = 2;
        angle = DA.dot(normalize(Diagonal.Diag));
        if (abs(angle) < 0.95)
        {
            Diagonal.Diag = Vec2f(Polygons[polygonIndex].Corners[1].x - Polygons[polygonIndex].Corners[3].x,
                                    Polygons[polygonIndex].Corners[1].y - Polygons[polygonIndex].Corners[3].y);
            Diagonal.indexOfCorner1 = 1;
            Diagonal.indexOfCorner2 = 3;
        }
        startingPoint[polygonIndex] = Diagonal.indexOfCorner1;
        // For square A
        if (polygonIndex == orderOfPolygons[0])
        {
            // Select the corner which is farther from the center of D
            dist = calcDistanceBet2Points(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner1], Polygons[orderOfPolygons[3]].Center);
            dist2 = calcDistanceBet2Points(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner2], Polygons[orderOfPolygons[3]].Center);
            if (dist2 > dist)
                startingPoint[polygonIndex] = Diagonal.indexOfCorner2;
        }
        // For square D
        else if (polygonIndex == orderOfPolygons[3])
        {
            // Select the corner which is closer to the center of A
            dist = calcDistanceBet2Points(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner1], Polygons[orderOfPolygons[0]].Center);
            dist2 = calcDistanceBet2Points(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner2], Polygons[orderOfPolygons[0]].Center);
            if (dist2 < dist)
                startingPoint[polygonIndex] = Diagonal.indexOfCorner2;
        }
        // For remaining squares
        else
        {
            // Consider vector DV as the vector joining D's center to the current polygon's center.
            // Select the corner which is on the same side of DV as that of A's center.
            Vec3f DV = Vec3f(Polygons[polygonIndex].Center.x - Polygons[orderOfPolygons[3]].Center.x,
                            Polygons[polygonIndex].Center.y - Polygons[orderOfPolygons[3]].Center.y,
                            0);
            Vec3f cP1 = DV.cross(DA3);

            Vec3f DCorner = Vec3f(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner1].x - Polygons[orderOfPolygons[3]].Center.x,
                                    Polygons[polygonIndex].Corners[Diagonal.indexOfCorner1].y - Polygons[orderOfPolygons[3]].Center.y,
                                    0);
            Vec3f cP2 = DV.cross(DCorner);

            if (cP1.val[2]*cP2.val[2] < -1)
                startingPoint[polygonIndex] = Diagonal.indexOfCorner2;
        }
    }

    for (int polygonIndex = 0; polygonIndex < 6; polygonIndex++)
    {
        cornerIndex = startingPoint[orderOfPolygons[polygonIndex]];
        for (int k=0; k<4; k++)
        {
            Corners[cornerLabel-1] = Polygons[orderOfPolygons[polygonIndex]].Corners[cornerIndex];
            cornerLabel++;
            cornerIndex = (cornerIndex + 1) % 4;
        }
    }
}

void drawCornerLabels(Mat & contourImg, Point2f (&Corners)[24])
{
    // Display corner labels
    char cLabel[3];
    for (int i=1; i<=24; i++)
    {
        sprintf(cLabel, "%d", i);
        putText(contourImg, cLabel, Corners[i-1], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    }
}

Mat_<double> detectCorners(
	Mat const frame,
	char const* inputWindowHandle,
	char const* thresholdedWindowHandle,
	char const* contourWindowHandle)
{
	if (inputWindowHandle)
	{
		imshow(inputWindowHandle, frame);
	}

	Mat grayImage, thresholdedImage;
	cvtColor(frame, grayImage, CV_RGB2GRAY);
	adaptiveThreshold(grayImage, thresholdedImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 75, 0);
	
	// Canny(frame, thresholdedImage, 50, 200, 3);
	
	if (thresholdedWindowHandle)
	{
		imshow(thresholdedWindowHandle, thresholdedImage);
	}
	
	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;
	findContours(
		thresholdedImage,
		contours,
		hierarchy,
		CV_RETR_TREE,
		CV_CHAIN_APPROX_SIMPLE,
		Point(0, 0));

	int numberOfContours = contours.size();
	vector<Point> approx;
	vector< vector<Point> > quadrilaterals;

	vector<int> numberOfChildContours;
	vector<int> parentContours;
	vector<int>::iterator itr;	
	vector<int> quadrilateralIndices;
	
	for (int i = 0; i < numberOfContours; i++)
	{
		if (contours[i].size() < 4 || hierarchy[i].val[3] == -1)
		{
			continue;
		}
		approxPolyDP(
			Mat(contours[i]),
			approx,
			arcLength(Mat(contours[i]), true) * APPROX_POLY_PARAMETER,
			true);
		if (approx.size() == 4 &&
			fabs(contourArea(Mat(approx))) > 100 &&
			isContourConvex(Mat(approx)))
		{
			itr = find(parentContours.begin(), parentContours.end(), hierarchy[i].val[3]);
			if (itr == parentContours.end())
			{
				parentContours.push_back(hierarchy[i].val[3]);
				numberOfChildContours.push_back(1);
			}
			else
			{
				numberOfChildContours[distance(parentContours.begin(), itr)]++;
			}
			quadrilateralIndices.push_back(i);
			quadrilaterals.push_back(approx);
		}
	}
	
	itr = max_element(numberOfChildContours.begin(), numberOfChildContours.end());
	int indexOfOuterSquare = parentContours[distance(numberOfChildContours.begin(), itr)];
	bool allCornersDetected = (*itr == 6);

	// If all squares are not detected, return failure.
	if (!allCornersDetected)
	{
		if (contourWindowHandle)
		{
			Mat contourImg = Mat::zeros(thresholdedImage.size(), CV_8UC3);
			imshow(contourWindowHandle, contourImg);
		}
		return Mat_<double>();  // empty
	}

	int cIndex, polygonIndex = 0;
	double maxPolyArea = 0, PolyArea = 0;
	_Polygon Polygons[6];
	Moments mo;

	int numberOfSelectedContours = quadrilateralIndices.size();
	int orderOfPolygons[6] = { 0 };
	int contourIndices[6];
	for (int i = 0; i < numberOfSelectedContours; i++)
	{
		cIndex = quadrilateralIndices[i];
		if (hierarchy[cIndex].val[3] != indexOfOuterSquare)
		{
			continue;
		}
		
		PolyArea = fabs(contourArea(Mat(quadrilaterals[i])));
		Polygons[polygonIndex].Corners = quadrilaterals[i];
		mo = moments(contours[cIndex], false);
		Polygons[polygonIndex].Center = Point2f(mo.m10 / mo.m00, mo.m01 / mo.m00);
		if (PolyArea > maxPolyArea)
		{
			maxPolyArea = PolyArea;
			orderOfPolygons[0] = polygonIndex;
		}
		contourIndices[polygonIndex] = cIndex;
		polygonIndex++;
	}

	Point2f Corners[24];
	labelPolygons(Polygons, orderOfPolygons);
	labelCorners(Polygons, orderOfPolygons, Corners);

	if (contourWindowHandle)
	{
		Mat contourImg = Mat::zeros(thresholdedImage.size(), CV_8UC3);
		for (int i = 0; i < 6; i++)
		{
			cIndex = contourIndices[i];
			drawContours(
				contourImg,
				contours,
				cIndex,
				Scalar(255, 0, 0),
				1, 8,
				hierarchy,
				0,
				Point());
		}
		drawPolygonLabels(contourImg, Polygons, orderOfPolygons);
		drawCornerLabels(contourImg, Corners);
		imshow(contourWindowHandle, contourImg);
	}

	Mat_<double> cornerMatrix(24, 2);
	for (int i = 0; i < 24; i++)
	{
		cornerMatrix(i, 0) = Corners[i].x;
		cornerMatrix(i, 1) = Corners[i].y;
	}
	return cornerMatrix;
}
