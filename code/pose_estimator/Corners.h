/*
*  drones-267, automated landing using pose estimation
*  Copyright (C) {2013}  {Constantin Berzan, Nahush Bhanage, Sunil Shah}
*  
*  https://github.com/ssk2/drones-267
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
* 
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* 
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/opencv.hpp>
using namespace cv;

/**
 * Detect corners of landing pad in the provided image.
 *
 * If succeeds, returns 24 points in the order described in the paper.
 * If fails, returns an empty matrix.
 *
 * Inputs:
 * frame = input image
 *
 * Returns:
 * imagePts (24x2) = image points
 */

class _Polygon
{
public:
    char Label[2];
    vector<Point> Corners;
    Point2f Center;

    _Polygon()
    {
        Label[0] = '-';
        Label[1] = '\0';
    }
};

struct _Diagonal
{
    Vec2f Diag;
    int indexOfCorner1;
    int indexOfCorner2;
};

Mat_<double> detectCorners(
    Mat const frame,
    char const* inputWindowHandle = NULL,
    char const* cannyWindowHandle = NULL,
    char const* contourWindowHandle = NULL);
