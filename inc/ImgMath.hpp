#ifndef _IMGMATH_HPP_
#define _IMGMATH_HPP_
#include <Head.hpp>

using namespace cv;
using namespace std;

double GetEuclidean(IN Point2f& p1, IN Point2f& p2);
Point2f GetMean(IN vector<Point>& inPoints);

#endif