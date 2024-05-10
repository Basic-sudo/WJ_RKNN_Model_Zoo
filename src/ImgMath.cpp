#include "ImgMath.hpp"

double GetEuclidean(IN Point2f& p1, IN Point2f& p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

Point2f GetMean(IN vector<Point>& inPoints)
{
    int num = inPoints.size();
    float sumX = 0.0f;
    float sumY = 0.0f;
    if (!num) {
        cout << "get_mean erro" << endl;
        return {0.0f, 0.0f};
    } else {
        for (int i=0; i<num; i++) {
            sumX += inPoints[i].x;
            sumY += inPoints[i].y;
        }
        return {float(sumX/num), float(sumY/num)};
    }
}