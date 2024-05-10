#ifndef _IMG_FEATURE_HPP_
#define _IMG_FEATURE_HPP_
#include <Head.hpp>
#include "ImgCore.hpp"

using namespace cv;
using namespace std;

class ImgFeature : public ImgCore {
private:
public:
    ImgFeature(){}
    ~ImgFeature(){}
    uint8 GetOSTUThreshold(IN const Mat& inChImg);
    int BinarySege(IN const Mat& inImg, OUT Mat& outImg, IN ST_DETECT_INFO* p_detectFlag);
    int CheckLocalColor(IN const Mat& inImg, OUT Mat& outImg, IN Rect& inRect);
    int CheckGlobalColor(IN const Mat& inImg, OUT Mat& outImg);
};

class ImgContours : public ImgFeature {
private:
    int label; // 轮廓编号
    Point2f central; // 轮廓重心
    vector<Point> points; // 轮廓点集合
public:
    ImgContours(){}
    ImgContours(IN int inLabel, IN Point2f inCentral, IN vector<Point> inPoints) {
        label = inLabel;
        central = inCentral;
        points = inPoints;
    }
    ~ImgContours(){}
    static bool CompareContourAreas(IN const vector<Point>& contour1, IN const vector<Point>& contour2) {
        double area1 = contourArea(contour1);
        double area2 = contourArea(contour2);
        return (area1 > area2);
    }
    int SizerContours(IN const Mat& inImg, IN_OUT vector<vector<Point>>& inContours, IN vector<Vec4i>& inHierarchy);
    int MergeContours(IN const Mat& inImg, IN_OUT vector<vector<ImgContours>>& mergeContours);
    int Recursion(IN ImgContours& P, IN vector<ImgContours>& inTours, IN_OUT vector<vector<ImgContours>>& outTours, IN uint8 iClass, IN const float T);
    int RoiCheck(IN_OUT Rect& inRect); // rect maybe erro, this is normoal, which is rect's x1=x0+width y1=y0+heigh 
    int RoiCheck(IN_OUT Rect& inRect, IN int x0, IN int y0, IN int x1, IN int y1);
    int RoiRecognize(IN Rect& inRect, IN_OUT Rect& outRect);
    int RoiExpanded(IN Rect& inRect, IN_OUT Rect& outRect);
    int ExtractRoi(IN vector<vector<ImgContours>>& inContours, IN_OUT vector<Fire>& fireROI);
    int DrawContours(IN vector<Fire>& fireROI, IN_OUT Mat& outImg);
};

class ImgBlocks : public ImgContours {
private:
public: 
    Point p;
    int pSite;
    int pIndex[2]; // 索引点 x, y
    uint8 pResult;
    Rect pBlock;
    float pIoU;
    float areaBlock;
    ImgBlocks() {
        p = {0,0};
        pIndex[0] = 0; pIndex[1] = 0;
        pBlock = {int(pIndex[0]*BLOCK_COL), int(pIndex[0]*BLOCK_ROW), int(BLOCK_COL), int(BLOCK_ROW)}; // rect:x2-x1+1=width
        areaBlock = (COL*ROW)/(BLOCK*BLOCK);
    }
    ImgBlocks(IN Point& inP, IN int inPosition) {
        int area = 0;
        int x = 0, y = 0, width = 0, height = 0;
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (inP.x < 0 || inP.y < 0) {
            cout << "point is erro, should >= 0" << endl;
        }
        this->p = inP;
        this->pIoU = 0.0f;
        this->pResult = 0;
        this->pSite = inPosition;
        this->areaBlock = (COL*ROW)/(BLOCK*BLOCK);
        // 边界问题(需要考虑):越过交叉
        //   0 1 2
        // 0
        // 1
        // 2
        if (!this->pSite) {
            // 左上点
            this->pIndex[0] = ( p.x >= int(1*BLOCK_COL)+1) ? 
                              ((p.x >= int(2*BLOCK_COL)+1) ? 2 : 1) : 0; // x
            this->pIndex[1] = ( p.y >= int(1*BLOCK_ROW)+1) ? 
                              ((p.y >= int(2*BLOCK_ROW)+1) ? 2 : 1) : 0; // y
        } else {
            // 右下点
            this->pIndex[0] = ( p.x >= int(1*BLOCK_COL)-1) ? 
                              ((p.x >= int(2*BLOCK_COL)-1) ? 2 : 1) : 0; 
            this->pIndex[1] = ( p.y >= int(1*BLOCK_ROW)-1) ? 
                              ((p.y >= int(2*BLOCK_ROW)-1) ? 2 : 1) : 0;
        }
        
        x = int(this->pIndex[0]*BLOCK_COL);
        y = int(this->pIndex[1]*BLOCK_ROW);
        width = int(BLOCK_COL);
        height = int(BLOCK_ROW);
        this->pBlock = {x, y, width-1, height-1}; // 直接相加
        RoiCheck(this->pBlock, x0, y0, x1, y1);
        switch (this->pSite) {
            case 0: // 首点
                if (x1>=p.x-1 && y1>=p.y-1) { // 考虑边界
                    area = abs((x1 - p.x)*(y1 - p.y));
                    this->pIoU = float(area / this->areaBlock); 
                } else {
                    cout << "pSite is 0, but area erro" << endl;
                }
                break;
            case 1: // 尾点
                if (p.x+1>=x0 && p.y+1>=y0) { // 考虑边界
                    area = abs((p.x - x0)*(p.y - y0));
                    this->pIoU = float(area / this->areaBlock); 
                } else {
                    cout << "pSite is 1, but area erro" << endl;
                }
                break;
            default:
                break;
        }
        // 放弃pIoU这种思路，简直就是给自己挖坑
        if (this->pIoU > 0.01) {
            this->pResult = 1;
        } else {
            this->pResult = 0;
        }
    }
    ~ImgBlocks(){}
};
#endif