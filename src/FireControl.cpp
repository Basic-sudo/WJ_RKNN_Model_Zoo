#include <FireControl.hpp>
#include "ImgCore.hpp"
#include "ImgProcess.hpp"
#include "ImgFeature.hpp"

extern int ret;
extern Mat G_srcImg;
extern Mat G_segImg;
extern ImgCore G_fireCore;
extern ImgProcess G_fireProcess;
extern ImgFeature G_fireFeature;
extern ImgContours G_fireContours;
extern float minTemp;
extern float maxTemp;
extern float centerTemp;

extern Fire G_fire; // 最后筛选的火焰区域
extern vector<Fire> G_fireVector;
extern vector<vector<ImgContours>> G_contours;

static int ZeroPre(IN const Mat& inImg, OUT Mat& outImg, IN ST_DETECT_INFO *p_detectFlag);
static int OneSega(IN const Mat& inImg, OUT Mat& outImg, IN ST_DETECT_INFO *p_detectFlag);
static int TwoCase(IN const Mat& inImg, IN_OUT Mat& outImg, IN ST_DETECT_INFO *p_detectFlag);
static int ThreeSizer(IN vector<Fire>& fireROI, IN_OUT Fire& inFire, IN ST_DETECT_INFO *p_detectFlag);

int SearchFire(IN_OUT Mat& inHeatImg)
{
    ReadTempValues();
    DisplayTempImg(inHeatImg);
    if (maxTemp>120.0f) { // 温度满足
        goto ok;
    }
    return 0;
ok:
    return 1;
}

int DetectFire(IN const Mat& inImg, OUT Mat& outImg, IN ST_DETECT_INFO *p_detectFlag)
{
    Mat srcImg = inImg.clone();
        outImg = inImg.clone();
    Mat enhImg; // enhance image
    ret = ZeroPre(srcImg, enhImg, p_detectFlag);
    if (ret != 0) {
        cout << "[ERRO] ZeroPre erro is: " << ret << endl;
        goto erro1;
    }
    ret = OneSega(enhImg, G_segImg, p_detectFlag);
    if (ret != 0) {
        cout << "[ERRO] OneSega erro is: " << ret << endl;
        goto erro1;
    }
    ret = TwoCase(G_segImg, outImg, p_detectFlag); // 确定实例框
    if (ret != 0) {
        cout << "[ERRO] TwoCase erro is: " << ret << endl;
        goto erro2;
    }
        cout << "ok!" << endl;
    ret = ThreeSizer(G_fireVector, G_fire, p_detectFlag); // 筛选框
    if (ret >= 0) {
        if (ret == 0) {
            // 正常
            goto exit;
        } else if (ret == 1) {
            // 筛选了
            if (p_detectFlag->m_u8RealSortFlag) {
                // 有火 
                p_detectFlag->m_u8RealSortFlag = 0;
                goto ok;
            } else { 
                // 无火
                goto exit;
            }
        }
    } else if (ret < 0) {
        cout << "[ERRO] ThreeSizer erro is: " << ret << endl;
        goto erro3;
    }
    return 0;
ok:
    return 1;
exit:
    return 0;
erro1:
    return -1;
erro2:
    return -2;
erro3:
    return -3;
}

static int ZeroPre(IN const Mat& inImg, OUT Mat& outImg, IN ST_DETECT_INFO *p_detectFlag)
{
    Mat srcImg;
    if (inImg.empty()) {
        cout << "[INFO] ZeroPre: inImg is null" << endl;
        goto erro1;
    } else {
        srcImg = inImg.clone();
    }
    ret = G_fireProcess.ColorEnhence(srcImg, outImg);
    if (ret != 0) {
        cout << "[INFO] ZeroPre: ColorEnhence erro" << endl;
        goto erro2;
    } else {
#if BUG_CAM_TEMP
        G_fireCore.ShowImg("ZeroPre", outImg, 1); // 保存行为
#endif
    }
    return 0;
erro1:
    return -1;
erro2:
    return -2;
}

static int OneSega(IN const Mat& inImg, OUT Mat& outImg, IN ST_DETECT_INFO *p_detectFlag)
{
    Mat srcImg;
    outImg.create(ROW, COL, CV_8UC1); outImg = {0}; // 全部抹黑
    if (inImg.empty()) {
        cout << "OneSega: inImg is null" << endl;
        goto erro1;
    } else {
        srcImg = inImg.clone();
    }
    ret = G_fireFeature.BinarySege(srcImg, outImg, p_detectFlag);
    if (ret != 0) {
        cout << "[ERRO] OneSega: BinarySege erro" << endl;
        goto erro2;
    } else {
#if BUG_CAM_TEMP
        g_fireCore.ShowImg("OneSega", outImg, 1); // 保存行为
#endif
    }
    return 0;
erro1:
    return -1;
erro2:
    return -2;
}

static int TwoCase(IN const Mat& inImg, IN_OUT Mat& outImg, IN ST_DETECT_INFO *p_detectFlag)
{
    int ret = 0;
    Mat srcImg = {};
    // outImg = inImg.clone(); // 画在当前图(此帧是分割图)
    if (inImg.empty() || outImg.empty()) {
        goto erro1;
    } else {
        srcImg = inImg.clone();
    }
    ret = G_fireContours.MergeContours(srcImg, G_contours);
    if (ret != 0) {
        goto erro2;
    }
    ret = G_fireContours.ExtractRoi(G_contours, G_fireVector);
    if (ret == 0) {
        // 正常退出(掉帧情况)
        p_detectFlag->m_u8RealSegFlag = 0; // 未提取到实例
        if (p_detectFlag->m_u8ReceptFlag) { // reserve
            p_detectFlag->m_u8ReceptNums++;
            if (p_detectFlag->m_u8ReceptNums > 5) {
                // 连续5帧
                p_detectFlag->m_u8ReceptNums = 0; // 接受区域的时间
                p_detectFlag->m_u8ReceptFlag = 0;
                if (G_fireVector.size()) {
                    G_fireVector.clear();     
                }
            }
        }
    } else if (ret > 0) {
        p_detectFlag->m_u8RealSegFlag = 1; // 实际分割检测到的
        p_detectFlag->m_u8ReceptFlag = 1;
        p_detectFlag->m_u8ReceptNums = 0;
    } else if (ret < 0) {
        goto erro3;
    }
    ret = G_fireContours.DrawContours(G_fireVector, outImg); // 检测到画框，未检测到画绿框
    if (ret != 0) {
        goto erro4;
    }
    return 0;
erro:
    return 0;
erro1:
    return -1;
erro2:
    return -2;
erro3:
    return -3;
erro4:
    return -4;
}

extern Mat MLX_IMG;
static int ThreeSizer(IN vector<Fire>& fireROI, IN_OUT Fire& inFire, IN ST_DETECT_INFO *p_detectFlag)
{
    if (fireROI.size()==0 || inFire.realRect.area()>0) {
        if (inFire.realRect.area() > 0) {
            // 清空
            inFire = {0, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
        } 
        if (fireROI.size() == 0) {
            cout << "ThreeSizer fireROI.size() is null" << endl;
            goto noFire; 
        }
    } 
    if (p_detectFlag->m_u8RealSegFlag) {
        // 筛选（初步筛最大的框）
        if (1/*SearchFire(MLX_IMG)*/) {
            uint8 label = fireROI[0].label; 
            for (uint32 i=0; i<fireROI.size(); i++) {
                if (fireROI[label].realRect.area() < fireROI[i].realRect.area() && 
                    fireROI[label].expdRect.area() < fireROI[i].expdRect.area()) {
                    label = i;
                }
            }
            inFire = {label, fireROI[label].realRect, fireROI[label].recoRect, fireROI[label].expdRect};
            goto isFire; // 有框有火
        } else {
            // 未筛选到=>确实无火
            // 数据和标志清空
            p_detectFlag->m_u8RealSegFlag = 0;
            p_detectFlag->m_u8ReceptFlag = 0;
            p_detectFlag->m_u8ReceptNums = 0;
            p_detectFlag->m_u8DetectNoFireNums = 0;
            p_detectFlag->m_u8DetectOkFireNums = 0;
            inFire = {0, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
            if (fireROI.size()) { // 一定要先判断，否则导致二次释放内存！！！
                fireROI.clear();
            }
            goto noFire; 
        }
    } 
    // else if (p_detectFlag->m_u8ReceptFlag) { // 接受区域时
    //     // 筛选最大的框
    //     uint8 label = fireROI[0].label; 
    //     for (uint32 i=0; i<fireROI.size(); i++) {
    //         if (fireROI[label].realRect.area() < fireROI[i].realRect.area() && 
    //             fireROI[label].expdRect.area() < fireROI[i].expdRect.area()) {
    //             label = i;
    //         }
    //     }
    //     inFire = {label, fireROI[label].realRect, fireROI[label].recoRect, fireROI[label].expdRect};
    //     goto noFire; // 有框的情况
    // }
    return 0; 
isFire: // 有框有火
    p_detectFlag->m_u8RealSortFlag = 1;
    return 1;
noFire: // 无火（无框、有框无火、识别无火）=> 上续处理
    p_detectFlag->m_u8RealSortFlag = 0;
    return 1;
}


