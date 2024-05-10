#ifndef _IMG_CORE_HPP_
#define _IMG_CORE_HPP_
#include <Head.hpp>

using namespace cv;
using namespace std;

class ImgCore {
private:
public:
    VideoCapture cap;
    Mat frameNow;   // 当前帧
    uint64 frameNum;  // 帧数
    ImgCore() {}
    ~ImgCore() {}
    int OpenVideo(IN VideoCapture& capture, IN uint8 stabalNum); // 检查是否打开正确,并稳定几帧
    int ReadImg(IN VideoCapture& capture, OUT Mat& outimg); 
    int ShowImg(IN String dialogID, IN const Mat& inImg, IN unsigned char waitFlag);
};

#endif