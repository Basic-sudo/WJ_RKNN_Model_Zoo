#ifndef _IMG_PROCESS_HPP_
#define _IMG_PROCESS_HPP_
#include <Head.hpp>
#include "ImgCore.hpp"

using namespace cv;
using namespace std;

typedef enum {
    E_boxFilter = 0,
    E_meanFilter,
    E_gaussianFilter,
    E_medianFilter,
} ENUM_BLUR;

class ImgProcess : public ImgCore {
private:
public:
    ImgProcess() {}
    ~ImgProcess() {}
    int Smooth(IN const Mat& inImg, OUT Mat& outImg, IN unsigned int blurMode);
    int ColorEnhence(IN const Mat& inImg, OUT Mat& outImg);
};

#endif