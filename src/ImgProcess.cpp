#include "ImgProcess.hpp"
#include "ImgFeature.hpp"

int ImgProcess::Smooth(IN const Mat& inImg, OUT Mat& outImg, IN unsigned int blurMode)
{
	if (inImg.empty()) {
		cout << "Smooth Mat is empty!" << endl;
		goto erro1;
	}
    switch(blurMode){
        /* 平滑滤波 */
        case E_boxFilter:
            boxFilter(inImg, outImg, inImg.depth(), Size(3, 3), Point(-1,-1), false); // 后面1个参数都用默认值
            break;
        case E_meanFilter:
            blur(inImg, outImg, Size(10, 10));
            break;
        case E_gaussianFilter:
            GaussianBlur(inImg, outImg, Size(5, 5), 0, 0);
            break;
        case E_medianFilter:
            /* 消除椒盐噪声 */
            medianBlur(inImg, outImg, 9);
            break;
        default:
            break;
	}
	return 0;
erro1:
	return -1;
}

int ImgProcess::ColorEnhence(IN const Mat& inImg, OUT Mat& outImg)
{
	double R, G, B;
	vector<Mat>imageRGB;
	if (inImg.empty()) {
		cout << "Mat is empty!" << endl;
		goto erro1;
	}
    /* 通道颜色增强:采用白平衡(最简单) */
	//RGB三通道分离
	split(inImg, imageRGB);
	// //求原始图像的RGB分量的均值
	B = mean(imageRGB[0])[0];
	G = mean(imageRGB[1])[0];
	R = mean(imageRGB[2])[0];
	//需要调整的RGB分量的增益
	double KR, KG, KB;
	KB = (R + G + B) / (3 * B);
	KG = (R + G + B) / (3 * G);
	KR = (R + G + B) / (3 * R);
	//调整RGB三通道各自的值
	imageRGB[0] = imageRGB[0] * KB;
	imageRGB[1] = imageRGB[1] * KG;
	imageRGB[2] = imageRGB[2] * KR;
	// 增强
	equalizeHist(imageRGB[0], imageRGB[0]);
	equalizeHist(imageRGB[1], imageRGB[1]);
	equalizeHist(imageRGB[2], imageRGB[2]);

	//RGB三通道图像合并
	merge(imageRGB, outImg);
	return 0;
erro:
	return 0;
erro1:
	return -1;
}

