#include "ImgFeature.hpp"
#include "ImgProcess.hpp"

float ST = 50.00f; // RT下的ST值 (55~65)

extern Mat g_orgImg;
extern Mat g_srcImg;
extern Mat g_segImg;
extern Fire g_fire;
extern vector<Fire> g_fireVector;
extern ImgCore g_fireCore;
extern ImgProcess g_fireProcess;
extern ImgContours g_fireContours;
extern ST_DETECT_INFO g_detectFlag;

uint8 ImgFeature::GetOSTUThreshold(IN const Mat& inChImg)
{
	if (inChImg.channels() != 1) {
		cout << "GetOSTUThreshold inChImg.channels() not 1" << endl;
		goto erro1; // 跳过实例化
	}
	{
		// uint16 t_u16i, t_u16j;

		/*灰度直方图参数*/
		uint16 t_u16HistoGramAr[256];//灰度直方图  
		uint32 t_int32MinValue, t_int32MaxValue;

		uint32 t_u32Amount = 0;//像素点总数
		uint32 t_u32PixelIntegral = 0;//灰度值总数

		uint32 t_u32PixelBack = 0;//前景像素点总数
		uint32 t_u32PixelIntegralBack = 0;//前景灰度值

		int32  t_int32PixelFore = 0;//背景像素点总数
		int32  t_int32PixelIntegralFore = 0;//背景灰度值
		
		/*OSTU*/
		f32   t_F32OmegaBack, t_F32OmegaFore, t_F32MicroBack, t_F32MicroFore, t_F32SigmaB, t_F32Sigma; //类间方差：浮点型更精确
		
		uint8  t_u8Threshold = 0;
		uint8  t_u8NewThreshold = 0;//用于迭代

		/*获取灰度直方图*/ 
#		pragma omp parallel for num_threads(4)
		for (uint16 t_u16j=0; t_u16j<256; t_u16j++) {
			t_u16HistoGramAr[t_u16j] = 0;    //初始化灰度直方图
		}
		/*可以考虑压缩*/
#		pragma omp parallel for num_threads(4)
		for (uint16 t_u16j = 0; t_u16j < ROW; t_u16j++) {
			for (uint16 t_u16i = 0; t_u16i < COL; t_u16i++) {
				t_u16HistoGramAr[(uint8)inChImg.at<uchar>(t_u16j, t_u16i)]++; //统计灰度级中每个像素在整幅图像中的个数
			}
		}
		//for (uint16 t_u16j = 0; t_u16j < IMGH; t_u16j += 2)
		//{
		//	for (uint16 t_u16i = 0; t_u16i < IMGW; t_u16i += 2)
		//	{
		//		t_u16HistoGramAr[(Uint8)Img[t_u16j][t_u16i]]++; //获取灰度直方图：统计灰度级中每个像素在整幅图像中的个数
		//	}
		//}
		for (t_int32MinValue = 0; t_int32MinValue < 256 && t_u16HistoGramAr[t_int32MinValue] == 0; t_int32MinValue++) {
			;    //获取最小灰度的值
		}
		for (t_int32MaxValue = 255; t_int32MaxValue > t_int32MinValue && t_u16HistoGramAr[t_int32MinValue] == 0; t_int32MaxValue--) {
			;    //获取最大灰度的值
		}
		if (t_int32MaxValue == t_int32MinValue) {
			return((uint8)(t_int32MaxValue));    // 图像中只有一个颜色
		}
		if (t_int32MinValue + 1 == t_int32MaxValue) {
			return((uint8)(t_int32MinValue));    // 图像中只有二个颜色
		}

		/*OSTU大律法*/
		for (uint16 t_u16j = (uint16)t_int32MinValue; t_u16j <= t_int32MaxValue; t_u16j++) {
			t_u32Amount += t_u16HistoGramAr[t_u16j];    //  像素总数
		}
		t_u32PixelIntegral = 0;
		for (uint16 t_u16j = (uint16)t_int32MinValue; t_u16j <= t_int32MaxValue; t_u16j++) {
			t_u32PixelIntegral += t_u16HistoGramAr[t_u16j] * t_u16j;//灰度值总数
		}
		t_F32SigmaB = -1;
		for (uint16 t_u16j = (uint16)t_int32MinValue; t_u16j < t_int32MaxValue; t_u16j++) {
			t_u32PixelBack = t_u32PixelBack + t_u16HistoGramAr[t_u16j];    //前景像素点数
			t_int32PixelFore = t_u32Amount - t_u32PixelBack;         //背景像素点数
			t_F32OmegaBack = (f32)t_u32PixelBack / t_u32Amount;//前景像素百分比
			t_F32OmegaFore = (f32)t_int32PixelFore / t_u32Amount;//背景像素百分比
			t_u32PixelIntegralBack += t_u16HistoGramAr[t_u16j] * t_u16j;  //前景灰度值
			t_int32PixelIntegralFore = t_u32PixelIntegral - t_u32PixelIntegralBack;//背景灰度值
			t_F32MicroBack = (f32)t_u32PixelIntegralBack / t_u32PixelBack;   //前景灰度百分比
			t_F32MicroFore = (f32)t_int32PixelIntegralFore / t_int32PixelFore;   //背景灰度百分比
			t_F32Sigma = t_F32OmegaBack * t_F32OmegaFore * (t_F32MicroBack - t_F32MicroFore) * (t_F32MicroBack - t_F32MicroFore);//计算类间方差
			if (t_F32Sigma > t_F32SigmaB) {                   //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
				t_F32SigmaB = t_F32Sigma;
				t_u8Threshold = (uint8)t_u16j;
			}
		}
		return t_u8Threshold;                        //返回最佳阈值;
	}
erro1:
	return -1;
}

int ImgFeature::BinarySege(IN const Mat& inImg, OUT Mat& outImg, IN ST_DETECT_INFO *p_detectFlag)
{
	if (inImg.empty()) {
		cout << "BinarySege: inImg is null" << endl;
		goto erro1;
	}
	{
		Mat temImg;  // 中间过程
		Mat srcImg; 
		if (p_detectFlag->m_u8ReceptFlag) {
			// 局部(上帧发现火源=>筛选后的一个区域（最大）
			if (g_fire.expdRect.area()>0) {
				temImg = inImg(g_fire.expdRect).clone();
				CheckLocalColor(temImg, outImg, g_fire.expdRect);
			}
		} else {
			// 全局
			temImg = inImg.clone();
			CheckGlobalColor(temImg, outImg);
		}
		// erode(detImg, detImg, Mat(3, 3, CV_8UC1));
		// g_fireProcess.Smooth(detImg, outImg, E_medianFilter); // 中值滤波
		// dilate(detImg, detImg, Mat(5, 5, CV_8UC1), Point(-1, -1), 3); // 6次
		cout << "BinarySege ok" << endl;
		return 0;
	}
exit:
	return 0;
erro1:
	return -1;
}

int ImgFeature::CheckLocalColor(IN const Mat& inImg, OUT Mat& outImg, IN Rect& inRect)
{
	if (inImg.empty() || inRect.area() <= 0) {
		if (inImg.empty()) {
			cout << "CheckLocalColor:inImg is null" << endl;
			goto erro1;
		} 
		if (!inRect.area()) {
			cout << "CheckLocalColor:inRect is null" << endl;
			goto erro2;
		}
	}
	{
		Mat srcImg; 
		vector<Mat> multiRGB;
		int x0 = inRect.tl().x; 
		int y0 = inRect.tl().y;
		int x1 = inRect.br().x; 
		int y1 = inRect.br().y; 
		srcImg = inImg.clone(); 
		split(srcImg, multiRGB); // RGB
#ifdef BUG_INFO
		cout << "[INFO-LOCAL] g_fire.expdRect x0: " << x0 << " ; g_fire.expdRect x1: " << x1 << endl;
		cout << "[INFO-LOCAL] g_fire.expdRect y0: " << y0 << " ; g_fire.expdRect y1: " << y1 << endl;
#endif
		uint8 RT = GetOSTUThreshold(multiRGB[2]); // OSTU
		if (RT == 0) {
			cout << "[WARN] CheckLocalColor: GetOSTUThreshold RT is 0" << endl;
			goto exit;
		} else if (RT < 0) {
			cout << "[ERRO] CheckLocalColor: GetOSTUThreshold RT erro" << endl;
			goto erro3;
		} else {	
			float threshold = float(ST/RT);
			printf("[INFO-LOCAL] ST: %0.2f \n", ST);
			printf("[INFO-LOCAL] RT: %d \n", RT);
			printf("[INFO-LOCAL] Threshold (ST/RT): %0.2f \n", threshold);
#			pragma omp parallel for num_threads(4)
			for (int i=y0; i<y1; i++) {
				for (int j=x0; j<x1; j++) {
					float B = multiRGB[0].at<uchar>(i-y0, j-x0);
					float G = multiRGB[1].at<uchar>(i-y0, j-x0);
					float R = multiRGB[2].at<uchar>(i-y0, j-x0);
					// HSI=>S
					float maxValue = max(max(B, G), R);
					float minValue = min(min(B, G), R);
					double S = (1 - (3.0 * minValue / (R + G + B)));
					// RT (115~135)
					// ST (55~65)
					double SE = (255 - R) * threshold; // SE means Evaluation variable: ST is value under RT, R with negtive relation 
					
					// if (i == j) {
					// 	printf("[CHECK-Local] S: %0.2f \n", float(S)); // means satuation
					// 	printf("[CHECK-Local] SE: %0.2f \n", float(SE));
					// }
					
					// R > RT  R>=G>=B  S>=((255-R)*(ST/RT))  
					if (S > SE && // satuation
						R >= RT &&
						R > G && G > B) { 
						outImg.at<uchar>(i, j) = 255; 
					} else {
						outImg.at<uchar>(i, j) = 0;
					}
				}
			}
		}
		return 0;
	}
exit:
	return 0;
erro1:
	return -1;
erro2:
	return -2;
erro3:
	return -3;
}

int ImgFeature::CheckGlobalColor(IN const Mat& inImg, OUT Mat& outImg)
{
	if (inImg.empty()) {
		cout << "CheckGlobalColor: inImg is null" << endl;
		goto erro1;
	} 
	{
		Mat srcImg; 
		vector<Mat> multiRGB;
		srcImg = inImg.clone();
		split(srcImg, multiRGB); // RGB
#ifdef BUG_INFO
		cout << "[INFO-LOCAL] g_fire.expdRect x0: " << x0 << " ; g_fire.expdRect x1: " << x1 << endl;
		cout << "[INFO-LOCAL] g_fire.expdRect y0: " << y0 << " ; g_fire.expdRect y1: " << y1 << endl;
#endif
		uint8 RT = GetOSTUThreshold(multiRGB[2]); // OSTU
		if (RT == 0) {
			cout << "[WARN] CheckGlobalColor: GetOSTUThreshold RT is 0" << endl;
			goto exit;
		} else if (RT < 0) {
			cout << "[ERRO] CheckGlobalColor: GetOSTUThreshold RT erro" << endl;
			goto erro2;
		} else {	
			float threshold = float(ST/RT);  
			printf("[INFO-GLOBAL] ST: %0.2f \n", ST);
			printf("[INFO-GLOBAL] RT: %d \n", RT);
			printf("[INFO-GLOBAL] Threshold (ST/RT): %0.2f \n", threshold);
#			pragma omp parallel for num_threads(4)
			for (int i=0; i<ROW; i++) {
				for (int j=0; j<COL; j++) {
					float B = multiRGB[0].at<uchar>(i, j);
					float G = multiRGB[1].at<uchar>(i, j);
					float R = multiRGB[2].at<uchar>(i, j);
					// HSI=>S
					float maxValue = max(max(B, G), R);
					float minValue = min(min(B, G), R);
					double S = (1 - (3.0 * minValue / (R + G + B)));
					// RT (115~135)
					// ST (55~65)
					double SE = (255 - R) * threshold; // SE means Evaluation variable: ST is value under RT, R with negtive relation 
					
					// cout << "i=" << i << "; j=" << j << endl;
					// if (i == j) {
					// 	printf("[CHECK-Global] S: %0.2f \n", float(S)); // means satuation
					// 	printf("[CHECK-Global] SE: %0.2f \n", float(SE));
					// }

					// R > RT  R>=G>=B  S>=((255-R)*(ST/RT))  
					if (S > SE && // satuation
						R >= RT &&
						R >= G && G >= B) { 
						outImg.at<uchar>(i, j) = 255; 
					} else {
						outImg.at<uchar>(i, j) = 0;
					}
				}
			}
		}
		return 0;
	}
exit:
	return 0;
erro1:
	return -1;
erro2:
	return -2;
}

int ImgContours::SizerContours(IN const Mat& inImg, IN_OUT vector<vector<Point>>& inContours, IN vector<Vec4i>& inHierarchy)
{
	if (!inContours.size() || !inHierarchy.size()) {
		cout << "SizerContours is null" << endl;
		goto exit;
	} else if (inContours.size() != inHierarchy.size()) {
		cout << "SizerContours=>findContours is null" << endl;
		goto erro1;
	}
	{
		vector<vector<Point>> outContours = {};
		vector<vector<Point>> largestContours = {};
		// 检查白色轮廓（有效性）
		for (uint32 i=0; i<inContours.size(); i++) {
			if (inHierarchy[i][3]<0) {
				// 没有最外层轮廓
				outContours.push_back(inContours[i]); // 最外层
			}
		}
		// 提取前CLASS个最大面积的轮廓
		// 排序
		sort(outContours.begin(), outContours.end(), CompareContourAreas);
		// 提取前CLASS个最大面积的轮阔
		for (uint32 i=0; i<CLASS && i<outContours.size(); i++) {
			largestContours.push_back(outContours[i]);
		}
		if (largestContours.size()) {
			inContours.clear();
			for (uint32 i=0; i<largestContours.size(); i++) {
				inContours.push_back(largestContours[i]);
			}
		}
		return 0;
	}
exit:
	return 0;	
erro1:
	return -1;
}


unordered_set<int> dp; // 用于记录已经遍历过的点
// 可以优化为模板函数 (vector<vector<ImgContours>>& mergeContours)[N]
int ImgContours::MergeContours(IN const Mat& inImg, IN_OUT vector<vector<ImgContours>>& mergeContours) 
{
	if (inImg.empty() || mergeContours.size()) {
		// 先检查 IN_OUT => 清空
		if (mergeContours.size()) {
			for (uint32 i=0; i<mergeContours.size(); i++) {
				mergeContours[i].clear(); // 清空
			}
		}
		// 检查 IN
		if (inImg.empty()) {
			goto exit;
		}
	}
	{
		int ret = 0;
		Moments M;
		Point2f tempP = {0.0f, 0.0f};
		vector<ImgContours> tempContours = {};
		vector<vector<Point>> contours = {};//膨胀轮廓点集（几个轮廓=>几个点集合）
		vector<Vec4i> hierarchy = {};
		// CHAIN_APPROX_NONE:存储所有边界点；
		// CHAIN_APPROX_SIMPLE:只存储有用的点，去掉冗余点，压缩轮廓。
		findContours(inImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		SizerContours(inImg, contours, hierarchy); // 筛选一次,最外层的轮廓
#ifdef DISPLAY_DEBUG
		drawContours(g_srcImg, contours, -1, Scalar(0, 255, 0), 3); // g
#endif

		// auto constexpression = [&contours]() -> const int { return contours.size();};
		// constexpr int contours_class = (const int)constexpression;

		if (!contours.size()) {
			goto exit;
		} else {
			// 求轮廓重心(点数少易错)，重构数据结构
			for (uint32 i=0; i<contours.size(); i++) {
				if (contours[i].size() > 1000) {
					// 超过10个点求重心
					M = moments(contours[i]);
					tempP.x = double(M.m10 / M.m00);
					tempP.y = double(M.m01 / M.m00);
				} else {
					// 点数少求均值
					tempP = GetMean(contours[i]); 
				}
				if (!isnan(tempP.x) && !isnan(tempP.y)) {
					tempContours.emplace_back(i, tempP, contours[i]); // 0,1,2...
				} else {
					cout << "[ERRO] Moments clac erro" << endl;
					goto erro1;
				}
			}
			// 合并轮廓
			if (contours.size() == 1) {
				mergeContours[0].emplace_back(tempContours[0].label, tempContours[0].central, tempContours[0].points); // 只有一个轮廓
			} else {
				if (dp.size()) {
					dp.clear();
				}
				uint8 iClass = 0;
				while(tempContours.size()) {
					// ret = Recursion(tempContours[i], tempContours, mergeContours, iClass++, 100.0);
					ret = Recursion(tempContours[0], tempContours, mergeContours, iClass++, TR);
					if (ret != 0) {
						cout << "[ERRO-findContours] Recursion erro" << endl;
						goto erro2;
					}
					// 删除已经遍历的点
					for (uint32 j=0; j<tempContours.size(); ) {
						if (dp.find(tempContours[j].label) != dp.end()) {
							tempContours.erase(tempContours.begin() + j); // 删除当前点（拷贝/引用需要恢复）
						} else {
							j++;
						}
					}
				}
			}
		}
		return 0;
	}
exit:
	return 0;
erro1:
	return -1;
erro2:
	return -2;
}

int ImgContours::Recursion(IN ImgContours& P, IN vector<ImgContours>& inTours, IN_OUT vector<vector<ImgContours>>& outTours, IN uint8 iClass, IN const float T)
{
	if (!P.points.size() || !inTours.size()) {
		goto exit;
	}
	if (iClass > CLASS) {
		cout << "[ERRO] RecursionProcess erro" << endl;
		goto erro1;
	} 
	{
		int num = inTours.size();
		vector<ImgContours> tempTours = {}; // 拷贝
		for (int i = 0; i < num; i++) {
			tempTours.emplace_back(inTours[i]); // 拷贝
		}
		// 递归
		if (!num) {
			goto exit; // 递归终止
		} else {
			for (int j=0, index=0; j<num; j++) {
				if (dp.find(inTours[j].label) != dp.end()) {
					continue; // 找到，已经遍历过
				}
				double dest = GetEuclidean(P.central, inTours[j].central);
				if (dest < T) {
					dp.emplace(inTours[j].label);
					tempTours.erase(tempTours.begin() + (j-index++)); // 后index++（细节） 删除当前点（拷贝/引用需要恢复）：此处存在坐标对应关系
					if (P.label == tempTours[j].label) { // 自己
						outTours[iClass].emplace_back(inTours[j].label, inTours[j].central, inTours[j].points);
						continue;
					} else {
						outTours[iClass].emplace_back(inTours[j].label, inTours[j].central, inTours[j].points);
						Recursion(inTours[j], tempTours, outTours, iClass, T);
					}
				}
			}
		}
		return 0;
	}
exit:
	return 0;
erro1:
	return -1;
}

int ImgContours::ExtractRoi(IN vector<vector<ImgContours>>& inContours, IN_OUT vector<Fire>& fireROI)
{
	if (!inContours[0].size()) {
		goto erro;
	}
	if (fireROI.size()) { // 一定要先释放
		fireROI.clear();
	}
	{
		vector<Point> points = {};
		vector<vector<Point>>::iterator iter;
		vector<vector<Point>> contoursPoints = {}; // 空
		for (uint32 i=0; i<inContours.size(); i++) {
			points.clear(); // 注意清空
			if (!inContours[i].size()) {
				break; // 小加速
			} 
			for (uint32 j=0; j<inContours[i].size(); j++) {
				points.insert(points.end(),
							inContours[i][j].points.begin(),
							inContours[i][j].points.end());
			}
			contoursPoints.emplace_back(points);
		}
		iter = contoursPoints.begin(); // contours_merge
		for (uint32 i=0; iter!=contoursPoints.end(); iter++, i++) {
			Rect realRect = boundingRect(*iter);
			Rect recoRect = {0, 0, 0, 0};
			Rect expdRect = {0, 0, 0, 0};
			if (realRect.area() > 0) { 
				if (RoiRecognize(realRect, recoRect) != 0) {
					cout << "[ERRO] ExtractRoi=>RoiRecognize is erro" << endl;
					goto erro1;
				}
				if (RoiExpanded(realRect, expdRect) != 0) {
					cout << "[ERRO] ExtractRoi=>RoiExpanded is erro" << endl;
					goto erro2;
				}
				fireROI.emplace_back(i, realRect, recoRect, expdRect);
			} else {
				cout << "[ERRO] ExtractRoi realRect area < 0" << endl;
				goto erro3;
			}
		}
		if (fireROI.size()) {
			goto ok;
		}
		return 0;
	}
ok:
	return 1;
erro:
	return 0;
erro1:	
	return -1;
erro2:	
	return -2;
erro3:
	return -3;
}

int ImgContours::RoiCheck(IN_OUT Rect& inRect)
{
	if (inRect.area() <= 0) {
		if (inRect.area() == 0) {
			cout << "[WARN] RoiCheck:rect is null, not should introduction!" << endl;
			goto exit;
		} else if (inRect.area() < 0) {
			cout << "[ERRO] RoiCheck:rect area null < 0" << endl;
			goto erro1;
		}
	} else {
		// 检查坐标值
		int x0 = inRect.tl().x;
		int y0 = inRect.tl().y;
		// rect maybe erro
		int x1 = inRect.br().x; // rect's width and heigh is x0+width and y0+heigh
		int y1 = inRect.br().y; // rect's width and heigh is x0+width and y0+heigh
		if(x0 < 0) {
			cout << "[INFO] RoiCheck:Rect x0 < 0: " << endl;
			goto erro2;
		}
		if (y0 < 0) {
			cout << "[INFO] RoiCheck:Rect y0 < 0: " << endl;
			goto erro2;
		} 
		if (x1 > COL-1) {
			cout << "[INFO] RoiCheck:Rect x1 > COL-1: " << endl;
			goto erro2;
		}
		if (y1 > ROW-1) {
			cout << "[INFO] RoiCheck:Rect y1 > ROW-1: " << endl;
			goto erro2;
		}
		if (x0>x1 && y0>y1) {
			cout << "[ERRO] RoiCheck:Rect is illegal: " << endl;
			goto erro3;
		}
	}
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

int ImgContours::RoiCheck(IN_OUT Rect& inRect, IN int x0, IN int y0, IN int x1, IN int y1)
{
	if (inRect.area() <= 0) {
		if (inRect.area() == 0) {
			cout << "RoiCheck:rect is null, not should introduction!" << endl;
			goto exit;
		} else if (inRect.area() < 0) {
			cout << "RoiCheck:rect area null < 0" << endl;
			goto erro1;
		}
	} else {
		// 检查坐标值
		if(inRect.tl().x < 0) {
			x0 = 0;
		} else {
			x0 = inRect.x;
		}
		if (inRect.tl().y < 0) {
			y0 = 0;
		} else {
			y0 = inRect.y;
		}
		if (inRect.br().x > COL-1) {
			x1 = COL-1;
		} else {
			x1 = inRect.x + inRect.width; // 在OpenCV中Rect是这样计算右下点的坐标的
		}
		if (inRect.br().y > ROW-1) {
			y1 = ROW-1;
		} else { 
			y1 = inRect.y + inRect.height; // 在OpenCV中Rect是这样计算右下点的坐标的
		}

		if (x0==x1 || y0==y1) {
			cout << "[WARN] Rect area is empty!!!" << endl;
			goto exit;
		} else if (x1>x0 && y1>y0) {
			inRect = {x0, y0, x1-x0, y1-y0}; // 修正
		} else if (x1<x0 && y1<y0) {
			inRect = {x1, y1, x0-x1, y0-y1}; // 修正
		} else if (x1<x0 || y1<y0) {
			if (x1<x0) {
				inRect = {x1, y0, x0-x1, y1-y0}; // 修正
			}
			if (y1<y0) {
				inRect = {x0, y1, x1-x0, y0-y1}; // 修正
			}
		} else {
			cout << "[ERRO] Rect is erro" << endl;
			goto erro2;
		}
	}
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

int ImgContours::RoiRecognize(IN Rect& inRect, IN_OUT Rect& outRect)
{
	{
		int ret = RoiCheck(inRect, inRect.tl().x, inRect.tl().y, inRect.br().x, inRect.br().y);
		if (ret != 1) {
			if (ret == 0) {
				goto exit;
			} 
			if (ret < 0) {
				std::cout << "RoiExpanded=>RoiCheck is erro" << endl;
				goto erro1;
			}	
		}
		if (outRect.area()) {
			outRect = {0, 0, 0, 0};
		}
	}
	{
		int cX = 0, cY = 0;
		int point_x = 0, point_y = 0;
		int roiWidth  = CNN_COL;
		int roiHeight = CNN_ROW;
		// 确定检测框大小
		if (inRect.width > roiWidth) {
			roiWidth = inRect.width; 
		}
		if (inRect.height > roiHeight) {
			roiHeight = inRect.height;
		}
		// 中心 毛糙点更好
		cX = inRect.x + (inRect.width/2);
		cY = inRect.y + (inRect.height/2);
		point_x = int(cX - (roiWidth/2));
		point_y = int(cY - (roiHeight/2));
		// 限幅处理(顺序不能错，先减后判，避免roiwidth=COL)
		if (point_x < 0) 
			point_x = 0;
		if (point_y < 0) 
			point_y = 0;
		if (point_x + roiWidth > COL-1) {
			if (roiWidth >= COL) {
				point_x = 0; // 避免减出来为负的情况
			} else {
				point_x = COL - roiWidth - 1;
			}
		}
		if (point_y + roiHeight > ROW-1) {
			if (roiHeight >= ROW) {
				point_y = 0; // 避免减出来为负的情况
			} else {
				point_y = ROW - roiHeight - 1;
			}
		}
		outRect = {point_x, point_y, roiWidth, roiHeight};
		return 0;
	}
exit:
	return 0;
erro1:
	return -1;
}

int ImgContours::RoiExpanded(IN Rect& inRect, IN_OUT Rect& outRect)
{	
	{
		int ret = RoiCheck(inRect, inRect.tl().x, inRect.tl().y, inRect.br().x, inRect.br().y);
		if (ret != 1) {
			if (ret == 0) {
				goto exit;
			} 
			if (ret < 0) {
				std::cout << "RoiExpanded=>RoiCheck is erro" << endl;
				goto erro1;
			}	
		}
		if (outRect.area()) {
			outRect = {0, 0, 0, 0};
		}
	}
	{
		Point p0 = inRect.tl(); // 首点
		Point p1 = inRect.br(); // 尾点
		ImgBlocks b0 = {p0, 0}; // 首点 0
		ImgBlocks b1 = {p1, 1}; // 尾点 1
		int x0 = 0, y0 = 0, x1 = 0, y1 = 0; // width = 0, height = 0;
		if (b0.pIndex[0] == b1.pIndex[0] && b0.pIndex[1] == b1.pIndex[1]) {
			// 同一块(扩一点更好)
			x0 = b0.pBlock.tl().x-2;
			y0 = b0.pBlock.tl().y-2;
			x1 = b0.pBlock.br().x+2;
			y1 = b0.pBlock.br().y+2;
		} else if (b0.pIndex[0] == b1.pIndex[0] || b0.pIndex[1] == b1.pIndex[1]) {
			if (b0.pIndex[0] == b1.pIndex[0]) {
				// 在一排
				if (b0.pResult && b1.pResult) {
					x0 = b0.pBlock.tl().x-2;
					y0 = b0.pBlock.tl().y-2;
					x1 = b1.pBlock.br().x+2;
					y1 = b1.pBlock.br().y+2;
				} else if (b0.pResult || b1.pResult) {
					if (b0.pResult) {
						x0 = b0.pBlock.tl().x-2;
						y0 = b0.pBlock.tl().y-2;
						x1 = b1.pBlock.br().x+2 - b1.pBlock.width/2;
						if (p1.y > b1.pBlock.br().y - b1.pBlock.height/2) {
							y1 = b1.pBlock.br().y+2;
						} else {
							y1 = b1.pBlock.br().y+2 - b1.pBlock.height/2;
						}
					}
					if (b1.pResult) {
						x1 = b1.pBlock.br().x+2;
						y1 = b1.pBlock.br().y+2;
						x0 = b0.pBlock.tl().x-2 + b0.pBlock.width/2;
						if (p0.y > b0.pBlock.tl().y + b0.pBlock.height/2) {
							y0 = b0.pBlock.tl().y-2 + b0.pBlock.height/2;
						} else {
							y0 = b0.pBlock.tl().y-2; // 靠上
						}
					}
				} else {
					// 都不满足
					x0 = b0.pBlock.tl().x-2;
					y0 = b0.pBlock.tl().y-2 + b0.pBlock.height/2;
					x1 = b1.pBlock.br().x+2;
					y1 = b1.pBlock.br().y+2 - b0.pBlock.height/2;
				}
			}
			if (b0.pIndex[1] == b1.pIndex[1]) {
				// 在一列
				if (b0.pResult && b1.pResult) {
					x0 = b0.pBlock.tl().x-2;
					y0 = b0.pBlock.tl().y-2;
					x1 = b1.pBlock.br().x+2;
					y1 = b1.pBlock.br().y+2;
				} else if (b0.pResult || b1.pResult) {
					if (b0.pResult) {
						x0 = b0.pBlock.tl().x-2;
						y0 = b0.pBlock.tl().y-2;
						y1 = b1.pBlock.br().y+2 - b1.pBlock.height/2;
						if (p1.x > b1.pBlock.br().x - b1.pBlock.width/2) {
							x1 = b1.pBlock.br().x+2;
						} else {
							x1 = b1.pBlock.br().x+2 - b1.pBlock.width/2;
						}
					}
					if (b1.pResult) {
						x1 = b1.pBlock.br().x+2;
						y1 = b1.pBlock.br().y+2;
						y0 = b0.pBlock.tl().y-2 + b0.pBlock.height/2;
						if (p0.x > b0.pBlock.tl().x + b0.pBlock.width/2) {
							x0 = b0.pBlock.tl().x-2 +b0.pBlock.width/2;
						} else {
							x0 = b0.pBlock.tl().x-2; // 靠上
						}
					}
				} else {
					// 都不满足
					x0 = b0.pBlock.tl().x-2 + b0.pBlock.width/2;
					y0 = b0.pBlock.tl().y-2;
					x1 = b1.pBlock.br().x+2 - b1.pBlock.width/2;
					y1 = b1.pBlock.br().y+2;
				}
			}
		} else {
			// 不在一行也不在一列
			if (b0.pResult && b1.pResult) {
				x0 = b0.pBlock.tl().x-2;
				y0 = b0.pBlock.tl().y-2;
				x1 = b1.pBlock.br().x+2;
				y1 = b1.pBlock.br().y+2;
			} else if (b0.pResult || b1.pResult) {
				if (b0.pResult) {
					x0 = b0.pBlock.tl().x-2;
					y0 = b0.pBlock.tl().y-2;
					if (p1.x<b1.pBlock.width/2 && p1.y<b1.pBlock.height/2) {
						x1 = b1.pBlock.br().x+2 - b1.pBlock.height/2;
						y1 = b1.pBlock.br().y+2 - b1.pBlock.height/2;
					} else if (p1.x<b1.pBlock.width/2 || p1.y<b1.pBlock.height/2) {
						if (p1.x<b1.pBlock.width/2) {
							x1 = b1.pBlock.br().x+2 - b1.pBlock.height/2;
							y1 = b1.pBlock.br().y+2;
						}
						if (p1.y<b1.pBlock.height/2) {
							x1 = b1.pBlock.br().x+2;
							y1 = b1.pBlock.br().y+2 - b1.pBlock.height/2;
						}
					} else {
						x1 = b1.pBlock.br().x+2;
						y1 = b1.pBlock.br().y+2;
					}
				}
				if (b1.pResult) {
					x1 = b1.pBlock.br().x+2;
					y1 = b1.pBlock.br().y+2;
					if (p0.x>b0.pBlock.width/2 && p0.y>b0.pBlock.height/2) {
						x0 = b0.pBlock.tl().x-2 + b0.pBlock.height/2;
						y0 = b0.pBlock.tl().y-2 + b0.pBlock.height/2;
					} else if (p0.x>b0.pBlock.width/2 || p0.y>b0.pBlock.height/2) {
						if (p0.x>b0.pBlock.width/2) {
							x0 = b0.pBlock.tl().x-2 + b0.pBlock.height/2;
							y0 = b0.pBlock.tl().y-2;
						}
						if (p0.y>b0.pBlock.height/2) {
							x0 = b0.pBlock.tl().x-2;
							y0 = b0.pBlock.tl().y-2 + b0.pBlock.height/2;
						}
					} else {
						x0 = b1.pBlock.tl().x-2;
						y0 = b1.pBlock.tl().y-2;
					}
				}
			} else {
				x0 = b0.pBlock.tl().x-2 + b0.pBlock.height/2;
				y0 = b0.pBlock.tl().y-2 + b0.pBlock.height/2;
				x1 = b1.pBlock.br().x+2 - b1.pBlock.height/2;
				y1 = b1.pBlock.br().y+2 - b1.pBlock.height/2;
			}
		}
		// 限幅处理(必须)
		outRect = {x0, y0, x1-x0, y1-y0};
		if (RoiCheck(outRect, outRect.tl().x, outRect.tl().y, outRect.br().x, outRect.br().y) != 1) {
			cout << "RoiExpanded=>RoiExpanded is erro" << endl;
			goto erro2;
		}
		cout << "[INFO-RoiExpanded] b0.pResult: " << int(b0.pResult) << endl;
		cout << "[INFO-RoiExpanded] b1.pResult: " << int(b1.pResult) << endl;
		cout << "[INFO-RoiExpanded] x0=" << outRect.tl().x << " x1=" << outRect.br().x <<endl;
		cout << "[INFO-RoiExpanded] y0=" << outRect.tl().y << " y1=" << outRect.br().y <<endl;
		return 0;
	}
exit:
	return 0;
erro1:
	return -1;
erro2:
	return -2;
}

int ImgContours::DrawContours(IN vector<Fire>& fireROI, IN_OUT Mat& outImg)
{
	if (!fireROI.size() || outImg.empty()) {
		// 先清空
		if (!fireROI.size()) {
			cout << "DrawContours: init is null" << endl;
			goto exit; // 正常退出
		}
		if (outImg.empty()) {
			cout << "DrawContours: Img is empty" << endl;
			goto erro1;
		}
	}
	{
#if DISPLAY_DEBUG 
		Point x1_start(int(1*BLOCK_COL), 0);
		Point x1_end(int(1*BLOCK_COL), ROW-1);
		Point x2_start(int(2*BLOCK_COL), 0);
		Point x2_end(int(2*BLOCK_COL), ROW-1);
		Point y1_start(0, int(1*BLOCK_ROW));
		Point y1_end(COL-1, int(1*BLOCK_ROW));
		Point y2_start(0, int(2*BLOCK_ROW));
		Point y2_end(COL-1, int(2*BLOCK_ROW));
		line(outImg, x1_start, x1_end, Scalar(125,125,125), 5);
		line(outImg, x2_start, x2_end, Scalar(125,125,125), 5);
		line(outImg, y1_start, y1_end, Scalar(125,125,125), 5);
		line(outImg, y2_start, y2_end, Scalar(125,125,125), 5);
#endif
		for (uint32 i=0; i<fireROI.size(); i++) {
			if (g_detectFlag.m_u8RealSegFlag) {
				// 实际检测到
#if BUG_CAM_SAVE
				cout << "save breakpoint" << endl;
				cout << "recoRect.tl().x: " << fireROI[i].recoRect.tl().x << endl;
				cout << "recoRect.tl().y: " << fireROI[i].recoRect.tl().y << endl;
				cout << "recoRect.br().x: " << fireROI[i].recoRect.br().x << endl;
				cout << "recoRect.br().y: " << fireROI[i].recoRect.br().y << endl;
				Mat tmpImg = g_orgImg(fireROI[i].recoRect).clone();
				g_fireCore.ShowImg("tmpImg save", tmpImg, 1);
#endif
#if DISPLAY_DEBUG
				rectangle(g_srcImg, fireROI[i].realRect, Scalar(0, 0, 255), 2); // r
#endif
#if DISPLAY_DEBUG
				rectangle(outImg, fireROI[i].realRect, Scalar(0, 0, 255), 2); // r
				rectangle(outImg, fireROI[i].recoRect, Scalar(255, 0, 0), 2); // b
#endif
			}
			if (g_detectFlag.m_u8ReceptFlag) {
				// 接受时间
				// 上帧仍然有保留
				// rectangle(outImg, fireROI[i].realRect, Scalar(0, 0, 255), 2); // r
				// rectangle(outImg, fireROI[i].recoRect, Scalar(255, 0, 0), 2); // b
#if DISPLAY_DEBUG
				rectangle(outImg, fireROI[i].expdRect, Scalar(0, 255, 0), 2); // g
#endif
			}
		}
		return 0;
	}
exit:
	return 0;
erro1:
	return -1;
}
