#include <ImgCore.hpp>

int ImgCore::OpenVideo(IN VideoCapture& capture, IN uint8 stabalNum)
{
    if (!capture.isOpened()) {
        cout << "[ERRO] Error opening video file" << endl;
        return -1;
    } else {
        // 打开成功，配置参数
        cout << "[INFO] opening video file ok!" << endl;
        capture.set( 3 /* CV_CAP_PROP_FRAME_WIDTH */, COL);
        capture.set( 4 /*CV_CAP_PROP_FRAME_HEIGHT */, ROW);
        // 检查参数
        double rate  = capture.get(CAP_PROP_FPS);//帧率
        long totalFrameNumber = capture.get(CAP_PROP_FRAME_COUNT); // 总帧数（视频）
        int WIDTH  = capture.get(CAP_PROP_FRAME_WIDTH);//  宽度
        int HEIGHT = capture.get(CAP_PROP_FRAME_HEIGHT);// 高度
        cout << "[INFO] 帧率为:" << rate << endl;
        cout << "[INFO] 总帧数:" << totalFrameNumber << endl;
        cout << "[INFO] 宽度为:" << WIDTH << endl;
        cout << "[INFO] 高度为:" << HEIGHT << endl;
    } 
    while (stabalNum--) {
        cap.read(this->frameNow); // 首帧
        this->frameNum++;
        continue;
    }
    return 0;
}

int ImgCore::ReadImg(IN VideoCapture& capture, OUT Mat& outImg)
{
    capture >> this->frameNow;
    this->frameNum++;
    if(frameNow.empty()) {          //判断输入的视频帧是否为空的
        cout << "[ERRO] frame erro" << endl;
        return -1;
    } else {
        cout << "[INFO] frame ok" << endl;
        cout << "[INFO] frameNum:" << this->frameNum << endl;
        outImg = this->frameNow.clone();
        if (this->frameNum > 1000) {
            this->frameNum = 0;
        }
    }
    return 0;
}

int ImgCore::ShowImg(IN String dialogID, IN const Mat& inImg, IN unsigned char waitFlag)
{
#if DISPLAY_DEBUG
    char ckey = 0;
	namedWindow(dialogID, WINDOW_NORMAL);
	imshow(dialogID, inImg);
	if (waitFlag) {
        ckey = waitKey(0);
        // 扫描键盘
        switch (ckey) {
            case 27: // 按下Esc键则退出程序  
                destroyAllWindows();
                return 0; // 退出
                break;
            case 115: //按下s(小写)
                cout << "[INFO] Please input y or n" << endl;
                ckey = waitKey(0);
                if (ckey == 121) {
                    // 按下 y
                    cout << "[INFO] save in PositiveSample" << endl;

                } else if (ckey == 110) {
                    // 按下 n
                    cout << "[INFO] save in NegativeSample" << endl;
    
                } else if (ckey == 13 || ckey == 32) {
                    // 按下回车或空格
                    cout << "[INFO] exit" << endl;
                } else {
                    cout << "[INFO] input erro" << endl;
                    goto erro1;
                }
            case 13:  // 按下回车:跳帧        
            case 32:  // 按下空格:跳帧        
                destroyWindow(dialogID);
                break;
            default:
                cout << "[ERRO] input erro" << endl;
                goto erro1;
                break;
        }
    }
erro1:
    return -1;
#endif
}
