#include <Head.hpp>
#include "ImgCore.hpp"
#include "ImgProcess.hpp"
#include "ImgFeature.hpp"
#include "rknn_api.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <queue>
#include <string>
#include <sstream>
#include <opencv2/dnn/dnn.hpp>
#include "thread"

using namespace std;
using namespace cv;
rknn_context ctx;

#if BILI
Mat MLX_IMG(SENSOR_H*2, SENSOR_W*2, CV_8UC1);
#else
Mat MLX_IMG(SENSOR_H, SENSOR_W, CV_8UC1); // 黑白/热力
#endif

int Filter = 10;
int ret; // 状态
uint32 tX; uint32 tX_mean;
uint32 tY; uint32 tY_mean;
const uint8_t HEADER = 'A';
const uint8_t FOOTER = 'B';
// 中间图
Mat G_orgImg;
Mat G_srcImg;
Mat G_segImg;
Mat G_outImg;
// 对象(饿汉模式)
ImgCore G_fireCore;
ImgProcess G_fireProcess;
ImgFeature G_fireFeature;
ImgContours G_fireContours;
Fire G_fire; // 最后筛选的火焰区域
vector<Fire> G_fireVector;
vector<vector<ImgContours>> G_contours(CLASS);
queue<uint32> tXFilter;
queue<uint32> tYFilter;

//1.open camera mlx and port
SerialPort port("/dev/ttyS4");
void receive_thread(void) {
    std::cout<<"receive_thread start"<<endl;
}

uint8 G_receive = 0;
uint8 G_buffer[12]; // 1120 0240 ys
template<typename T, int N>



int SetUp(void) {   
    
    int rknn_fd = -1;

// 初始化RKNN环境
    ret = rknn_init(&ctx, "yolov8.rknn", 0, 0);
     if (ret < 0) {
        print ("rknn_init fail! ret=%d\n", ret);
        goto erro4;
    }
    ret = MLX90640Init();
    if (ret != 0) {
        std::cout << "[ERRO] MLX90640Init erro" << endl;
        goto erro1;
    }
    ret = G_fireProcess.OpenVideo(cap, 100); // 稳定100帧(30帧一秒)
    if (ret != 0) {
        std::cout << "[ERRO] OpenVideo erro" << endl;
        goto erro2;
    }
    ret = SerialPortInit(port, "/dev/ttyS4"); // 监听线程开启
    if (ret != 0) {
        std::cout << "[ERRO] SerialPort erro" << endl;
        goto erro3;
    }
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

int KeyOperate(uint8 inDelay)
{
    // 更新键盘指令  
    char ckey = waitKey(inDelay); // 1ms，利于直接显示图像
    // 扫描键盘
    switch (ckey){
        case 27: // 按下Esc键则退出程序  
            destroyAllWindows();
            return 0; // 退出
            break;
        case 13:  // 按下回车:跳帧        
        case 32:  // 按下空格:跳帧        
            destroyAllWindows();
            break;
        default:
            break;
    }
    return 1;
}

ST_DETECT_INFO G_detectFlag {
    .m_u8DetectNoFireNums = 0,              // 检测到没有火焰的次数
    .m_u8DetectOkFireNums = 0,              // 检测到有火焰的次数
    .m_u8ReceptFlag = 0,                    // 接收到数据标志位
    .m_u8ReceptNums = 0,                    // 接收到数据次数
    .m_u8RealSegFlag = 0,                   // 实际分段标志位
    .m_u8RealSortFlag = 0                   // 实际排序标志位
}; // 检测标志位
ENUM_FIRE_PROCESS_STATE G_globalState = e_searchFire; // 全局状态
int frameControl(uint8 inDelay); // 帧控制

uint8 G_receive = 0;
uint8 G_buffer[12]; // 1120 0240 ys    //缓冲区可能用于存储特定格式的数据，例如坐标或命令信息
template<typename I_T, int N>
int LoadBuffer(I_T (&inArray)[N], int I_X, int I_Y);

void processInferenceResults(rknn_output* outputs, size_t actual_size[]) {
    float* outputData = reinterpret_cast<float*>(outputs[0].buf);

    int numAnchors = outputs[0].size / (outputData[0] * 4 + 1); // outputData[0]为检测到的物体数量
    float confidenceThreshold = 0.5; // 可根据需要调整置信度阈值
    for (int i = 0; i < numAnchors; ++i) {
        float objScore = outputData[i * (outputData[0] + 5) + 4];
        if (objScore > confidenceThreshold) { // 如果置信度高于阈值
            int classId = max_element(outputData + i * (outputData[0] + 5) + 5, outputData + i * (outputData[0] + 5) + 5 + outputData[0]) - (outputData + i * (outputData[0] + 5) + 5);
            if (classId > 0) { // classId=0通常是背景
                float tX = outputData[i * (outputData[0] + 5)];
                float tY = outputData[i * (outputData[0] + 5) + 1];
                float w = outputData[i * (outputData[0] + 5) + 2];
                float h = outputData[i * (outputData[0] + 5) + 3];

                // 转换为图像坐标
                int left = static_cast<int>((tX - w/2) * G_srcImg.cols);
                int top = static_cast<int>((tY - h/2) * G_srcImg.rows);
                int width = static_cast<int>(w * G_srcImg.cols);
                int height = static_cast<int>(h * G_srcImg.rows);

                // 确保坐标在图像范围内
                left = std::max(0, std::min(left, G_srcImg.cols - 1));
                top = std::max(0, std::min(top, G_srcImg.rows - 1));
                width = std::max(1, std::min(width, G_srcImg.cols - left));
                height = std::max(1, std::min(height, G_srcImg.rows - top));

                // 在这里处理或绘制边界框，例如：
                rectangle(G_srcImg, Point(left, top), Point(left + width, top + height), Scalar(0, 255, 0), 2);
                putText(G_srcImg, to_string(classId), Point(left, top - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
            }
        }
    }

    // 释放输出缓冲区等资源
    for(int i = 0; i < 4; ++i) {
        rknn_free_outputs(ctx, outputs, 1);
    }
}

void prepareInputsOutputs(rknn_context ctx, const Mat& inputImage, rknn_input* inputs, rknn_output* outputs, size_t* actual_size) {
    // 准备输入数据
    Mat blob;
    auto input_size = cv::Size(640, 640); // 根据YOLOv8模型调整
    blobFromImage(inputImage, blob, 1./255., input_size, Scalar(), true, false);

    // 设置输入数据
    inputs[0].index = 0;
    inputs[0].size = blob.total() * blob.elemSize();
    inputs[0].pass_through = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = blob.data;

    // 设置输出数据
    for (int i = 0; i < 4; ++i) {
        outputs[i].want_float = 1;
    }

    // 这里可以根据需要设置其他输出信息

    // 设置实际大小
    actual_size[0] = outputs[0].size;
    actual_size[1] = outputs[1].size;
    actual_size[2] = outputs[2].size;
    actual_size[3] = outputs[3].size;
}

void DetectFire_ret(int I_ret, ST_DETECT_INFO& IO_detectFlag, ENUM_FIRE_PROCESS_STATE& IO_detectState) {
    if (I_ret == 0) {
        // 无火 （连续10帧）

        IO_detectFlag.m_u8DetectNoFireNums++;
        if (IO_detectFlag.m_u8DetectNoFireNums > 3) {
            IO_detectFlag.m_u8DetectOkFireNums = 0;
        }
        if (IO_detectFlag.m_u8DetectNoFireNums > 10) {
            IO_detectFlag.m_u8DetectNoFireNums = 0;
            IO_detectFlag.m_u8DetectOkFireNums = 0;
            IO_detectFlag.m_u8ReceptFlag = 0;
            IO_detectFlag.m_u8ReceptNums = 0;
            IO_detectFlag.m_u8RealSegFlag = 0;
            IO_detectFlag.m_u8RealSortFlag = 0;
            if (G_fireVector.size()) { // 一定要先判断，否则导致二次释放内存！！！
                G_fireVector.clear();
            }
            IO_detectState = e_searchFire;
        }
    } else {
        // 有火 (连续3帧) 
        IO_detectFlag.m_u8DetectNoFireNums = 0;
        IO_detectFlag.m_u8DetectOkFireNums++;
        if (IO_detectFlag.m_u8DetectOkFireNums > 10) {
            IO_detectState = e_locateFire;
        } else {
            IO_detectState = e_detectFire;
        }
    }
}

template<typename T, int N>
int LoadBuffer(IN T (&inArray)[N], IN int inX, IN int inY) {
    if (inArray==nullptr || N==0) {
        cout << "inNums is nullptr or N is 0" << endl;
        goto exit;
    } else {
        for (uint8 i=0; i<=N; i++) {
            inArray[i] = 0; // 清空buffer
        }
    }
    {
        string s;
        string s1, s2;
        stringstream ss1, ss2; 
        ss1.clear(); // 清空流
        ss2.clear(); // 清空流
        ss1 << inX; s1 = ss1.str();// 将整数写入流=>并转入字符串（不带\0）
        ss2 << inY; s2 = ss2.str();// 将整数写入流=>并转入字符串（不带\0）
        if (ss1.str().length() > 4 || ss2.str().length() > 4) {
            cout << "LoadBuffer: ss1 length erro" << endl;
            goto erro1;
        } else {
            for (uint8 i=4; i>0; i--) {
                if (i > ss1.str().length()) {
                    s1 = '0' + s1; // 补0
                }
                if (i > ss2.str().length()) {
                    s2 = '0' + s2; // 补0
                }
            }
            s = 'A' + ' ' + s1 + ' ' +  s2 + ' ' +  'B'; // 补空
            cout << "[INFO-PORT-s] " << s << endl;
        }
        if (s.length() != N) {
            cout << "[ERRO] LoadBuffer s erro" << endl;
            goto erro2;
        } else {
            uint8 i = 0; // 去除最后一位 \0
            while (i <= N-1) {
                inArray[i] = s[i];
                i++;
            }
        }
    }
    return 0;
exit:
    return 0;
erro1:
    return -1;
erro2:
    return -2;
}

bool Receive(const std::vector<uint8_t>& data) {
    // 确保数据至少包含包头+数据+包尾，即至少3个字节
    if (data.size() < 3) {
        std::cerr << "[ERRO] Data frame too short to contain header, data, and footer." << std::endl;
        return false;
    }

    // 检查包头和包尾
    const uint8_t HEADER = "A";
    const uint8_t FOOTER = "B";
    if (data.front() != HEADER || data.back() != FOOTER) {
        std::cerr << "[ERRO] Invalid header or footer detected." << std::endl;
        return false;
    }

    // 提取布尔值字节（假设位于包头和包尾之间）
    bool value = (data[1] != 0); // 假设第二个字节为布尔值，1代表true，0代表false
    std::cout << "[INFO] Received bool value: " << (value ? "true" : "false") << std::endl;
    return value;
}



int main(int argc, char *argv[]) {

     thread mytobj(receive_thread);
    //mytobj.join();

    ret = SetUp();
    if (ret != 0) {
        std::cout << "[ERRO] SetUp erro" << endl;
        goto exit;
    }
    while(KeyOperate(1)) {
        ret = G_fireCore.ReadImg(cap, G_srcImg); // 需要放外部视频一直流动！
        // 调整图像亮度和对s比度
        // convertScaleAbs(g_srcImg, g_srcImg, 0.8, 6);
        if (ret != 0) {
            std::cout << "[ERRO] ReadImg erro" << endl;
            goto erro;
        }
#if BUG_CAM_TEMP
        G_orgImg = G_srcImg.clone();
        G_fireCore.ShowImg("AdjustOrinImg", G_orgImg, 0); // 不等
#endif
        switch (G_globalState) {
        case e_searchFire:
            ret = SearchFire(MLX_IMG);
#if DISPLAY_DEBUG
            namedWindow("MLX", WINDOW_NORMAL);
            imshow("MLX", MLX_IMG);
#endif 
            if (ret >= 0) {
                if (ret == 0) {
                    break; // 正常
                } else {
                    G_globalState = e_detectFire;
                } 
            } else {
                std::cout << "[ERRO] SearchFire erro" << endl;
                goto erro;
            }
            break;
        case e_detectFire:
            if (G_receive == 6) {
                G_receive = 0;
                G_globalState = e_searchFire;
                break;
            }

            if (G_globalState == e_detectFire) {
    // 图像预处理，符合模型输入要求
             Mat blob;
             auto input_size = cv::Size(640, 640); // 根据YOLOv8模型调整
             blobFromImage(G_srcImg, blob, 1./255., input_size, Scalar(), true, false);    //将图像转换为Blob（Blob是用于深度学习网络输入的标准化数据结构）

    // 执行推理
             rknn_input inputs[1];
             rknn_output outputs[4];           //yolov8的输出层数量
             size_t actual_size[4];
             prepareInputsOutputs(ctx, blob, inputs, outputs, actual_size);

             ret = rknn_run(ctx, NULL, inputs, 1, outputs, 4, NULL);
             if (ret < 0) {
                 std::cout <<"rknn_run fail! "<< endl;
                 return -1;
                 }else{
                    DetectFire_ret(ret, G_detectFlag, G_globalState);
                    processInferenceResults(outputs, actual_size);
                 }
}

#if DISPLAY_DEBUG
            namedWindow("SrcImg", WINDOW_NORMAL);
            imshow("SrcImg", G_srcImg);
            namedWindow("SegImg", WINDOW_NORMAL);
            imshow("SegImg", G_segImg);
            namedWindow("OutImg", WINDOW_NORMAL);
            imshow("OutImg", G_outImg);
#endif
            break;
        case e_locateFire:
            std::cout << "okokokokokokokokokokokokokokokokokokokokokokok!" << endl;
            G_detectFlag.m_u8DetectNoFireNums = 0;
            // g_detectFlag.m_u8DetectOkFireNums = 0; // do not!
            /* 发送给下位机 */
            tX = 0; tY = 0;
            tX = (G_fire.realRect.br().x - G_fire.realRect.tl().x)/2 + G_fire.realRect.tl().x;
            tY = G_fire.realRect.br().y;
            // tY = (g_fire.realRect.br().y - g_fire.realRect.tl().y)/2 + g_fire.realRect.tl().y;
            // filter operator
            // static int nSize = 0;
            // static uint32 tX_Total = 0;
            // static uint32 tY_Total = 0;
            // if (tX!=0 && tY!=0) {
            //     if (nSize > Filter) {
            //         // tX
            //         tX_Total -= tXFilter.front();
            //         tXFilter.pop();
            //         tXFilter.push(tX);
            //         tX_Total += tXFilter.back(); 
            //         // tY
            //         tY_Total -= tYFilter.front();
            //         tYFilter.pop();
            //         tYFilter.push(tY);
            //         tY_Total += tYFilter.back();
            //     } else {
            //         nSize++;
            //         //tX
            //         tXFilter.push(tX);
            //         tX_Total += tXFilter.back(); 
            //         //tY
            //         tYFilter.push(tY);
            //         tY_Total += tYFilter.back();
            //     }
            // }
            // tX_mean = int(tX_Total/nSize);
            // tY_mean = int(tY_Total/nSize);
            tX_mean = tX;
            tY_mean = tY;
            std::cout << "[INFO-PORT] tX_mean: " << int(tX_mean) << endl;
            std::cout << "[INFO-PORT] tY_mean: " << int(tY_mean) << endl;
            LoadBuffer(G_buffer, tX_mean, tY_mean); // 装载
            // 发送
            std::cout << "[INFO-PORT] port ok!" << endl;
            G_globalState = e_detectFire; // 回到侦察阶段
            break;
        default:
            std::cout << "[ERRO] global state erro" << endl;
            goto erro;
        }
    }
    return 0;
exit:
    return 0;
erro:
    return -1;

    std::vector<uint8_t> receivedData{'A', 1, 'B'}; // 假设接收到的是代表true的字节
    bool boolValue = Receive(receivedData);

    // 根据接收到的布尔值进行后续处理
    if (boolValue = 0) {
        std::cout <<"正在移动到火焰" <<endl;
    } else {
        std::cout << "成功移动到火焰" <<endl;
        G_globalState = e_searchFire;

        ret=SearchFire(MLX_IMG);
        if(ret=0){
            std::cout<<"灭火成功"<<endl;
        }
        G_globalState = e_searchFire;
    }
    return 0;
}