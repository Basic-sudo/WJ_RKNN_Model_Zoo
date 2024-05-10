
#include <Head.hpp>
#include "ImgCore.hpp"
#include "ImgProcess.hpp"
#include "ImgFeature.hpp"

using namespace std;
using namespace cv;

#if BILI
Mat MLX_IMG(SENSOR_H*2, SENSOR_W*2, CV_8UC1);
#else
Mat MLX_IMG(SENSOR_H, SENSOR_W, CV_8UC1); // 黑白/热力
#endif

int Filter = 10;
int ret; // 状态
uint32 tX; uint32 tX_mean;
uint32 tY; uint32 tY_mean;
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



VideoCapture cap(4);
SerialPort port("/dev/ttyS4");

int SetUp(void); // 初始化
int KeyOperate(uint8 inDelay); // 键盘

ST_DETECT_INFO G_detectFlag {
    .m_u8DetectNoFireNums = 0,
    .m_u8DetectOkFireNums = 0,
    .m_u8ReceptFlag = 0,
    .m_u8ReceptNums = 0,
    .m_u8RealSegFlag = 0,
    .m_u8RealSortFlag = 0
}; // 检测标志位
ENUM_FIRE_PROCESS_STATE G_globalState = e_searchFire; // 全局状态
int frameControl(uint8 inDelay); // 帧控制

uint8 G_receive = 0;
uint8 G_buffer[12]; // 1120 0240 ys
template<typename T, int N>
int LoadBuffer(I_T (&inArray)[N], int I_X, int I_Y);

int main(int argc, char *argv[]) {
    ret = SetUp();
    if (ret != 0) {
        cout << "[ERRO] SetUp erro" << endl;
        goto exit;
    }
    while(KeyOperate(1)) {
        ret = G_fireCore.ReadImg(cap, G_srcImg); // 需要放外部视频一直流动！
        // 调整图像亮度和对s比度
        // convertScaleAbs(g_srcImg, g_srcImg, 0.8, 6);
        if (ret != 0) {
            cout << "[ERRO] ReadImg erro" << endl;
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
                cout << "[ERRO] SearchFire erro" << endl;
                goto erro;
            }
            break;
        case e_detectFire:
            if (G_receive == 6) {
                G_receive = 0;
                G_globalState = e_searchFire;
                break;
            }
            ret = DetectFire(G_srcImg, G_outImg, &G_detectFlag); // 输出框(3阶段)
            if (ret >= 0) {
                DetectFire_ret(ret, G_detectFlag, G_globalState);
            } else {
                cout << "[ERRO] DetectFire erro" << endl;
                goto erro;
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
            cout << "okokokokokokokokokokokokokokokokokokokokokokok!" << endl;
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
            cout << "[INFO-PORT] tX_mean: " << int(tX_mean) << endl;
            cout << "[INFO-PORT] tY_mean: " << int(tY_mean) << endl;
            LoadBuffer(G_buffer, tX_mean, tY_mean); // 装载
            port.write(G_buffer,12); // 发送
            cout << "[INFO-PORT] port ok!" << endl;
            G_globalState = e_detectFire; // 回到侦察阶段
            break;
        default:
            cout << "[ERRO] global state erro" << endl;
            goto erro;
        }
    }
    return 0;
exit:
    return 0;
erro:
    return -1;
}

int SetUp(void) {   
    ret = MLX90640Init();
    if (ret != 0) {
        cout << "[ERRO] MLX90640Init erro" << endl;
        goto erro1;
    }
    ret = G_fireProcess.OpenVideo(cap, 100); // 稳定100帧(30帧一秒)
    if (ret != 0) {
        cout << "[ERRO] OpenVideo erro" << endl;
        goto erro2;
    }
    ret = SerialPortInit(port, "/dev/ttyS4"); // 监听线程开启
    if (ret != 0) {
        cout << "[ERRO] SerialPort erro" << endl;
        goto erro3;
    }
    return 0;
erro1:
    return -1;
erro2:
    return -2;
erro3:
    return -3;
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
            s = s1 + ' ' +  s2 + ' ' + 'y' + 's'; // 补空
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
