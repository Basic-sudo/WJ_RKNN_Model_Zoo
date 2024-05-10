#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_
#include <Head.hpp>

using namespace cv;
using namespace std;

/* 显示 */
// 无定义：全不显示
// 定义为1：展示效果
#define DISPLAY 1
#if DISPLAY
#define DISPLAY_DEBUG 1 // 1 DEBUG 
#else
#define DISPLAY_DEBUG 0 // 0 NODEBUG
#endif

/* 调试中间过程 */
#define BUG 0
#if BUG // 分模块调试
#define BUG_INFO
// 调试MLX
#define BUG_MLX 0
#if BUG_MLX
#else
// 调试CAM
#define BUG_CAM 1
#if BUG_CAM
#define BUG_CAM_TEMP 0 // bug cam 的中间过程
#define BUG_CAM_SAVE 0 // bug cam 的中间过程
#endif
#endif
// 调试串口
#define BUG_BUF 1
#endif

#define COL 640 // 640 1920
#define ROW 480 // 480 1080
#define CNN_COL 224
#define CNN_ROW 224

#define BLOCK 3
#define BLOCK_COL (COL/BLOCK)
#define BLOCK_ROW (ROW/BLOCK)
 
#define CLASS 10  // 分组(最多筛选10个轮廓)
#define TR (COL*0.15) // 聚类

class Fire {
private:
public:
    uint8 label;
    Rect realRect; // 真实
    Rect recoRect; // 识别
    Rect expdRect; // 扩大
    Fire(){}
    Fire(uint8 inLabel, Rect inRealRect, Rect inRecoRect, Rect inExpdRect) {
        label = inLabel;
        realRect = inRealRect;
        recoRect = inRecoRect;
        expdRect = inExpdRect;
    }
};
#endif
