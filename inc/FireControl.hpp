#ifndef _FIRECONTROL_HPP_
#define _FIRECONTROL_HPP_
#include <Head.hpp>

typedef enum _en_fire_process_state_ {
    e_searchFire = 1, /* 搜索火焰阶段 */
    e_detectFire,     /* 侦测火焰阶段 */
    e_locateFire,     /* 定位火焰阶段 */
} ENUM_FIRE_PROCESS_STATE;

typedef struct _st_detect_flag_ {
    uint8 m_u8DetectNoFireNums; // 全局帧数控制
    uint8 m_u8DetectOkFireNums; // 全局帧数控制

    uint8 m_u8ReceptFlag; // 接受框的标志
    uint8 m_u8ReceptNums;

    uint8 m_u8RealSegFlag;  // 实际分割检测到的标志
    uint8 m_u8RealSortFlag; // 实际分类检测到的标志(3个状态)
} ST_DETECT_INFO;

int SearchFire(IN_OUT Mat& inHeatImg); 
int DetectFire(IN const Mat& inImg, OUT Mat& outImg, IN ST_DETECT_INFO *p_detectFlag);

#endif