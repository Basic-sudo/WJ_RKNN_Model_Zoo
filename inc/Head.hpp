#pragma once

#define IN       /*标记作为输入参数：该类数据需要入口有效性检测*/
#define OUT      /*标记作为输出参数*/
#define IN_OUT   /*标记作为输入输出参数（在外部循环使用）：该类数据需要入口清空数据*/

typedef unsigned char       uint8;  
typedef signed   char       int8;
typedef unsigned short      uint16; 
typedef signed   short      int16;
typedef unsigned long       uint32;
typedef signed   long       int32;
// typedef unsigned long long  uint64;
// typedef signed   long long  int64;
typedef float               f32;
typedef double              f64;

/* 基本头文件PIXL接口 */
#include "stdlib.h"
#include <assert.h>   /*测试一个条件并可能使程序终止(断言)*/
#include <math.h>      
#include <fcntl.h> /*文件控制定义*/
#include <stdio.h> /*标准输入输出定义*/
#include <stdlib.h>  /*标准函数库定义*/
#include <stdint.h>   /*指定可以扩展为一个特定整数类型的常数的类似函数的宏*/
#include <unistd.h>  /*Unix 标准函数定义*/
#include <memory.h>   /*提供了一组内存操作函数*/
#include <chrono>     /**/
#include <string>/**/
#include <cstring>/**/
#include <sstream> // 字符串流
#include <fstream>/**/
#include <iostream> // 终端io输出
#include <vector>/**/
#include <queue> // 队列容器
#include <map>/**/
#include <unordered_map>
#include <set>
#include <unordered_set>

#include <omp.h>

/* POSIX接口 */
#include <thread>
#include <termios.h> /* POSIX 终端控制定义 串口 */
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* opencv接口 */
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

/* MLX90640接口 */
#include <MLX90640_API.h>
#include <MLX90640_APP.h>
#include <MLX90640_LINUX_I2C_Driver.h>

/* 串口接口 */
#include <SerialPort.hpp>

/* 自定义头 */
#include <Config.hpp>
#include "FireControl.hpp"
// #include "ImgCore.hpp"
#include "ImgMath.hpp"
// #include "ImgProcess.hpp"
// #include "ImgFeature.hpp"