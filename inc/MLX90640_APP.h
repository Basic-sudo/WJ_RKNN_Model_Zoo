#ifndef _MLX90640_APP_H_
#define _MLX90640_APP_H_
#include <Head.hpp>

using namespace cv;
using namespace std;

#define BILI 0 // 是否启用双线性插值

#define MLX_I2C_ADDR 0x33// 0x33
#define SENSOR_W 32
#define SENSOR_H 24 // 768 pixels

#define EMMISIVITY 0.95f // 1
#define TA_SHIFT 8 // 8 // Default shift for MLX90640 in open air

// Valid frame rates are 1, 2, 4, 8, 16, 32 and 64
// The i2c baudrate is set to 1mhz to support these
#define FPS 64
// 根据FPS帧率确定帧间隔时间
#define FRAME_TIME_MICROS (1000000/FPS) // 1s = 1000ms = 1000000us
#define OFFSET_MICROS 850 // 帧率偏移

// 另一版本
#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06
#define	 RefreshRate FPS8HZ 

// start with some initial colors
extern float minTemp;
extern float maxTemp;
extern float centerTemp;

#ifdef __cplusplus
extern "C" {
#endif 

#define min(a,b)  ((a)<(b)?(a):(b))
#define max(a,b)  ((a)>(b)?(a):(b))
#define abs(x)    ((x)>0?(x):(-x))
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt))) 

int MLX90640Init(void);
int ReadTempValues(void);
void DisplayTempImg(IN_OUT Mat& inImg);

#ifdef __cplusplus
}
#endif

#endif