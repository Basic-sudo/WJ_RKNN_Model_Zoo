#include <MLX90640_APP.h>

static int status;

/* MLX相关变量 */
paramsMLX90640 mlx90640;          // 参数读取 => 解析参数
static uint16 eeMLX90640[832];  // EEPROM读取的值(EEPROM 中的地址 0x2440~0x273F 内包含了每个像素的校准信息（出厂时已完成校准）)
static uint16 eeFrame[834];     // 32*24+2 读取一帧中寄存器中的值
static float mlx90640To[32 * 24]; // 用于存储 MLX90640 的温度数据 32*24=768 // array for the 32 x 24 measured tempValues
static float InsertValues[24][32]; // 图像矩阵（拷贝）
static float InterpolationValues[24*2][32*2]; // 插值矩阵

// start with some initial colors
Point minPoint;
Point maxPoint;
float minTemp = 20.0f;
float maxTemp = 40.0f;
float centerTemp = 0.0f;

// variables for interpolated colors
uint8 red, green, blue;

static int is_in_array(float x, float y, float height, float width);
static void bilinera_interpolation(float (*in_array)[32], float height, float width, 
                                   float (*out_array)[64], float out_height, float out_width);
static uint16 BSP_LCD_GetColor565(uint8 red, uint8 green, uint8 blue);
static uint16 TempToColor(float val);
static int eightNeighborRegion(IN float* const toTemp, IN_OUT Mat& inImg, IN vector<vector<bool>>& visited, IN int threshold, int x, int y);
static void GetTempScale(IN float *const toTemp, OUT Point& maxPoint, OUT Point& minPoint);
static void DrawLegend(void);
static void DrawPicture(IN float *const toTemp, IN_OUT Mat &inImg);

static void DrawMeasurement(void);

static int is_in_array(float x, float y, float height, float width)
{
    if (x >= 0 && x < width && y >= 0 && y < height)
        return 1;
    else
        return 0;
}

// 32列 => 64列
static void bilinera_interpolation(float (*in_array)[32], float height, float width,
                                   float (*out_array)[64], float out_height, float out_width)
{
    double h_times = (double)out_height / (double)height,
           w_times = (double)out_width / (double)width;
    float x1, y1, x2, y2, f11, f12, f21, f22;
    double x, y;

    for (int i = 0; i < out_height; i++)
    {
        for (int j = 0; j < out_width; j++)
        {
            x = j / w_times;
            y = i / h_times;
            x1 = (float)(x - 1);
            x2 = (float)(x + 1);
            y1 = (float)(y + 1);
            y2 = (float)(y - 1);
            f11 = is_in_array(x1, y1, height, width) ? in_array[(int)y1][(int)x1] : 0;
            f12 = is_in_array(x1, y2, height, width) ? in_array[(int)y2][(int)x1] : 0;
            f21 = is_in_array(x2, y1, height, width) ? in_array[(int)y1][(int)x2] : 0;
            f22 = is_in_array(x2, y2, height, width) ? in_array[(int)y2][(int)x2] : 0;
            if (i > 1 && i < out_height - 2 && j > 2 && j < out_width - 2)
            {
                out_array[i][j] = (float)(((f11 * (x2 - x) * (y2 - y)) +
                                           (f21 * (x - x1) * (y2 - y)) +
                                           (f12 * (x2 - x) * (y - y1)) +
                                           (f22 * (x - x1) * (y - y1))) /
                                          ((x2 - x1) * (y2 - y1)));
            }
            else if (i <= 1 || i >= out_height - 2)
            {
                out_array[i][j] = (float)((f11 * (x2 - x) + f21 * (x - x1)) / (x2 - x1));
            }
            else if (j <= 2 || j >= out_width - 2)
            {
                out_array[i][j] = (float)((f11 * (y2 - y) + f12 * (y - y1)) / (y2 - y1));
            }
        }
    }
}

static int eightNeighborRegion(IN float *const toTemp, IN_OUT Mat& inImg, IN vector<vector<bool>>& visited, IN int threshold, int x, int y) 
{
#if BILI
    int width = SENSOR_W*2;
    int height = SENSOR_H*2;
#else
    int width = SENSOR_W;
    int height = SENSOR_H;
#endif
    if (toTemp == nullptr) {
        cout << "DrawPicture toTemp is nullptr" << endl;
    }
    if (x<0 || x>=width || 
        y<0 || y>=height) {
        goto exit; // 超出图像边界，返回
    }
    // 判断当前点是否已经访问过或对应像素值是否大于阈值
    if (visited[x][y] || fabs(toTemp[height*y + x]-maxTemp)>threshold) {
        goto exit; // 已访问过或不满足条件，返回
    }

    visited[x][y] = true; // 标记当前点为已访问
    inImg.at<uchar>(x, y) = 255; // 白色
    
    // 递归访问当前点的8邻域
    eightNeighborRegion(toTemp, inImg, visited, threshold, x - 1, y - 1);
    eightNeighborRegion(toTemp, inImg, visited, threshold, x, y - 1);
    eightNeighborRegion(toTemp, inImg, visited, threshold, x + 1, y - 1);
    eightNeighborRegion(toTemp, inImg, visited, threshold, x - 1, y);
    eightNeighborRegion(toTemp, inImg, visited, threshold, x + 1, y);
    eightNeighborRegion(toTemp, inImg, visited, threshold, x - 1, y + 1);
    eightNeighborRegion(toTemp, inImg, visited, threshold, x, y + 1);
    eightNeighborRegion(toTemp, inImg, visited, threshold, x + 1, y + 1);
    return 0;
exit:
    return 0;
}

static void GetTempScale(IN float *const toTemp, OUT Point& minPoint, OUT Point& maxPoint)
{
#if BILI
    int width = SENSOR_W*2;
    int height = SENSOR_H*2;
#else
    int width = SENSOR_W;
    int height = SENSOR_H;
#endif
    minTemp = 255.0f;
    minPoint.x = 0; minPoint.y = 0;
    maxTemp = 0.0f;
    maxPoint.x = 0; maxPoint.y = 0;
    for (int y=0; y<height; y++) {
        for (int x=0; x<width; x++) {
            if (minTemp > toTemp[height*y + x]) {
                minPoint.x = x; minPoint.y = y;
            }
            minTemp = min(minTemp, toTemp[height*y + x]);
            if (maxTemp < toTemp[height*y + x]) {
                maxPoint.x = x; maxPoint.y = y;
            }
            maxTemp = max(maxTemp, toTemp[height*y + x]);
        }
    }
    // Measure and print center temperature
    centerTemp = (toTemp[383 - 16] +
                  toTemp[383 - 15] +
                  toTemp[384 + 15] +
                  toTemp[384 + 16]) / 4;
    cout << "minTemp: " << minTemp << " ; " << "minPoint.x: " << minPoint.x << " ; " << "minPoint.y: " << minPoint.y << endl;
    cout << "maxTemp: " << maxTemp << " ; " << "maxPoint.x: " << maxPoint.x << " ; " << "maxPoint.y: " << maxPoint.y << endl;
    cout << "centerTemp: " << centerTemp << endl;
}

static void DrawPicture(IN float *const toTemp, IN_OUT Mat &inImg) 
{
#if BILI
    int width = SENSOR_W*2;
    int height = SENSOR_H*2;
#else
    int width = SENSOR_W;
    int height = SENSOR_H;
#endif

#if 1
    // 黑白图
    inImg = {0}; // 抹黑
    vector<vector<bool>> visited(width, vector<bool>(height, false)); // 傻缓存表
    if (toTemp == nullptr) {
        cout << "DrawPicture toTemp is nullptr" << endl;
    }
    eightNeighborRegion(toTemp, inImg, visited, 3, maxPoint.x, maxPoint.y); // 3°
#else
    // 热力图
    uint16 pixels[width * height] = {0}; // 热力图
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            pixels[y * width + x] = TempToColor(toTemp[height*y + x]); // TempToColor(toTemp[(31 - x) + (y * 32)]); // 需要翻转
        }
    }  
    // Mat
    for (int y = 0; y < width; y++) {
        for (int x = 0; x < height; x++) {
            inImg.at<Vec3b>(x, y)[2] = (pixels[y * width + x] & 0xF800) >> 11; // 红色
            inImg.at<Vec3b>(x, y)[1] = (pixels[y * width + x] & 0x07E0) >> 5;  // 绿色
            inImg.at<Vec3b>(x, y)[0] = (pixels[y * width + x] & 0x001F) >> 0;  // 蓝色
        }
    }
#endif
}

// Draw a legend.
static void DrawLegend(void)
{
    // int j = 0;
    // // if (minTemp < 0.0f) minTemp = 0.0f; // don't let the scale go below zero (can happen if minTemp < 0 and maxTemp < 0)

    // float inc = (maxTemp - minTemp) / 224.0f;
    // for (int ii = minTemp; ii < maxTemp; ii += inc)
    // {
    //     // LCD_FAST_DrawLine(8+ + j++, 292,8+ + j+ +20 , 292, TempToColor(ii));
    //     piclib_fill_color(0 + j++, 280, 7, 16, TempToColor(ii));
    // }

    // memset(tempBuffer, 0, sizeof(tempBuffer));
    // sprintf(tempBuffer, (char *)"%2.2f", minTemp); // minTemp输入到tempBuffer中 %2.1f 保留一位小数
    // POINT_COLOR = BLUE;
    // LCD_ShowString(5, 260, 100, 16, 16, "MIN");
    // LCD_ShowString(5, 300, 200, 16, 16, (u8 *)tempBuffer);

    // memset(tempBuffer, 0, sizeof(tempBuffer));
    // sprintf(tempBuffer, (char *)"%2.2f", maxTemp); // maxTemp输入到tempBuffer中 %2.1f 保留一位小数
    // POINT_COLOR = RED;
    // LCD_ShowString(200, 260, 100, 16, 16, "MAX");
    // LCD_ShowString(200, 300, 200, 16, 16, (u8 *)tempBuffer);
}

// Draw a circle + measured value.
static void DrawMeasurement(void)
{
    // POINT_COLOR = BLACK;
    // // Mark center measurement
    // LCD_Draw_Circle(120, 84, 2);
    // LCD_Draw_Circle(120, 84, 3);
    // LCD_Draw_Circle(120, 84, 4);

    // // Measure and print center temperature
    // centerTemp = (tempValues[383 - 16] + tempValues[383 - 15] + tempValues[384 + 15] + tempValues[384 + 16]) / 4;
    // printf("%2.2f ", centerTemp);

    // memset(tempBuffer, 0, sizeof(tempBuffer));
    // sprintf(tempBuffer, (char *)"%2.2f ", centerTemp);
    // // BSP_LCD_DisplayStringAt(88,220,(uint8_t *)tempBuffer,LEFT_MODE,LCD_COLOR_RED);
    // POINT_COLOR = RED;
    // LCD_ShowString(100, 205, 100, 16, 16, (u8 *)tempBuffer);
    // POINT_COLOR = BLACK;
    // LCD_ShowString(5, 180, 240, 16, 16, "The center temperature is:");
    // LCD_ShowString(50, 230, 240, 16, 16, "Degrees Celsius"); // 显示单位 摄氏度
}

uint16 BSP_LCD_GetColor565(uint8 red, uint8 green, uint8 blue)
{
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue) >> 3); // 1111 1000    1111 1100
}

static uint16 TempToColor(float val)
{
    /*
      pass in value and figure out R G B
      several published ways to do this I basically graphed R G B and developed simple linear equations
      again a 5-6-5 color display will not need accurate temp to R G B color calculation

      equations based on
      http://web-tech.ga-usa.com/2012/05/creating-a-custom-hot-to-cold-temperature-color-gradient-for-use-with-rrdtool/index.html

    */
    // variables for row/column interpolation
    float a = 0.0f, b = 0.0f, c = 0.0f, d = 0.0f;

    // 设置温度截止点（插值点）
    a = minTemp + (maxTemp - minTemp) * 0.2121f;
    b = minTemp + (maxTemp - minTemp) * 0.3182f;
    c = minTemp + (maxTemp - minTemp) * 0.4242f;
    d = minTemp + (maxTemp - minTemp) * 0.8182f;

    red = constrain(255.0f / (c - b) * val - ((b * 255.0f) / (c - b)), 0, 255);

    if ((val > minTemp) & (val < a))
    {
        green = constrain(255.0f / (a - minTemp) * val - (255.0f * minTemp) / (a - minTemp), 0, 255);
    }
    else if ((val >= a) & (val <= c))
    {
        green = 255;
    }
    else if (val > c)
    {
        green = constrain(255.0f / (c - d) * val - (d * 255.0f) / (c - d), 0, 255);
    }
    else if ((val > d) | (val < a))
    {
        green = 0;
    }

    if (val <= b)
    {
        blue = constrain(255.0f / (a - b) * val - (255.0f * b) / (a - b), 0, 255);
    }
    else if ((val > b) & (val <= d))
    {
        blue = 0;
    }
    else if (val > d)
    {
        blue = constrain(240.0f / (maxTemp - d) * val - (d * 240.0f) / (maxTemp - d), 0, 240);
    }

    // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
    return BSP_LCD_GetColor565(red, green, blue);
}

int MLX90640Init(void)
{
    MLX90640_I2CGeneralReset();
    // 设置控制寄存器（刷新率、校准模式）
    switch (FPS)
    { // 64
    case 1:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b001);
        break;
    case 2:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b010);
        break;
    case 4:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b011);
        break;
    case 8:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b100);
        break;
    case 16:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b101);
        break;
    case 32:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b110);
        break;
    case 64:
        MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b111);
        break;
    default:
        printf("Unsupported framerate: %d", FPS);
        break; // return 1;
    }
    // 设置棋盘交错模式
    sleep(1);
    MLX90640_SetChessMode(MLX_I2C_ADDR);
    // 读EEPROM=>加载系统默认参数
    sleep(1);
    status = MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    if (status != 0) {
        cout << "load system parameters error with code" << endl;
        goto erro1;
    }   
    // 从 MLX90640 的 EEPROM 中提取并计算参数
    sleep(1);
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0) {
        cout << "extract system parameters error with code" << endl;
        goto erro2;
    }
    sleep(1);
    cout << "MLX90640 init ok!" << endl;
    return 0;
erro1:
    return -1;
erro2:
    return -2;
}

// Read pixel data from MLX90640.
int ReadTempValues(void)
{
    // Read both subpages
    for (uint8 x = 0; x < 2; x++) {
        status = MLX90640_GetFrameData(MLX_I2C_ADDR, eeFrame);
        if (status < 0) {
            cout << "Get eeFrame Error: " << status << endl;
            goto erro1;
        }

        float vdd = MLX90640_GetVdd(eeFrame, &mlx90640); // 获取 MLX90640 的电压 3.3、5
        float Ta = MLX90640_GetTa(eeFrame, &mlx90640);   // 获取 MLX90640 的环境温度

        float tr = Ta - TA_SHIFT; // Reflected temperature based on the sensor ambient temperature

        MLX90640_CalculateTo(eeFrame, &mlx90640, EMMISIVITY, tr, mlx90640To);
        MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, mlx90640To, 1, &mlx90640);  // 修正 MLX90640 的坏像素
        MLX90640_BadPixelsCorrection((&mlx90640)->outlierPixels, mlx90640To, 1, &mlx90640); // 修正 MLX90640 的坏像素
#if BILI
        float *p = mlx90640To;
        for (int h = 0; h < 24; h++) {
            for (int w = 0; w < 32; w++) {
                InsertValues[h][w] = *(p + h * 32 + w);
            }
        }
        // bilinera_interpolation(InsertValues, 24, 32, InterpolationValues, 24, 32); // 此处为对比显示   注释1-2 共两处
        bilinera_interpolation(InsertValues, 24, 32, InterpolationValues, 48, 64);    // 此处为单显示双线性插值 注释 2-2 共两处
#endif
    }
    return 0;
erro1:
    return -1;
}
void DisplayTempImg(IN_OUT Mat &inImg)
{
    GetTempScale(mlx90640To, minPoint, maxPoint);
    // 画bar
    DrawLegend(); // draw the color legend bar
    
#if BILI
    DrawPicture(&InterpolationValues[0][0], inImg);
#else
    DrawPicture(mlx90640To, inImg);
#endif
    DrawMeasurement();
}
