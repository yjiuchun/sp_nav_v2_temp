/**
 * @file plan_select.cpp
 * @author lp
 * @brief 获取策略id，在参数服务器中设置后开启决策节点并关闭本程序
 * @version 0.1
 * @date 2024-04-23
 *
 * @copyright Copyright (c) 2024
 *
 */
// #include <ros/ros.h>
// #include <ros/package.h>
// #include <opencv2/opencv.hpp>
// #include <Eigen/Geometry>
// #include <vector>
// #include "tools/yaml_read.hpp"

// int main(int argc, char **argv)
// {
//     cv::Mat img;
//     img.resize(500,500);
//     cv::imshow("img",img);
//     cv::waitKey(0);
//     setlocale(LC_CTYPE, "zh_CN.utf8");
//     ros::init(argc, argv, "plan_select");
//     ros::NodeHandle nh;
//     int param_plan_id=0;
//     ros::param::set("plan_id",param_plan_id);
//     return 0;
// }
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/package.h>
#include <cstdlib>

using namespace cv;
using namespace std;
int number;
bool running;
void start()
{
    ros::param::set("plan_id", number);
    const char *scriptCommand = "./your_script.sh"; 
    int returnValue = system(scriptCommand);
    ros::Duration(2).sleep();
    running=false;
}
void close()
{
    ros::param::set("decision_node_run", 0);
    ros::Duration(1).sleep();
    ros::param::set("decision_node_run", 1);
}
void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        if (x > 50 && x < 150 && y > 50 && y < 150)
        {
            if (number > 0)
                number--;
        }
        if (x > 350 && x < 450 && y > 50 && y < 150)
        {
            number++;
        }
        if (x > 50 && x < 450 && y > 250 && y < 350)
        {
            start();
        }
         if (x > 50 && x < 450 && y > 375 && y < 475)
        {
            close();
        }
    }
}
int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "plan_select");
    ros::NodeHandle nh;
    int param_plan_id = 1;
    ros::param::set("plan_id", param_plan_id);
    running=true;
    // 创建一个窗口
    namedWindow("Select plan");

    // 初始化数字和按钮
    number = 0;
    Rect decreaseButton(50, 50, 100, 100);
    Rect increaseButton(350, 50, 100, 100);
    Rect startButto(50, 250, 400, 100);
    Rect restartButton(50, 375, 400, 100);
    while (running)
    {
        // 创建一个空白画布
        Mat canvas(500, 500, CV_8UC3, Scalar(255, 255, 255));

        // 绘制数字
        string text = to_string(number);
        putText(canvas, text, Point(200, 150), FONT_HERSHEY_SIMPLEX, 5, Scalar(0, 0, 0), 10);

        // 绘制按钮
        rectangle(canvas, decreaseButton, Scalar(200, 200, 200), -1);
        rectangle(canvas, increaseButton, Scalar(200, 200, 200), -1);
        rectangle(canvas, startButto, Scalar(200, 200, 200), -1);
        rectangle(canvas, restartButton, Scalar(200, 200, 200), -1);
        putText(canvas, "-", Point(75, 120), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 0), 2);
        putText(canvas, "+", Point(375, 120), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 0), 2);
        putText(canvas, "start", Point(170, 320), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 0), 2);
        putText(canvas, "close", Point(170, 440), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 0), 2);

        // 显示画布
        imshow("Select plan", canvas);
        cv::setMouseCallback("Select plan", onMouse, &canvas);
        // 检测鼠标点击事件
        int key = waitKey(1);
        if (key == 27) // 按下ESC退出
            break;
    }

    // 关闭窗口
    destroyAllWindows();

    return 0;
}
