#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <vector>
#include "tools/yaml_read.hpp"
tools::yaml_reader::Ptr yaml_reader_ptr_;
tools::yaml_reader::Ptr yaml_reader_ptr_nav;
std::vector<Eigen::Vector2d> points_;
cv::Mat resized_image;
cv::Mat image;
int num = 0;
cv::Point startPoint, endPoint;
std::vector<double> alpha; // 逆时针为正
double delta;
double pos_x, pos_y;
double resolution;
int k = 5; // 放缩比例
int with;
int height;
// 初始化地图信息
void map_init()
{
    yaml_reader_ptr_nav = std::make_shared<tools::yaml_reader>(ros::package::getPath("fast_lio") + "/map/nav.yaml");
    if (yaml_reader_ptr_nav->readYAML())
    {
        YAML::Node config = yaml_reader_ptr_nav->getConfig();
        resolution = config["resolution"].as<double>();
        std::vector<double> origin = config["origin"].as<std::vector<double>>();
        pos_x = -origin[0];
        pos_y = -origin[1];
        // std::cout << "resolution: " << resolution << "  origin: " << origin[0] << "  " << origin[1] << std::endl;
    }
}
// 初始化点集
void points_init()
{
    yaml_reader_ptr_ = std::make_shared<tools::yaml_reader>(ros::package::getPath("sp_decision") + "/config/points_original.yaml");
    if (yaml_reader_ptr_->readYAML())
    {
        YAML::Node config = yaml_reader_ptr_->getConfig();
        for (const auto &node : config["points"])
        {
            Eigen::Vector2d point;
            point[0] = node["x"].as<double>();
            point[1] = node["y"].as<double>();
            points_.push_back(point);
        }
        for (const auto &point : points_)
        {
            // std::cout << "x: " << point[0] << "  y:  " << point[1] << std::endl;
        }
    }
}
// 映射点集到地图
void project_points()
{
    // 计算旋转矩阵
    cv::Point zero(k * pos_x / resolution, k * (height - (pos_y / resolution)));
    cv::circle(resized_image, zero, 10, cv::Scalar(255, 0, 0), -1); // 红色实心圆
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(delta * CV_PI / 180), -sin(delta * CV_PI / 180), sin(delta * CV_PI / 180), cos(delta * CV_PI / 180);
    // std::cout << "Matrix:" << rotation_matrix << std::endl;
    //  投影点集
    std::vector<Eigen::Vector2d> projected_points;
    for (const auto &point : points_)
    {

        Eigen::Vector2d projected_point = rotation_matrix * point;
        // std::cout << "x:" << projected_point[0] << "  y:" << projected_point[1] << std::endl;
        projected_points.push_back(projected_point);
    }
    for (const auto &point : projected_points)
    {
        int x = static_cast<int>(k * (point[0] + pos_x) / resolution);
        int y = static_cast<int>(k * (height - (point[1] + pos_y) / resolution));
        // std::cout << "x: " << x << "  y:  " << y << std::endl;
        cv::Point cvPoint(x, y);
        cv::circle(resized_image, cvPoint, 10, cv::Scalar(0, 0, 255), -1); // 红色实心圆
    }
}
/**
 * @brief 鼠标点击画线，画水平线计算旋转角度
 *
 */
void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && num == 0)
    {
        startPoint = cv::Point(x, y);
        // std::cout << "Point:" << startPoint << std::endl;
        num++;
    }
    else if (event == cv::EVENT_LBUTTONDOWN && num == 1)
    {
        endPoint = cv::Point(x, y);
        cv::line(*((cv::Mat *)userdata), startPoint, endPoint, cv::Scalar(0, 0, 255), 2);
        // std::cout << "Point:" << startPoint << std::endl;
        cv::imshow("PGM Image", *((cv::Mat *)userdata));
        num = 0;

        // 计算角度
        Eigen::Vector2d start(startPoint.x, startPoint.y);
        Eigen::Vector2d end(endPoint.x, endPoint.y);
        Eigen::Vector2d diff = end - start;
        double angle = -std::atan2(diff(1), diff(0)) * 180 / CV_PI;
        std::cout << "Angle: " << angle << std::endl;
        alpha.push_back(angle);
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        cv::Point cvPoint = cv::Point(x, y);
        cv::circle(resized_image, cvPoint, 10, cv::Scalar(0, 255, 0), -1); // 红色实心圆
        //std::cout << "Point:" << cvPoint << std::endl;
        float x1 =  static_cast<double>(x) /  static_cast<double>(k) * resolution - pos_x;
        float y1 = - static_cast<double>(y) /  static_cast<double>(k) * resolution + height * resolution - pos_y;
        Eigen::Vector2f point(x1, y1);
        Eigen::Vector2f point_rotated =  point;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << point_rotated[0]; // Set precision to 3 decimal places
        std::string result_x = ss.str();
         std::stringstream ss_y;
        ss_y << std::fixed << std::setprecision(2) << point_rotated[1]; // Set precision to 3 decimal places
        std::string result_y = ss_y.str();
        std::string text = "(" + result_x + ", " + result_y + ")";
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        cv::Scalar color = cv::Scalar(255, 0, 0);
        int thickness = 1;
        cv::Point textOrg(x + 15, y); // 将文本放在点的右边，稍微偏移一些
        cv::putText(resized_image, text, textOrg, font, fontScale, color, thickness);
    }
}
/**
 * @brief 计算平均角度
 *
 */
void calculate_average_angle()
{
    double sum = 0;
    for (int i = 0; i < alpha.size(); i++)
    {
        sum += alpha[i];
    }
    delta = sum / alpha.size();
    std::cout << "Average Angle: " << delta << std::endl;
}
// 重置
void reset()
{
    cv::resize(image, resized_image, cv::Size(), k, k, cv::INTER_LINEAR);
    num = 0;
    alpha.clear();
    delta = 0;
    cv::destroyAllWindows();
}
// 输出更新后的点集
void output_points()
{
    // 创建YAML节点
    YAML::Node rootNode;
    std::vector<Eigen::Vector2d> projected_points;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(delta * CV_PI / 180), -sin(delta * CV_PI / 180), sin(delta * CV_PI / 180), cos(delta * CV_PI / 180);
    for (const auto &point : points_)
    {
        Eigen::Vector2d projected_point = rotation_matrix * point;
        // std::cout << "x:" << projected_point[0] << "  y:" << projected_point[1] << std::endl;
        projected_points.push_back(projected_point);
    }
    // 添加每个点到YAML节点
    for (size_t i = 0; i < projected_points.size(); ++i)
    {
        YAML::Node pointNode;
        pointNode["id"] = i + 1; // id从1开始
        pointNode["x"] = projected_points[i][0];
        pointNode["y"] = projected_points[i][1];
        rootNode["points"].push_back(pointNode);
    }

    // 输出到文件
    std::ofstream fout(ros::package::getPath("sp_decision") + "/config/points.yaml");
    fout << rootNode;
    std::cout << "file saved successfully." << std::endl;
}
int main()
{
    std::cout << "输入c计算角,p显示点位,r重置,鼠标右键点击获取map坐标系下的坐标,q退出" << std::endl;
    points_init();
    map_init();
    // 读取PGM图像
    image = cv::imread(ros::package::getPath("fast_lio") + "/map/nav.pgm", cv::IMREAD_COLOR);
    height = image.size().height;
    with = image.size().width;
    // std::cout << "height: " << height << "  width: " << with << std::endl;
    if (image.empty())
    {
        std::cerr << "Error: Could not open or find the image" << std::endl;
        return -1;
    }

    cv::resize(image, resized_image, cv::Size(), k, k, cv::INTER_LINEAR);
    char key = 'a';
    while (key != 'q')
    {                                                    // 创建窗口并显示图像
        cv::namedWindow("PGM Image", cv::WINDOW_NORMAL); // 使用WINDOW_NORMAL以便调整窗口大小
        cv::imshow("PGM Image", resized_image);
        cv::setMouseCallback("PGM Image", onMouse, &resized_image);
        // 等待按键
        key = cv::waitKey(1);
        if (key == 'c')
        {
            calculate_average_angle();
        }
        if (key == 'p')
        {
            project_points();
        }
        if (key == 'r')
        {
            reset();
        }
        if (key == 'o')
        {
            output_points();
        }
    }
    // 关闭窗口
    cv::destroyAllWindows();
    return 0;
}
