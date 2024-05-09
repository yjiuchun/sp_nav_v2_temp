#include "perception/cloud_tracker.hpp"

// // 定义点云数据结构
// struct Point3D {
//     float x;
//     float y;
//     float z;
// };

// // 投影三维点云到二维平面，并统计点云数量
// cv::Mat projectAndCountPointCloud(const std::vector<Point3D>& pointCloud, float grid_resolution) {
//     // 计算二维平面栅格数量
//     float x_max = 0, y_max = 0;
//     for (const auto& point : pointCloud) {
//         x_max = std::max(x_max, point.x);
//         y_max = std::max(y_max, point.y);
//     }
//     int num_rows = std::ceil(x_max / grid_resolution);
//     int num_cols = std::ceil(y_max / grid_resolution);

//     // 创建二维图像
//     cv::Mat image(num_rows, num_cols, CV_8UC1, cv::Scalar(0));

//     // 统计每个栅格的点云数量
//     std::vector<int> grid_counts(num_rows * num_cols, 0);
//     for (const auto& point : pointCloud) {
//         int row = std::floor(point.x / grid_resolution);
//         int col = std::floor(point.y / grid_resolution);
//         if (row >= 0 && row < num_rows && col >= 0 && col < num_cols) {
//             grid_counts[row * num_cols + col]++;
//         }
//     }

//     // 归一化处理
//     int max_count = *std::max_element(grid_counts.begin(), grid_counts.end());
//     for (int i = 0; i < num_rows; ++i) {
//         for (int j = 0; j < num_cols; ++j) {
//             int count = grid_counts[i * num_cols + j];
//             image.at<uchar>(i, j) = static_cast<uchar>(255-255 * count*2 / max_count);
//             image.at<uchar>(i-1, j) = static_cast<uchar>(255-255 * count*2 / max_count);
//             image.at<uchar>(i, j-1) = static_cast<uchar>(255-255 * count*2 / max_count);
//             image.at<uchar>(i-1, j) = static_cast<uchar>(255-255 * count*2 / max_count);
//         }
//     }

//     return image;
// }

// int main() {
//     // 生成随机的三维点云数据
//     std::vector<Point3D> pointCloud;
//     for (int i = 0; i < 2000; ++i) {
//         pointCloud.push_back({static_cast<float>(rand()) / RAND_MAX*10 , static_cast<float>(rand()) / RAND_MAX*10, 0});
//     }

//     // 投影点云到二维平面并统计数量，栅格分辨率为5厘米
//     float grid_resolution = 0.05;
//     cv::Mat image = projectAndCountPointCloud(pointCloud, grid_resolution);

//     // 放大图像
//     cv::Mat enlarged_image;
//     cv::resize(image, enlarged_image, cv::Size(), 5, 5); // 放大5倍
//     std::cout<<image.size().height<<"  "<<image.size().width<<std::endl;

//     // 显示结果
//     cv::imshow("Point Cloud Projection", enlarged_image);
//     cv::waitKey(0);

//     return 0;
// }
using namespace sp_decision;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_process");
    ros::NodeHandle pnh("~");
    CloudTracker tracker;
    ros::spin();
}