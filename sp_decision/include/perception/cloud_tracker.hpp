#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/surface/mls.h>

namespace sp_decision
{
    class costmap
    {
    public:
        /**
         * @brief 创建costmap
         *
         * @param width 格子数
         * @param height 格子数
         * @param resolution 每个格子的分辨率(m)
         */
        costmap(int width, int height, int resolution)
        {
            this->width = width;
            this->height = height;
            this->resolution = resolution;
            int init_grid = 0;
            for (int i = 0; i < width * height; i++)
            {
                grid.push_back(init_grid);
            }
        }

        int get(int x, int y)
        {
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                return -1;
            }
            return grid[x + y * width];
        }
        std::vector<int> grid; // 按行储存
        int width;
        int height;
        int resolution;
        cv::Mat costmap_image;
    private:
    };
    class CloudTracker
    {
    public:
        CloudTracker();
        void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg);
        void tracked_object_pub();
        void draw_costmap(float x, float y);
        void map_show();

    private:
        int num;
        cv::Rect2d bbox;
        cv::Ptr<cv::Tracker> tracker;
        ros::NodeHandle nh_;
        ros::Subscriber cloud_sub_;
        ros::Publisher tracked_object_pub_;
        cv::Mat costmap; // 投影地图
    };
    CloudTracker::CloudTracker()
    {
        num = 0;
        tracker = cv::TrackerKCF::create();
        cloud_sub_ = nh_.subscribe("/cloud_registered_body", 1, &CloudTracker::cloud_callback, this);
        costmap = cv::Mat(160, 160, CV_8UC1, cv::Scalar(0));
    }
    void CloudTracker::map_show()
    {
        cv::Mat enlarged_image;
        cv::resize(costmap, enlarged_image, cv::Size(), 5, 5); // 放大5倍
        cv::imshow("costmap", enlarged_image);
    }
    void CloudTracker::draw_costmap(float x, float y)
    {
        int radius = 1;
        // 修改半径内的像素值
        for (int i = x - radius; i <= x + radius; ++i)
        {
            for (int j = y - radius; j <= y + radius; ++j)
            {
                // 检查像素是否在图像范围内
                if (i >= 0 && i < costmap.rows && j >= 0 && j < costmap.cols)
                {
                    // 增加像素值
                    costmap.at<uchar>(i, j) += 100 - 20 * abs(x - i + y - j);
                    ROS_INFO("pix %f,", 100 - 20 * abs(x - i + y - j));
                }
            }
        }
    }
    void CloudTracker::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg)
    {
        costmap.setTo(cv::Scalar(0));
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2 ROSPCL_output;
        pcl::fromROSMsg(*laserCloudMsg, *pcl2cloud);
        if ((pcl2cloud->points.size()) == 0)
        {
            return;
        }
        else
        {
            long point_num = 0;
            for (long i = 0; i <= pcl2cloud->points.size(); i = i + 1)
            {
                pcl2cloud->points[i].x *= 20;
                pcl2cloud->points[i].x += 80;
                pcl2cloud->points[i].y *= 20;
                pcl2cloud->points[i].y = -pcl2cloud->points[i].y + 80;

                // ROS_INFO("x:%f,y:%f", pcl2cloud->points[i].x, pcl2cloud->points[i].y);
                if (pcl2cloud->points[i].x >= 0 && pcl2cloud->points[i].x <= 160 && pcl2cloud->points[i].y >= 0 && pcl2cloud->points[i].y <= 160)
                    draw_costmap(pcl2cloud->points[i].x, pcl2cloud->points[i].y);
            }
        }
        if (num == 0)
        {
            bbox = cv::selectROI(costmap, false);
            tracker->init(costmap, bbox);
            num++;
        }
        else
        {
            bool ok = tracker->update(costmap, bbox);
            if (ok)
            {
                // cv::rectangle(costmap, bbox, cv::Scalar(255, 0, 0), 2, 1);
            }
            else
            {
                // 如果跟踪失败，输出错误信息
                std::cerr << "Tracking failure detected." << std::endl;
            }

            // // 显示帧
            // cv::imshow("Tracking", costmap);
        }
        // map_show();
        // cv::waitKey(10);
    }
}