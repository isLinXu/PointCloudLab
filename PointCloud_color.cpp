//
// Created by linxu on 2022/3/3.
//

#include "PointCloud_color.h"


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

int	main (int argc,char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  //加载点云
    pcl::io::loadPCDFile("test.pcd", *cloud);

    cv::Mat img = cv::imread("test.jpeg");  //读入图片

    int length = img.rows * img.cols;
    uchar *R_array = new uchar[length]; //动态创建数组存放rgb值
    uchar*G_array = new uchar[length];
    uchar*B_array = new uchar[length];

    for (int r = 0; r < img.rows; r++) //遍历图片存放rgb值
    {
        for (int c = 0; c < img.cols; c++)
        {
            B_array[r*img.cols+c] = img.at<cv::Vec3b>(r, c)[0];//注意opencvz中Scalar(B,G,R)不是R，G，B这个顺序
            G_array[r*img.cols+c] = img.at<cv::Vec3b>(r, c)[1];
            R_array[r*img.cols+c] = img.at<cv::Vec3b>(r, c)[2];
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGBA(new pcl::PointCloud<pcl::PointXYZRGB>);  //创建XYZRGB点云
    cloud_RGBA->width = length;  //点云初始化，点云数量和图像像素数量一样
    cloud_RGBA->height = 1;
    cloud_RGBA->is_dense = true;
    cloud_RGBA->resize(length);
    for(int t = 0; t < length; t++) //给点云赋值（RGB值和XYZ坐标）
    {
        cloud_RGBA->points[t].r = R_array[t];
        cloud_RGBA->points[t].g = G_array[t];
        cloud_RGBA->points[t].b = B_array[t];
        cloud_RGBA->points[t].x = cloud->points[t].x;
        cloud_RGBA->points[t].y = cloud->points[t].y;
        cloud_RGBA->points[t].z = cloud->points[t].z;
    }

    cv::imshow("image", img);  //显示图p片

    pcl::visualization::PCLVisualizer viewer;  //显示点云
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_RGBA);
    viewer.addCoordinateSystem(1000.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
