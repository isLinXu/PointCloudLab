//
// Created by linxu on 2022/3/3.
//

#include <iostream>

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

int Pcl_test() {
    std::cout << "Test PCL !!!" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05) {
        for (float angle(0.0); angle <= 360.0; angle += 5.0) {
            pcl::PointXYZRGB point;
            point.x = 0.5 * cosf(pcl::deg2rad(angle));
            point.y = sinf(pcl::deg2rad(angle));
            point.z = z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float *>(&rgb);
            point_cloud_ptr->points.push_back(point);
        }
        if (z < 0.0) {
            r -= 12;
            g += 12;
        } else {
            g -= 12;
            b += 12;
        }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    pcl::visualization::CloudViewer viewer("test");
    viewer.showCloud(point_cloud_ptr);
    while (!viewer.wasStopped()) {};
    return 0;
}


typedef pcl::PointXYZRGB point;

//RGB colour visualisation example
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    //创建3D窗口并添加点云
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}


int Point_Cloud(string pcd_path, string rgb_path) {
    //! 点云对象处理
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloudRGB_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    //点云对象的读取
    pcl::PCDReader reader;

    //读取点云到cloud中
    reader.read(pcd_path, *point_cloud_ptr);

    // 打印点云信息
    std::cerr << "PointCloud before filtering: " << point_cloud_ptr->width * point_cloud_ptr->height
              << " data points (" << pcl::getFieldsList(*point_cloud_ptr) << ").";

    //将XYZI类型转化为XYZRGB类型
    for (int i = 0; i < point_cloud_ptr->points.size(); i++) {
        point p;
        p.x = point_cloud_ptr->points[i].x;
        p.y = point_cloud_ptr->points[i].y;
        p.z = point_cloud_ptr->points[i].z;
        p.r = 0;
        p.g = 0;
        p.b = 0;
        point_cloudRGB_ptr->points.push_back(p);
    }

    // 处理RGB图像
    Mat img = imread(rgb_path);
    if (img.channels() != 3) {
        cout << "RGB pics needed." << endl;
        return 0;
    }
    cout << "Pic loaded." << endl;
    imshow("pic", img);
    int rows = img.rows;
    int cols = img.cols;

    unsigned char red, green, blue;
    float p_u, p_v, p_w;//pics_uv1;(u for cols, v for lines!!!)
    float c_x, c_y, c_z, c_i;//clouds_xyz、intensity;

    Mat P2 = (Mat_<float>(3, 4)
            << 7.215377000000e+02, 0.000000000000e+00, 6.095593000000e+02, 4.485728000000e+01, 0.000000000000e+00, 7.215377000000e+02, 1.728540000000e+02, 2.163791000000e-01, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 2.745884000000e-03);
    Mat R0_rect = (Mat_<float>(4, 4)
            << 9.999239000000e-01, 9.837760000000e-03, -7.445048000000e-03, 0, -9.869795000000e-03, 9.999421000000e-01, -4.278459000000e-03, 0, 7.402527000000e-03, 4.351614000000e-03, 9.999631000000e-01, 0, 0, 0, 0, 1);
    Mat Tr_velo_to_cam = (Mat_<float>(4, 4)
            << 7.533745000000e-03, -9.999714000000e-01, -6.166020000000e-04, -4.069766000000e-03, 1.480249000000e-02, 7.280733000000e-04, -9.998902000000e-01, -7.631618000000e-02, 9.998621000000e-01, 7.523790000000e-03, 1.480755000000e-02, -2.717806000000e-01, 0, 0, 0, 1);

    Mat trans = cv::Mat_<float>(3, 4);
    trans = P2 * R0_rect * Tr_velo_to_cam;
    Mat c_tmp = cv::Mat_<float>(4, 1);
    Mat p_result = cv::Mat_<float>(3, 1);


    for (int nIndex = 0; nIndex < point_cloud_ptr->points.size(); nIndex++) {
        //将点云坐标转化到图像坐标系中
        c_x = point_cloud_ptr->points[nIndex].x;
        c_y = point_cloud_ptr->points[nIndex].y;
        c_z = point_cloud_ptr->points[nIndex].z;

        c_tmp = (Mat_<float>(4, 1) << c_x, c_y, c_z, 1);
        p_result = trans * c_tmp;
        // cout << "mat = " << p_result << endl;

        p_w = p_result.at<float>(2, 0);
        p_u = p_result.at<float>(0, 0);
        p_v = p_result.at<float>(1, 0);
        p_u = (float) (p_u / p_w);
        p_v = (float) (p_v / p_w);

        //RGB图像外的点云设置为粉红色，RGB图像内的点云设置为对应点彩色图像中的颜色
        //注意边界条件是cols-1和rows-1（原代码这里有问题）
        if ((p_u < 0) || (p_u > cols - 1) || (p_v < 0) || (p_v > rows - 1) || (p_w < 0)) {
            point_cloudRGB_ptr->points[nIndex].r = 128;
            point_cloudRGB_ptr->points[nIndex].g = 2;
            point_cloudRGB_ptr->points[nIndex].b = 64;
            continue;
        }
        point_cloudRGB_ptr->points[nIndex].r = img.at<Vec3b>(p_v, p_u)[2];//not (p_u,p_v)!
        point_cloudRGB_ptr->points[nIndex].g = img.at<Vec3b>(p_v, p_u)[1];
        point_cloudRGB_ptr->points[nIndex].b = img.at<Vec3b>(p_v, p_u)[0];
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(point_cloudRGB_ptr);

    // 主循环
    while (!viewer->wasStopped()) {
        viewer->spinOnce();  //运行视图
//        std::sleep(100);
    }
    return (0);
}


int main() {
    std::cout << "Hello, World!" << std::endl;
//    Pcl_test();
    string pcd_path = "/home/linxu/CLionProjects/PointCloudLab/data/test/1.pcd";
    string rgb_pah = "/home/linxu/CLionProjects/PointCloudLab/data/test/000001.png";
    Point_Cloud(pcd_path, rgb_pah);
    return 0;
}
