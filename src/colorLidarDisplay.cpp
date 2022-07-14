#include "livox_camera_associate/common.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

parameter para;
ros::Publisher pubLaserCloudColor;
ros::Publisher pubCloudImage;

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& pcl, const nav_msgs::OdometryConstPtr& odom)
{
  pcl::PointCloud<PointXYZIT>::Ptr laserCloudIn(new pcl::PointCloud<PointXYZIT>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudColor(new pcl::PointCloud<pcl::PointXYZRGB>());
  cv::Mat imageIn;
  cv::Mat imageOut;
  // Solve all of perception here...
  // ROS_INFO("image time = %f, pcl time = %f\n", image->header.stamp.toSec(), pcl->header.stamp.toSec());

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image,  sensor_msgs::image_encodings::TYPE_8UC3);
  imageIn = cv_ptr->image;

  pcl::fromROSMsg(*pcl, *laserCloudIn);

  cv::Mat rgb_img;

  if (imageIn.type() == CV_8UC3) { // image是RGB图
    rgb_img = imageIn;
  } else if (imageIn.type() == CV_8UC1) { // image是灰度图
    cv::cvtColor(imageIn, rgb_img, cv::COLOR_GRAY2BGR);
  }

  std::vector<cv::Point3f> pts_3d;
  int density = 1;
  int color_intensity_threshold_ = 10;
  // 遍历每个雷达点
  for (size_t i = 0; i < laserCloudIn->size(); i += density) {
    PointXYZIT point = laserCloudIn->points[i];
    // 计算点的深度
    float depth = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    // 有效点的深度不能太远，也不能太近
    // if (depth > 2 && depth < 50 &&
    //     point.intensity >= color_intensity_threshold_) {
    // 存储
    if(depth<30){
      pts_3d.emplace_back(cv::Point3f(point.x, point.y, point.z));
    }
  }

  Eigen::AngleAxisf rotation_vector3;
  rotation_vector3.fromRotationMatrix(para.M_extLivoxToCam.block<3, 3>(0, 0));
  cv::Mat camera_matrix;
  cv::eigen2cv(para.M_camIntrinsicMatrix, camera_matrix);
  Eigen::Matrix<float, 1, 5> eigen_distortion_coeff = Eigen::Map<Eigen::Matrix<float, 1, 5>>(para.distortionCoeff.data());
  cv::Mat distortion_coeff;
  cv::eigen2cv(eigen_distortion_coeff, distortion_coeff);
  cv::Mat r_vec;
  float angle = rotation_vector3.angle();
  Eigen::Matrix<float, 3, 1> angle_vec = angle*rotation_vector3.axis();
  cv::eigen2cv(angle_vec, r_vec);
  cv::Mat t_vec;
  Eigen::Matrix<float, 3, 1> t = para.M_extLivoxToCam.block<3, 1>(0, 3);
  cv::eigen2cv(t, t_vec);

  std::vector<cv::Point2f> pts_2d;
  // 将3d点投影到相机的2d像素面（考虑上针孔模型和畸变参数）
  cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
                    pts_2d);

  // Image的size
  int image_rows = rgb_img.rows;
  int image_cols = rgb_img.cols;

  laserCloudColor->clear();
  for (size_t i = 0; i < pts_2d.size(); i++) {
    // 确保投影点没有超出图像的尺寸
    // cout<<pts_2d[i].x<<", "<<pts_2d[i].y<<endl;
    if (pts_2d[i].x >= 0 && pts_2d[i].x < image_cols && pts_2d[i].y >= 0 &&
        pts_2d[i].y < image_rows) {
      // 获取图像中的原像素
      cv::Scalar color =
          rgb_img.at<cv::Vec3b>((int)pts_2d[i].y, (int)pts_2d[i].x);
      // 判断RGB
      if (color[0] == 0 && color[1] == 0 && color[2] == 0) {
        continue;
      }
      if (pts_3d[i].x > 100) {
        continue;
      }
      // 给点云赋色
      pcl::PointXYZRGB p;
      p.x = pts_3d[i].x;
      p.y = pts_3d[i].y;
      p.z = pts_3d[i].z;
      // p.a = 255;
      p.b = color[0];
      p.g = color[1];
      p.r = color[2];
      laserCloudColor->points.push_back(p);
    }
  }
  laserCloudColor->width = laserCloudColor->points.size();
  laserCloudColor->height = 1;

  Eigen::Vector3f t_odom(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
  Eigen::Quaternionf q_odom(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  for(int i=0; i<laserCloudColor->size(); i++)
  {
    Eigen::Vector3f thisPoint(laserCloudColor->points[i].x, laserCloudColor->points[i].y, laserCloudColor->points[i].z);
    thisPoint = q_odom * thisPoint + t_odom;
    laserCloudColor->points[i].x = thisPoint[0];
    laserCloudColor->points[i].y = thisPoint[1];
    laserCloudColor->points[i].z = thisPoint[2];
  }

  // cout<<laserCloudColor->points.size()<<endl;

  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg(*laserCloudColor, tempCloud);
  tempCloud.header.frame_id = "odom";
  tempCloud.header.stamp = pcl->header.stamp;
  if (pubLaserCloudColor.getNumSubscribers() != 0)
      pubLaserCloudColor.publish(tempCloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  para.load(nh);

  pubLaserCloudColor = nh.advertise<sensor_msgs::PointCloud2>("color_lidar_cloud", 1);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/galaxy_camera/image_raw", 50);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "lio_sam/deskew/cloud_deskewed", 50);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "lio_sam/mapping/odometry", 50);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, pcl_sub, odom_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}