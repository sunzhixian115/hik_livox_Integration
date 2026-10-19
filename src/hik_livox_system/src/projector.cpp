#include <rclcpp/rclcpp.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <dist_msg/msg/dist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

using Image = sensor_msgs::msg::Image;
using Detections = vision_msgs::msg::Detection2DArray;
using Cloud = livox_interfaces::msg::CustomMsg;

class Projector : public rclcpp::Node {
 public:
  Projector() : Node("projector") {
    auto camera = declare_parameter("camera_topic", "/image_raw");
    auto detection = declare_parameter("detection_topic", "/armor_detections");
    auto lidar = declare_parameter("lidar_topic", "/livox/stamped");
    debug_ = declare_parameter("debug", true);
    bbox_scale_ = declare_parameter("bbox_size", 0.8);
    max_age_ = declare_parameter("max_data_age", 0.5);
    max_image_age_ = declare_parameter("max_image_age", 0.2);
    max_debug_ = declare_parameter("debug_max_points", 5000);
    debug_scale_ = declare_parameter("debug_scale", 0.5);
    debug_rate_ = declare_parameter("debug_rate", 10.0);
    width_ = declare_parameter("image_width", 1440);
    height_ = declare_parameter("image_height", 1080);
    if (max_age_ <= 0 || max_image_age_ <= 0 || max_debug_ < 0 ||
        debug_scale_ <= 0 || debug_scale_ > 1 || debug_rate_ <= 0 ||
        width_ <= 0 || height_ <= 0 || bbox_scale_ <= 0 || bbox_scale_ > 1)
      throw std::runtime_error("Invalid projector limits");
    auto intrinsic = declare_parameter("intrinsic_path", "");
    auto extrinsic = declare_parameter("extrinsic_path", "");
    auto k = read_matrix(intrinsic, 9);
    auto t = read_matrix(extrinsic, 16);
    for (int i=0; i<9; ++i) k_.val[i]=k[i];
    for (int i=0; i<16; ++i) t_.val[i]=t[i];
    pub_ = create_publisher<dist_msg::msg::Dist>("/distances", 10);
    detection_sub_ = create_subscription<Detections>(detection, 1,
      [this](Detections::ConstSharedPtr msg){ detections_=msg; });
    if (debug_) {
      image_sub_ = create_subscription<Image>(camera, rclcpp::SensorDataQoS().keep_last(2),
        [this](Image::ConstSharedPtr msg){
          images_.push_back(msg);
          while (images_.size()>20) images_.pop_front();
        });
      debug_pub_ = create_publisher<Image>("/debug_image", 2);
    }
    cloud_sub_ = create_subscription<Cloud>(lidar, 10,
      std::bind(&Projector::process, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Native single-frame projector: full points for range, %.0f%% image, <=%d drawn points",
                debug_scale_*100, max_debug_);
  }
 private:
  struct Point { double range, u, v; };
  static std::vector<double> read_matrix(const std::string &path, size_t count) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open calibration: "+path);
    std::vector<double> values; std::string line;
    while (std::getline(in,line) && values.size()<count) {
      std::istringstream row(line); double x;
      while (row>>x) values.push_back(x);
    }
    if (values.size()!=count || !std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x);}))
      throw std::runtime_error("Invalid calibration: "+path);
    return values;
  }
  static double seconds(const builtin_interfaces::msg::Time &s) {
    return s.sec+1e-9*s.nanosec;
  }
  void process(Cloud::ConstSharedPtr cloud) {
    auto start=std::chrono::steady_clock::now();
    const double stamp=seconds(cloud->header.stamp);
    auto detections=detections_;
    bool fresh=detections && std::abs(stamp-seconds(detections->header.stamp))<=max_age_;
    std::vector<Point> projected;
    projected.reserve(cloud->points.size());
    for (const auto &p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      auto c=t_*cv::Vec4d(p.x,p.y,p.z,1.0);
      if (!std::isfinite(c[2]) || c[2]<=1e-6) continue;
      auto uv=k_*cv::Vec3d(c[0],c[1],c[2]);
      double u=uv[0]/uv[2], v=uv[1]/uv[2];
      if (!std::isfinite(u) || !std::isfinite(v) || u<0 || u>=width_ || v<0 || v>=height_) continue;
      projected.push_back({std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z),u,v});
    }
    dist_msg::msg::Dist result;
    result.header=cloud->header;
    size_t hits=0;
    if (fresh) {
      for (const auto &d : detections->detections) {
        if (d.results.empty() || result.obj_classes.size()>=255) continue;
        double x=d.bbox.center.position.x, y=d.bbox.center.position.y;
        double w=d.bbox.size_x*bbox_scale_/2, h=d.bbox.size_y*bbox_scale_/2;
        std::vector<double> ranges;
        for (const auto &p : projected)
          if (p.u>=x-w && p.u<=x+w && p.v>=y-h && p.v<=y+h) ranges.push_back(p.range);
        double median=std::numeric_limits<double>::quiet_NaN();
        if (!ranges.empty()) {
          hits+=ranges.size();
          auto mid=ranges.begin()+ranges.size()/2;
          std::nth_element(ranges.begin(),mid,ranges.end()); median=*mid;
          if (ranges.size()%2==0) median=(median+*std::max_element(ranges.begin(),mid))/2;
        }
        result.obj_classes.push_back(d.results.front().hypothesis.class_id);
        result.distances.push_back(median);
      }
    }
    result.count=result.distances.size(); pub_->publish(result);
    // Rendering never gates the full-point distance computation.
    if (debug_ && !images_.empty() && stamp-last_debug_>=0.9/debug_rate_) {
      auto image=images_.back();
      // Prefer the source image of the detections when it is still within the age limit.
      double target=fresh?seconds(detections->header.stamp):stamp;
      for (const auto &candidate:images_)
        if (std::abs(seconds(candidate->header.stamp)-target)<std::abs(seconds(image->header.stamp)-target)) image=candidate;
      if (std::abs(stamp-seconds(image->header.stamp))>max_image_age_) image=images_.back();
      if (std::abs(stamp-seconds(image->header.stamp))<=max_image_age_) {
        auto view=cv_bridge::toCvShare(image,"rgb8"); cv::Mat output;
        cv::resize(view->image,output,cv::Size(),debug_scale_,debug_scale_,cv::INTER_AREA);
        size_t step=max_debug_>0?std::max<size_t>(1,(projected.size()+max_debug_-1)/max_debug_):1;
        for(size_t i=0;i<projected.size();i+=step) {
          const auto &p=projected[i];
          int u=int(p.u*output.cols/image->width), v=int(p.v*output.rows/image->height);
          if (u<0 || v<0 || u>=output.cols || v>=output.rows) continue;
          double f=std::min(1.0,p.range/60.0);
          output.at<cv::Vec3b>(v,u)=cv::Vec3b(255*f,255*(1-f),255*(1-f));
        }
        debug_pub_->publish(*cv_bridge::CvImage(image->header,"rgb8",output).toImageMsg());
        last_debug_=stamp;
      }
    }
    auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,
      "points=%zu valid=%zu boxes=%u hits=%zu detection_age=%.3fs processing=%.2fms",
      cloud->points.size(),projected.size(),result.count,hits,
      detections?stamp-seconds(detections->header.stamp):-1.0,ms);
  }
  bool debug_; double bbox_scale_,max_age_,max_image_age_,debug_scale_,debug_rate_,last_debug_=0;
  int max_debug_,width_,height_; cv::Matx33d k_; cv::Matx44d t_;
  Detections::ConstSharedPtr detections_; std::deque<Image::ConstSharedPtr> images_;
  rclcpp::Publisher<dist_msg::msg::Dist>::SharedPtr pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<Detections>::SharedPtr detection_sub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv); cv::setNumThreads(1);
  rclcpp::spin(std::make_shared<Projector>()); rclcpp::shutdown();
}
