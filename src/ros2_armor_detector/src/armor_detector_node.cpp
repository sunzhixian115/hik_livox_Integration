#include "network_deployment_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class ArmorDetectorNode final : public rclcpp::Node
{
public:
    ArmorDetectorNode()
        : Node("armor_detector")
    {
        const std::string model_config = declare_parameter(
            "model_config",
            "/home/szx/works/hik_livox_workspace/src/ros2_armor_detector/config/armor_v8_cpu.json");
        const std::string model_root = declare_parameter(
            "model_root",
            "/home/szx/works/hik_livox_workspace/models/openvino");
        const std::string model_key = declare_parameter("model_key", "armor_v8");
        const std::string input_topic = declare_parameter("input_topic", "/image_raw");
        const std::string detection_topic = declare_parameter("detection_topic", "/armor_detections");
        const std::string debug_topic = declare_parameter("debug_topic", "/armor_debug");
        my_color_ = declare_parameter("my_color", 2);
        publish_debug_ = declare_parameter("publish_debug", true);
        cv::setNumThreads(1);
        const int cpu_threads = declare_parameter("cpu_threads", 4);
        if (cpu_threads < 0)
            throw std::invalid_argument("cpu_threads must be non-negative");
        const auto threads_value = std::to_string(cpu_threads);
        setenv("NNDEPLOYMENT_CPU_THREADS", threads_value.c_str(), 1);

        RCLCPP_INFO(get_logger(), "model config: %s", model_config.c_str());
        RCLCPP_INFO(get_logger(), "model root: %s", model_root.c_str());
        RCLCPP_INFO(get_logger(), "model key: %s", model_key.c_str());
        RCLCPP_INFO(get_logger(), "input topic: %s", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "detection topic: %s", detection_topic.c_str());

        model_ = std::make_unique<ArmorModel>(
            JsonConfig{model_config, model_key, model_root});

        detection_publisher_ = create_publisher<vision_msgs::msg::Detection2DArray>(
            detection_topic, 10);
        if (publish_debug_)
        {
            debug_publisher_ = create_publisher<sensor_msgs::msg::Image>(
                debug_topic, 10);
        }

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            input_topic,
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &message)
    {
        try
        {
            const auto started = std::chrono::steady_clock::now();
            const cv_bridge::CvImageConstPtr cv_image =
                cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
            const std::vector<NetArmorResult> results =
                model_->netProcess(cv_image->image, my_color_);
            const double inference_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "image conversion + model: %.2f ms, detections=%zu", inference_ms, results.size());

            vision_msgs::msg::Detection2DArray detections;
            detections.header = message->header;
            for (const NetArmorResult &result : results)
            {
                if (result.points.empty())
                    continue;

                double min_x = result.points.front().x;
                double max_x = result.points.front().x;
                double min_y = result.points.front().y;
                double max_y = result.points.front().y;
                for (const cv::Point2d &point : result.points)
                {
                    min_x = std::min(min_x, point.x);
                    max_x = std::max(max_x, point.x);
                    min_y = std::min(min_y, point.y);
                    max_y = std::max(max_y, point.y);
                }

                vision_msgs::msg::Detection2D detection;
                detection.header = message->header;
                detection.id = result.class_name;
                detection.bbox.center.position.x = (min_x + max_x) * 0.5;
                detection.bbox.center.position.y = (min_y + max_y) * 0.5;
                detection.bbox.center.theta = 0.0;
                detection.bbox.size_x = std::max(0.0, max_x - min_x);
                detection.bbox.size_y = std::max(0.0, max_y - min_y);

                vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
                hypothesis.hypothesis.class_id = result.class_name.empty()
                    ? std::to_string(result.armor_id)
                    : result.class_name;
                hypothesis.hypothesis.score = result.score;
                detection.results.push_back(hypothesis);
                detections.detections.push_back(detection);
            }
            detection_publisher_->publish(detections);

            if (publish_debug_ && debug_publisher_)
            {
                cv::Mat debug_image = cv_image->image.clone();
                drawResults(debug_image, results);
                debug_publisher_->publish(
                    *cv_bridge::CvImage(message->header, sensor_msgs::image_encodings::BGR8,
                                        debug_image).toImageMsg());
            }
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "armor inference failed: %s", error.what());
        }
    }

    static void drawResults(cv::Mat &image, const std::vector<NetArmorResult> &results)
    {
        for (const NetArmorResult &result : results)
        {
            if (result.points.size() < 4)
                continue;
            for (std::size_t index = 0; index < result.points.size(); ++index)
            {
                const cv::Point start = result.points[index];
                const cv::Point end = result.points[(index + 1) % result.points.size()];
                cv::line(image, start, end, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                cv::circle(image, start, 3, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
            }
            const cv::Point2d &anchor = result.points.front();
            const std::string label = result.class_name + " " +
                std::to_string(static_cast<int>(std::round(result.score * 100.0))) + "%";
            cv::putText(image, label, cv::Point(static_cast<int>(anchor.x),
                                                static_cast<int>(anchor.y) - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1,
                        cv::LINE_AA);
        }
    }

    std::unique_ptr<ArmorModel> model_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
    int my_color_ = 2;
    bool publish_debug_ = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
