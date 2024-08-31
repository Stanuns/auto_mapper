#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/std_msgs/msg/color_rgba.hpp"
#include "rosidl_runtime_cpp/bounded_vector.hpp"

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;
using std_msgs::msg::ColorRGBA;
using std::to_string;
using std::abs;

using namespace std;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        markerArrayPublisher_ = this->create_publisher<MarkerArray>("/frontiers", 10);
    //   timer_ = this->create_wall_timer(
    //   500ms, std::bind(&MinimalPublisher::timer_callback, this));
        Frontier frontier1, frontier2, frontier3;
        frontier1.centroid.x = 3;
        frontier1.centroid.y = -2;
        frontier2.centroid.x = 4;
        frontier2.centroid.y = -3;
        frontier3.centroid.x = 5;
        frontier3.centroid.y = -4;
        frontier_list.push_back(frontier1);
        frontier_list.push_back(frontier2);
        frontier_list.push_back(frontier3);
        DrawMarkers(frontier_list);

        Frontier frontier4, frontier5;
        frontier4.centroid.x = -3;
        frontier4.centroid.y = -1;
        frontier5.centroid.x = -4;
        frontier5.centroid.y = -4;
        frontier_list2.push_back(frontier4);
        frontier_list2.push_back(frontier5);
        DrawMarkers(frontier_list2);
        // for(int i = 0; i < 5; i++){
        //     ;
        // }
    }

  private:
    void timer_callback()
    {
    //   auto message = std_msgs::msg::String();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // markerArrayPublisher_->publish(message);
        ;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<MarkerArray>::SharedPtr markerArrayPublisher_;
    size_t count_;

    // MarkerArray markersMsg_;
    // int markerId_;
    struct Frontier {
        Point centroid;
        vector<Point> points;
        string getKey() const{to_string(centroid.x) + "," + to_string(centroid.y);}
    };
    vector<Frontier> frontier_list;
    vector<Frontier> frontier_list2;
    int tag = 1;

    void DrawMarkers(const vector<Frontier> &frontiers) {
        // markersMsg_
        MarkerArray markersMsg_;
        int markerId_= 0 ;
        vector<Marker> &mm_markers = markersMsg_.markers;
        ColorRGBA colour;
        colour.r = 1;
        colour.g = 1;
        colour.b = 0;
        colour.a = 1.0;
        
        RCLCPP_INFO(get_logger(), "DrawMarkers:frontiers.size(): %d ", frontiers.size());

        for (const auto &frontier: frontiers) {
            RCLCPP_INFO(get_logger(), "visualising: %f,%f ", frontier.centroid.x, frontier.centroid.y);

            Marker m;

            m.header.frame_id = "map";
            m.header.stamp = now();
            m.frame_locked = true;

            m.action = Marker::ADD;
            m.ns = "frontiers";
            m.id = ++markerId_;
            RCLCPP_INFO(get_logger(), "m.id: %d", m.id );
            m.type = Marker::SPHERE;
            m.pose.position = frontier.centroid;
            m.scale.x = 0.3;
            m.scale.y = 0.3;
            m.scale.z = 0.3;
            m.color = colour;
            mm_markers.push_back(m);           
        }
        if(tag == 2){
            markerArrayPublisher_->publish(markersMsg_);
        }
        tag++;
        RCLCPP_INFO(get_logger(), "debug test .... ");
    }

    // void ClearMarkers() {
    //     for (auto &m: markersMsg_.markers) {
    //         m.action = Marker::DELETE;
    //     }
    //     markerArrayPublisher_->publish(markersMsg_);
    // }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}