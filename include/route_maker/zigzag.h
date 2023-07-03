#ifndef ZIGZAG_H
#define ZIGZAG_H

#include <ros/ros.h>
#include <string>
#include <cmath>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "brushee_navigation_msgs/Node.h"
#include "brushee_navigation_msgs/Edge.h"
#include "brushee_navigation_msgs/NodeEdge.h"

class Zigzag
{
    public:
        Zigzag();
        void process();

    private:
        //struct
        struct point
        {
            double x;
            double y;
        };
        struct node
        {
            int id;
            int type;
            geometry_msgs::Pose pose;
            std::string direction;
        };
        struct edge
        {
            int command;
            int start;
            int end;
            bool skippable;
        };
        //method
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void clicked_callback(const geometry_msgs::PointStamped::ConstPtr &msg);

        void map_roughther(nav_msgs::OccupancyGrid map);
        void path_maker(nav_msgs::OccupancyGrid map, double move_direction, point start, point goal);
        void set_turn_point(point p, double move_direction, point& turn_point);
        void reset_turn_point(point p, double move_direction, point& turn_point);
        double set_moving_direction(nav_msgs::OccupancyGrid map, double current_direction, point current_point);
        double get_distance(point a, point b);
        double adjust_angle(double angle);
        void publish_path();
        void fill_in_msg(std::vector<node> nodes, brushee_navigation_msgs::NodeEdge path);
        geometry_msgs::Pose point_to_rosmsg(point p);
        int xy_to_map(nav_msgs::OccupancyGrid map, double x, double y);
        bool check_is_closed(point p);

        //param
        point clicked_;
        point start_;

        std::vector<point> closed_;
        std::vector<point> open_;

        double width_; //[m]
        double length_; //[m]

        bool get_map_;
        bool is_invalid_;

        int clicked_counter_;
        double move_direction_;
        double moved_direction_;
        double v_grav_;

        std::string directions_[2] = {"head","keep"};
        std::vector<edge> edges_;
        std::vector<node> nodes_;

        //member
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_map_;
        ros::Subscriber sub_clicked_;
        ros::Publisher pub_node_edge_;
        ros::Publisher pub_path_;
        ros::Publisher pub_passed_map_;

        nav_msgs::OccupancyGrid rough_map_;
        nav_msgs::OccupancyGrid raw_map_;
        nav_msgs::OccupancyGrid passed_map_;
        geometry_msgs::PointStamped clicked_real_;
        nav_msgs::Path nav_path_;

        brushee_navigation_msgs::Node node_msg_;
};
#endif // ZIGZAG_H

