#ifndef MAP_EXPANDER_H
#define MAP_EXPANDER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>

class MapExpander
{
    public:
        MapExpander();
        void process();

    private:
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void make_fat_map(nav_msgs::OccupancyGrid raw,nav_msgs::OccupancyGrid &edit);

        //param
        double width_;
        double length_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_map_;
        ros::Publisher pub_map_;
};

#endif
