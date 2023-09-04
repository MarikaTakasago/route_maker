#include "route_maker/map_expander.h"

MapExpander::MapExpander(): private_nh_("~")
{
    //param
    private_nh_.param("width",width_,0.6); //meter
    private_nh_.param("length",length_,0.8); //meter

    //sub&pub
    sub_map_ = nh_.subscribe("/map",10,&MapExpander::map_callback,this);
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/expanded_map",10);
    is_received_ = false;
}

void MapExpander::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!is_received_) std::cout<<"get map"<<std::endl;
    is_received_ = true;
    nav_msgs::OccupancyGrid raw = *msg;
    nav_msgs::OccupancyGrid edited;
    edited_map_.header = raw.header;
    edited_map_.info = raw.info;
    edited_map_.data.resize(edited_map_.info.width*edited_map_.info.height,-1);
    for(int i=0;i<raw.info.width*raw.info.height;i++) edited_map_.data[i] = raw.data[i];

    make_fat_map(raw,edited_map_);
}

void MapExpander::make_fat_map(nav_msgs::OccupancyGrid raw, nav_msgs::OccupancyGrid &edit)
{
    std::cout<<"start expansion"<<std::endl;
    int len_x = int(width_/raw.info.resolution)/2;
    int len_y = int(length_/raw.info.resolution)/2;
    for(int i=0;i<raw.info.width;i++)
    {
        for(int j=0;j<raw.info.height;j++)
        {
            if(raw.data[i+j*raw.info.width]==100)
            {
                for(int k=-len_x;k<=len_x;k++)
                {
                    for(int l=-len_x;l<=len_x;l++)
                    {
                        if(i+k>=0 && i+k<raw.info.width && j+l>=0 && j+l<raw.info.height)
                        {
                            edit.data[i+k+(j+l)*raw.info.width] = 100;
                        }
                    }
                }
            }
        }
    }
    std::cout<<"end expansion"<<std::endl;
}

void MapExpander::process()
{
    // ros::spin();
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        pub_map_.publish(edited_map_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"map_expander");
    MapExpander map_expander;
    map_expander.process();
    return 0;
}
