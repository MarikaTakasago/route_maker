#include "route_maker/zigzag.h"

Zigzag::Zigzag(): private_nh_("~")
{
    //param
    private_nh_.param("width",width_,0.1);
    private_nh_.param("length",length_,0.8);

    //sub&pub
    sub_map_ = nh_.subscribe("/map",10,&Zigzag::map_callback,this);
    sub_clicked_ = nh_.subscribe("/clicked_point",10,&Zigzag::clicked_callback,this);

    pub_node_edge_ = nh_.advertise<brushee_navigation_msgs::NodeEdge>("/new_node_edge",1);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/new_path",1);
    pub_passed_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/passed_map",1);

    //flag
    get_map_ = false;
    clicked_counter_ = 0;
    is_invalid_ = false;

    //header
    nav_path_.header.frame_id ="map";
}

void Zigzag::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    raw_map_ = *msg;
    passed_map_ = *msg;
    map_roughther(raw_map_);
    std::cout<<"get map"<<std::endl;
    get_map_ = true;
}

void Zigzag::clicked_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    clicked_real_ = *msg;
    if(get_map_)
    {
        std::cout<<"click!!"<<std::endl;
        //get clicked_point
        float x = std::ceil(clicked_real_.point.x * 100) / 100; //if raw x = 1.22222, x = 1.22
        float y = std::ceil(clicked_real_.point.y * 100) / 100;
        clicked_.x = double(x);
        clicked_.y = double(y);
        clicked_counter_++;

        switch(clicked_counter_)
        {
            case 1:
                start_ = clicked_;
                std::cout<<"get start point ("<<clicked_.x<<","<<clicked_.y<<")"<<std::endl;
                break;

            case 2:
                move_direction_ = std::atan2(clicked_.y - start_.y, clicked_.x - start_.x);
                std::cout<<"get move direction: "<<move_direction_<<std::endl;
                break;

            case 3:
                std::cout<<"get goal point ("<<clicked_.x<<","<<clicked_.y<<")"<<std::endl;
                path_maker(raw_map_,move_direction_,start_,clicked_);
                publish_path();
                clicked_counter_ = 0; //reset counter
                break;

            default:
                std::cout<<"now in default..."<<std::endl;
                break;
        }
    }
}

void Zigzag::map_roughther(nav_msgs::OccupancyGrid raw_map)
{

}

void Zigzag::path_maker(nav_msgs::OccupancyGrid map, double move_direction, point start, point goal)
{
    // put start point
    geometry_msgs::Pose pose = point_to_rosmsg(start);
    struct node start_node = {0,0,pose,directions_[1]};
    nodes_.push_back(start_node);
    struct point p = {start.x,start.y};
    passed_map_.data[xy_to_map(map,start.x,start.y)] = 100;
    closed_.push_back(p);
    std::cout<<"closed start point"<<std::endl;
    std::cout<<"node0("<<nodes_[0].pose.position.x<<","<<nodes_[0].pose.position.y<<")"<<std::endl;
    geometry_msgs::PoseStamped pose_st;
    pose_st.pose = pose;
    nav_path_.poses.push_back(pose_st);
    pub_path_.publish(nav_path_);

    struct point checking_point;
    checking_point.x = start.x;
    checking_point.y = start.y;
    checking_point.x += map.info.resolution * cos(move_direction);
    checking_point.y += map.info.resolution * cos(move_direction);

    //set vurtial gravity direction
    double v_grav_ = adjust_angle(set_moving_direction(map,move_direction,start)+M_PI);

    // start making cleaning path
    // start from "start point" and move to "move_direction" direction untill hit obstacle
    // then, move to "move_direction + 90" direction, untill move width/2
    // then, move to "move_direction + 180" direction, untill hit objstacle
    // repeat this untill all cell closed
    int i = 1;
    double moving_direction = move_direction;
    double line_length = 0;
    double old_line_length = 0;
    bool long_move = false;
    while(1)
    {
        //move to move_direction(cleaning line)
        double moving_direction = move_direction;
        old_line_length = line_length;
        line_length = 0;
        bool go_forward = 0;
        std::cout<<"old_line_length: "<<old_line_length<<std::endl;
        while(1)
        {
            if(i>1)
            {
                double old_direction = adjust_angle(std::atan2(nodes_[i-2].pose.position.y - nodes_[i-3].pose.position.y,nodes_[i-2].pose.position.x - nodes_[i-3].pose.position.x));
                // std::cout<<"old: "<<old_direction<<std::endl;
                if(!long_move && fabs(move_direction - old_direction) < 0.01) moving_direction = move_direction + M_PI;
                if(long_move)
                {
                    moving_direction = old_direction;
                    std::cout<<"continue cleaning"<<std::endl;
                    std::cout<<"old_line_length: "<<old_line_length<<std::endl;
                }
            }
            //check if next cell is obstacle
            checking_point.x += map.info.resolution * std::cos(moving_direction);
            checking_point.y += map.info.resolution * std::sin(moving_direction);
            line_length += map.info.resolution;
            if(i>1 && old_line_length + length_/2 <= line_length && !go_forward)
            {
                std::cout<<"long??"<<std::endl;
                geometry_msgs::Pose pose = point_to_rosmsg(checking_point);
                struct node n = {i,0,pose,directions_[1]};
                nodes_.push_back(n);
                closed_.push_back(checking_point);
                std::cout<<"go_down_node"<<i<<"("<<nodes_[i].pose.position.x<<","<<nodes_[i].pose.position.y<<")"<<std::endl;
                geometry_msgs::PoseStamped pose_st;
                pose_st.pose = pose;
                nav_path_.poses.push_back(pose_st);
                pub_path_.publish(nav_path_);
                i++;
                //if gravity direction is unclosed move gravity direction
                struct point grav_p;
                grav_p.x = checking_point.x + map.info.resolution * std::cos(v_grav_);
                grav_p.y = checking_point.y + map.info.resolution * std::sin(v_grav_);
                std::cout<<"x,y: "<<grav_p.x<<","<<grav_p.y<<std::endl;
                double lm_length = map.info.resolution;
                while(1)
                {
                    grav_p.x += map.info.resolution * std::cos(v_grav_);
                    grav_p.y += map.info.resolution * std::sin(v_grav_);
                    lm_length += map.info.resolution;
                    if(passed_map_.data[xy_to_map(passed_map_,grav_p.x,grav_p.y)] == 100)
                    {
                        if(lm_length <= width_*2)
                        {
                            go_forward = 1;
                            break;
                        }
                        else
                        {
                            checking_point.x = grav_p.x - 0.3 * std::cos(v_grav_);
                            checking_point.y = grav_p.y - 0.3 * std::sin(v_grav_);

                            geometry_msgs::Pose pose = point_to_rosmsg(checking_point);
                            struct node n = {i,0,pose,directions_[1]};
                            nodes_.push_back(n);
                            closed_.push_back(checking_point);
                            std::cout<<"long_move_end_node"<<i<<"("<<nodes_[i].pose.position.x<<","<<nodes_[i].pose.position.y<<")"<<std::endl;
                            geometry_msgs::PoseStamped pose_st;
                            pose_st.pose = pose;
                            nav_path_.poses.push_back(pose_st);
                            pub_path_.publish(nav_path_);
                            i++;
                            long_move = true;
                            std::cout<<"long_move"<<std::endl;
                            old_line_length = line_length;
                            line_length = 0;
                            std::cout<<"old_line_length: "<<old_line_length<<std::endl;
                            break;
                        }
                    }
                }
            }
            // if invalid index
            int next_index = xy_to_map(map,checking_point.x,checking_point.y);
            if(next_index < 0 || next_index > map.data.size())
            {
                std::cout<<"invalid index"<<std::endl;
                is_invalid_ = true;
                break;
            }
            //if hit obstacle
            else if(map.data[next_index] == 100)
            {
                // put checking_point in nodes
                // consider length between base_link and head
                checking_point.x -= length_/2 * std::cos(moving_direction);
                checking_point.y -= length_/2 * std::sin(moving_direction);
                geometry_msgs::Pose pose = point_to_rosmsg(checking_point);
                struct node n = {i,0,pose,directions_[0]};
                nodes_.push_back(n);
                closed_.push_back(checking_point);
                passed_map_.data[xy_to_map(map,checking_point.x,checking_point.y)] = 100;
                std::cout<<"hit_obs_node"<<i<<"("<<nodes_[i].pose.position.x<<","<<nodes_[i].pose.position.y<<")"<<std::endl;
                geometry_msgs::PoseStamped pose_st;
                pose_st.pose = pose;
                nav_path_.poses.push_back(pose_st);
                pub_path_.publish(nav_path_);
                pub_passed_map_.publish(passed_map_);
                i++;
                long_move = false;
                break;
            }
            //if not obstacle
            else
            {
                closed_.push_back(checking_point);
                passed_map_.data[xy_to_map(map,checking_point.x,checking_point.y)] = 100;
            }
        }
        if(is_invalid_)
        {
            std::cout<<"!!!!! invalid index !!!!!"<<std::endl;
            break;
        }


        double next_moving_direction = set_moving_direction(map,moving_direction,checking_point);
        if(next_moving_direction == -100 || fabs(next_moving_direction - v_grav_)<0.1)
        {
            std::cout<<"path end"<<std::endl;
            break;
        }

        //move (only running)
        std::cout<<"only run"<<std::endl;
        point run_start = checking_point;
        double distance = get_distance(checking_point, run_start);
        long_move = false;
        while(1)
        {
            //check if next cell is obstacle
            struct point next_point;
            next_point.x = checking_point.x + map.info.resolution * std::cos(next_moving_direction);
            next_point.y = checking_point.y + map.info.resolution * std::sin(next_moving_direction);
            distance += get_distance(checking_point,next_point);
            int next_index = xy_to_map(map,next_point.x,next_point.y);
            // if invalid index
            if(next_index < 0 || next_index > map.data.size())
            {
                std::cout<<"invalid index"<<std::endl;
                is_invalid_ = true;
                break;
            }
            if(distance >= width_ || map.data[next_index] == 100)
            {
                // put checking_point in nodes
                geometry_msgs::Pose pose = point_to_rosmsg(checking_point);
                struct node n = {i,0,pose,directions_[1]};
                nodes_.push_back(n);
                closed_.push_back(checking_point);
                passed_map_.data[xy_to_map(map,checking_point.x,checking_point.y)] = 100;
                std::cout<<"start_cleaning_node"<<i<<"("<<nodes_[i].pose.position.x<<","<<nodes_[i].pose.position.y<<")"<<std::endl;
                geometry_msgs::PoseStamped pose_st;
                pose_st.pose = pose;
                nav_path_.poses.push_back(pose_st);
                pub_path_.publish(nav_path_);
                pub_passed_map_.publish(passed_map_);
                i++;
                break;
            }
            else
            {
                closed_.push_back(checking_point);
                double close_x1 = checking_point.x;
                double close_y1 = checking_point.y;
                double close_x2 = checking_point.x;
                double close_y2 = checking_point.y;
                double closed_len = 0;
                int flag = 0;
                while(1)
                {
                    if(closed_len < length_/2)
                    {
                        passed_map_.data[xy_to_map(map,close_x1,close_y1)] = 100;
                        close_x1 += map.info.resolution * std::cos(moving_direction);
                        close_y1 += map.info.resolution * std::sin(moving_direction);
                    }
                    else break;
                    // if(closed_len < line_length)
                    // {
                    //     passed_map_.data[xy_to_map(map,close_x,close_y)] = 100;
                    //     close_x += map.info.resolution * std::cos(moving_direction);
                    //     close_y += map.info.resolution * std::sin(moving_direction);
                    // }
                    closed_len += map.info.resolution;
                    // if(closed_len >line_length) flag =2;
                }
                checking_point.x = next_point.x;
                checking_point.y = next_point.y;
            }
        }
        if(is_invalid_)
        {
            std::cout<<"!!!!! invalid index !!!!!"<<std::endl;
            break;
        }
    }
}

geometry_msgs::Pose Zigzag::point_to_rosmsg(point p)
{
    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    return pose;
}

void Zigzag::set_turn_point(point p, double moving_direction, point& turn_point)
{
    turn_point.x = p.x - length_ / 2 * std::cos(moving_direction);
    turn_point.y = p.y - length_ / 2 * std::sin(moving_direction);
}
void Zigzag::reset_turn_point(point p, double moving_direction, point& turn_point)
{
    turn_point.x = p.x + length_ / 2 * std::cos(moving_direction);
    turn_point.y = p.y + length_ / 2 * std::sin(moving_direction);
}

double Zigzag::set_moving_direction(nav_msgs::OccupancyGrid map,double current_direction,point current_point)
{
    // deside turn right or turn left
    double next_direction1 = current_direction + M_PI/2;
    double next_direction2 = current_direction - M_PI/2;
    point v1_next = current_point;
    point v2_next = current_point;
    double distance1 = get_distance(v1_next,current_point);
    double distance2 = get_distance(v2_next,current_point);
    bool is_wall1 = false;
    bool is_wall2 = false;
    bool is_closed1 = false;
    bool is_closed2 = false;

    while(distance1 <= width_ + 0.1|| distance2 <= width_ + 0.1)
    {

        v1_next.x += map.info.resolution * std::cos(next_direction1);
        v1_next.y += map.info.resolution * std::sin(next_direction1);
        distance1 += map.info.resolution;
        int v1_index = xy_to_map(map,v1_next.x, v1_next.y);
        is_closed1 = check_is_closed(v1_next);
        if(map.data[v1_index] != 0 || is_closed1) is_wall1 = true;

        v2_next.x += map.info.resolution * std::cos(next_direction2);
        v2_next.y += map.info.resolution * std::sin(next_direction2);
        distance2 += map.info.resolution;
        int v2_index = xy_to_map(map,v2_next.x,v2_next.y);
        is_closed2 = check_is_closed(v2_next);
        if(map.data[v2_index] != 0 || is_closed2) is_wall2 = true;

        if(is_wall1 || is_wall2) break;
    }

    std::cout<<"current_direction: "<<current_direction<<", d1: "<<next_direction1<<", d2: "<<next_direction2<<std::endl;
    std::cout<<"is1: "<<is_wall1<<", is2: "<<is_wall2<<std::endl;

    if(is_wall1 && is_wall2) return -100;
    if(is_wall1) return (next_direction2);
    if(is_wall2) return (next_direction1);
    if(!is_wall1 && !is_wall2)
    {
        double up_dir = next_direction1;
        if(fabs(adjust_angle(next_direction2)-v_grav_) < fabs(adjust_angle(next_direction1)-v_grav_)) up_dir = next_direction2;
        return up_dir;
    }
}

double Zigzag::adjust_angle(double angle)
{
    //adjust -M_PI ~ M_PI;
    double new_angle = angle;
    while(new_angle < -M_PI) new_angle += 2*M_PI;
    while(new_angle > M_PI) new_angle -= 2*M_PI;
    if(-M_PI < new_angle && new_angle < M_PI) return new_angle;
    else return new_angle+2*M_PI;
    // while(new_angle<0) new_angle += 2*M_PI;
    // while (new_angle>2*M_PI) new_angle-=2*M_PI;
    // return new_angle;
}

double Zigzag::get_distance(point a, point b)
{
    return std::sqrt(std::pow(a.x - b.x,2) + std::pow(a.y - b.y,2));
}

void Zigzag::publish_path()
{
    //make edges
    int command = 1;
    int size = nodes_.size();
    std::vector<brushee_navigation_msgs::Edge> edges;
    for(int i=0;i<size;i++)
    {
        brushee_navigation_msgs::Edge edge;
        if(i == 0 || i == size-1) command = 0;
        edge.command = command;
        edge.start_node_id = nodes_[i].id;
        edge.end_node_id = nodes_[i+1].id;
        edge.skippable = false;
        edges.push_back(edge);
    }

    //publish path
    brushee_navigation_msgs::NodeEdge path;
    fill_in_msg(nodes_,path);
    path.edges = edges;
    pub_node_edge_.publish(path);
}

void Zigzag::fill_in_msg(std::vector<node> nodes,brushee_navigation_msgs::NodeEdge path)
{
    for(auto n : nodes)
    {
        brushee_navigation_msgs::Node node;
        node.id = n.id;
        node.type = n.type;
        node.pose = n.pose;
        node.direction = n.direction;
        path.nodes.push_back(node);
    }
}


int Zigzag::xy_to_map(nav_msgs::OccupancyGrid map, double x, double y)
{
    int index = (int)((y - map.info.origin.position.y) / map.info.resolution) * map.info.width + (int)((x - map.info.origin.position.x) / map.info.resolution);
    return index;
}

bool Zigzag::check_is_closed(point p)
{
    // if(passed_map_.data[xy_to_map(passed_map_,p.x,p.y)] == 100) return true;
    // return false;

    for(int i=0;i<closed_.size();i++)
    {
        if(fabs(closed_[i].x-p.x)<0.001 && fabs(p.y-closed_[i].y)<0.001) return true;
    }
    return false;
}

void Zigzag::process()
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"zigzag");
    Zigzag zigzag;
    zigzag.process();
    return 0;
}
