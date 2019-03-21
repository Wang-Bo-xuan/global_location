#include <ls_global_location/ls_global_location.h>

LS_SLAM::LS_GLOBAL_LOCSTION::LS_GLOBAL_LOCSTION(ros::NodeHandle *nh)
{
  this->map_sub_ = new ros::Subscriber();
  *this->map_sub_ = nh->subscribe("/map",1,&LS_GLOBAL_LOCSTION::MapCallBack,this);
  this->lidar_sub_ = new ros::Subscriber();
  *this->lidar_sub_ = nh->subscribe("/scan",1,&LS_GLOBAL_LOCSTION::LidarCallBack,this);
  this->init_pose_pub_ = new ros::Publisher();
  *this->init_pose_pub_ = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10);
  this->cost_map_pub_ = new ros::Publisher();
  *this->cost_map_pub_ = nh->advertise<nav_msgs::OccupancyGrid>("/cost_map",10);
  this->socre_map_pub_ = new ros::Publisher();
  *this->socre_map_pub_ = nh->advertise<nav_msgs::OccupancyGrid>("/socre_map",10);

  this->init();
}

LS_SLAM::LS_GLOBAL_LOCSTION::~LS_GLOBAL_LOCSTION()
{
  if(this->map_sub_)
  {
    delete this->map_sub_;
    this->map_sub_ = NULL;
  }

  if(this->lidar_sub_)
  {
    delete this->lidar_sub_;
    this->lidar_sub_ = NULL;
  }

  if(this->init_pose_pub_)
  {
    delete this->init_pose_pub_;
    this->init_pose_pub_ = NULL;
  }

  if(this->cost_map_pub_)
  {
    delete this->cost_map_pub_;
    this->cost_map_pub_ = NULL;
  }

  if(this->socre_map_pub_)
  {
    delete this->socre_map_pub_;
    this->socre_map_pub_ = NULL;
  }
}

void LS_SLAM::LS_GLOBAL_LOCSTION::init(void)
{
  this->get_map_ = false;
  this->get_lidar_ = false;
  this->start_find_ = true;
  this->finish_find_ = false;

  memset(this->cost_map_,-1,sizeof(this->cost_map_));
}

void LS_SLAM::LS_GLOBAL_LOCSTION::MapCallBack(const nav_msgs::OccupancyGrid &msg)
{
  if(false == this->get_map_)
  {
    this->get_map_ = true;

    this->map_.info.resolution = msg.info.resolution;
    this->map_.info.width = msg.info.width;
    this->map_.info.height = msg.info.height;
    this->map_.info.origin.position.x = msg.info.origin.position.x;
    this->map_.info.origin.position.y = msg.info.origin.position.y;

    for(long int i = 0;i < msg.data.size();i ++)
    {
      this->map_.data.push_back(msg.data[i]);
    }

    ROS_INFO("map_size:%ld",this->map_.data.size());
    ROS_INFO("map_height:%d",this->map_.info.height);
    ROS_INFO("map_width:%d",this->map_.info.width);
    ROS_INFO("map_resolution:%.2lf",this->map_.info.resolution);
    ROS_INFO("map_origin_x:%.2lf",this->map_.info.origin.position.x);
    ROS_INFO("map_origin_y:%.2lf",this->map_.info.origin.position.y);
  }
}

void LS_SLAM::LS_GLOBAL_LOCSTION::LidarCallBack(const sensor_msgs::LaserScan &msg)
{
  if(false == this->get_lidar_)
  {
    this->get_lidar_ = true;

    this->lidar_.angle_increment = msg.angle_increment;
    this->lidar_.angle_min = msg.angle_min;
    this->lidar_.angle_max = msg.angle_max;
    this->lidar_.range_min = msg.range_min;
    this->lidar_.range_max = msg.range_max;

    for(int i = 0;i < msg.ranges.size();i ++)
    {
      this->lidar_.ranges.push_back(msg.ranges[i]);
    }

    ROS_INFO("lidar_size:%d",this->lidar_.ranges.size());
    ROS_INFO("lidar_angle_increment:%.2lf",this->lidar_.angle_increment);
    ROS_INFO("lidar_angle_min:%.2lf",this->lidar_.angle_min);
    ROS_INFO("lidar_angle_max:%.2lf",this->lidar_.angle_max);
    ROS_INFO("lidar_range_min:%.2lf",this->lidar_.range_min);
    ROS_INFO("lidar_range_max:%.2lf",this->lidar_.range_max);
  }
}

void LS_SLAM::LS_GLOBAL_LOCSTION::InitPosePublish(void)
{
  this->init_pose_pub_->publish(this->pose_);
  this->finish_find_ = true;
}

void LS_SLAM::LS_GLOBAL_LOCSTION::FindInitPose(void)
{
  if(false == this->get_map_)
  {
    ROS_WARN("can not get the map_data...");

    return ;
  }

  if(false == this->get_lidar_)
  {
    ROS_WARN("can not get the lidar_data...");

    return ;
  }

  if(true == this->start_find_)
  {
    ROS_WARN("start finding");

    this->GenerateGlobalMapCostMatrix();
    this->RetrievalInitPose();

    this->start_find_ = false;
  }
}

void LS_SLAM::LS_GLOBAL_LOCSTION::GenerateGlobalMapCostMatrix(void)
{
  int p = 0,q = 0;
  int fx[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};

  for(int i = 0;i < this->map_.info.height;i ++)
  {
    for(int j = 0;j < this->map_.info.width;j ++)
    {
      if(this->map_.data[i*this->map_.info.width+j] > 90)
      {
        this->cost_map_[i][j] = 0;
        cost_que_[0][q][0] = i;
        cost_que_[0][q][1] = j;
        q ++;
      }
    }
  }

  for(int s = 0;;s ++)
  {
    if(0 == q)
    {
      break;
    }

    for(int i = 0; i < q; i ++)
    {
      int x = cost_que_[s & 1][i][0];
      int y = cost_que_[s & 1][i][1];

      for(int j = 0; j < 4; j ++)
      {
        if(x+fx[j][0] >= 0 && x+fx[j][0] < this->map_.info.height && y+fx[j][1] >= 0 && y+fx[j][1] < this->map_.info.width)
        {
          if(-1 == this->cost_map_[x+fx[j][0]][y+fx[j][1]])
          {
            if(s > 100)
            {
              this->cost_map_[x+fx[j][0]][y+fx[j][1]] = 100;
            }
            else
            {
              this->cost_map_[x+fx[j][0]][y+fx[j][1]] = s + 1;
            }
            cost_que_[~s & 1][p][0] = x+fx[j][0];
            cost_que_[~s & 1][p][1] = y+fx[j][1];
            p ++;
          }
        }
      }
    }
    q = p;
    p = 0;
  }

  this->Display();
}

void LS_SLAM::LS_GLOBAL_LOCSTION::RetrievalInitPose(void)
{
  long mini = 99999999;
  int ansi = 0,ansj = 0,ansk = 0;

  for(int i = 0; i < this->map_.info.height; i += 4)
  {
    for(int j = 0; j < this->map_.info.width; j += 4)
    {
      if(0 == this->map_.data[i*this->map_.info.width+j])
      {
        for(int k = 0;k < 360;k += 15)
        {
          int sum = 0;
          double ck = cos(k*3.14159/180);
          double sk = sin(k*3.14159/180);

          for(int t = 0; t < this->lidar_.ranges.size(); t ++)
          {
            if(this->lidar_.ranges[t] > 30 || std::isnan(this->lidar_.ranges[t]) || std::isinf(this->lidar_.ranges[t]))
            {
              continue ;
            }

            double angle = this->lidar_.angle_min+this->lidar_.angle_increment*t;
            double range = this->lidar_.ranges[t];
            double lx = range*cos(angle);
            double ly = range*sin(angle);
            int x = j + (lx*ck-ly*sk)/this->map_.info.resolution;
            int y = i + (lx*sk+ly*ck)/this->map_.info.resolution;

            if(x < 0 || x >= this->map_.info.width || y < 0 || y >= this->map_.info.height)
            {
              sum += (this->map_.info.width + this->map_.info.height);
            }
            else
            {
              sum += this->cost_map_[y][x];
            }
          }

          if(sum < mini)
          {
            mini = sum;
            ansi = i;
            ansj = j;
            ansk = k;
          }
        }
      }
    }
  }

  this->pose_.header.frame_id = "/map";
  this->pose_.pose.pose.position.x = this->map_.info.origin.position.x + ansj*this->map_.info.resolution;
  this->pose_.pose.pose.position.y = this->map_.info.origin.position.y + ansi*this->map_.info.resolution;
  this->pose_.pose.pose.orientation.x = tf::createQuaternionFromYaw(ansk*3.14159/180).getX();
  this->pose_.pose.pose.orientation.y = tf::createQuaternionFromYaw(ansk*3.14159/180).getY();
  this->pose_.pose.pose.orientation.z = tf::createQuaternionFromYaw(ansk*3.14159/180).getZ();
  this->pose_.pose.pose.orientation.w = tf::createQuaternionFromYaw(ansk*3.14159/180).getW();
  this->InitPosePublish();

  ROS_ERROR("robot's init pose on  map:");
  ROS_ERROR("x:%.2lf",this->pose_.pose.pose.position.x);
  ROS_ERROR("y:%.2lf",this->pose_.pose.pose.position.y);
  ROS_ERROR("yaw:%.2lf",tf::getYaw(this->pose_.pose.pose.orientation));

  ROS_INFO("i:%d j:%d",ansj,ansi);
}

void LS_SLAM::LS_GLOBAL_LOCSTION::Display(void)
{
  nav_msgs::OccupancyGrid costmap_;
  costmap_.header.frame_id = "/map";
  costmap_.info.height = 1000;
  costmap_.info.width = 1000;
  costmap_.info.resolution = this->map_.info.resolution;
  costmap_.info.origin.position.x = this->map_.info.origin.position.x;
  costmap_.info.origin.position.y = this->map_.info.origin.position.y;
  costmap_.info.origin.orientation.x = this->map_.info.origin.orientation.x;
  costmap_.info.origin.orientation.y = this->map_.info.origin.orientation.y;
  costmap_.info.origin.orientation.z = this->map_.info.origin.orientation.z;
  costmap_.info.origin.orientation.w = this->map_.info.origin.orientation.w;

  for(int i = 0;i < 1000;i ++)
  {
    for(int j = 0;j < 1000;j ++)
    {
      costmap_.data.push_back(this->cost_map_[i][j]);
    }
  }
  this->cost_map_pub_->publish(costmap_);
}

using namespace LS_SLAM;
int main(int argc,char *argv[])
{
  ros::init(argc,argv,"ls_global_location_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10.0);

  LS_GLOBAL_LOCSTION location(&nh);

  while(ros::ok())
  {
    location.FindInitPose();

    if(true == location.finish_find_)
    {
      ROS_WARN("finished find");

      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
