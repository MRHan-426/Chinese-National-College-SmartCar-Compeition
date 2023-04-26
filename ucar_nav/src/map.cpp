#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
using namespace std;
int i=0;
int j=0;
int w=81;
int h =81;
ros::Publisher map_publisher;
nav_msgs::OccupancyGrid map_;

void initMap();
int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_map");
  ros::NodeHandle nh;
  initMap();
  map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true); 
  while(1)
  {
    map_publisher.publish(map_);
  }
  // ros::spin();
}

void initMap()
{
    map_.header.frame_id = "map";
    map_.info.height =h;
    map_.info.width = w;
    map_.info.resolution = 0.05;
    map_.info.origin.position.x = -w/2* map_.info.resolution; 
    map_.info.origin.position.y = -h/2* map_.info.resolution;

    map_.info.origin.orientation.x=0;
    map_.info.origin.orientation.y=0;
    map_.info.origin.orientation.z=0;
    map_.info.origin.orientation.w = 1.0;
    map_.data.assign( w*h, 0);
    for(i=0,j=0;i<=80;i++)
    {  
      map_.data.at(j*w+i) = 100;
    }
    for(i=20,j=20;i<=80;i++)
    {  
      map_.data.at(j*w+i) = 100;
    }
    for(i=0,j=40;i<=40;i++)
    {  
      map_.data.at(j*w+i) = 100;
    }
    for(i=20,j=50;i<=40;i++)
    {  
      map_.data.at(j*w+i) = 100;
    }
    for(i=10,j=60;i<=20;i++)
    {  
      map_.data.at(j*w+i) = 100;
    }
    for(i=40,j=60;i<=70;i++)
    {  
      map_.data.at(j*w+i) = 100;
    }
    for(i=0,j=80;i<=80;i++)
    {  
      map_.data.at(j*w+i) = 100;
    }


    for(i=0,j=0;j<=80;j++)
    {  
      map_.data.at(j*w+i) = 100;
    }
    for(i=20,j=0;j<=80;j++)
    {  
      if((j>=10&&j<=30)||(j>=50&&j<=60))
        {map_.data.at(j*w+i) = 100;}
    }
    for(i=40,j=0;j<=80;j++)
    {  
      if((j>=0&&j<=10)||(j>=50&&j<=60))
        {map_.data.at(j*w+i) = 100;}
    }
    for(i=50,j=20;j<=70;j++)
    {  
      map_.data.at(j*w+i) = 100;
    }
    for(i=80,j=0;j<=80;j++)
    {  
      map_.data.at(j*w+i) = 100;
    }


}

