#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"//要和订阅的消息类型所匹配

#include "math.h"
#include <iostream>

#define block_value 0.1

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)//回调函数
{
  //ROS_INFO("angle_min: %f", msg->angle_min);
  //ROS_INFO("angle_max: %f", msg->angle_max);
  //ROS_INFO("range_min: %f", msg->range_min);
  //ROS_INFO("range_max: %f", msg->range_max);
  //ROS_INFO("angle_increment: %f", msg->angle_increment);
  //ROS_INFO("ranges: %f", msg->ranges);
  //ROS_INFO("intensities: %f", msg->intensities);
  //ROS_INFO("sizes: %d", msg->ranges.size());
  //ROS_INFO("/");
  float angle_sum = 0;
  float ranges[720];
  for(int i=0;i<720;i++)
  {
    ranges[i]=msg->ranges[i];
  }
  float X[720]={0},Y[720]={0},K[720]={0};
  float gap_num[10]={0}; int gap_count = 0;
  int Num = 0;
  /////////////////////////////////////
  //去除障碍点
  for(int i=0;i<720 - 1;i++)
  {
    //挑选去除障碍物的点
    float ranges_minus = ranges[i + 1] - ranges[i];
    if(ranges_minus > block_value)//大于某个阈值，认为是突变点
    {
      gap_num[gap_count] = i;
      ++gap_count;
    }
  }
  for(int t=0;t<gap_count - 1;t = t + 2)
  {
    for(int tt=gap_num[t];tt<gap_num[t + 1];tt++)
    {
      ranges[tt]=0;
    }
  }
  ///////////////////////////////////
  //找出斜率
  for(int i=0;i<720;i++)
  {
    angle_sum = angle_sum + msg->angle_increment;
    //ROS_INFO("angle_sum: %f", angle_sum);
    //ROS_INFO("ranges: %f", msg->ranges[i]);

    float tmp_ranges = ranges[i];
    if(tmp_ranges!=0)
    {
      X[Num]=tmp_ranges*cos(angle_sum);
      Y[Num]=tmp_ranges*sin(angle_sum);
      //ROS_INFO("%f",X[Num]);
      //ROS_INFO("%f",Y[Num]);
      ++Num;
    }
  }
  for(int i=0;i<Num - 2;i++)
  {
    K[i] = (Y[i+1] - Y[i])/(X[i+1] - X[i]);
    //ROS_INFO("%f",K[i]);
  }
  //过滤错误点和特殊点
  float K_sum = 0; int number = 0;
  for(int i=1;i<Num - 2;i++)  
  {
    //ROS_INFO("O:%f",K[i]);
    if((K[i - 1]>0 && K[i]<0 && K[i + 1]>0)||(K[i - 1]<0 && K[i]>0 && K[i + 1]<0))
      K[i]=0;
    if(abs(K[i - 1]) > 10)
      K[i]=0;
    if(K[i]!=0)
    {
      K_sum = K_sum + K[i];
      ++number;
    }
    //ROS_INFO("N:%f",K[i]);
  } 
  int time = 0;
  for(int i=0;i<Num - 11;i = i + 10)
  {
    if((K[i + 10] - K[i]) > 2)
    {
      ++time;
    }
  }
  //ROS_INFO("%d",time);
  //ROS_INFO("K_sum: %f",K_sum);
  float avg = K_sum/number;
  //ROS_INFO("avg: %f",avg);
  int flag = 0;
  if(abs(avg)<0.15 && time<5)
  {
    ROS_INFO("rectangle");
    flag = 1;
  }
  if(abs(avg)<0.5 && time>5)
  {
    ROS_INFO("circle");
    flag = 1;
  }
  if(abs(avg)>1 && time<10)//time>10
  {
    ROS_INFO("triangle");
    flag = 1;
  }
  if(flag == 1)
  {
    ros::shutdown();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");//初始化ros，向master注册一个叫“listener”的节点

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);
  //订阅者向master注册自己需要订阅的话题"chatter"，消息队列大小是1000，chatterCallback是回调函数
  //回调函数的意义是，当订阅者从自己所订阅的话题上接收到消息，回调函数自动执行

  ros::spin();//因为不在while循环体中，所以它要不断执行，而不是用spinOnce()，只执行一次

  return 0;
}

