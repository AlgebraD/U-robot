#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>  
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);  //#这是要传递的信息。
   
  unsigned int num_readings = 1;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];  //# 准备创建虚拟的激光信息，这分别是平率，不太懂。。。
 
  double count = 0;
  ros::Rate r(1.0);     //#发送数据的速率
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }
    ros::Time scan_time = ros::Time::now();      //#每隔一秒产生一个虚假的数据

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 0.000000314;//3.14 / num_readings;
    scan.time_increment = 0.000025;//(1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;                  // #这些是消息内容。与前一节交相辉映

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = 1;//intensities[i];
    }

    scan_pub.publish(scan);
    count += 0.1;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "laser_frame"));

    r.sleep();
  }
}
