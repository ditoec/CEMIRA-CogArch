#include <ros/ros.h>
#include <leap_motion/leapros.h>
#include <ros/package.h>
#include <unistd.h>
#include "svm.h"

int status;

std::string model_path;

float calculate_dist(geometry_msgs::Point point1,geometry_msgs::Point point2){
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	float z = point1.z - point2.z;
	return sqrt((x*x)+(y*y)+(z*z)); 
}

 svm_node get_feature(geometry_msgs::Point point1,geometry_msgs::Point point2,float scale, int count){
 	float distance = calculate_dist(point1,point2);
 	svm_node node = {count,(distance/scale)};
	return node;
 }

void leapCallback(const leap_motion::leapros::ConstPtr& data)
{
  if(!(data->palmpos.x==0&&data->palmpos.y==0&&data->palmpos.z==0)){
    svm_node nodes [21];
  	int count = 1;
    float scale = calculate_dist(data->middle_metacarpal,data->palmpos);

    nodes[count-1] = get_feature(data->palmpos,data->thumb_proximal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->thumb_intermediate,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->thumb_distal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->thumb_tip,scale,count);

    count++;
    nodes[count-1] = get_feature(data->palmpos,data->index_proximal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->index_intermediate,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->index_distal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->index_tip,scale,count);

    count++;
    nodes[count-1] = get_feature(data->palmpos,data->middle_proximal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->middle_intermediate,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->middle_distal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->middle_tip,scale,count);

    count++;
    nodes[count-1] = get_feature(data->palmpos,data->ring_proximal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->ring_intermediate,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->ring_distal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->ring_tip,scale,count);

    count++;
    nodes[count-1] = get_feature(data->palmpos,data->pinky_proximal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->pinky_intermediate,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->pinky_distal,scale,count);
    count++;
    nodes[count-1] = get_feature(data->palmpos,data->pinky_tip,scale,count);
    
    count++;
    svm_node node = {(-1,-1)};
    nodes[count-1] = node;

    svm_model *model = svm_load_model(model_path.c_str());
    if(!model){
      ROS_INFO("Model cannot be loaded!");
      ros::shutdown(); 
    }
    status = svm_predict(model,nodes);
 	if (status==1)
  	{
    		ROS_INFO("Pointing gesture detected!");
  	}
  	else if (status==2)
    	{
      		ROS_INFO("Open palm gesture detected!");
    	}
	else if (status==3)
    	{
      		ROS_INFO("Close hand gesture detected!");
    	}
	else
    	{
      		ROS_INFO("Other gesture detected!");
    	}   
  }
}

int main( int argc, char** argv )
{
	status = 0;
  ros::init(argc, argv,"leap_predictor");
  ros::NodeHandle n;
  ros::Subscriber leap_sub = n.subscribe("/leapmotion/data", 100,leapCallback);
  if(leap_sub){
    ROS_INFO("Leap Data Subscriber intitated!");
  }
  else{
    ROS_INFO("Leap Data Subscriber failed!"); 
  }

  std::string package_path = ros::package::getPath("leap");
  std::stringstream ss;
  ss << package_path << "/config/hand.model";
  model_path = ss.str();
  ROS_INFO(model_path.c_str());

  ros::spin();
  return 0;
}
