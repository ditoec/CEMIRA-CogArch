#include <ros/ros.h>
#include <leap_motion/leapros.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>
using namespace std;

int status;

std::string model_path;

string category;

float calculate_dist(geometry_msgs::Point point1,geometry_msgs::Point point2){
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	float z = point1.z - point2.z;
	return sqrt((x*x)+(y*y)+(z*z)); 
}

 string append_data(geometry_msgs::Point point1,geometry_msgs::Point point2,float scale, int count){
 	float distance = calculate_dist(point1,point2);
 	std::stringstream ss;
	ss << count << ":" << std::fixed << std::setprecision(5) << (distance/scale);
	return " " + ss.str();
 }

void leapCallback(const leap_motion::leapros::ConstPtr& data)
{
  if(!(data->palmpos.x==0&&data->palmpos.y==0&&data->palmpos.z==0)){
  	string dataset = category;
  	int count = 0;
    float scale = calculate_dist(data->middle_metacarpal,data->palmpos);

    count++;
    dataset = dataset + append_data(data->palmpos,data->thumb_proximal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->thumb_intermediate,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->thumb_distal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->thumb_tip,scale,count);

    count++;
    dataset = dataset + append_data(data->palmpos,data->index_proximal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->index_intermediate,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->index_distal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->index_tip,scale,count);

    count++;
    dataset = dataset + append_data(data->palmpos,data->middle_proximal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->middle_intermediate,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->middle_distal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->middle_tip,scale,count);

    count++;
    dataset = dataset + append_data(data->palmpos,data->ring_proximal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->ring_intermediate,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->ring_distal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->ring_tip,scale,count);

    count++;
    dataset = dataset + append_data(data->palmpos,data->pinky_proximal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->pinky_intermediate,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->pinky_distal,scale,count);
    count++;
    dataset = dataset + append_data(data->palmpos,data->pinky_tip,scale,count);

    int count0=0,count1=0,count2=0,count3=0;
  	ifstream myfilein (model_path.c_str());
 	if (myfilein.is_open())
  	{
  		std::string line;
		while(std::getline(myfilein, line))
		{
    		if(line[0]=='0')count0++;
    		else if(line[0]=='1')count1++;
    		else if(line[0]=='2')count2++;
		else if(line[0]=='3')count3++;
		}
		myfilein.close();
	}
	ofstream myfileout (model_path.c_str(),ios::app);
	if (myfileout.is_open())
  	{
    	myfileout <<  dataset + "\n";
    	myfileout.close();
    	status=1;
    	ROS_INFO("Success writing data to file");
    	ROS_INFO("Number of 0=%d, Number of 1=%d, Number of 2=%d, Number of 3=%d",count0,count1,count2,count3);
  	}
  	else{
  		status=-1;
  		ROS_INFO("Unable to open file");	
  	}
  }
  else{
  	status=-1;
  	ROS_INFO("Unable to detect hand!");
  } 
}

int main( int argc, char** argv )
{
  if (argc < 2){
  	ROS_INFO("I need a category parameter! 0=Other Gesture, 1=Pointing Gesture, 2=Open Palm Gesture 3=Close Hand Gesture");
  }
  else{
  	status = 0;
    category = argv[1];	
    ros::init(argc, argv,"leap_sampler");
    ros::NodeHandle n;

    std::string package_path = ros::package::getPath("leap");
    std::stringstream ss;
    ss << package_path << "/config/hand.model";
    model_path = ss.str();
    ROS_INFO(model_path.c_str());

    ros::Subscriber leap_sub = n.subscribe("/leapmotion/data", 100,leapCallback);
    if(leap_sub){
      ROS_INFO("Leap Data Subscriber intitated!");
    }
    else{
      ROS_INFO("Leap Data Subscriber failed!"); 
    }
    while(status==0) ros::spinOnce();
    if(status==1){
      ROS_INFO("Leap Data recording done!");
    }
    else{
      ROS_INFO("Leap Data recording failed!"); 
    }
  }
  return 0;
}
