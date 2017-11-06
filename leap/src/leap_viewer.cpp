#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <leap_motion/leapros.h>
#include "svm.h"

ros::Publisher marker_pub;

int pointsSize = 7;
int linesSize = 8;
int arrowsSize = 2;

float last_x = 0.0;
float last_y = 0.0;
float last_z = 0.0;

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

geometry_msgs::Point calc_intersect(geometry_msgs::Point p0,geometry_msgs::Point p1, float x,float y,float z,float c ){
    float t = (c-(x*p0.x)-(y*p0.y)-(z*p0.z))/((x*(p1.x-p0.x))+(y*(p1.y-p0.y))+(z*(p1.z-p0.z)));
    geometry_msgs::Point pOut;
    pOut.x = p0.x + t*(p1.x-p0.x);
    pOut.y = p0.y + t*(p1.y-p0.y);
    pOut.z = p0.z + t*(p1.z-p0.z);
    return pOut;
}

void leapCallback(const leap_motion::leapros::ConstPtr& data)
{
  visualization_msgs::Marker pointsArray[pointsSize];
  visualization_msgs::Marker lineStripArray[linesSize];
  visualization_msgs::Marker arrowArray[arrowsSize];
  visualization_msgs::Marker gestureText;
  visualization_msgs::MarkerArray allArray;
  int i = 0;

  float size =0;
  if(!(data->palmpos.x==last_x&&data->palmpos.y==last_y&&data->palmpos.z==last_z)){

    last_x = data->palmpos.x;
    last_y = data->palmpos.y;
    last_z = data->palmpos.z;

    gestureText.header.frame_id = "leap";
    gestureText.header.stamp = ros::Time::now();
    gestureText.ns = "leap";
    gestureText.action = visualization_msgs::Marker::ADD;
    gestureText.id = i;
    gestureText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    gestureText.pose.position.x = data->palmpos.x + 0.15;
    gestureText.pose.position.y = data->palmpos.y + 0.15;
    gestureText.pose.position.z = data->palmpos.z + 0.15;
    gestureText.scale.z = 0.05;
    gestureText.color.b = 1.0f;
    gestureText.color.a = 1.0;
    i++;

    for (uint32_t j = 0; j < pointsSize;j++){
      pointsArray[j].header.frame_id = "leap";
      pointsArray[j].header.stamp = ros::Time::now();
      pointsArray[j].ns = "leap";
      pointsArray[j].action = visualization_msgs::Marker::ADD;
      pointsArray[j].id = i;
      pointsArray[j].type = visualization_msgs::Marker::POINTS;
      if(j==(pointsSize-1)){
        size = 0.025;
      }
      else{
        size = 0.015;
      }
      pointsArray[j].scale.x = size;
      pointsArray[j].scale.y = size;
      pointsArray[j].color.r = 1.0f;
      pointsArray[j].color.a = 1.0;
      i++;
    }

    for (uint32_t j = 0; j < linesSize;j++){
      lineStripArray[j].header.frame_id = "leap";
      lineStripArray[j].header.stamp = ros::Time::now();
      lineStripArray[j].ns = "leap";
      lineStripArray[j].action = visualization_msgs::Marker::ADD;
      lineStripArray[j].id = i;
      lineStripArray[j].type = visualization_msgs::Marker::LINE_STRIP;
      lineStripArray[j].scale.x = 0.01;
      lineStripArray[j].color.b = 1.0f;
      lineStripArray[j].color.a = 1.0;
      i++;
    }

    for (uint32_t j = 0; j < arrowsSize;j++){
      arrowArray[j].header.frame_id = "leap";
      arrowArray[j].header.stamp = ros::Time::now();
      arrowArray[j].ns = "leap";
      arrowArray[j].action = visualization_msgs::Marker::ADD;
      arrowArray[j].id = i;
      arrowArray[j].type = visualization_msgs::Marker::ARROW;
      arrowArray[j].scale.x = 0.015;
      arrowArray[j].scale.y = 0.025;
      arrowArray[j].color.g = 1.0f;
      arrowArray[j].color.a = 1.0;
      i++;
    }

    pointsArray[0].points.push_back(data->thumb_metacarpal);
    pointsArray[0].points.push_back(data->thumb_proximal);
    pointsArray[0].points.push_back(data->thumb_intermediate);
    pointsArray[0].points.push_back(data->thumb_distal);
    pointsArray[0].points.push_back(data->thumb_tip);

    lineStripArray[0].points.push_back(data->thumb_metacarpal);
    lineStripArray[0].points.push_back(data->thumb_proximal);
    lineStripArray[0].points.push_back(data->thumb_intermediate);
    lineStripArray[0].points.push_back(data->thumb_distal);
    lineStripArray[0].points.push_back(data->thumb_tip);

    pointsArray[1].points.push_back(data->index_metacarpal);
    pointsArray[1].points.push_back(data->index_proximal);
    pointsArray[1].points.push_back(data->index_intermediate);
    pointsArray[1].points.push_back(data->index_distal);
    pointsArray[1].points.push_back(data->index_tip);

    lineStripArray[1].points.push_back(data->index_metacarpal);
    lineStripArray[1].points.push_back(data->index_proximal);
    lineStripArray[1].points.push_back(data->index_intermediate);
    lineStripArray[1].points.push_back(data->index_distal);
    lineStripArray[1].points.push_back(data->index_tip);

    pointsArray[2].points.push_back(data->middle_metacarpal);
    pointsArray[2].points.push_back(data->middle_proximal);
    pointsArray[2].points.push_back(data->middle_intermediate);
    pointsArray[2].points.push_back(data->middle_distal);
    pointsArray[2].points.push_back(data->middle_tip);

    lineStripArray[2].points.push_back(data->middle_metacarpal);
    lineStripArray[2].points.push_back(data->middle_proximal);
    lineStripArray[2].points.push_back(data->middle_intermediate);
    lineStripArray[2].points.push_back(data->middle_distal);
    lineStripArray[2].points.push_back(data->middle_tip);

    pointsArray[3].points.push_back(data->ring_metacarpal);
    pointsArray[3].points.push_back(data->ring_proximal);
    pointsArray[3].points.push_back(data->ring_intermediate);
    pointsArray[3].points.push_back(data->ring_distal);
    pointsArray[3].points.push_back(data->ring_tip);

    lineStripArray[3].points.push_back(data->ring_metacarpal);
    lineStripArray[3].points.push_back(data->ring_proximal);
    lineStripArray[3].points.push_back(data->ring_intermediate);
    lineStripArray[3].points.push_back(data->ring_distal);
    lineStripArray[3].points.push_back(data->ring_tip);

    pointsArray[4].points.push_back(data->pinky_metacarpal);
    pointsArray[4].points.push_back(data->pinky_proximal);
    pointsArray[4].points.push_back(data->pinky_intermediate);
    pointsArray[4].points.push_back(data->pinky_distal);
    pointsArray[4].points.push_back(data->pinky_tip);

    pointsArray[5].points.push_back(data->palmpos);

    lineStripArray[4].points.push_back(data->pinky_metacarpal);
    lineStripArray[4].points.push_back(data->pinky_proximal);
    lineStripArray[4].points.push_back(data->pinky_intermediate);
    lineStripArray[4].points.push_back(data->pinky_distal);
    lineStripArray[4].points.push_back(data->pinky_tip);

    lineStripArray[5].points.push_back(data->thumb_metacarpal);
    lineStripArray[5].points.push_back(data->index_metacarpal);
    lineStripArray[5].points.push_back(data->middle_metacarpal);
    lineStripArray[5].points.push_back(data->ring_metacarpal);
    lineStripArray[5].points.push_back(data->pinky_metacarpal);

    lineStripArray[6].points.push_back(data->thumb_proximal);
    lineStripArray[6].points.push_back(data->index_proximal);
    lineStripArray[6].points.push_back(data->middle_proximal);
    lineStripArray[6].points.push_back(data->ring_proximal);
    lineStripArray[6].points.push_back(data->pinky_proximal);

    arrowArray[0].points.push_back(data->palmpos);
    geometry_msgs::Point normEnd;
    normEnd.x = data->palmpos.x + (data->normal.x/20);
    normEnd.y = data->palmpos.y + (data->normal.y/20);
    normEnd.z = data->palmpos.z + (data->normal.z/20);
    arrowArray[0].points.push_back(normEnd);

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

    int gesture = 0;
    svm_model *model = svm_load_model("/home/dito/hand.model");
    if(!model){
      ROS_INFO("Model cannot be loaded!"); 
    }
    else{
      gesture = svm_predict(model,nodes);
    }
    if (gesture==0)gestureText.text = "Other Gesture";
    else if (gesture==1)gestureText.text = "Pointing";
    else if (gesture==2)gestureText.text = "Open Palm";
    else if (gesture==3)gestureText.text = "Close Hand";

    if (gesture==1){
      geometry_msgs::Point pointingPoint = calc_intersect(data->index_proximal,data->index_tip,0,1,0,-0.01125);
      if(pointingPoint.z < 0 && pointingPoint.z > -100 && std::abs(pointingPoint.x) < 100){
        arrowArray[1].points.push_back(data->index_proximal);
        arrowArray[1].points.push_back(data->index_tip);

        pointsArray[5].points.push_back(pointingPoint);
        geometry_msgs::Point basePoint;
        basePoint.x = data->index_proximal.x;
        basePoint.z = data->index_proximal.z;
        basePoint.y = 0;
        lineStripArray[7].points.push_back(basePoint);
        lineStripArray[7].points.push_back(pointingPoint);
      }
      else{
        arrowArray[1].action = 2;
        pointsArray[5].action = 2;
        lineStripArray[7].action = 2;
      }
    }
  
    allArray.markers.resize(1+pointsSize+linesSize+arrowsSize);
    allArray.markers[0] = gestureText;  
    i = 1;
    for (uint32_t j = 0; j < pointsSize;j++){
      allArray.markers[i] = pointsArray[j];
      i++;
    }
    for (uint32_t j = 0; j < linesSize;j++){
      allArray.markers[i] = lineStripArray[j];
      i++;
    }
    for (uint32_t j = 0; j < arrowsSize;j++){
      allArray.markers[i] =  arrowArray[j];
      i++;
    }
  }
  else{
    allArray.markers.resize(pointsSize+linesSize+arrowsSize);
    
    for (uint32_t j = 0; j < pointsSize;j++){
      pointsArray[j].header.frame_id = "leap";
      pointsArray[j].header.stamp = ros::Time::now();
      pointsArray[j].ns = "leap";
      pointsArray[j].action = 3;
      pointsArray[j].id = i;
      pointsArray[j].type = visualization_msgs::Marker::POINTS;
      allArray.markers[i] = pointsArray[j];
      i++;
    }

    for (uint32_t j = 0; j < linesSize;j++){
      lineStripArray[j].header.frame_id = "leap";
      lineStripArray[j].header.stamp = ros::Time::now();
      lineStripArray[j].ns = "leap";
      lineStripArray[j].action = 3;
      lineStripArray[j].id = i;
      lineStripArray[j].type = visualization_msgs::Marker::LINE_STRIP;
      allArray.markers[i] = lineStripArray[j];
      i++;
    }

    for (uint32_t j = 0; j < arrowsSize;j++){
      arrowArray[j].header.frame_id = "leap";
      arrowArray[j].header.stamp = ros::Time::now();
      arrowArray[j].ns = "leap";
      arrowArray[j].action = 3;
      arrowArray[j].id = i;
      arrowArray[j].type = visualization_msgs::Marker::ARROW;
      allArray.markers[i] =  arrowArray[j];
      i++;
    }
  }
  
  marker_pub.publish(allArray);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv,"leap_viewer");
  ros::NodeHandle n;
  ros::Subscriber leap_sub = n.subscribe("/leapmotion/data", 100,leapCallback);
  if(leap_sub){
    ROS_INFO("Leap Data Subscriber intitated!");
  }
  else{
    ROS_INFO("Leap Data Subscriber failed!"); 
  }
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("/leapmotion/marker", 10);
  if(marker_pub){
    ROS_INFO("Leap Data Visualizer intitated!");
  }
  else{
    ROS_INFO("Leap Data Visualizer failed!"); 
  }
  ros::spin();
}
