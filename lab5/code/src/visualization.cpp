#include "scan_matching_skeleton/visualization.h"

PointVisualizer::PointVisualizer(ros::Publisher& pub, string ns, string frame_id) : pub(pub), ns(ns),
      frame_id(frame_id) {
  dots.header.frame_id = frame_id;
  dots.ns = ns;
  dots.action = visualization_msgs::Marker::ADD;
  dots.pose.orientation.w = 1.0;
  dots.id = num_visuals;
  dots.type = visualization_msgs::Marker::POINTS;
  dots.scale.x = dots.scale.y = 0.005;
  ++num_visuals;
}

void PointVisualizer::addPoints(vector<Point>& points, std_msgs::ColorRGBA color) {
  for (Point p : points) {
    dots.points.push_back(p.getPoint());
    dots.colors.push_back(color);
  }
}

void PointVisualizer::publishPoints() {
  dots.header.stamp = ros::Time::now();
  pub.publish(dots);
  ROS_INFO("published dots");
  dots.points.clear();
  dots.colors.clear();
}

CorrespondenceVisualizer::CorrespondenceVisualizer(ros::Publisher& pub, string ns, string frame_id) : pub(pub), ns(ns),
      frame_id(frame_id) {
  line_list.header.frame_id = frame_id;
  line_list.ns = ns;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = num_visuals;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.001;
  num_visuals++;
}

void CorrespondenceVisualizer::addCorrespondences(vector<Correspondence> correspondences) {
  std_msgs::ColorRGBA col;
  col.r = 1.0; col.b = 0.0; col.g = 0.0; col.a = 1.0;
  for (Correspondence c : correspondences) {
    line_list.points.push_back(c.p->getPoint());
    line_list.colors.push_back(col);
    line_list.points.push_back(c.getPiGeo());
    line_list.colors.push_back(col);
  }
}

void CorrespondenceVisualizer::publishCorrespondences() {
  line_list.header.stamp = ros::Time::now();
  pub.publish(line_list);
  ROS_INFO("published dots");
  line_list.points.clear();
  line_list.colors.clear();
}
