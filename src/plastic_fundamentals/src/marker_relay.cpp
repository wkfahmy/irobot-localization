#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plastic_fundamentals/PublishMarker.h>
#include <plastic_fundamentals/Line.h>
#include <plastic_fundamentals/Point.h>

class MarkerRelay {
public:
    MarkerRelay()
    {
        ros::NodeHandle nh;

        service_ = nh.advertiseService("marker_service", &MarkerRelay::publishMarkerCallback, this);

        marker_pub_ = nh.advertise<visualization_msgs::Marker>("/markers", 10);

    }


    void pointMarker(const plastic_fundamentals::PublishMarker::Request &req)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "points";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.01;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        visualization_msgs::Marker direction_marker;
        direction_marker.header.frame_id = "map";
        direction_marker.header.stamp = ros::Time::now();
        direction_marker.ns = "points_direction";
        direction_marker.id = 1;
        direction_marker.type = visualization_msgs::Marker::LINE_LIST;
        direction_marker.action = visualization_msgs::Marker::ADD;
        direction_marker.pose.orientation.w = 1.0;
        direction_marker.scale.x = 0.03;
        direction_marker.color.r = 0.0;
        direction_marker.color.g = 0.0;
        direction_marker.color.b = 1.0;
        direction_marker.color.a = 1.0;


        for (const plastic_fundamentals::Point& point : req.points) {
            geometry_msgs::Point pt;
            pt.x = point.x;
            pt.y = point.y;
            pt.z = 0.0;
            marker.points.push_back(pt);

            geometry_msgs::Point direction;
            direction.x = point.x + 0.05 * cos(point.w);
            direction.y = point.y + 0.05 * sin(point.w);
            direction.z = 0.0;
            direction_marker.points.push_back(pt);
            direction_marker.points.push_back(direction);
        }

        marker_pub_.publish(marker);


        direction_marker.lifetime = ros::Duration(0);

        marker_pub_.publish(marker);
        marker_pub_.publish(direction_marker);
    }

    void robotMarker(const plastic_fundamentals::PublishMarker::Request &req)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "robot";
        marker.id = 2;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.pose.position.x = req.robot.x;
        marker.pose.position.y = req.robot.y;
        marker.pose.position.z = 0.01;
        marker.pose.orientation.w = req.robot.w;

        marker_pub_.publish(marker);

        visualization_msgs::Marker direction_marker;
        direction_marker.header.frame_id = "map";
        direction_marker.header.stamp = ros::Time::now();
        direction_marker.ns = "robot_direction";
        direction_marker.id = 3;
        direction_marker.type = visualization_msgs::Marker::LINE_STRIP;
        direction_marker.action = visualization_msgs::Marker::ADD;
        direction_marker.scale.x = 0.05;

        direction_marker.color.r = 1.0;
        direction_marker.color.g = 1.0;
        direction_marker.color.b = 0.0;
        direction_marker.color.a = 1.0;

        geometry_msgs::Point p1;
        p1.x = req.robot.x;
        p1.y = req.robot.y;
        p1.z = 0.06;

        geometry_msgs::Point p2;
        double line_length = 0.2;
        p2.x = req.robot.x + line_length * cos(req.robot.w);
        p2.y = req.robot.y + line_length * sin(req.robot.w);
        p2.z = 0.06;

        direction_marker.points.push_back(p1);
        direction_marker.points.push_back(p2);

        marker_pub_.publish(direction_marker);
    }


    void lineMarker(const plastic_fundamentals::PublishMarker::Request &req)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "lines";
        marker.id = 5;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        for (const plastic_fundamentals::Line& line : req.lines) {
            geometry_msgs::Point p1, p2;
            p1.x = line.x1;
            p1.y = line.y1;
            p1.z = 0.0;
            p2.x = line.x2;
            p2.y = line.y2;
            p2.z = 0.0;
            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }

        marker.lifetime = ros::Duration(0);

        marker_pub_.publish(marker);
    }

    void rayMarker(const plastic_fundamentals::PublishMarker::Request &req)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "rays";
        marker.id = 4;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;

        for (const plastic_fundamentals::Line& line : req.lines) {
            geometry_msgs::Point p1, p2;
            p1.x = line.x1;
            p1.y = line.y1;
            p1.z = 0.0;
            p2.x = line.x2;
            p2.y = line.y2;
            p2.z = 0.0;
            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }

        marker.lifetime = ros::Duration(0);

        marker_pub_.publish(marker);
    }

    bool publishMarkerCallback(plastic_fundamentals::PublishMarker::Request &req,
                                  plastic_fundamentals::PublishMarker::Response &res) {
        if (req.marker_type == "PointMarker") {
            pointMarker(req);
        }
        else if (req.marker_type == "RobotMarker") {
            robotMarker(req);
        }
        else if (req.marker_type == "LineMarker") {
            lineMarker(req);
        }
        else if (req.marker_type == "RayMarker") {
            rayMarker(req);
        }
        else {
            ROS_WARN("Unknown marker type received.");
        }

        res.success = true;
        return true;
    }

private:
    ros::ServiceServer service_;
    ros::Publisher marker_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "marker_relay");
    MarkerRelay marker_relay;
    ros::spin();
    return 0;
}