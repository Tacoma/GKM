#ifndef POLYGON_ARRAY_H
#define POLYGON_ARRAY_H


class PolygonPublisher {


    PolygonPublisher() {

    }

    ~PolygonPublisher() {


    }



    void PolygonPublisher::publishPolygons() {
//     jsk_recognition_msgs::PolygonArray markers;
//     markers.header.frame_id = "world";
//     markers.header.stamp = ros::Time::now();
//
//     geometry_msgs::PolygonStamped polyStamped;
//     polyStamped.header.stamp = ros::Time::now();
//     polyStamped.header.frame_id = "world";
//
//     for (int i=0; i<planes_.size(); i++) {
//         MyPointcloud::ConstPtr hull = planes_[i]->getHull();
//         for (int i=0; i < hull->size(); i++) {
//             geometry_msgs::Point32 point;
//             point.x = hull->points[i].x;
//             point.y = hull->points[i].y;
//             point.z = hull->points[i].z;
//             polyStamped.polygon.points.push_back(point);
//         }
//         markers.polygons.push_back(polyStamped);
//     }
//     pub_markers_.publish(markers);
    }

};



#endif //POLYGON_ARRAY_H
