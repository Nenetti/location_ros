#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import rospy
from location.msg import Location
from location.srv import *
from std_msgs.msg import String, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
import csv
import os

DEFAULT_PATH = os.environ['HOME'] + '/map_pos.csv'


class Data:
    def __init__(self, location, markers):
        self.location = location
        self.markers = markers


class Register:

    def __init__(self):
        self.marker_id = 0

    ####################################################################################################
    def create_marker(self, message):
        # type: (Location) -> (Marker, Marker)
        r = random.random()
        g = random.random()
        b = random.random()
        scale = Vector3(0.5, 0.5, 0.5)
        frame_id = "map"
        ns = "location_markers"

        marker_str = Marker()
        marker_str.header.stamp = rospy.Time.now()
        marker_str.header.frame_id = frame_id
        marker_str.ns = ns
        marker_str.id = self.marker_id
        marker_str.type = Marker.TEXT_VIEW_FACING
        marker_str.text = message.name
        marker_str.action = Marker.ADD
        marker_str.pose.position = Point(message.pose.position.x, message.pose.position.y, 0.5)
        marker_str.scale = scale
        marker_str.color = ColorRGBA(r, g, b, 1)
        self.marker_id += 1

        marker_sphere = Marker()
        marker_sphere.header.stamp = rospy.Time.now()
        marker_sphere.header.frame_id = frame_id
        marker_sphere.ns = ns
        marker_sphere.id = self.marker_id
        marker_sphere.type = Marker.SPHERE
        marker_sphere.action = Marker.ADD
        marker_sphere.pose.position = Point(message.pose.position.x, message.pose.position.y, 0)
        marker_sphere.scale = scale
        marker_sphere.color = ColorRGBA(r, g, b, 0.5)
        self.marker_id += 1

        return [marker_sphere, marker_str]

    def start(self):
        ####################################################################################################
        def save(message):
            # type: (String) -> None
            path = message.data
            if path == "":
                path = DEFAULT_PATH
            with open(path, "w") as file:
                writer = csv.writer(file)
                for key in locations:
                    location = locations[key].location
                    line = [location.name, location.pose.position.x, location.pose.position.y, location.pose.position.z,
                            location.pose.orientation.x,
                            location.pose.orientation.y, location.pose.orientation.z, location.pose.orientation.w]
                    writer.writerow(line)
                    print(line)

        ####################################################################################################
        def load(message):
            # type: (String) -> None
            path = message.data
            if path == "":
                path = DEFAULT_PATH
            with open(path, "r") as file:
                reader = csv.reader(file)
                for line in reader:
                    location = Location(line[0],
                                        Pose(Point(float(line[1]), float(line[2]), float(line[3])),
                                             Quaternion(float(line[4]), float(line[5]), float(line[6]),
                                                        float(line[7]))))
                    register(location)

        ####################################################################################################
        def register(message):
            # type: (Location) -> None
            print("[Register] %s (%s %s)" % (message.name, message.pose.position.x, message.pose.position.y))
            markers = self.create_marker(message)
            locations[message.name] = Data(message, markers)
            for marker in markers:
                pub.publish(marker)

        ####################################################################################################
        def delete(message):
            # type: (String) -> None
            if message.data in locations:
                data = locations.pop(message.data)
                print("[Delete] %s (%s %s)" % (
                    data.location.name, data.location.pose.position.x, data.location.pose.position.y))
                for marker in data.markers:
                    marker.action = Marker.DELETE
                    pub.publish(marker)

        ####################################################################################################
        def request_location(request):
            # type: (Request_LocationRequest) -> Request_LocationResponse
            result = locations.get(request.name)
            if result is not None:
                return Request_LocationResponse(result)
            return None

        ####################################################################################################
        locations = {}
        rospy.init_node('location_concepts', anonymous=False)
        rospy.Subscriber("location/register", Location, register)
        rospy.Subscriber("location/register/save", String, save)
        rospy.Subscriber("location/register/load", String, load)
        rospy.Subscriber("location/register/delete", String, delete)
        rospy.Service("location/request", Request_Location, request_location)
        pub = rospy.Publisher("location/marker", Marker, queue_size=10)
        rospy.spin()


if __name__ == "__main__":
    Register().start()
