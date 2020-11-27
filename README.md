# Routing-ROS-Module

This module is actually a ROS service, which returns the points of a polyline in the long/lat coordinates systems, from a starting point to an end point. It's using the Mapzen API to calculate the quickest path to do so.

## How does it work ?

Using CURL, the service calls Mapzen API providing coordinates of the start and the end of the route, and the API returns directions, encoded in the spline format. We decode that spline, and put it in the output, it's as simple as that.

## Installation

`git clone` in your `catkin_ws/src` and you're ready to go! 
 
## Changes in original project
*Thnx https://github.com/oxidase/rosrm*

Valhalla API was changed to OSRM API

In current implementation we request the route from the hosted remote server (router.project-osrm.org) via sockets. No local map or built osrm_backend required. 

But you can use service_type ROS parameter to change the host. Local - local osrm/Service - hosted remote server (router.project-osrm.org).


 
