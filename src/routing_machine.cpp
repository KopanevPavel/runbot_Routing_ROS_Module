#include "ros/ros.h"
#include "routing_machine/ParseWptsService.h"
#include <routing_machine/RouteService.h>
#include <routing_machine/MatchService.h>
#include <iostream>
#include <ctype.h>
#include <cstring>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <string.h>
#include <vector>
#include <sstream> 
#include <ros/console.h>

#include <osrm/osrm.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/storage_config.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/match_parameters.hpp>
#include <osrm/json_container.hpp>
#include <util/json_renderer.hpp>

#include <tf/transform_datatypes.h>

using namespace std;

struct waypointsCoords {
    std::vector<double> latitude;
	std::vector<double> longitude;
};

// ===> Prototypes
string apiCall(string website, string parameters);
string isolateShape(string input);
string getStatus(string input);
waypointsCoords getVecFromStr(std::string input);
int decodePolyline(string encoded, std::vector<double> *latitude, std::vector<double> *longitude);
string formateParameters(double slat, double slng, double elat, double elng);
string formateParametersOSRM(double slat, double slng, double elat, double elng);
bool getRouting(routing_machine::ParseWptsService::Request  &req, routing_machine::ParseWptsService::Response &res);

namespace
{
    auto convert_coordinate(osrm::json::Array &coordinate)
    {
        geometry_msgs::Point result;
        result.x = coordinate.values.at(0).get<osrm::util::json::Number>().value;
        result.y = coordinate.values.at(1).get<osrm::util::json::Number>().value;
        result.z = 0.;

        return result;
    }

    auto convert_coordinates(osrm::json::Array &coordinates)
    {
        std::vector<geometry_msgs::Point> result;
        for (auto &coordinate : coordinates.values)
        {
            result.emplace_back(convert_coordinate(coordinate.get<osrm::util::json::Array>()));
        }

        return result;
    }

    auto convert_lane(osrm::json::Object &lane)
    {
        routing_machine::Lane result;

        result.valid = lane.values["valid"].which() == 4;
        auto &indications = lane.values["indications"].get<osrm::util::json::Array>().values;
        for (auto &indication : indications)
        {
            result.indications.emplace_back(indication.get<osrm::util::json::String>().value);
        }

        return result;
    }

    auto convert_intersection(osrm::json::Object &intersection)
    {
        routing_machine::Intersection result;

        result.in = intersection.values.count("in") ? intersection.values["in"].get<osrm::util::json::Number>().value : 0;
        result.out = intersection.values.count("out") ? intersection.values["out"].get<osrm::util::json::Number>().value : 0;
        result.location = convert_coordinate(intersection.values["location"].get<osrm::util::json::Array>());

        auto &bearings = intersection.values["bearings"].get<osrm::util::json::Array>().values;
        for (auto &bearing : bearings)
        {
            result.bearings.emplace_back(bearing.get<osrm::util::json::Number>().value);
        }

        auto &entries = intersection.values["entry"].get<osrm::util::json::Array>().values;
        for (auto &entry : entries)
        {
            result.entry.emplace_back(entry.which() == 4);
        }

        auto lanes = intersection.values.find("lanes");
        if (lanes != intersection.values.end())
        {
            for (auto &lane : lanes->second.get<osrm::util::json::Array>().values)
            {
                result.lanes.emplace_back(convert_lane(lane.get<osrm::util::json::Object>()));
            }
        }

        return result;
    }

    auto convert_maneuver(osrm::json::Object &maneuver)
    {
        routing_machine::Maneuver result;
        result.type = maneuver.values["type"].get<osrm::util::json::String>().value;
        result.modifier = maneuver.values["modifier"].get<osrm::util::json::String>().value;
        result.exit = maneuver.values.count("exit") ? maneuver.values["exit"].get<osrm::util::json::Number>().value : 0;
        result.bearing_before = maneuver.values["bearing_before"].get<osrm::util::json::Number>().value;
        result.bearing_after = maneuver.values["bearing_after"].get<osrm::util::json::Number>().value;
        result.location = convert_coordinate(maneuver.values["location"].get<osrm::util::json::Array>());

        return result;
    }

    auto convert_step(osrm::json::Object &step)
    {
        routing_machine::RouteStep result;
        result.name = step.values["name"].get<osrm::util::json::String>().value;
        result.mode = step.values["mode"].get<osrm::util::json::String>().value;
        result.distance = step.values["distance"].get<osrm::util::json::Number>().value;
        result.duration = step.values["duration"].get<osrm::util::json::Number>().value;
        result.weight = step.values["weight"].get<osrm::util::json::Number>().value;
        result.maneuver = convert_maneuver(step.values["maneuver"].get<osrm::util::json::Object>());

        auto &geometry = step.values["geometry"].get<osrm::util::json::Object>().values;
        result.coordinates = convert_coordinates(geometry["coordinates"].get<osrm::util::json::Array>());

        auto &intersections = step.values["intersections"].get<osrm::util::json::Array>().values;
        for (auto &intersection : intersections)
        {
            result.intersections.emplace_back(convert_intersection(intersection.get<osrm::util::json::Object>()));
        }

        return result;
    }

    auto convert_annotation(osrm::json::Object &annotation)
    {
        routing_machine::Annotation result;

        auto copy_values = [&annotation](const std::string &key, auto &vec) {
            auto &values = annotation.values;
            auto it = values.find(key);
            if (it != values.end())
            {
                for (auto &v : it->second.get<osrm::util::json::Array>().values)
                {
                    vec.emplace_back(v.get<osrm::util::json::Number>().value);
                }
            }
        };

        copy_values("duration", result.duration);
        copy_values("nodes", result.nodes);
        copy_values("distance", result.distance);
        copy_values("weight", result.weight);
        copy_values("datasources", result.datasources);
        copy_values("speed", result.speed);

        return result;
    }

    auto convert_leg(osrm::json::Object &leg)
    {
        routing_machine::RouteLeg result;
        result.summary = leg.values["summary"].get<osrm::util::json::String>().value;
        result.distance = leg.values["distance"].get<osrm::util::json::Number>().value;
        result.duration = leg.values["duration"].get<osrm::util::json::Number>().value;
        result.weight = leg.values["weight"].get<osrm::util::json::Number>().value;

        auto &steps = leg.values["steps"].get<osrm::util::json::Array>().values;
        for (auto &step : steps)
        {
            result.steps.emplace_back(convert_step(step.get<osrm::util::json::Object>()));
        }

        if (leg.values.count("annotation"))
        {
            result.annotation = convert_annotation(leg.values["annotation"].get<osrm::util::json::Object>());
        }

        return result;
    }

    auto convert_route(osrm::json::Object &route)
    {
        routing_machine::Route result;
        result.weight_name = route.values["weight_name"].get<osrm::util::json::String>().value;
        result.distance = route.values["distance"].get<osrm::util::json::Number>().value;
        result.duration = route.values["duration"].get<osrm::util::json::Number>().value;
        result.weight = route.values["weight"].get<osrm::util::json::Number>().value;

        auto &geometry = route.values["geometry"].get<osrm::util::json::Object>().values;
        result.coordinates = convert_coordinates(geometry["coordinates"].get<osrm::util::json::Array>());

        auto &legs = route.values["legs"].get<osrm::util::json::Array>().values;
        for (auto &leg : legs)
        {
            result.legs.emplace_back(convert_leg(leg.get<osrm::util::json::Object>()));
        }

        return result;
    }

    auto convert_matching(osrm::json::Object &matching)
    {
        routing_machine::Matching result;
        result.confidence = matching.values["confidence"].get<osrm::util::json::Number>().value;

        auto &geometry = matching.values["geometry"].get<osrm::util::json::Object>().values;
        result.coordinates = convert_coordinates(geometry["coordinates"].get<osrm::util::json::Array>());

        auto &legs = matching.values["legs"].get<osrm::util::json::Array>().values;
        for (auto &leg : legs)
        {
            result.legs.emplace_back(convert_leg(leg.get<osrm::util::json::Object>()));
        }

        return result;
    }

    template<typename Request, typename Parameters>
    void convert_base_parameters(const Request &request, Parameters &parameters)
    {
        for (const auto &radius : request.radiuses)
        {
            parameters.radiuses.push_back(radius == 0. ? boost::optional<double>() : radius);
        }

        for (const auto &approach : request.approaches)
        {
            parameters.approaches.push_back(static_cast<osrm::engine::Approach>(approach));
        }

        parameters.exclude = request.exclude;
        parameters.steps = request.steps;
        parameters.number_of_alternatives = request.number_of_alternatives;
        parameters.annotations_type = static_cast<osrm::engine::api::RouteParameters::AnnotationsType>(request.annotation);
        parameters.overview = static_cast<osrm::engine::api::RouteParameters::OverviewType>(request.overview);
        parameters.continue_straight = request.continue_straight;
    }
}

struct OSRMProxy
{
    OSRMProxy(const osrm::OSRM& osrm) : osrm(osrm) {}

    bool route(routing_machine::RouteService::Request  &req, routing_machine::RouteService::Response &res)
    {
        // Only if the client ask for waypoints
        if(req.get_wpts == true)
        {
            // Variables declaration
            // std::vector<double> latitude;
            // std::vector<double> longitude;

            waypointsCoords calculatedWaypoints;

            float endLatitude;
            float endLongitude;
            float startLatitude;
            float startLongitude;
            bool goTo;

            string apiReturn;
            string shape;
            string status;
            string apiReturnJSON;
            string coordinatesJSON;

            // First test : we check if the interface has been launched. If so, it should have set the parameters
            // Second test : we check if the user has provided a route by checking goTo's value.
            if (ros::param::get("/routing_machine/destination/goTo", goTo) && goTo == true)
            {
                // If the parameters goTo has been set, the 4 others should have been too. We get their values.
                ros::param::get("/routing_machine/start/latitude", startLatitude);
                ros::param::get("/routing_machine/start/longitude", startLongitude);
                ros::param::get("/routing_machine/destination/latitude", endLatitude);
                ros::param::get("/routing_machine/destination/longitude", endLongitude);

                ROS_INFO("Start lat : %f lon : %f - End : lat : %f lon : %f", startLatitude, startLongitude, endLatitude, endLongitude);


                // Set route parameters
                osrm::RouteParameters parameters;

                // Set common parameters
                convert_base_parameters(req, parameters);

                // Set fixed parameters
                parameters.geometries = osrm::engine::api::RouteParameters::GeometriesType::GeoJSON;

                // Set request waypoints and bearings
                for (const auto &waypoint : req.waypoints)
                {
                    auto longitude = osrm::util::FloatLongitude{waypoint.position.x};
                    auto latitude = osrm::util::FloatLatitude{waypoint.position.y};
                    parameters.coordinates.emplace_back(longitude, latitude);
                }

                for (const auto &bearing : req.bearings)
                {
                    parameters.bearings.push_back(osrm::engine::Bearing{bearing.bearing, bearing.range});
                }

                if (!parameters.IsValid())
                {
                    res.code = "IncorrectParameters";
                    return true;
                }

                // Find a route
                osrm::json::Object response;
                const auto status = osrm.Route(parameters, response);

                // std::stringstream sstr;
                // osrm::util::json::render(sstr, response);
                // ROS_INFO("%s", sstr.str().c_str());

                // Convert JSON response into a ROS message
                res.code = response.values["code"].get<osrm::util::json::String>().value;
                if (status != osrm::engine::Status::Ok)
                    throw ros::Exception(response.values["message"].get<osrm::util::json::String>().value);

                for (auto &route : response.values["routes"].get<osrm::json::Array>().values)
                {
                    res.routes.emplace_back(convert_route(route.get<osrm::json::Object>()));
                }

                res.success = true;
                return true;
            }
            else
            {
                // The interface has not been launched or the user hasn't provided any route
                ROS_INFO("No destination set.");
                res.success = false;
                return true;
            }
        }
        else
        {
            res.success = false;
            return true;
        }
    }

    bool match(routing_machine::MatchService::Request  &req, routing_machine::MatchService::Response &res)
    {
        // Set map matching parameters
        osrm::MatchParameters parameters;
        std::vector<unsigned> timestamps;

        // Set common parameters
        convert_base_parameters(req, parameters);

        // Set fixed parameters
        parameters.geometries = osrm::engine::api::RouteParameters::GeometriesType::GeoJSON;

        // Set request waypoints, bearings and timestamps
        for (const auto &waypoint : req.waypoints)
        {
            auto longitude = osrm::util::FloatLongitude{waypoint.pose.position.x};
            auto latitude = osrm::util::FloatLatitude{waypoint.pose.position.y};
            parameters.coordinates.emplace_back(longitude, latitude);

            tf::Quaternion orientation(waypoint.pose.orientation.x, waypoint.pose.orientation.y,
                                       waypoint.pose.orientation.z, waypoint.pose.orientation.w);
            if (fabs(orientation.length2() - 1 ) < tf::QUATERNION_TOLERANCE / 2)
            {
                const auto yaw = tf::getYaw(orientation);
                const auto bearing = static_cast<short>(std::round((180. * yaw / M_PI) + (yaw < 0 ? 360. : 0.)));
                parameters.bearings.push_back(osrm::engine::Bearing{bearing, 10});
            }
            else
            {
                parameters.bearings.push_back(boost::none);
            }

            timestamps.push_back(waypoint.header.stamp.toNSec() / 1000000000ull);
        }

        if (std::any_of(timestamps.begin(), timestamps.end(), [](auto x) { return x != 0; } ))
            parameters.timestamps = std::move(timestamps);

        parameters.gaps = static_cast<osrm::engine::api::MatchParameters::GapsType>(req.gaps);
        parameters.tidy = req.tidy;

        if (!parameters.IsValid())
        {
            res.code = "IncorrectParameters";
            return true;
        }

        // Do a map-matching
        osrm::json::Object response;
        const auto status = osrm.Match(parameters, response);

        // std::stringstream sstr;
        // osrm::util::json::render(sstr, response);
        // ROS_INFO("%s", sstr.str().c_str());

        // Convert JSON response into a ROS message
        res.code = response.values["code"].get<osrm::util::json::String>().value;
        if (status != osrm::engine::Status::Ok)
            throw ros::Exception(response.values["message"].get<osrm::util::json::String>().value);

        for (auto &matching : response.values["matchings"].get<osrm::json::Array>().values)
        {
            res.matchings.emplace_back(convert_matching(matching.get<osrm::json::Object>()));
        }

        return status == osrm::engine::Status::Ok;
    }

    const osrm::OSRM &osrm;
};


// ===> main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "routing_machine");
	ros::NodeHandle n;

    std::string server_type;

    if (n.getParam("/routing_machine/server_type", server_type)) ;
    else server_type = "server";

    // ros::Publisher waypoints_publisher = n.advertise<routing_machine::OutputCoords>("routing_machine/waypoints_coords", 10);
    ROS_INFO("Routing Machine ready !");
    if (!server_type.empty()) ROS_INFO("Server type:  %s",server_type.c_str());

    //ros::ServiceServer service1 = n.advertiseService("routing_machine/get_wpts", getRouting);
    if (server_type == "server") {
	    ros::ServiceServer service1 = n.advertiseService("routing_machine/get_wpts", getRouting);

        // Start service loop
        ROS_INFO("Routing machine service is ready");
        ros::AsyncSpinner spinner(0);
        spinner.start();
        ros::waitForShutdown();
    }
    else if (server_type == "local") {
        // Get parameters and fill config structures
        osrm::EngineConfig config;

        std::string base_path;
        n.param("base_path", base_path, std::string());
        config.storage_config = osrm::StorageConfig(base_path);

        std::string algorithm;
        n.param<std::string>("algorithm", algorithm, "CH");
        std::transform(algorithm.begin(), algorithm.end(), algorithm.begin(), ::tolower);
        config.algorithm =
                algorithm == "mld" ? osrm::engine::EngineConfig::Algorithm::MLD :
                algorithm == "corech" ? osrm::engine::EngineConfig::Algorithm::CoreCH :
                osrm::engine::EngineConfig::Algorithm::CH;

        n.param("use_shared_memory", config.use_shared_memory, base_path.empty());

        // Create OSRM engine and ROS<->OSRM proxy
        ROS_INFO("Starting ROSRM with %s using %s",
                 config.use_shared_memory ? "shared memory" : base_path.c_str(),
                 algorithm.c_str());

        try
        {
            osrm::OSRM osrm(config);
            OSRMProxy proxy(osrm);

            // Advertise OSRM service
            ros::ServiceServer service2 = n.advertiseService("routing_machine/get_wpts_local", &OSRMProxy::route, &proxy);
            ros::ServiceServer service3 = n.advertiseService("routing_machine/match_local", &OSRMProxy::match, &proxy);

            // Start service loop
            ROS_INFO("Routing machine service is ready");
            ros::AsyncSpinner spinner(0);
            spinner.start();
            ros::waitForShutdown();
        }
        catch(const std::exception& exc)
        {
            ROS_ERROR(exc.what());
            return 1;
        }

    }
    else ROS_ERROR("Unsupported server type used! Pick server or local option!");

	ros::spin();

	return 0;
}

// ===> getRouting : principal function
bool getRouting(routing_machine::ParseWptsService::Request  &req, routing_machine::ParseWptsService::Response &res)
{
	// Only if the client ask for waypoints
	if(req.get_wpts == true)
	{
		// Variables declaration
		// std::vector<double> latitude;
		// std::vector<double> longitude;

		waypointsCoords calculatedWaypoints;
		
		float endLatitude;
		float endLongitude;
		float startLatitude;
		float startLongitude;
		bool goTo;
		
		string apiReturn;
		string shape;
		string status;
		string apiReturnJSON;
		string coordinatesJSON;

		// First test : we check if the interface has been launched. If so, it should have set the parameters
		// Second test : we check if the user has provided a route by checking goTo's value.
		if (ros::param::get("/routing_machine/destination/goTo", goTo) && goTo == true)
		{
			// If the parameters goTo has been set, the 4 others should have been too. We get their values.
			ros::param::get("/routing_machine/start/latitude", startLatitude);
			ros::param::get("/routing_machine/start/longitude", startLongitude);
			ros::param::get("/routing_machine/destination/latitude", endLatitude);
			ros::param::get("/routing_machine/destination/longitude", endLongitude);

			ROS_INFO("Start lat : %f lon : %f - End : lat : %f lon : %f", startLatitude, startLongitude, endLatitude, endLongitude);

			// Start message
			ROS_INFO("Routing Machine : WIP !");

            // First, call the API
            // apiReturn = apiCall("valhalla.mapzen.com",formateParameters(startLatitude,startLongitude,endLatitude,endLongitude));
            apiReturn = apiCall("router.project-osrm.org",formateParametersOSRM(startLatitude,startLongitude,endLatitude,endLongitude));

            // Test the return. If we have an error, it means that the API is disconnected or the car isn't connected
            // to the internet, so no need to go further.
            if (apiReturn == "error")
            {
                ROS_FATAL("Routing Machine : Impossible to connect to routing API. Routing failed.");
                res.success = false;
            }
            else
            {
                // Data has arrived! Next, extract the shape.
                ROS_INFO("Routing Machine : Got information from API, processing.");
                // shape = isolateShape(apiReturn);
                status = getStatus(apiReturn);

                ROS_INFO("Check API return : \n%s",apiReturn.c_str());
                ROS_INFO("Check API status : %s",status.c_str());

                // Here, two possibilities.
                // - The export is a JSON object, containing the shape, so we use string function manipulation
                //   to extract it, and after we'll be able to decode it with the polyline algorith.
                // - The export is not parsable. That's mean that instead of a nice JSON object, we got an error
                //   coming from the API. It could be pretty much anything, so we display it for debugging purpose.
                //   For example, it could be a coordinate that doesn't exists.

                if(status != "Ok")
                {
                    ROS_FATAL("Routing Machine : Unable to parse data");
                    // ROS_FATAL("Check API return : \n%s",apiReturn.c_str());
                    ROS_FATAL("Check API status : %s",status.c_str());

                    res.success = false;
                }
                else
                {
                    ROS_INFO("Routing Machine : Parsed correctly");
                    // Now we can decode the shape, using the google algorithm.
                    // More info here : https://developers.google.com/maps/documentation/utilities/polylinealgorithm
                    // We give the function two pointers, so we get our vectors directly written.
                    // decodePolyline(apiReturn, &latitude, &longitude);

                    // decodedJSON = decodeJSON(apiReturn.substr(apiReturn.find("{\"code\""), apiReturn.npos));
                    apiReturnJSON = apiReturn.substr(apiReturn.find("{\"code\""), apiReturn.npos);

                    // ROS_INFO("apiReturnJSON : \n%s",apiReturnJSON.c_str());
                    // TODO: add good JSON parser
                    coordinatesJSON = apiReturnJSON.substr(apiReturnJSON.find("{\"coordinates\"")+16, apiReturnJSON.find(",\"type\"")-apiReturnJSON.find("{\"coordinates\"")-17);
                    ROS_INFO("coordinatesJSON : \n%s",coordinatesJSON.c_str());

                    // TODO: add support for local server using https://github.com/oxidase/rosrm and https://github.com/Project-OSRM/osrm-backend/blob/master/example/example.cpp

                    calculatedWaypoints = getVecFromStr(coordinatesJSON);

                    calculatedWaypoints.latitude.push_back(endLatitude);
                    calculatedWaypoints.latitude.insert(calculatedWaypoints.latitude.begin(), startLatitude);
                    calculatedWaypoints.longitude.push_back(endLongitude);
                    calculatedWaypoints.longitude.insert(calculatedWaypoints.longitude.begin(), startLongitude);

                    // And now we make a nice output.
                    res.latitude = calculatedWaypoints.latitude;
                    res.longitude = calculatedWaypoints.longitude;
                    res.num_wpts = calculatedWaypoints.latitude.size();
                    res.success = true;

                    // routing_machine::OutputCoords waypointsMsg;
                    // waypointsMsg.latitude = calculatedWaypoints.latitude;
                    // waypointsMsg.longitude = calculatedWaypoints.longitude;
                    // waypoints_publisher.publish(waypointsMsg);
                }
			}

			ROS_INFO("Routing Machine : Finish! %ld waypoints.", calculatedWaypoints.latitude.size());
			return true;
		}
		else
		{
			// The interface has not been launched or the user hasn't provided any route
			ROS_INFO("No destination set.");
			res.success = false;
			return true;
		}
	}
	else
	{
		res.success = false;
		return true;
	}
}

waypointsCoords getVecFromStr(std::string input)
{
	string pairCoordinates;
	waypointsCoords result;
	
	while (!input.empty()) {
		std::size_t start = input.find("[")+1; 
		std::size_t end = input.find("]")-1;  
		pairCoordinates = input.substr(start, end-start+1);
		try {
			input.erase(start-1, end-start+3);
		}
		catch (...) {
			// std::cout << "Cannot erase" << std::endl;
			break;
		}
			
		// ROS_INFO("Pairs : \n%s",pairCoordinates.c_str());

		std::stringstream iss(pairCoordinates);
		double coodinate;
		std::vector<double> coodinates;
		
		while (iss >> coodinate){
			coodinates.push_back(coodinate);
			if (iss.peek() == ',')
        		iss.ignore();
			// std::cout << coodinate << std::endl;
		}

		result.latitude.push_back(coodinates[0]);
		result.longitude.push_back(coodinates[1]);
		
	}
	
	/* for (int i = 0; i<result.latitude.size(); i++) {
		std::cout << result.latitude[i] << std::endl;
		std::cout << result.longitude[i] << std::endl;
	} */

	return result;
}

// ===> apiCall : open a socket on the website asked, with the parameters, and return the whole result.
string apiCall(string website, string parameters) 
{
	// Adapted from : http://www.mzan.com/article/17685466-http-request-by-sockets-in-c.shtml

	// Variables declarations
	int sock;
	struct sockaddr_in client;
	int PORT = 80;
	string output;
	string realoutput;
	struct hostent * host = gethostbyname(website.c_str());

	// Program starts
	ROS_INFO("Routing Machine : Contact API at %s%s",website.c_str(), parameters.c_str());

	if ( (host == NULL) || (host->h_addr == NULL) ) {
		ROS_FATAL("Routing Machine : Error contacting DNS. Are you connected to the internet?");
		return "error";
	}

	bzero(&client, sizeof(client));
	client.sin_family = AF_INET;
	client.sin_port = htons( PORT );
	memcpy(&client.sin_addr, host->h_addr, host->h_length);

	sock = socket(AF_INET, SOCK_STREAM, 0);

	if (sock < 0) {
		ROS_FATAL("Routing Machine : Error creating socket.");
		return "error";
	}

	if ( connect(sock, (struct sockaddr *)&client, sizeof(client)) < 0 ) {
		close(sock);
		ROS_FATAL("Routing Machine : Could not connect to API.");
		return "error";
	}

	stringstream ss;
	ss << "GET "<< parameters << " HTTP/1.0\r\n"
		 << "Host: " << website << "\r\n"
		 << "Accept: application/json\r\n"
		 << "Content-Type: application/x-www-form-urlencoded\r\n"
		 << "\r\n\r\n";
	string request = ss.str();

	if (send(sock, request.c_str(), request.length(), 0) != (int)request.length()) {
		ROS_FATAL("Routing Machine : Error sending request.");
		return "error";
	}

	// Copy buffer to output
	char cur; 
	int i = 0;
	int j = 0;
	while ( read(sock, &cur, 1) > 0 ) {
		output += cur;
		i++;
	}

	return output;
}

// ===> isolateShape : make a little string manipulation to separate the shape from the whole API return.
string isolateShape(string input)
{
	std::size_t start = input.find("[{\"shape\":\"") + 11; 
	
	if(start == string::npos + 11) 
	{
		return "error";
	}
	else
	{	
		std::size_t end = input.find("\"", start+12);  
		return input.substr(start, end-start);
	}
}

string getStatus(string input)
{
	/*
	Keyword code response values:

	Ok	Request could be processed as expected.
	InvalidUrl	URL string is invalid.
	InvalidService	Service name is invalid.
	InvalidVersion	Version is not found.
	InvalidOptions	Options are invalid.
	InvalidQuery	The query string is synctactically malformed.
	InvalidValue	The successfully parsed query parameters are invalid.
	NoSegment	One of the supplied input coordinates could not snap to street segment.
	TooBig	The request size violates one of the service specific request size restrictions.
	*/

	std::size_t start = input.find("{\"code\"")+9;  
	std::size_t end = input.find("\"", start); 

	return input.substr(start, end-start);
	
}

// ===> decodePolyline : decode the shape, which is encoded with a google algorithm
int decodePolyline(string encoded, std::vector<double> *latitude, std::vector<double> *longitude)
{
	//Adapted from : https://github.com/paulobarcelos/ofxGooglePolyline
	// More info here : https://developers.google.com/maps/documentation/utilities/polylinealgorithm
	std::vector<float> points;
	int len = encoded.length();
	int index = 0;
	float lat = 0;
	float lng = 0;

	while (index < len) {
		char b;
		int shift = 0;
		int result = 0;
		do {
			b = encoded.at(index++) - 63;
			result |= (b & 0x1f) << shift;
			shift += 5;
		} while (b >= 0x20);
		float dlat = ((result & 1) ? ~(result >> 1) : (result >> 1));
		lat += dlat;

		shift = 0;
		result = 0;
		do {
			b = encoded.at(index++) - 63;
			result |= (b & 0x1f) << shift;
			shift += 5;
		} while (b >= 0x20);
		float dlng = ((result & 1) ? ~(result >> 1) : (result >> 1));
		lng += dlng;

		latitude -> push_back(lat * (float)1e-6);
		longitude -> push_back(lng * (float)1e-6);
	}

	return 0;
}

// ===> formateParameters : using the start and finish coordinates, it formates the string with the params in it
string formateParameters(double slat, double slng, double elat, double elng)
{
	char output[500];
	sprintf(output,"/route?json={\"locations\":[{\"lat\":%f,\"lon\":%f},{\"lat\":%f,\"lon\":%f}],\"costing\":\"pedestrian\",\"directions_options\":{\"units\":\"miles\"}}&api_key=valhalla-RsYgicy",slat,slng,elat,elng);
	return output;
}


// ===> formateParametersOSRM : using the start and finish coordinates, it formates the string with the params in it
string formateParametersOSRM(double slat, double slng, double elat, double elng)
{
	char output[500];
	// Modes: driving, cycling, walking
	// According to https://github.com/Project-OSRM/osrm-backend/blob/master/docs/http.md
	sprintf(output,"/route/v1/cycling/%f,%f;%f,%f?overview=full&geometries=geojson",slat,slng,elat,elng);

	return output;
}