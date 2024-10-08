#ifndef BT_STRING_SERIALIZE_H
#define BT_STRING_SERIALIZE_H

#include <vector>
#include <string>
#include <sstream>
#include <unordered_map>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
class bt_string_serialize
{
public:
    /* This class is used to serialize and deserialize strings for the behavior tree
    */
    bt_string_serialize(){};
    static int stringToInt(const std::string& ss){
        // convert string to int
        return std::stoi(ss);
    }

    static std::string intToString(const int& i){
        // convert int to string
        return std::to_string(i);
    }

    static std::vector<double> stringToVector(const std::string& ss){
        // convert string to vector
        std::vector<double> result;
        std::stringstream stream(ss);

        for (double i; stream >> i;) {
            result.push_back(i);
            if (stream.peek() == ',')
                stream.ignore();
        }

        return result;
    }

    static std::vector<int> stringToVectorInt(const std::string& ss){
        // convert string to vector
        std::vector<int> result;
        std::stringstream stream(ss);

        for (int i; stream >> i;) {
            result.push_back(i);
            if (stream.peek() == ',')
                stream.ignore();
        }

        return result;
    }

    static std::string vectorToString(const std::vector<double>& vec){
        // convert vector to string
        std::string result;
        for (size_t i = 0; i < vec.size(); i++){
            result += std::to_string(vec[i]);
            if (i != vec.size() - 1){
                result += ",";
            }
        }
        return result;
    }

    static std::string geoMsgPtToString(const geometry_msgs::msg::Point& pt){
        // convert geometry_msgs::msg::Point to string
        std::string result;
        result += std::to_string(pt.x) + ",";
        result += std::to_string(pt.y) + ",";
        result += std::to_string(pt.z);
        return result;
    }

    static std::string poseToString(const geometry_msgs::msg::Pose& pose){
        // convert geometry_msgs::msg::Pose to string
        std::string result;
        result += std::to_string(pose.position.x) + ",";
        result += std::to_string(pose.position.y) + ",";
        result += std::to_string(pose.position.z) + ",";
        result += std::to_string(pose.orientation.x) + ",";
        result += std::to_string(pose.orientation.y) + ",";
        result += std::to_string(pose.orientation.z) + ",";
        result += std::to_string(pose.orientation.w);
        return result;
    }

    static geometry_msgs::msg::Pose stringToPose(const std::string& ss){
        // convert string to geometry_msgs::msg::Pose
        geometry_msgs::msg::Pose result;
        std::stringstream stream(ss);
        std::string item;

        for (int i = 0; std::getline(stream, item, ','); i++){
            switch (i){
                case 0:
                    result.position.x = std::stod(item);
                    break;
                case 1:
                    result.position.y = std::stod(item);
                    break;
                case 2:
                    result.position.z = std::stod(item);
                    break;
                case 3:
                    result.orientation.x = std::stod(item);
                    break;
                case 4:
                    result.orientation.y = std::stod(item);
                    break;
                case 5:
                    result.orientation.z = std::stod(item);
                    break;
                case 6:
                    result.orientation.w = std::stod(item);
                    break;
            }
        }

        return result;
    }

    static std::unordered_map<std::string, double> stringToMap(const std::string& ss){
        std::unordered_map<std::string, double> result;
        std::stringstream ss_stream(ss);
        std::string item;

        while (std::getline(ss_stream, item, ',')) {
            std::stringstream item_stream(item);
            std::string key;
            double value;

            if (std::getline(item_stream, key, ':')) {
                item_stream >> value;
                result[key] = value;
            }
        }

        return result;
    }

    static std::string mapToString(const std::unordered_map<std::string, double>& map){
        std::string result;
        for (auto const& [key, val] : map){
            result += key + ":" + std::to_string(val) + ",";
        }
        return result;
    }
};
#endif