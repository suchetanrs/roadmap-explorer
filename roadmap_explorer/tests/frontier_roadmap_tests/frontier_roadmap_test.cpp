#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <fstream>
#include <vector>
#include <string>

class PointSaver : public rclcpp::Node
{
public:
    PointSaver() : Node("point_saver")
    {
        // Subscriber to /clicked_point topic
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&PointSaver::pointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointSaver node has started.");
    }

private:
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        std::ofstream file("points.csv", std::ios::app);
        if (file.is_open())
        {
            // Write the point to the file in CSV format: x,y,z
            file << msg->point.x << "," << msg->point.y << "," << msg->point.z << "\n";
            file.close();
            RCLCPP_INFO(this->get_logger(), "Point saved: [%f, %f, %f]", msg->point.x, msg->point.y, msg->point.z);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file for writing.");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

void loadPointsFromFile(const std::string &filename)
{
    std::ifstream file(filename);
    std::string line;
    if (file.is_open())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading points from file:");
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string x, y, z;
            std::getline(ss, x, ',');
            std::getline(ss, y, ',');
            std::getline(ss, z, ',');
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point: [%s, %s, %s]", x.c_str(), y.c_str(), z.c_str());
        }
        file.close();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open file for reading.");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointSaver>();

    // Optional: Load points from file at startup
    loadPointsFromFile("points.csv");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}