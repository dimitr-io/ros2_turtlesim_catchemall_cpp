#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim_catchemall_cpp/msg/turtle_info.hpp"
#include "turtlesim_catchemall_cpp/srv/get_closest_turtle.hpp"
#include "turtlesim_catchemall_cpp/srv/catch_turtle.hpp"
#include <random>
#include <unordered_map>
#include <limits>
#include <cmath>

using namespace std::chrono_literals;
namespace catchemall_msg = turtlesim_catchemall_cpp::msg;
namespace catchemall_srv = turtlesim_catchemall_cpp::srv;

/// @brief Node that spawns and kills turtles through service requests, and keeps track of alive turtles.
class TurtleSpawnerNode : public rclcpp::Node 
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner"), random_number_engine_(std::random_device{}()), dist_pos_(0.0, 11.0), dist_theta_(0, 2*M_PI)
    {
        this->declare_parameter("spawn_timer_period_sec", 1.0); // default spawn timer 1 second
        double spawn_timer_period_ = this->get_parameter("spawn_timer_period_sec").as_double();

        spawn_request_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!spawn_request_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the turtlesim spawn server...");
        }

        kill_request_client_ = this->create_client<turtlesim::srv::Kill>("kill");

        while (!kill_request_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the turtlesim kill server...");
        }

        // spawn_turtle_callback(); // spawn first target turtle

        spawn_timer_ = this->create_wall_timer(std::chrono::duration<double>(spawn_timer_period_), 
                                               std::bind(&TurtleSpawnerNode::spawn_turtle_callback, this));
        
        get_closest_turtle_server_ = this->create_service<catchemall_srv::GetClosestTurtle>
                                    ("get_closest_turtle",
                                     std::bind(&TurtleSpawnerNode::get_closest_turtle_server_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        catch_turtle_server_ = this->create_service<catchemall_srv::CatchTurtle>
                                    ("catch_turtle", std::bind(&TurtleSpawnerNode::catch_turtle_server_callback, this, std::placeholders::_1, std::placeholders::_2));
    
        RCLCPP_INFO(this->get_logger(), "Turtle Spawner Initialized.");
    }

private:
    /// @brief Sends service request to turtlesim_node to spawn a turtle at a random position in the turtlesim window
    void spawn_turtle_callback()
    {
        spawn_request_ = std::make_shared<turtlesim::srv::Spawn::Request>();
        
        // Random x and y spawn location in [0, 11.0]
        spawn_request_->x = dist_pos_(random_number_engine_); spawn_request_->y = dist_pos_(random_number_engine_); 
        
        // Random theta for in [0, 2pi]
        spawn_request_->theta = dist_theta_(random_number_engine_);
        
        spawn_request_client_->async_send_request(spawn_request_, std::bind(&TurtleSpawnerNode::spawn_response_callback, this, std::placeholders::_1));
    }

    /// @brief Callback to handle response from spawn service request. Add spawned turtle info to all_turtles_info_ set.
    /// @param future Service response future object
    void spawn_response_callback(const rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        auto response = future.get();

        catchemall_msg::TurtleInfo turtle;
        turtle.x = this->spawn_request_->x; 
        turtle.y = this->spawn_request_->y;
        turtle.name = response->name.c_str();
        
        all_turtles_info_[turtle.name] = turtle;

    }
 
    /// @brief Callback from get_closest_turtle_server, finds closest distance turtle and to request position places it in response
    void get_closest_turtle_server_callback(const rclcpp::Client<catchemall_srv::GetClosestTurtle>::Request::SharedPtr& request,
                                            const rclcpp::Client<catchemall_srv::GetClosestTurtle>::Response::SharedPtr& response) 
    {
        if (all_turtles_info_.size() > 0)
        {

            double x = request->x; double y = request->y;

            catchemall_msg::TurtleInfo* closest_turtle_ptr = nullptr;
            double closest_dist = std::numeric_limits<double>::max(); 

            for (auto& [name, turtle] : all_turtles_info_)
            {
                double dist = std::sqrt((turtle.x - x) * (turtle.x - x) + (turtle.y - y) * (turtle.y - y));
                if (dist < closest_dist) 
                {
                    closest_dist = dist;
                    closest_turtle_ptr = &turtle;
                }
            }

            response->closest_turtle.x = closest_turtle_ptr->x;
            response->closest_turtle.y = closest_turtle_ptr->y;
            response->closest_turtle.name = closest_turtle_ptr->name;
            response->success = true;
        }
        else
        {
            response->success = false;
        }

    }
    
    // @brief Callback function for catch turtle request by turtle_controller_node
    void catch_turtle_server_callback(const rclcpp::Client<catchemall_srv::CatchTurtle>::Request::SharedPtr& request,
                                     const rclcpp::Client<catchemall_srv::CatchTurtle>::Response::SharedPtr& response)
    {
        if (all_turtles_info_.find(request->name) != all_turtles_info_.end()) // if name found
        {
            auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
            kill_request->name = request->name;
            kill_request_client_->async_send_request(kill_request, [](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future){future.get();});
            all_turtles_info_.erase(request->name);
            response->success = true;
        }
        else response->success = false;
    }
    
    // Declare clients, relevant timers, servers for interaction with turtlesim_node
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_request_client_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    rclcpp::Client<turtlesim::srv::Spawn>::Request::SharedPtr spawn_request_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_request_client_;

    // Servers for interaction with turtle_controller_node
    rclcpp::Service<catchemall_srv::GetClosestTurtle>::SharedPtr get_closest_turtle_server_;
    rclcpp::Service<catchemall_srv::CatchTurtle>::SharedPtr catch_turtle_server_;

    // map to store all spawned turtles' info (keys are turtle names)
    std::unordered_map<std::string, catchemall_msg::TurtleInfo> all_turtles_info_;
    
    // for generating random spawn locations in turtlesim grid
    std::mt19937 random_number_engine_;
    std::uniform_real_distribution<double> dist_pos_;
    std::uniform_real_distribution<double> dist_theta_;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
