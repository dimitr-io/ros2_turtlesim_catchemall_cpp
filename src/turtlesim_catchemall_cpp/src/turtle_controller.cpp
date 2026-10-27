#include "rclcpp/rclcpp.hpp"
#include "turtlesim_catchemall_cpp/msg/turtle_info.hpp"
#include "turtlesim_catchemall_cpp/srv/get_closest_turtle.hpp"
#include "turtlesim_catchemall_cpp/srv/catch_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <random>
#include <set>

using namespace std::chrono_literals;
namespace catchemall_msg = turtlesim_catchemall_cpp::msg;
namespace catchemall_srv = turtlesim_catchemall_cpp::srv;

/// @brief Node that controls the main turtle (my_turtle) to catch target turtle
class TurtleControllerNode : public rclcpp::Node 
{
public:
    TurtleControllerNode() : Node("turtle_controller"), heading_error_integral_(0.0), heading_error_prev_(0), catch_distance_(0.2)
    {
        initialize_parameters();

        my_turtle_ctrl_cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        get_closest_turtle_client_ = this->create_client<catchemall_srv::GetClosestTurtle>("get_closest_turtle"); 

        my_turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>
                              ("/turtle1/pose", 10,
                               std::bind(&TurtleControllerNode::my_turtle_pose_sub_callback, this, std::placeholders::_1));
        
        catch_turtle_client_ = this->create_client<catchemall_srv::CatchTurtle>("catch_turtle");

        while (!get_closest_turtle_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the turtle_spawner get_closest_turtle server...");
        }

        while (!catch_turtle_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the turtle_spawner catch_turtle server...");
        }

        my_turtle_ctrl_timer_ = this->create_timer(std::chrono::duration<double>(1.0 / my_turtle_ctrl_freq_hz_), std::bind(&TurtleControllerNode::my_turtle_ctrl_callback, this));

        my_turtle_ctrl_last_time_ = this->now();

        my_turtle_state_ = TurtleCatcherState::SEARCHING_FOR_TARGET;

        RCLCPP_INFO(this->get_logger(), "Turtle Controller Initialized");
    }

private:
    enum class TurtleCatcherState
    {
        SEARCHING_FOR_TARGET,
        WAITING_FOR_TARGET_RESPONSE,
        CHASING_TARGET,
        WAITING_FOR_CATCH_RESPONSE
    };

    void my_turtle_ctrl_callback()
    {
        switch(my_turtle_state_) {
            case(TurtleCatcherState::SEARCHING_FOR_TARGET):
                // turtle_ctrl_cmd_publish();
                get_closest_turtle_request();
                break;
            
            case(TurtleCatcherState::WAITING_FOR_TARGET_RESPONSE):
                // turtle_ctrl_cmd_publish();
                break;

            case(TurtleCatcherState::CHASING_TARGET):
                my_turtle_distance_from_target_ = calc_distance_from_target();
                if (my_turtle_distance_from_target_ < catch_distance_)
                {
                    catch_turtle_request();
                }  
                turtle_ctrl_cmd_publish();
                break;
        
            case(TurtleCatcherState::WAITING_FOR_CATCH_RESPONSE):
                turtle_ctrl_cmd_publish();
                break;

        }
    }

    // Whenever a new pose is received, decide on control command
    void my_turtle_pose_sub_callback(const turtlesim::msg::Pose& msg)
    {
        my_turtle_pose_ = msg;
    }

    // Service request to get turtle closest to my_turtle
    void get_closest_turtle_request()
    {
        auto request = std::make_shared<catchemall_srv::GetClosestTurtle::Request>();
        request->x = my_turtle_pose_.x; request->y = my_turtle_pose_.y;
        get_closest_turtle_client_->async_send_request(request, std::bind(&TurtleControllerNode::get_closest_turtle_response_callback, this, std::placeholders::_1));
        my_turtle_state_ = TurtleCatcherState::WAITING_FOR_TARGET_RESPONSE;
    }

    void get_closest_turtle_response_callback(rclcpp::Client<catchemall_srv::GetClosestTurtle>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            target_turtle_info_ = response->closest_turtle;
            my_turtle_state_ = TurtleCatcherState::CHASING_TARGET;
            heading_error_integral_ = 0.0;
        }
        else
        {
            my_turtle_state_ = TurtleCatcherState::SEARCHING_FOR_TARGET;
        }
    }

    // Commands published whenever new my_turtle_pose received
    void turtle_ctrl_cmd_publish()
    {
        auto msg = geometry_msgs::msg::Twist();
        if (my_turtle_state_ == TurtleCatcherState::CHASING_TARGET) {
            if (my_turtle_distance_from_target_ < my_turtle_distance_low_threshold_)
                msg.linear.x = my_turtle_min_velocity_; // at low distance reduce speed to prevent missing the target
            else   
                msg.linear.x = my_turtle_velocity_;
            
            // compute heading error
            double error = calc_desired_heading() - my_turtle_pose_.theta;
            error = std::atan2(std::sin(error), std::cos(error)); // normalize to [-pi, pi]

            // compute dt for integral
            rclcpp::Time now = this->now();
            double dt = (now - my_turtle_ctrl_last_time_).seconds();
            
            //
            heading_error_integral_ += error * dt;
            msg.angular.z = heading_kp_ * error + heading_ki_ * heading_error_integral_;
            heading_error_prev_ = error;
            my_turtle_ctrl_last_time_ = now;
        }
        else {
            msg.linear.x = 0;
            msg.angular.z = 0;
        }
        my_turtle_ctrl_cmd_publisher_->publish(msg);
    }

    void catch_turtle_request()
    {
        turtle_ctrl_cmd_publish();
        auto request = std::make_shared<catchemall_srv::CatchTurtle::Request>();
        request->name = target_turtle_info_.name;
        catch_turtle_client_->async_send_request(request, std::bind(&TurtleControllerNode::catch_turtle_response_callback, this, std::placeholders::_1));
        my_turtle_state_ = TurtleCatcherState::WAITING_FOR_CATCH_RESPONSE;

    }

    void catch_turtle_response_callback(rclcpp::Client<catchemall_srv::CatchTurtle>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success) 
        {
            my_turtle_state_ = TurtleCatcherState::SEARCHING_FOR_TARGET;

            if (my_turtle_velocity_ < my_turtle_max_velocity_) my_turtle_velocity_ += my_turtle_velocity_increment_;
            heading_error_integral_ = 0.0;
        }

    }
    
    double calc_desired_heading()
    {
        return std::atan2(target_turtle_info_.y - my_turtle_pose_.y, target_turtle_info_.x - my_turtle_pose_.x);
    }

    double calc_distance_from_target()
    {
        return std::sqrt((my_turtle_pose_.x - target_turtle_info_.x) * (my_turtle_pose_.x - target_turtle_info_.x) + 
                                (my_turtle_pose_.y - target_turtle_info_.y) * (my_turtle_pose_.y - target_turtle_info_.y));;
    }

    void initialize_parameters()
    {
        this->declare_parameter("my_turtle_ctrl_freq_hz", 100.0);
        my_turtle_ctrl_freq_hz_ = this->get_parameter("my_turtle_ctrl_freq_hz").as_double();

        this->declare_parameter("heading_kp", 4.0);
        heading_kp_ = this->get_parameter("heading_kp").as_double();

        this->declare_parameter("heading_ki", 20.0);
        heading_ki_ = this->get_parameter("heading_ki").as_double();

        this->declare_parameter("my_turtle_init_velocity", 2.0);
        my_turtle_velocity_ = this->get_parameter("my_turtle_init_velocity").as_double();

        this->declare_parameter("my_turtle_max_velocity", 10.0);
        my_turtle_max_velocity_ = this->get_parameter("my_turtle_max_velocity").as_double();

        this->declare_parameter("my_turtle_min_velocity", 1.0);
        my_turtle_min_velocity_ = this->get_parameter("my_turtle_min_velocity").as_double();

        this->declare_parameter("my_turtle_distance_low_threshold", 0.8);
        my_turtle_distance_low_threshold_ = this->get_parameter("my_turtle_distance_low_threshold").as_double();

        this->declare_parameter("my_turtle_velocity_increment", 0.2);
        my_turtle_velocity_increment_ = this->get_parameter("my_turtle_velocity_increment").as_double();

    }

    // declare clients, publishers timers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr my_turtle_ctrl_cmd_publisher_;
    rclcpp::Client<catchemall_srv::GetClosestTurtle>::SharedPtr get_closest_turtle_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr my_turtle_pose_sub_;
    rclcpp::Client<catchemall_srv::CatchTurtle>::SharedPtr catch_turtle_client_;
    rclcpp::TimerBase::SharedPtr my_turtle_ctrl_timer_;

    // turtle info
    TurtleCatcherState my_turtle_state_;
    turtlesim::msg::Pose my_turtle_pose_;
    catchemall_msg::TurtleInfo target_turtle_info_;
    double my_turtle_distance_from_target_;
    double my_turtle_ctrl_freq_hz_;
    double my_turtle_velocity_;
    double my_turtle_max_velocity_;
    double my_turtle_velocity_increment_; // how much velocity increases for each turtle caught
    double my_turtle_min_velocity_; // reduced velocity for when very near target
    double my_turtle_distance_low_threshold_; // distance below which target is considered to be "very near"
 
    double heading_kp_; // proportional gain for tracking of desired heading
    double heading_ki_; // integral gain for tracking of desired heading
    double heading_error_integral_;
    double heading_error_prev_;
    rclcpp::Time my_turtle_ctrl_last_time_; // last time at which ctrl command was published

    double catch_distance_; // distance from target turtle at which it can be caught
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
