#include <ros/ros.h>
#include "std_msgs/String.h"

class Cmd_Node
{
public:
    Cmd_Node()
    {
        // Initialize the node handle
        nh_ = ros::NodeHandle();

        // Create a subscriber that listens to the "jetson_commands" topic
        subscription_ = nh_.subscribe("jetson_commands", 1, &Cmd_Node::topicCallback, this); 
        // Publisher to send messages (if required, initialize publisher as needed)
        publisher_ = nh_.advertise<std_msgs::String>("cmd_response", 10); 
    }

    // Callback function to process received messages
    void topicCallback(const std_msgs::String::ConstPtr& msg)
    {
        std_msgs::String new_msg;

        // Handling the command strings and setting new messages
        if (msg->data == "high") {
            new_msg.data = "High command received!";
        }
        else if (msg->data == "low") {
            new_msg.data = "Low command received!";
        }
        else if (msg->data == "left") {
            new_msg.data = "Turn left!";
        }    
        else if (msg->data == "right") {
            new_msg.data = "Turn right!";
        }  
        else if (msg->data == "back") {
            new_msg.data = "Move back!";
        }  
        else if (msg->data == "stop") {
            new_msg.data = "Stop command received!";
        }   
        else {
            new_msg.data = "Invalid command!";
        }

        // Publish the new message to a topic if required
        publisher_.publish(new_msg);
    }

private:
    ros::NodeHandle nh_;  // Node handle
    ros::Subscriber subscription_;  // Subscriber object
    ros::Publisher publisher_;  // Publisher object
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_node");  // Initialize the ROS node
    Cmd_Node cmd_node;  // Create an instance of the Cmd_Node class

    ros::spin();  // Keep the program alive to receive messages

    return 0;
}

