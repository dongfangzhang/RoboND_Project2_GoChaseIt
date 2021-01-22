#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if (!client.call(srv))
	ROS_ERROR("Failed to Move the Robot");
    ros::Duration(1).sleep();
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int ball_position=img.width;
    int avg_pixel=0;
    bool ball_found=false;
    
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for(size_t j{0};j<img.height;j++)
    {
	for(size_t i{0};i<img.step;i++)
	{
		avg_pixel = (img.data[(j*img.step+i)]+img.data[j*img.step+1+i]+img.data[(j*img.step+i+2)])/3.0;
		if(avg_pixel==white_pixel)
		{
			
			int camera_pos=i+1;
			//turn left if in the left section , camera_pos <800			
			if(camera_pos<=ball_position)
				drive_robot(0.2,0.5);
			//go straight if in the left section 800<camera_pos<1600
			if(camera_pos>ball_position && camera_pos<=2.0*ball_position)
				drive_robot(0.5,0);
			//turn right if in the right section  camera_pos>1600
			if(camera_pos> 2.0*ball_position)
				drive_robot(0.2,-0.5);
			ball_found=true;
			return;
		}
	}
    }
    ROS_INFO_STREAM("BALL NOT FOUND");

    if(!ball_found)
	drive_robot(0.0,0.0);
    return;
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
