#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ROS_INFO_STREAM("Moving the arm to the center");

  // Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget srv;

  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  if (!client.call(srv))
  {
    ROS_ERROR("Failed to call service command_robot");
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;
  int height = img.height;
  int width = img.width;
  std::vector<uint8_t> data = img.data;//get the image data

  // TODO: Loop through each pixel in the image and check if there's a bright white one
  // Then, identify if this pixel falls in the left, mid, or right side of the image
  // Depending on the white ball position, call the drive_bot function and pass velocities to it
  // Request a stop when there's no white ball seen by the camera

  // Loop through each pixel in the image and check if its equal to the first one
  uint8_t r = 0;//red pixel
  uint8_t g = 0;//green pixl
  uint8_t b = 0;//blue pixel
  int pxlInd = 0;//which pixel in the image is it
  float avgx = 0.0;//holds the average of the columns of the pixel indexs that are white
  int minx = 0;//holds the min x saved
  int maxx = 0;//holds the max x saved
  int numSav = 0;//hold the number of pixels saved
  int col = 0;//image column
  float balld = 0.2;//ball actual diameter in meters
  float fx = 476.7030836014194;//focal x
  float cx = 400.5;//center x
  float zd = 2.0;//stay 2 meter back
  float kv = 0.5;//linear gain
  float kw = 0.002;//angular gain
  for (int i = 0; i < data.size(); i+=3)
  {
    // break out the three colors
    r = data.at(i);
    g = data.at(i+1);
    b = data.at(i+2);

    //get the column
    col = pxlInd%width;

    //if all three colors are equal to white_pixel then the pixel is white and index column is saved to the average, min, and max
    if ((r == white_pixel) && (g == white_pixel) && (b == white_pixel))
    {
      avgx += float(col);

      //if its the first pixel saved set to max and min otherwise compare
      if (numSav == 0)
      {
        minx = col;
        maxx = col;
      }
      else
      {
        if (col < minx)
        {
          minx = col;
        }

        if (col > maxx)
        {
          maxx = col;
        }
      }
      numSav++;
    }
    pxlInd++;
  }

  //if there are some pixels that are white then use the average to get the center and the max-min to get the distance to ball
  //otherwise send stop
  if (numSav > 0)
  {
    avgx /= float(numSav);//determine the average
    int balldpxl = maxx - minx;//diameter in pixels
    float z = fx*balld/float(balldpxl);//get the depth
    float lin_vel = kv*(z-zd);//if ball is farther than desired go positive otherwise go negative
    float ang_vel = kw*(cx-avgx);//if ball is to left of center go positive otherwise go negative
    drive_robot(lin_vel,ang_vel);
  }
  else
  {
    drive_robot(0.0,0.0);
  }
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
