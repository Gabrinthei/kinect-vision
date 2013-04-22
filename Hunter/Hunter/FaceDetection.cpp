#include <iostream>
#include <opencv.hpp>
#include <OpenNI.h>
#include "iostream"
#include "opencv.hpp"
#include "NiTE.h"


#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

void overlayImage(const cv::Mat &background, const cv::Mat &foreground,cv::Mat &output, cv::Point2i location)
{
  background.copyTo(output);


  // start at the row indicated by location, or at row 0 if location.y is negative.
  for(int y = std::max(location.y , 0); y < background.rows; ++y)
  {
    int fY = y - location.y; // because of the translation

    // we are done of we have processed all rows of the foreground image.
    if(fY >= foreground.rows)
      break;

    // start at the column indicated by location,

    // or at column 0 if location.x is negative.
    for(int x = std::max(location.x, 0); x < background.cols; ++x)
    {
      int fX = x - location.x; // because of the translation.

      // we are done with this row if the column is outside of the foreground image.
      if(fX >= foreground.cols)
        break;

      // determine the opacity of the foregrond pixel, using its fourth (alpha) channel.
      double opacity =((double)foreground.data[fY * foreground.step + fX * foreground.channels() + 3])/ 255.;


      // and now combine the background and foreground pixel, using the opacity,

      // but only if opacity > 0.
	  //double opacity = 150;
      for(int c = 0; opacity > 0 && c < output.channels(); ++c)
      {
        unsigned char foregroundPx =
          foreground.data[fY * foreground.step + fX * foreground.channels() + c];
        unsigned char backgroundPx =
          background.data[y * background.step + x * background.channels() + c];
        output.data[y*output.step + output.channels()*x + c] =
          backgroundPx * (1.-opacity) + foregroundPx * opacity;
      }
    }
  }
}




void updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
	USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
	USER_MESSAGE("Out of Scene")
	else if (user.isLost())
	USER_MESSAGE("Lost")

	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
				break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
				break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
				break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
				break;
		}
	}
}

int main(int argc, char** argv)
{

    cv::Mat image;
	cv::Mat changedImage;
    image = cv::imread("HunterHead.png", CV_LOAD_IMAGE_UNCHANGED); 
	cv::Mat bw; //= cv::Mat(cv::Size(320, 240), CV_8UC3, cv::Scalar(255, 255, 0));
	if(! image.data )                              // Check for invalid input
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
    }
	//cv::imshow("image", image);

	openni::Device device;
	openni::VideoStream color;
	openni::VideoFrameRef colorFrame;
	openni::Status rc = openni::STATUS_OK;
	

	rc = openni::OpenNI::initialize();
	rc = device.open(openni::ANY_DEVICE);
	rc = color.create(device, openni::SENSOR_COLOR);
	rc = color.start();



	nite::UserTracker userTracker;
	nite::Status niteRc;
	nite::NiTE::initialize();
	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK)
	{
		printf("Couldn't create user tracker\n");
		cv::waitKey(0);
		return 3;
	}
	printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

	nite::UserTrackerFrameRef userTrackerFrame;
	float min =10000.0;
	float max = -10000;
	cv::namedWindow("bw",1);
	while (true)
	{
		color.readFrame(&colorFrame);
		const openni::RGB888Pixel* imageBuffer = (const openni::RGB888Pixel*)colorFrame.getData();

		bw.create(colorFrame.getHeight(),colorFrame.getWidth(),CV_8UC3);
		memcpy(bw.data,imageBuffer,3*colorFrame.getHeight()*colorFrame.getWidth()*sizeof(uint8_t));

		//cv::cvtColor(bw,bw,CV_BGR2RGB);

		niteRc = userTracker.readFrame(&userTrackerFrame);
		if (niteRc != nite::STATUS_OK)
		{
			printf("Get next frame failed\n");
			continue;
		}

		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			updateUserState(user,userTrackerFrame.getTimestamp());
			if (user.isNew())
			{
				userTracker.startSkeletonTracking(user.getId());
			}
			else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
			{
				const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
				if (head.getPositionConfidence() > .5){
					float xer = head.getPosition().x;
					if (xer < min)
						min = xer;
					else if(xer>max)
						max = xer;
					printf("%d. x=(%5.2f, y= %5.2f, z= %5.2f)\n", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z);
					float newX, newY;
					userTracker.convertJointCoordinatesToDepth(head.getPosition().x, head.getPosition().y, head.getPosition().z, &newX, &newY);
					
					int size = 1+(40000/head.getPosition().z);
					printf("%d)\n", size);
					cv::resize(image,changedImage,cv::Size(size*2,size*2));
					overlayImage(bw, changedImage, bw, cv::Point2i(newX-changedImage.cols/2,newY-changedImage.rows/2));
				}
			}
		}
		cv::imshow("bw", bw);
		
		int key = cv::waitKey(10);
		if (key==27)
			break;
	}
	std::cout << "min " << min << " and max " << max << std::endl;
	nite::NiTE::shutdown();

}

