#include <iostream>
#include <opencv.hpp>
#include <OpenNI.h>

/*
int main(int argc, char** argv)
{
	std::cout << "Testing" << std::endl;
	cv::namedWindow("Test", 1);
	openni::Status rc = openni::STATUS_OK;

   openni::Device device;
	
   openni::VideoStream depth, color;
const char* deviceURI = openni::ANY_DEVICE;


openni::OpenNI::initialize();
rc = device.open(deviceURI);


if (rc != openni::STATUS_OK)
{
    printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
    openni::OpenNI::shutdown();
     return 8;
}

rc = depth.create(device, openni::SENSOR_DEPTH);
if (rc == openni::STATUS_OK)
{
rc = depth.start();
if (rc != openni::STATUS_OK)
{
printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
depth.destroy();
}
}
else
{
printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
}


rc = color.create(device, openni::SENSOR_COLOR);
if (rc == openni::STATUS_OK)
{
rc = color.start();
if (rc != openni::STATUS_OK)
{
printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
color.destroy();
}
}
else
{
printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
}


if (!depth.isValid() || !color.isValid())
{
printf("SimpleViewer: No valid streams. Exiting\n");
openni::OpenNI::shutdown();
return 2;
}


openni::VideoFrameRef pFrame;
openni::VideoFrameRef dep;


int k = 0;
cv::Mat f = cv::imread("hunterl.jpg", 1);
while(true){
depth.readFrame(&dep);
openni::DepthPixel* pDepth = (openni::DepthPixel*)dep.getData();
color.readFrame(&pFrame);

openni::RGB888Pixel *pColor = (openni::RGB888Pixel *) pFrame.getData();


cv::Mat frame = cv::Mat(cv::Size(320,240), CV_8UC3);
cv::Mat bw = cv::Mat(cv::Size(320, 240), CV_8UC3);

const int MIN = 1100;
const int MAX = 1900;
for(int i = 0; i<frame.rows; i++)
{
for(int j = 0; j<frame.cols; j++)
{
openni::RGB888Pixel pix = pColor[frame.cols*i+j];
ushort dep = pDepth[frame.cols*i+j];
if( dep > MIN && dep < MAX){
frame.at<cv::Vec3b>(i, j) = cv::Vec3b(pix.b, pix.g, pix.r);
}
else{ 
frame.at<cv::Vec3b>(i,j) = cv::Vec3b(255,255,255);
}
bw.at<cv::Vec3b>(i, j) = cv::Vec3b(pix.b, pix.g, pix.r);
}


}

cv::imshow("bw", bw);
cv::imshow("frame", frame);
int k = (int)cv::waitKey(10);
if(k == 27)
break;
}//end of while

openni::OpenNI::shutdown();

	return 0;
} 


*/

#include <OpenNI.h>
#include "iostream"
#include "opencv.hpp"


#include "NiTE.h"


#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

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
	cv::Mat bw; //= cv::Mat(cv::Size(320, 240), CV_8UC3, cv::Scalar(255, 255, 0));

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
	Status status = STATUS_ERROR;
	while (true)
	{
		color.readFrame(&colorFrame);
		const openni::RGB888Pixel* imageBuffer = (const openni::RGB888Pixel*)colorFrame.getData();

		bw.create(colorFrame.getHeight(),colorFrame.getWidth(),CV_8UC3);
		memcpy(bw.data,imageBuffer,3*colorFrame.getHeight()*colorFrame.getWidth()*sizeof(uint8_t));

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
					status = userTrackerFrame.convertJointCoordinatesToDepth(head.getPosition().x, head.getPosition().y, head.getPosition().z, &newX, &newY) | STATUS_OK;
					printf("%5.2f %5.2f)\n", newX, newY);
					cv::circle(bw, newX, newY)), 20, cv::Scalar(255,255,255), -1, 8, 0);
				}
			}
		}
		cv::imshow("bw", bw);
		int key = cv::waitKey(10);
		if (key==27 || status == STATUS_ERROR)
			break;
	}
	std::cout << "min " << min << " and max " << max << std::endl;
	nite::NiTE::shutdown();

}

