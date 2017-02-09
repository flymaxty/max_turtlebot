#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "FuzzyController.h"
#include "PIDController.h"

using namespace std;

struct location
{
	float x;
	float y;
	float theta;
}locate;

float GetTargetAngle(float localX, float localY, float localTheta, float targetX, float targetY)
{
	float targetAngle = 0;
	float deltaX = (targetX - localX), deltaY = (targetY-localY);
	if ((deltaX != 0) || (deltaY != 0))
	{
		if((cos(localTheta)*deltaY-sin(localTheta)*deltaX)>=0)
		{
			targetAngle = acos((cos(localTheta)*deltaX + sin(localTheta)*deltaY) / (sqrt(pow(deltaX, 2) + pow(deltaY, 2))));
		}
		else
		{
			targetAngle = -acos((cos(localTheta)*deltaX + sin(localTheta)*deltaY) / (sqrt(pow(deltaX, 2) + pow(deltaY, 2))));	
		}
	}
	else
	{
		targetAngle = localTheta;
	}
	return targetAngle;
}

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	static double roll, pitch, yaw;

	tf::Quaternion q(msg->pose.pose.orientation.x,
					 msg->pose.pose.orientation.y,
					 msg->pose.pose.orientation.z,
					 msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	locate.theta = yaw;

	locate.x = msg->pose.pose.position.x*100;
	locate.y = msg->pose.pose.position.y*100;


	ROS_INFO("x: %f, y: %f, omega: %f",
		locate.x, locate.y, locate.theta);
}

int main(int argc, char **argv)
{
	geometry_msgs::Twist setcmd;
	ros::init(argc, argv, "follow");
	ros::NodeHandle node;
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber sub = node.subscribe("amcl_pose", 1000, callback);
	ros::Rate loop_rate(80);

	ROS_INFO("Initilize................");
	float eX0=0, eX1=0, ecX=0, setVX=0;//zjm
	float eY0=0, eY1=0, ecY=0, setVY=0;
	float eW0=0, eW1=0, ecW=0, setVW=0;
	double xIntegral=0, wIntegral=0;
	fstream test;
	unsigned int timeCount = 0;
	float ellipseA = 50;
	float ellipseB = 80;
	float ellipseC = 400;
	float setPathX = 0;
	float setPathY = 0;
	float setPathW = 0;
	float startX = 0;
	float startY = 0;
	float startW = 0;

	struct timeval tv;

	double timeRecord = 0, timeStart = 0;

	FuzzyController XFuzzyController,  WFuzzyController;//zjm
	PIDController XPIDController,  WPIDController;

	float inputX = 0, inputW = 0;
	double deltaX = 0, deltaY = 0;
	XFuzzyController.FuzzyControllerInitialize(-3, 3, 50, -5, 5, 50, -30, 30, 2.3, 0.5, 0.12);
	WFuzzyController.FuzzyControllerInitialize(-3, 3, 50, -5, 5, 50, -3.14159/2, 3.14159/2, 1.5, 1, 0.3);
	XPIDController.PIDControllerInitialize(0, 10, 0, 0.0125);
	WPIDController.PIDControllerInitialize(0, 10, 0, 0.0125);
	test.open("/home/max/test.txt", ios::out | ios::trunc);
	test.close();
	test.open("/home/max/test.txt");

	ROS_INFO("Done!!!!!");

	while (ros::ok())
	{
		ros::spinOnce();

		gettimeofday(&tv, NULL);
		if (timeCount == 0)
		{
			startX = locate.x;//获取X坐标
			startY = locate.y;//获取Y坐标
			startW = locate.theta;//获取角度Theta
			timeStart = tv.tv_sec + (double)(tv.tv_usec / 1000) / 1000;
		}
		if (timeCount < 5000)
		{
			if (timeCount % 10 == 0)
			{
				setPathX = (float)20*cos(startW) + startX;//step
				setPathY = locate.y;

				//setPathX=startX-10;
				//setPathY = locate.y;
				
				//setPathX=ellipseA*cos((float)timeCount/ellipseC)-ellipseA+startX;//ellipsePath
				//setPathY=ellipseB*sin((float)timeCount/ellipseC)+startY;
			}
		}
		else{}

		inputW = GetTargetAngle(locate.x, locate.y, locate.theta, setPathX, setPathY);
		inputX = sqrt(pow(setPathX - locate.x,2)+pow(setPathY - locate.y,2))*cos(inputW);
		
		if ((inputX <= 1)&&(inputX >= -1))//FuzzyWithI
		{
			eW1 = eW0;
			ecW = eW0 - eW1;
			eW0 = eW1;
			eX1 = inputX;
			ecX = eX0 - eX1;
			eX0 = eX1;
			xIntegral=0;
			wIntegral=0;
		}
		else
		{
			eW1 = inputW;
			ecW = eW0 - eW1;
			eW0 = eW1;
			eX1 = inputX;
			ecX = eX0 - eX1;
			eX0 = eX1;
			xIntegral=xIntegral+eX1;//Fuzzy+I
			wIntegral=wIntegral+eW1;
		}
		setVX=XFuzzyController.FuzzyControlOutput(eX1,ecX)+0.00001*xIntegral;
		setVW=WFuzzyController.FuzzyControlOutput(eW1,ecW)+0.00000*wIntegral;
		
		/*
		eW1 = inputW;//Fuzzy
		ecW = eW0 - eW1;
		eW0 = eW1;
		eX1 = inputX;
		ecX = eX0 - eX1;
		eX0 = eX1;
		setVX=XFuzzyController.FuzzyControlOutput(eX1,ecX);
		setVW=WFuzzyController.FuzzyControlOutput(eW1,ecW);
		*/
			
		
		
		
		if(setVX>0.5)
		{
			setVX=0.5;
		}
		else if(setVX<-0.5)
		{
			setVX=-0.5;
		}
		else{}
		if(setVW>2)
		{
			setVW=2;
		}
		else if(setVW<-2)
		{
			setVW=-2;
		}
		else{}
		//setVX = XPIDController.PIDControlOutput(eX1);
		//setVW = WPIDController.PIDControlOutput(eW1);
		timeRecord = tv.tv_sec + (double)(tv.tv_usec / 1000)/1000-timeStart;
		test << fixed << timeRecord << " " << (setPathX - locate.x) << " " << (setPathY - locate.y) << " " << setVX << " " << setVW << " " << (locate.x - startX) << " " << (locate.y - startY) << " " << (setPathX - startX) << " " << (setPathY - startY) << " " << inputX << " " << inputW <<" "<< locate.theta << endl;
		if (timeCount < 5000)
		{
			setcmd.linear.x = setVX;
			//setcmd.angular.z = setVW;
			setcmd.angular.z = 0;
			timeCount += 1;
		}
		else
		{
			setcmd.linear.x = 0;
			setcmd.angular.z = 0;
		}
		//setcmd.linear.x = 0;


		pub.publish(setcmd);

		loop_rate.sleep();
	}

	test.close();

	//ros::spin();

	return 0;
}