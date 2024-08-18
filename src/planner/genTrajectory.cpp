/*
 * genTrajectory.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: João Benevides
 *      Modified: João Benevides - August 2024
 */

#include "planner/genTrajectory.h"

namespace DRONE {

    Planner::Planner() {

        init();

        srand(time(NULL)); /* initialize random seed: */
    }

    Planner::~Planner() {

    }

    //Setters

    void Planner::setIsControlStarted(bool state)
    {
        m_isControlStarted = state;
    }

    void Planner::setposeDesired(VectorFive poseDesiredValue)
    {
        m_parameters.poseDesired = poseDesiredValue;
    }

    // Getters

    bool Planner::getIsControlStarted(void)
    {
        return m_isControlStarted;
    }

    /* ###########################################################################################################################*/
    /* ###########################################################################################################################*/
    /* #####################################            REGULAR FUNCTIONS                 ########################################*/
    /* ###########################################################################################################################*/
    /* ###########################################################################################################################*/

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* 		Function: initPlanner
    *	  Created by: jrsbenevides
    *  Last Modified: jrsbenevides
    *
    *  	 Description: 1. Initialize essential functions for trajectory tracking;
    *				  2. Initialize parameters and default values;
    */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void Planner::init(void)
    {
        cout << "Starting Trajectory Planner ...\n" << endl;

        poseDesired = poseDesired.Zero();
        setIsControlStarted(false);
        m_isFirstTimePass = true;
        m_startTime = ros::Time::now().toSec();
        m_parameters.setDefaultParameters();

        loadTopics(n);
        loadSettings(n);

        //Set parameters after user input (loadSettings())
        m_parameters.setTrajectoryCoefficients();
        m_parameters.updateAngularFrequency();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* 		Function: load Topics
    *	  Created by: jrsbenevides
    *  Last Modified: jrsbenevides
    *
    *  	 Description: 1. Define the ROS Topics and its types of messages;
    */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void Planner::loadTopics(ros::NodeHandle& n)
    {

        waypoint_publisher = n.advertise<nav_msgs::Odometry>("/drone/waypoint", 1);
        joy_subscriber = n.subscribe<sensor_msgs::Joy>("/drone/joy", 1, &Planner::joyCallback, this);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* 		Function: load Settings
    *	  Created by: jrsbenevides
    *  Last Modified: jrsbenevides
    *
    *  	 Description: 1. Loads config parameters and loads them into the program by substituting previously created variables.
    *                    Those can be edited in "config/bebopParameters.yaml"
    */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void Planner::loadSettings(ros::NodeHandle& n)
    {

        float32 amplitude;
        if (n.getParam("/drone/amplitude", amplitude))
        {
            m_parameters.amplitude = amplitude;
            cout << "amplitude = " << amplitude << endl;
        }

        float32 velMed;
        if (n.getParam("/drone/velMed", velMed))
        {
            m_parameters.averageLinearSpeed = velMed;
            cout << "velMed = " << velMed << endl;
        }

        string trajectory;
        if (n.getParam("/drone/trajectory", trajectory))
        {
            setTrajectory(trajectory);
            cout << "trajectory = " << trajectory << endl;

        }

        vector<float32> poseDesired;
        if (n.getParam("/drone/poseDesired", poseDesired))
        {
            setposeDesired(VectorFive::Map(&poseDesired[0], 5));
            cout << "poseDesired = " << m_parameters.poseDesired << endl;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* 		Function: angle2quatZYX
    *	  Created by: rsinoue
    *  Last Modified: jrsbenevides Aug 18th 2024
    *
    *  	 Description: 1. Converts Euler angles into quaternions
    */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

    void Planner::angle2quatZYX(VectorQuat& q, const double& yaw, const double& pitch, const double& roll)
    {
        // q = [w, x, y, z]'
        const double cosHalfYaw = cos(0.5 * yaw);
        const double sinHalfYaw = sin(0.5 * yaw);
        const double cosHalfPitch = cos(0.5 * pitch);
        const double sinHalfPitch = sin(0.5 * pitch);
        const double cosHalfRoll = cos(0.5 * roll);
        const double sinHalfRoll = sin(0.5 * roll);

        q(0) = cosHalfYaw * cosHalfPitch * cosHalfRoll + sinHalfYaw * sinHalfPitch * sinHalfRoll;
        q(1) = cosHalfYaw * cosHalfPitch * sinHalfRoll - sinHalfYaw * sinHalfPitch * cosHalfRoll;
        q(2) = cosHalfYaw * sinHalfPitch * cosHalfRoll + sinHalfYaw * cosHalfPitch * sinHalfRoll;
        q(3) = sinHalfYaw * cosHalfPitch * cosHalfRoll - cosHalfYaw * sinHalfPitch * sinHalfRoll;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* 		Function: TrajPlanner
    *	  Created by: jrsbenevides
    *	  Last Modified: jrsbenevides in Aug 18th 2024
    *
    *  	 Description: Trajectory Planner
    *		   Steps: 1. Starts time count right after activating automatic control. This will also start
    *			 OBS: Improvements for 'ident' trajectory  are coming for next version.
    */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

    void Planner::planEightShape(nav_msgs::Odometry& goal, float32 timeSpent)
    {
        #if PRINT_LOG
        std::cout << "Eight-Shaped Trajectory" << std::endl;
        #endif
        goal.pose.pose.position = { m_parameters.amplitude * sin(m_parameters.angularFrequency * timeSpent),
                                    0.5F * m_parameters.amplitude * sin(2 * m_parameters.angularFrequency * timeSpent),
                                    0.0F };

        goal.twist.twist.linear = { m_parameters.angularFrequency * m_parameters.amplitude * cos(m_parameters.angularFrequency * timeSpent),
                                    m_parameters.angularFrequency * m_parameters.amplitude * cos(2 * m_parameters.angularFrequency * timeSpent),
                                    0.0F };

        goal.pose.pose.orientation = geometry_msgs::Quaternion{ 0.0F, 0.0F, 0.0F, 1.0F };

        goal.twist.twist.angular = { 0.0F, 0.0F, 0.0F };
    }

    void Planner::planCircle(nav_msgs::Odometry& goal, float32 timeSpent)
    {
        #if PRINT_LOG
        if (m_parameters.trajectory & circleZXY)
        {
            std::cout << "circle ZXY- Trajectory" << std::endl;
        }
        else
        {
            std::cout << "circle XY - Trajectory" << std::endl;
        }
        #endif
        const double zPositionValue = (m_parameters.trajectory & circleZXY) ? goal.pose.pose.position.y : 0.F;
        const double zVelocityValue = (m_parameters.trajectory & circleZXY) ? goal.twist.twist.linear.y : 0.F;

        goal.pose.pose.position = { m_parameters.amplitude * cos(m_parameters.angularFrequency * timeSpent),
                                    m_parameters.amplitude * sin(m_parameters.angularFrequency * timeSpent),
                                    zPositionValue };

        goal.twist.twist.linear = { -1.F * m_parameters.angularFrequency * m_parameters.amplitude * sin(m_parameters.angularFrequency * timeSpent),
                                    m_parameters.angularFrequency * m_parameters.amplitude * cos(m_parameters.angularFrequency * timeSpent),
                                    zVelocityValue };

        goal.pose.pose.orientation = geometry_msgs::Quaternion{ 0.0F, 0.0F, 0.0F, 1.0F };

        goal.twist.twist.angular = { 0.0F, 0.0F, 0.0F };
    }

    void Planner::planIdentification(nav_msgs::Odometry& goal, float32 timeSpent)
    {
        #if PRINT_LOG
        std::cout << "Identification - Trajectory" << std::endl;
        #endif
        if (timeSpent < 40.F)
        {
            goal.pose.pose.position = { 0.5 * m_parameters.amplitude * (cos(m_parameters.angularFrequency * timeSpent) + cos(timeSpent)),
                                        0.5 * m_parameters.amplitude * (sin(m_parameters.angularFrequency * timeSpent) + sin(timeSpent)),
                                        0.F };

            goal.twist.twist.linear = { -0.5 * m_parameters.amplitude * (m_parameters.angularFrequency * sin(m_parameters.angularFrequency * timeSpent) + sin(timeSpent)),
                                        0.5 * m_parameters.amplitude * (m_parameters.angularFrequency * cos(m_parameters.angularFrequency * timeSpent) + cos(timeSpent)),
                                        0.F };

            goal.pose.pose.orientation = geometry_msgs::Quaternion{ 0.0F, 0.0F, 0.0F, 1.0F };

            goal.twist.twist.angular = { -0.5F * m_parameters.amplitude * (m_parameters.angularFrequency * m_parameters.angularFrequency * cos(m_parameters.angularFrequency * timeSpent) + cos(timeSpent)),
                                        -0.5F * m_parameters.amplitude * (m_parameters.angularFrequency * m_parameters.angularFrequency * sin(m_parameters.angularFrequency * timeSpent) + sin(timeSpent)),
                                        0.0F };

        }
        else if (timeSpent < 80.F)
        {

            timeSpent -= 40.F;

            goal.pose.pose.position = { 0.F,
                                        0.F,
                                        0.5F * m_parameters.amplitude * (sin(m_parameters.angularFrequency * timeSpent + sin(timeSpent))) };

            goal.twist.twist.linear = { 0.F,
                                        0.F,
                                        0.5F * m_parameters.amplitude * (m_parameters.angularFrequency * cos(m_parameters.angularFrequency * timeSpent) + cos(timeSpent)) };

            goal.pose.pose.orientation = geometry_msgs::Quaternion{ 0.0F, 0.0F, 0.0F, 1.0F };

            goal.twist.twist.angular.x = 0.F;
            goal.twist.twist.angular.y = 0.F;
            goal.twist.twist.angular.z = -0.5F * m_parameters.amplitude * (m_parameters.angularFrequency * m_parameters.angularFrequency * sin(m_parameters.angularFrequency * timeSpent) + sin(timeSpent));

        }
        else
        {
            VectorQuat quatDesired;
            quatDesired = quatDesired.Zero();

            timeSpent -= 80.F;

            goal.pose.pose.position = { 0.0F, 0.0F, 0.0F };

            goal.twist.twist.linear = { 0.0F, 0.0F, 0.0F };

            yaw_desired = angles::normalize_angle(0.5F * 1.05F * (sin(m_parameters.angularFrequency * timeSpent) + sin(timeSpent)));

            angle2quatZYX(quatDesired, yaw_desired, 0.0F, 0.0F);

            goal.pose.pose.orientation = geometry_msgs::Quaternion{ quatDesired(0),
                                                                    quatDesired(1),
                                                                    quatDesired(2),
                                                                    quatDesired(3) };

            goal.twist.twist.angular = { -0.5F * 1.05F * (m_parameters.angularFrequency * m_parameters.angularFrequency * sin(m_parameters.angularFrequency * timeSpent) + sin(timeSpent)),
                                        0.0F,
                                        0.5F * 1.05F * (m_parameters.angularFrequency * cos(m_parameters.angularFrequency * timeSpent) + cos(timeSpent)) };
        }
    }

    void Planner::planStraightLine(nav_msgs::Odometry& goal, float32 timeSpent)
    {
        #if PRINT_LOG
        std::cout << "straightLine - Trajectory" << std::endl;
        #endif
        if (timeSpent <= poseDesired(4)) {

            float t2 = timeSpent * timeSpent;
            float t3 = t2 * timeSpent;
            float t4 = t3 * timeSpent;
            float t5 = t4 * timeSpent;

            goal.pose.pose.position = { cTx(0) * t3 + cTx(1) * t4 + cTx(2) * t5,
                                        cTy(0) * t3 + cTy(1) * t4 + cTy(2) * t5,
                                        cTz(0) * t3 + cTz(1) * t4 + cTz(2) * t5 };

            goal.twist.twist.linear = { 3.F * cTx(0) * t2 + 4 * cTx(1) * t3 + 5 * cTx(2) * t4,
                                        3.F * cTy(0) * t2 + 4 * cTy(1) * t3 + 5 * cTy(2) * t4,
                                        3.F * cTz(0) * t2 + 4 * cTz(1) * t3 + 5 * cTz(2) * t4 };

        }
        else {

            goal.pose.pose.position = { poseDesired(0),
                                        poseDesired(1),
                                        poseDesired(2) };

            goal.twist.twist.linear = { 0.0F, 0.0F, 0.0F };
        }

        goal.pose.pose.orientation = geometry_msgs::Quaternion{ 0.0F, 0.0F, 0.0F, 1.0F };

        goal.twist.twist.angular = { 0.0F, 0.0F, 0.0F };
    }

    void Planner::planWaypoint(nav_msgs::Odometry& goal)
    {
        #if PRINT_LOG
        std::cout << "wayPoint - Trajectory" << std::endl;
        #endif
        // Position Desired
        goal.pose.pose.position = { poseDesired(0),
                                    poseDesired(1),
                                    poseDesired(2) };

        // Linear Velocity Desired
        goal.twist.twist.linear = { 0.0F, 0.0F, 0.0F };

        // Quaternion Orientation Desired
        goal.pose.pose.orientation = geometry_msgs::Quaternion{ 0.0F, 0.0F, 0.0F, 1.0F };

        // Angular Velocity Desired
        goal.twist.twist.angular = { 0.0F, 0.0F, 0.0F };
    }

    void Planner::TrajPlanner(void)
    {
        nav_msgs::Odometry desiredGoal;

        if (getIsControlStarted())
        {
            if (m_isFirstTimePass)
            {
                cout << "Starting Clock Now..." << endl;
                m_startTime = ros::Time::now().toSec();
                m_isFirstTimePass = false;
            }

            float32 t = ros::Time::now().toSec() - m_startTime;

            #if PRINT_LOG
            std::cout << "Current Time: " << t << std::endl;
            #endif	    

            switch (m_parameters.trajectory)
            {
            case eightShape:
                planEightShape(desiredGoal, t);
                break;
            case circleXY:
                planCircle(desiredGoal, t);
                break;
            case ident:
                planIdentification(desiredGoal, t);
                break;
            case circleZXY:
                planCircle(desiredGoal, t);
                break;
            case straightLine:
                planStraightLine(desiredGoal, t);
                break;
            case wayPoint:
                planWaypoint(desiredGoal);
                break;
            case followTruck:
                //TODO: planFollowTruck()
                break;
            }
        }
        else
        {
            m_isFirstTimePass = true;
            desiredGoal.pose.pose.position.x = 0.0F;
            desiredGoal.pose.pose.position.y = 0.0F;
        }

        #if PRINT_LOG
        std::cout << "Current Time: " << t << std::endl;
        std::cout << "Trajectory: " << trajectory << std::endl;
        std::cout << "mGoal.pose.pose.position.x: " << mGoal.pose.pose.position.x << std::endl;
        #endif
        waypoint_publisher.publish(mGoal);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* 		Function: joyCallback
    *	  Created by: jrsbenevides
    *  Last Modified: jrsbenevides in Aug 18th 2024
    *
    *  	 Description: Unused buttons were used to special functions. These features are described below: For Xbox360 Joystick:
    *				  1. Button [14] = 'DIGITAL DOWN' = is responsible for resetting flag isOdomStarted. It allows the resetting
    *					 of local frame. The flag is raised again by a function "setPosition0()" after reset is done.
    *				  2. Button [6]  = 'SELECT' =  when kept pressed, it allows the controller node to run.
    * 		          ToDo: Map controller according to input (xbox360 or logitech)
    */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

    void Planner::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
        /*Enables "Automatic Control Mode" while pressing this button*/
        static const uint16_t BUTTON_AUTOMATIC_CONTROL_XBOX360 = 6U;
        m_isControlStarted = joy->buttons[BUTTON_AUTOMATIC_CONTROL_XBOX360];
    }
} //NAMESPACE DRONE


int main(int argc, char** argv)
{

    ros::init(argc, argv, "genTrajectory");

    try {

        DRONE::Planner wptNode;

        ros::Rate loop_rate(50);

        while (ros::ok())
        {
            wptNode.TrajPlanner();
            ros::spinOnce();
            loop_rate.sleep();
        }

        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_FATAL_STREAM("An error has occurred: " << e.what());
        exit(1);
    }

    return 0;
}

