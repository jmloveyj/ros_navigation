/*
 * Copyright (c) 2012, Stefano Rosa, Luca Carlone
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <custom_description/vfh_node.h>
#include <math.h>
#include <angles/angles.h>


VFH_node::VFH_node(ros::NodeHandle nh, ros::NodeHandle nh_private):
nh_(nh), nh_private_(nh_private),m_target_ok(false)
{
	ROS_INFO("Starting VFH");

    m_currentPos.x = 0.0;
    m_currentPos.y = 0.0;

    m_targetPos = m_currentPos;

    m_currentDir = 0;

	m_cell_size = 100;							// mm, cell dimension
	m_window_diameter = 60;						// number of cells
	m_sector_angle = 5;							// deg, sector angle

	if (!nh_private_.getParam ("m_safety_dist_0ms", m_safety_dist_0ms))
        m_safety_dist_0ms = 50; 				// mm, double, safe distance at 0 m/s

	if (!nh_private_.getParam ("m_safety_dist_1ms", m_safety_dist_1ms))
        m_safety_dist_1ms = 50; 				// mm, double, safe distance at 1 m/s

	if (!nh_private_.getParam ("m_max_speed", m_max_speed))
		m_max_speed= 200;						// mm/sec, int, max speed

	if (!nh_private_.getParam ("m_max_speed_narrow_opening", m_max_speed_narrow_opening))
		m_max_speed_narrow_opening= 200; 		// mm/sec, int, max speed in the narrow opening

	if (!nh_private_.getParam ("m_max_speed_wide_opening", m_max_speed_wide_opening))
		m_max_speed_wide_opening= 300; 			// mm/sec, int, max speed in the wide opening

	if (!nh_private_.getParam ("m_max_acceleration", m_max_acceleration))
		m_max_acceleration = 200;    			// mm/sec^2, int, max acceleration

	if (!nh_private_.getParam ("m_min_turnrate", m_min_turnrate))
		m_min_turnrate = 40;	 				// deg/sec, int, min turn rate <--- not used

	if (!nh_private_.getParam ("m_max_turnrate_0ms", m_max_turnrate_0ms))
		m_max_turnrate_0ms = 40;				// deg/sec, int, max turn rate at 0 m/s

	if (!nh_private_.getParam ("m_max_turnrate_1ms", m_max_turnrate_1ms))
		m_max_turnrate_1ms = 40;				// deg/sec, int, max turn rate at 1 m/s

	m_min_turn_radius_safety_factor = 1.0; 		// double ????

	if (!nh_private_.getParam ("m_free_space_cutoff_0ms", m_free_space_cutoff_0ms))
		m_free_space_cutoff_0ms = 2000000.0; 	//double, low threshold free space at 0 m/s

	if (!nh_private_.getParam ("m_obs_cutoff_0ms", m_obs_cutoff_0ms))
		m_obs_cutoff_0ms = 4000000.0;			//double, high threshold obstacle at 0 m/s

	if (!nh_private_.getParam ("m_free_space_cutoff_1ms", m_free_space_cutoff_1ms))
		m_free_space_cutoff_1ms = 2000000.0; 	//double, low threshold free space at 1 m/s

	if (!nh_private_.getParam ("m_obs_cutoff_1ms", m_obs_cutoff_1ms))
		m_obs_cutoff_1ms = 4000000.0;			//double, high threshold obstacle at 1 m/s

	if (!nh_private_.getParam ("m_weight_desired_dir", m_weight_desired_dir))
        m_weight_desired_dir = 5.0;				//double, weight desired direction

	if (!nh_private_.getParam ("m_weight_current_dir", m_weight_current_dir))
        m_weight_current_dir = 1.0;				//double, weight current direction

	if (!nh_private_.getParam ("m_robot_radius", m_robot_radius))
		m_robot_radius = 300.0;					// robot radius in mm

	m_vfh = new VFH_Algorithm(m_cell_size, m_window_diameter, m_sector_angle,
			m_safety_dist_0ms, m_safety_dist_1ms, m_max_speed,
			m_max_speed_narrow_opening, m_max_speed_wide_opening,
			m_max_acceleration, m_min_turnrate, m_max_turnrate_0ms,
			m_max_turnrate_1ms, m_min_turn_radius_safety_factor,
			m_free_space_cutoff_0ms, m_obs_cutoff_0ms, m_free_space_cutoff_1ms,
			m_obs_cutoff_1ms, m_weight_desired_dir, m_weight_current_dir);

	m_vfh->SetRobotRadius(m_robot_radius);
	m_vfh->Init();
	ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&VFH_node::goalCB, this, _1));


	// subscribe to topics
	scan_subscriber_ = nh_.subscribe(
			scan_topic_, 1, &VFH_node::scanCallback, this);
	odom_subscriber_ = nh_.subscribe(
			odom_topic_, 1, &VFH_node::odomCallback, this);
	// cmd_vel publisher
	vel_publisher_= nh_.advertise<geometry_msgs::Twist>("cmd_vel",5);

}

VFH_node::~VFH_node()
{
	// stop the robot
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x=0.0;
	cmd_vel.angular.z=0.0;
	vel_publisher_.publish(cmd_vel);
	delete m_vfh;
}

void VFH_node::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
	ROS_DEBUG("In ROS goal callback, wrapping the PoseStamped in the action message");
	m_target_ok = true;
	m_targetPos.x = goal->pose.position.x* 1000.0;
	m_targetPos.y = goal->pose.position.y* 1000.0;
}

void VFH_node::odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	ROS_DEBUG("odomCallback(): received odometry");
	m_robotVel = odom_msg->twist.twist.linear.x * 1000.0;
    m_currentPos.x = odom_msg->pose.pose.position.x*1000.0;
    m_currentPos.y = odom_msg->pose.pose.position.y*1000.0;
    m_currentDir =  tf::getYaw(odom_msg->pose.pose.orientation);
}


void VFH_node::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	ROS_DEBUG("scanCallback(): received scan, ranges %d",scan_msg->ranges.size());

	unsigned int n = scan_msg->ranges.size();
	for (unsigned i = 0; i < 361; i++)
		m_laser_ranges[i][0] = -1;

	int step=1;
	int startIndex=0;
	float laserSpan = scan_msg->angle_max - scan_msg->angle_min;

	if(laserSpan > M_PI || n>180) // in case we are using HOKUYO
	{
		startIndex = (- M_PI/2 - scan_msg->angle_min) /scan_msg->angle_increment;
		float rays_per_degree = (M_PI/180.0)/scan_msg->angle_increment;
		ROS_DEBUG("scanCallback(): startIndex %d, raysxdeg %f", startIndex, rays_per_degree);
		for (unsigned i = 0; i<180; i++)
		{
			step = int(rays_per_degree * i);
			// calculate position in laser frame
			if (startIndex+step > n-1) // probably this is not necessary :/
				step = step-1;

			double r = scan_msg->ranges[startIndex+step]*1000.0;

			if (r<10)
				r = scan_msg->range_max *1000.0;

			ROS_DEBUG("%d:%f\n",i,r);
			m_laser_ranges[i*2][0] = r;
			m_laser_ranges[i*2 + 1][0] = r;
		}
	}
	else
	{
		for (unsigned i = 0; i<180; i++) // in case we are using SICK
		{
			// calculate position in laser frame
			double r = scan_msg->ranges[i]*1000.0;
			m_laser_ranges[i*2][0] = r;
			m_laser_ranges[i*2 + 1][0] = r;
		}
	}

	// perform vfh+
	update();
}


void VFH_node::update()
{
    float desiredAngle,desiredDist,deltaX,deltaY;
    deltaX= m_targetPos.x - m_currentPos.x;
    deltaY= m_targetPos.y - m_currentPos.y;
    desiredDist = sqrt(deltaX*deltaX+deltaY*deltaY);
    desiredAngle = RAD2DEG(angles::normalize_angle_positive(atan2(deltaY,deltaX) - m_currentDir+M_PI/2));
    ROS_INFO_STREAM("current:"<<RAD2DEG(m_currentDir)<<" target: "<< RAD2DEG(angles::normalize_angle(atan2(deltaY,deltaX))) <<"angle: "<<desiredAngle);

    float currGoalDistanceTolerance=250;
    if(hypot(deltaX,deltaY)<currGoalDistanceTolerance)
    {
        m_target_ok = false;
    }
	if (!m_target_ok) return;
    m_vfh->Update_VFH(m_laser_ranges, (int) (m_robotVel), desiredAngle ,
			desiredDist, currGoalDistanceTolerance, chosen_speed,
			chosen_turnrate);

	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x=(float)(chosen_speed)/1000.0;
	cmd_vel.angular.z= DEG2RAD(chosen_turnrate);
	vel_publisher_.publish(cmd_vel);

    ROS_INFO_STREAM("target(:" <<m_targetPos.x<<","<<m_targetPos.y<<"),current(:" <<m_currentPos.x<<","<<m_currentPos.y<<"),delta: "<<deltaX<<","<<deltaY);
	ROS_INFO("chosen_speed %d, chosen_turnrate %d", chosen_speed,
			chosen_turnrate);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "VFH");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	VFH_node vfh_node(nh,nh_private);
	ros::spin();

	return 0;
}
