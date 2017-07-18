/*
 *  Orca-Components: Components for robotics.
 *
 *  Copyright (C) 2004
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include "custom_description/vfh_algorithm.h"

#include <stdio.h>
#include <assert.h>
#include <math.h>

/**
 * VFH_Algorithm constructor
 * @param cell_size local map cell size
 * @param window_diameter windows diameter
 * @param sector_angle sector angle
 * @param safety_dist_0ms safety obstacle distance at 0 m/s
 * @param safety_dist_1ms safety obstacle distance at 1 m/s
 * @param max_speed max allowed speed
 * @param max_speed_narrow_opening max allowed speed in narrow opening
 * @param max_speed_wide_opening max allowed speed in wide opening
 * @param max_acceleration max allowed acceleration
 * @param min_turnrate min turn rate
 * @param max_turnrate_0ms max turn rate at 0 m/s
 * @param max_turnrate_1ms max turn rate at 1 m/s
 * @param min_turn_radius_safety_factor safety factor
 * @param free_space_cutoff_0ms free space histogram threshold at 0 m/s
 * @param obs_cutoff_0ms obstacle histogram threshold at 0 m/s
 * @param free_space_cutoff_1ms free space histogram threshold at 1 m/s
 * @param obs_cutoff_1ms obstacle histogram threshold at 1 m/s
 * @param weight_desired_dir weight of the desired direction
 * @param weight_current_dir weight of the current direction
 */
VFH_Algorithm::VFH_Algorithm( double cell_size,
                              int window_diameter,
                              int sector_angle,
                              double safety_dist_0ms,
                              double safety_dist_1ms,
                              int max_speed,
                              int max_speed_narrow_opening,
                              int max_speed_wide_opening,
                              int max_acceleration,
                              int min_turnrate,
                              int max_turnrate_0ms,
                              int max_turnrate_1ms,
                              double min_turn_radius_safety_factor,
                              double free_space_cutoff_0ms,
                              double obs_cutoff_0ms,
                              double free_space_cutoff_1ms,
                              double obs_cutoff_1ms,
                              double weight_desired_dir,
                              double weight_current_dir )
    : CELL_WIDTH(cell_size),
      WINDOW_DIAMETER(window_diameter),
      SECTOR_ANGLE(sector_angle),
      SAFETY_DIST_0MS(safety_dist_0ms),
      SAFETY_DIST_1MS(safety_dist_1ms),
      Current_Max_Speed(max_speed),
      MAX_SPEED(max_speed),
      MAX_SPEED_NARROW_OPENING(max_speed_narrow_opening),
      MAX_SPEED_WIDE_OPENING(max_speed_wide_opening),
      MAX_ACCELERATION(max_acceleration),
      MIN_TURNRATE(min_turnrate),
      MAX_TURNRATE_0MS(max_turnrate_0ms),
      MAX_TURNRATE_1MS(max_turnrate_1ms),
      MIN_TURN_RADIUS_SAFETY_FACTOR(min_turn_radius_safety_factor),
      Binary_Hist_Low_0ms(free_space_cutoff_0ms),
      Binary_Hist_High_0ms(obs_cutoff_0ms),
      Binary_Hist_Low_1ms(free_space_cutoff_1ms),
      Binary_Hist_High_1ms(obs_cutoff_1ms),
      U1(weight_desired_dir),
      U2(weight_current_dir),
      Desired_Angle(90),
      Picked_Angle(90),
      Last_Picked_Angle(Picked_Angle),
      last_chosen_speed(0)
{
    this->Last_Binary_Hist = NULL;
    this->Hist = NULL;
    this->OriginHist=NULL;
    if ( SAFETY_DIST_0MS == SAFETY_DIST_1MS )
    {
        // For the simple case of a fixed safety_dist, keep things simple.
        NUM_CELL_SECTOR_TABLES = 1;
    }
    else
    {
        // AB: Made this number up...
        NUM_CELL_SECTOR_TABLES = 20;
    }
}

/**
 * Class destructor
 */
VFH_Algorithm::~VFH_Algorithm()
{
    if(this->Hist)
        delete[] Hist;
    if(this->OriginHist)
        delete[] OriginHist;
    if(this->Last_Binary_Hist)
        delete[] Last_Binary_Hist;
}

/**
 * Get the max turn rate at the given speed
 * @param speed current speed
 * @return max turn rate
 */
int VFH_Algorithm::GetMaxTurnrate( int speed )
{
    int val = ( MAX_TURNRATE_0MS - (int)(speed*( MAX_TURNRATE_0MS-MAX_TURNRATE_1MS )/1000.0) );

    if ( val < 0 )
        val = 0;

    return val;
}

/**
 * Set the current max speed
 * @param max_speed current max speed
 */
void VFH_Algorithm::SetCurrentMaxSpeed( int max_speed )
{
    this->Current_Max_Speed = MIN( max_speed, this->MAX_SPEED );
    this->Min_Turning_Radius.resize( Current_Max_Speed+1 );

    // small chunks of forward movements and turns-in-place used to
    // estimate turning radius, coz I'm too lazy to screw around with limits -> 0.
    double dx, dtheta;

    //
    // Calculate the turning radius, indexed by speed.
    // Probably don't need it to be precise (changing in 1mm increments).
    //
    // WARNING: This assumes that the max_turnrate that has been set for VFH is
    //          accurate.
    //
    for(int x=0;x<=Current_Max_Speed;x++)
    {
        dx = (double) x / 1e6; // dx in m/millisec
        dtheta = ((M_PI/180)*(double)(GetMaxTurnrate(x))) / 1000.0; // dTheta in radians/millisec
        Min_Turning_Radius[x] = (int) ( ((dx / tan( dtheta ))*1000.0) * MIN_TURN_RADIUS_SAFETY_FACTOR ); // in mm
    }
}


// Doesn't need optimization: only gets called once per update.
/**
 * Get the speed index ( for the current local map )
 * @param speed given speed
 * @return the index speed
 */
int
VFH_Algorithm::Get_Speed_Index( int speed )
{
    int val = (int) floor(((float)speed/(float)Current_Max_Speed)*NUM_CELL_SECTOR_TABLES);

    if ( val >= NUM_CELL_SECTOR_TABLES )
        val = NUM_CELL_SECTOR_TABLES-1;

//     printf("Speed_Index at %dmm/s: %d\n",speed,val);

    return val;
}

// Doesn't need optimization: only gets called on init plus once per update.
/**
 * Get the safety distance at the given speed
 * @param speed given speed
 * @return the safety distance
 */
int
VFH_Algorithm::Get_Safety_Dist( int speed )
{
    int val = (int) ( SAFETY_DIST_0MS + (int)(speed*( SAFETY_DIST_1MS-SAFETY_DIST_0MS )/1000.0) );

    if ( val < 0 )
        val = 0;

//    printf("Safety_Dist at %dmm/s: %d\n",speed,val);

    return val;
}

// AB: Could optimize this with a look-up table, but it shouldn't make much
//     difference: only gets called once per sector per update.
/**
 * Get the current low binary histogram threshold
 * @param speed given speed
 * @return the threshold
 */
float
VFH_Algorithm::Get_Binary_Hist_Low( int speed )
{
    return ( Binary_Hist_Low_0ms - (speed*( Binary_Hist_Low_0ms-Binary_Hist_Low_1ms )/1000.0) );
}

// AB: Could optimize this with a look-up table, but it shouldn't make much
//     difference: only gets called once per sector per update.
/**
 * Get the current high binary histogram threshold
 * @param speed given speed
 * @return the threshold
 */
float
VFH_Algorithm::Get_Binary_Hist_High( int speed )
{
    return ( Binary_Hist_High_0ms - (speed*( Binary_Hist_High_0ms-Binary_Hist_High_1ms )/1000.0) );
}

/**
 * Start up the VFH+ algorithm
 * @return 1
 */
int VFH_Algorithm::Init()
{
  int x, y, i;
  float plus_dir=0, neg_dir=0, plus_sector=0, neg_sector=0;
  bool plus_dir_bw, neg_dir_bw, dir_around_sector;
  float neg_sector_to_neg_dir=0, neg_sector_to_plus_dir=0;
  float plus_sector_to_neg_dir=0, plus_sector_to_plus_dir=0;
  int cell_sector_tablenum, max_speed_this_table;
  float r;

  CENTER_X = (int)floor(WINDOW_DIAMETER / 2.0);
  CENTER_Y = CENTER_X;
  HIST_SIZE = (int)rint(360.0 / SECTOR_ANGLE);

  // it works now; let's leave the verbose debug statement out
  /*
  printf("CELL_WIDTH: %1.1f\tWINDOW_DIAMETER: %d\tSECTOR_ANGLE: %d\tROBOT_RADIUS: %1.1f\tSAFETY_DIST: %1.1f\tMAX_SPEED: %d\tMAX_TURNRATE: %d\tFree Space Cutoff: %1.1f\tObs Cutoff: %1.1f\tWeight Desired Dir: %1.1f\tWeight Current_Dir:%1.1f\n", CELL_WIDTH, WINDOW_DIAMETER, SECTOR_ANGLE, ROBOT_RADIUS, SAFETY_DIST, MAX_SPEED, MAX_TURNRATE, Binary_Hist_Low, Binary_Hist_High, U1, U2);
  */

  VFH_Allocate();

  for(x=0;x<HIST_SIZE;x++) {
    Hist[x] = 0;
    OriginHist[x]=0;
    Last_Binary_Hist[x] = 1;
  }

  // For the following calcs:
  //   - (x,y) = (0,0)   is to the front-left of the robot
  //   - (x,y) = (max,0) is to the front-right of the robot
  //
  for(x=0;x<WINDOW_DIAMETER;x++) {
    for(y=0;y<WINDOW_DIAMETER;y++) {
      Cell_Mag[x][y] = 0;
      Cell_Dist[x][y] = sqrt(pow((CENTER_X - x), 2) + pow((CENTER_Y - y), 2)) * CELL_WIDTH;

      Cell_Base_Mag[x][y] = 15* pow((3000.0 - Cell_Dist[x][y]), 4) / 100000000.0;

      // Set up Cell_Direction with the angle in degrees to each cell
      if (x < CENTER_X) {
        if (y < CENTER_Y) {
          Cell_Direction[x][y] = atan((float)(CENTER_Y - y) / (float)(CENTER_X - x));
          Cell_Direction[x][y] *= (360.0 / 6.28);
          Cell_Direction[x][y] = 180.0 - Cell_Direction[x][y];
        } else if (y == CENTER_Y) {
          Cell_Direction[x][y] = 180.0;
        } else if (y > CENTER_Y) {
          Cell_Direction[x][y] = atan((float)(y - CENTER_Y) / (float)(CENTER_X - x));
          Cell_Direction[x][y] *= (360.0 / 6.28);
          Cell_Direction[x][y] = 180.0 + Cell_Direction[x][y];
        }
      } else if (x == CENTER_X) {
        if (y < CENTER_Y) {
          Cell_Direction[x][y] = 90.0;
        } else if (y == CENTER_Y) {
          Cell_Direction[x][y] = -1.0;
        } else if (y > CENTER_Y) {
          Cell_Direction[x][y] = 270.0;
        }
      } else if (x > CENTER_X) {
        if (y < CENTER_Y) {
          Cell_Direction[x][y] = atan((float)(CENTER_Y - y) / (float)(x - CENTER_X));
          Cell_Direction[x][y] *= (360.0 / 6.28);
        } else if (y == CENTER_Y) {
          Cell_Direction[x][y] = 0.0;
        } else if (y > CENTER_Y) {
          Cell_Direction[x][y] = atan((float)(y - CENTER_Y) / (float)(x - CENTER_X));
          Cell_Direction[x][y] *= (360.0 / 6.28);
          Cell_Direction[x][y] = 360.0 - Cell_Direction[x][y];
        }
      }

      // For the case where we have a speed-dependent safety_dist, calculate all tables
      for ( cell_sector_tablenum = 0;
            cell_sector_tablenum < NUM_CELL_SECTOR_TABLES;
            cell_sector_tablenum++ )
      {
        max_speed_this_table = (int) (((float)(cell_sector_tablenum+1)/(float)NUM_CELL_SECTOR_TABLES) *
                                      (float) MAX_SPEED);

        // printf("cell_sector_tablenum: %d, max_speed: %d, safety_dist: %d\n",
        // cell_sector_tablenum,max_speed_this_table,Get_Safety_Dist(max_speed_this_table));

        // Set Cell_Enlarge to the _angle_ by which a an obstacle must be
        // enlarged for this cell, at this speed
        if (Cell_Dist[x][y] > 0)
        {
          r = ROBOT_RADIUS + Get_Safety_Dist(max_speed_this_table);
          // Cell_Enlarge[x][y] = (float)atan( r / Cell_Dist[x][y] ) * (180/M_PI);
          Cell_Enlarge[x][y] = (float)asin( r / Cell_Dist[x][y] ) * (180/M_PI);
        }
        else
        {
          Cell_Enlarge[x][y] = 0;
        }

        Cell_Sector[cell_sector_tablenum][x][y].clear();
        plus_dir = Cell_Direction[x][y] + Cell_Enlarge[x][y];
        neg_dir  = Cell_Direction[x][y] - Cell_Enlarge[x][y];

        for(i=0;i<(360 / SECTOR_ANGLE);i++)
        {
            // Set plus_sector and neg_sector to the angles to the two adjacent sectors
            plus_sector = (i + 1) * (float)SECTOR_ANGLE;
            neg_sector = i * (float)SECTOR_ANGLE;

            if ((neg_sector - neg_dir) > 180) {
                neg_sector_to_neg_dir = neg_dir - (neg_sector - 360);
            } else {
                if ((neg_dir - neg_sector) > 180) {
                    neg_sector_to_neg_dir = neg_sector - (neg_dir + 360);
                } else {
                    neg_sector_to_neg_dir = neg_dir - neg_sector;
                }
            }

            if ((plus_sector - neg_dir) > 180) {
                plus_sector_to_neg_dir = neg_dir - (plus_sector - 360);
            } else {
                if ((neg_dir - plus_sector) > 180) {
                    plus_sector_to_neg_dir = plus_sector - (neg_dir + 360);
                } else {
                    plus_sector_to_neg_dir = neg_dir - plus_sector;
                }
            }

            if ((plus_sector - plus_dir) > 180) {
                plus_sector_to_plus_dir = plus_dir - (plus_sector - 360);
            } else {
                if ((plus_dir - plus_sector) > 180) {
                    plus_sector_to_plus_dir = plus_sector - (plus_dir + 360);
                } else {
                    plus_sector_to_plus_dir = plus_dir - plus_sector;
                }
            }

            if ((neg_sector - plus_dir) > 180) {
                neg_sector_to_plus_dir = plus_dir - (neg_sector - 360);
            } else {
                if ((plus_dir - neg_sector) > 180) {
                    neg_sector_to_plus_dir = neg_sector - (plus_dir + 360);
                } else {
                    neg_sector_to_plus_dir = plus_dir - neg_sector;
                }
            }

            plus_dir_bw = 0;
            neg_dir_bw = 0;
            dir_around_sector = 0;

            if ((neg_sector_to_neg_dir >= 0) && (plus_sector_to_neg_dir <= 0)) {
                neg_dir_bw = 1;
            }

            if ((neg_sector_to_plus_dir >= 0) && (plus_sector_to_plus_dir <= 0)) {
                plus_dir_bw = 1;
            }

            if ((neg_sector_to_neg_dir <= 0) && (neg_sector_to_plus_dir >= 0)) {
                dir_around_sector = 1;
            }

            if ((plus_sector_to_neg_dir <= 0) && (plus_sector_to_plus_dir >= 0)) {
                plus_dir_bw = 1;
            }

            if ((plus_dir_bw) || (neg_dir_bw) || (dir_around_sector)) {
                Cell_Sector[cell_sector_tablenum][x][y].push_back(i);
            }
        }
      }
    }
  }

//  assert( GlobalTime->GetTime( &last_update_time ) == 0 );

  gettimeofday(&last_update_time,0);

  return(1);
}

/**
 * Allocate the VFH+ memory
 */
int VFH_Algorithm::VFH_Allocate()
{
  std::vector<float> temp_vec;
  std::vector<int> temp_vec3;
  std::vector<std::vector<int> > temp_vec2;
  std::vector<std::vector<std::vector<int> > > temp_vec4;
  int x;

  Cell_Direction.clear();
  Cell_Base_Mag.clear();
  Cell_Mag.clear();
  Cell_Dist.clear();
  Cell_Enlarge.clear();
  Cell_Sector.clear();

  temp_vec.clear();
  for(x=0;x<WINDOW_DIAMETER;x++) {
    temp_vec.push_back(0);
  }

  temp_vec2.clear();
  temp_vec3.clear();
  for(x=0;x<WINDOW_DIAMETER;x++) {
    temp_vec2.push_back(temp_vec3);
  }

  for(x=0;x<WINDOW_DIAMETER;x++) {
    Cell_Direction.push_back(temp_vec);
    Cell_Base_Mag.push_back(temp_vec);
    Cell_Mag.push_back(temp_vec);
    Cell_Dist.push_back(temp_vec);
    Cell_Enlarge.push_back(temp_vec);
    temp_vec4.push_back(temp_vec2);
  }

  for(x=0;x<NUM_CELL_SECTOR_TABLES;x++)
  {
    Cell_Sector.push_back(temp_vec4);
  }

  Hist = new float[HIST_SIZE];
  OriginHist=new float[HIST_SIZE];
  Last_Binary_Hist = new float[HIST_SIZE];
  this->SetCurrentMaxSpeed( MAX_SPEED );

  return(1);
}

/**
 * Update the VFH+ state using the laser readings and the robot speed
 * @param laser_ranges the laser (or sonar) readings
 * @param current_speed the current robot speed
 * @param goal_direction the desired direction
 * @param goal_distance the desired distance
 * @param goal_distance_tolerance the distance tolerance from the goal
 * @param chosen_speed the chosen speed to drive the robot
 * @param chosen_turnrate the chosen turn rathe to drive the robot
 * @return 1
 */
int VFH_Algorithm::Update_VFH( double laser_ranges[361][2],
                               int current_speed,
                               float goal_direction,
                               float goal_distance,
                               float goal_distance_tolerance,
                               int &chosen_speed,
                               int &chosen_turnrate )
{
  int print = 0;

  this->Desired_Angle = goal_direction;
  this->Dist_To_Goal  = goal_distance;
  this->Goal_Distance_Tolerance = goal_distance_tolerance;

  //
  // Set current_pos_speed to the maximum of
  // the set point (last_chosen_speed) and the current actual speed.
  // This ensures conservative behaviour if the set point somehow ramps up beyond
  // the actual speed.
  // Ensure that this speed is positive.
  //
  int current_pos_speed;
  if ( current_speed < 0 )
  {
      current_pos_speed = 0;
  }
  else
  {
      current_pos_speed = current_speed;
  }


  if ( current_pos_speed < last_chosen_speed )
  {
      current_pos_speed = last_chosen_speed;
  }
//  printf("Update_VFH: current_pos_speed = %d\n",current_pos_speed);


  // Work out how much time has elapsed since the last update,
  // so we know how much to increase speed by, given MAX_ACCELERATION.
  timeval now;
  timeval diff;
  double  diffSeconds;
//  assert( GlobalTime->GetTime( &now ) == 0 );
  assert( gettimeofday(&now, 0) == 0 );

  TIMESUB( &now, &last_update_time, &diff );
  diffSeconds = diff.tv_sec + ( (double)diff.tv_usec / 1000000 );

  last_update_time.tv_sec = now.tv_sec;
  last_update_time.tv_usec = now.tv_usec;
//  printf("Update_VFH: Build_Primary_Polar_Histogram\n");
  if ( Build_Primary_Polar_Histogram(laser_ranges,current_pos_speed) == 0)
  {
      // Something's inside our safety distance: brake hard and
      // turn on the spot
      Picked_Angle = Last_Picked_Angle;
      Max_Speed_For_Picked_Angle = 0;
      Last_Picked_Angle = Picked_Angle;
  }
  else
  {
	  if (print) {
	    printf("Primary Histogram\n");
	    Print_Hist();
	  }

	  Build_Binary_Polar_Histogram(current_pos_speed);
	  if (print) {
	    printf("Binary Histogram\n");
	    Print_Hist();
	  }

	  Build_Masked_Polar_Histogram(current_pos_speed);
	  if (print) {
	    printf("Masked Histogram\n");
	    Print_Hist();
	  }

	  // Sets Picked_Angle, Last_Picked_Angle, and Max_Speed_For_Picked_Angle.
	  Select_Direction();

  }

//  printf("Picked Angle: %f\n", Picked_Angle);

  //
  // OK, so now we've chosen a direction.  Time to choose a speed.
  //

  // How much can we change our speed by?
  int speed_incr;
  if ( (diffSeconds > 0.3) || (diffSeconds < 0) )
  {
      // Either this is the first time we've been updated, or something's a bit screwy and
      // update hasn't been called for a while.  Don't want a sudden burst of acceleration,
      // so better to just pick a small value this time, calculate properly next time.
      speed_incr = 10;
  }
  else
  {
      speed_incr = (int) (MAX_ACCELERATION * diffSeconds);
  }

  if ( Cant_Turn_To_Goal() )
  {
//      printf("The goal's too close -- we can't turn tightly enough to get to it, so slow down...");
      speed_incr = -speed_incr;
  }

  // Accelerate (if we're not already at Max_Speed_For_Picked_Angle).
  chosen_speed = MIN( last_chosen_speed + speed_incr, Max_Speed_For_Picked_Angle );

//  printf("Max Speed for picked angle: %d\n",Max_Speed_For_Picked_Angle);

  // Set the chosen_turnrate, and possibly modify the chosen_speed
  Set_Motion( chosen_speed, chosen_turnrate, current_pos_speed );

  last_chosen_speed = chosen_speed;

  if (print)
    printf("CHOSEN: SPEED: %d\t TURNRATE: %d\n", chosen_speed, chosen_turnrate);

  return(1);
}


/**
 * The robot going too fast, such does it overshoot before it can turn to the goal?
 * @return true if the robot cannot turn to the goal
 */
bool VFH_Algorithm::Cant_Turn_To_Goal()
{
    // Calculate this by seeing if the goal is inside the blocked circles
    // (circles we can't enter because we're going too fast).  Radii set
    // by Build_Masked_Polar_Histogram.

    // Coords of goal in local coord system:
    float goal_x = this->Dist_To_Goal * cos( DTOR(this->Desired_Angle) );
    float goal_y = this->Dist_To_Goal * sin( DTOR(this->Desired_Angle) );

// AlexB: Is this useful?
//     if ( goal_y < 0 )
//     {
//         printf("Goal behind\n");
//         return true;
//     }

    // This is the distance between the centre of the goal and
    // the centre of the blocked circle
    float dist_between_centres;

//     printf("Cant_Turn_To_Goal: Dist_To_Goal = %f\n",Dist_To_Goal);
//     printf("Cant_Turn_To_Goal: Angle_To_Goal = %f\n",Desired_Angle);
//     printf("Cant_Turn_To_Goal: Blocked_Circle_Radius = %f\n",Blocked_Circle_Radius);

    // right circle
    dist_between_centres = hypot( goal_x - this->Blocked_Circle_Radius, goal_y );
    if ( dist_between_centres+this->Goal_Distance_Tolerance < this->Blocked_Circle_Radius )
    {
//        printf("Goal close & right\n");
        return true;
    }

    // left circle
    dist_between_centres = hypot( -goal_x - this->Blocked_Circle_Radius, goal_y );
    if ( dist_between_centres+this->Goal_Distance_Tolerance < this->Blocked_Circle_Radius )
    {
//        printf("Goal close & left.\n");
        return true;
    }

    return false;
}

/**
 * Difference between two integer angle
 * @param a1 first angle
 * @param a2 second angle
 * @return the difference
 */
float VFH_Algorithm::Delta_Angle(int a1, int a2)
{
  return(Delta_Angle((float)a1, (float)a2));
}

/**
 * Difference between two float angle
 * @param a1 first angle
 * @param a2 second angle
 * @return the difference
 */
float VFH_Algorithm::Delta_Angle(float a1, float a2)
{
  float diff;

  diff = a2 - a1;

  if (diff > 180) {
    diff -= 360;
  } else if (diff < -180) {
    diff += 360;
  }

  return(diff);
}

/**
 * Calculate the bisector between two angle
 * @param angle1 first angle
 * @param angle2 second angle
 * @return the bisector angle
 */
int VFH_Algorithm::Bisect_Angle(int angle1, int angle2)
{
  float a;
  int angle;

  a = Delta_Angle((float)angle1, (float)angle2);

  angle = (int)rint(angle1 + (a / 2.0));
  if (angle < 0) {
    angle += 360;
  } else if (angle >= 360) {
    angle -= 360;
  }

  return(angle);
}

/**
 * Select the candidate angle to decide the direction using the given weights
 * @return 1
 */
int VFH_Algorithm::Select_Candidate_Angle()
{
  unsigned int i;
  float weight, min_weight;

  if (Candidate_Angle.size() == 0)
  {
      // We're hemmed in by obstacles -- nowhere to go,
      // so brake hard and turn on the spot.
      Picked_Angle = Last_Picked_Angle;
      Max_Speed_For_Picked_Angle = 0;
      Last_Picked_Angle = Picked_Angle;
      return(1);
  }

  Picked_Angle = 90;
  min_weight = 10000000;

  for(i=0;i<Candidate_Angle.size();i++)
  {
      //printf("CANDIDATE: %f\n", Candidate_Angle[i]);
      weight = U1 * fabs(Delta_Angle(Desired_Angle, Candidate_Angle[i])) +
          U2 * fabs(Delta_Angle(Last_Picked_Angle, Candidate_Angle[i]));
      if (weight < min_weight)
      {
          min_weight = weight;
          Picked_Angle = Candidate_Angle[i];
          Max_Speed_For_Picked_Angle = Candidate_Speed[i];
      }
  }

  Last_Picked_Angle = Picked_Angle;

  return(1);
}

/**
 * Select the used direction
 * @return 1
 */
int VFH_Algorithm::Select_Direction()
{
  int start, i, left;
  float angle, new_angle;
  std::vector<std::pair<int,int> > border;
  std::pair<int,int> new_border;

  Candidate_Angle.clear();
  Candidate_Speed.clear();

  //
  // set start to sector of first obstacle
  //
  start = -1;

  // only look at the forward 180deg for first obstacle.
  for(i=0;i<HIST_SIZE/2;i++)
  {
      if (Hist[i] == 1)
      {
          start = i;
          break;
      }
  }

  if (start == -1)
  {
      Picked_Angle = Desired_Angle;
      Last_Picked_Angle = Picked_Angle;
      Max_Speed_For_Picked_Angle = Current_Max_Speed;

//      printf("No obstacles detected in front of us: full speed towards goal: %f, %f, %d\n",
//    		  Picked_Angle, Last_Picked_Angle, Max_Speed_For_Picked_Angle);

      return(1);
  }

  //
  // Find the left and right borders of each opening
  //

  border.clear();

  //printf("Start: %d\n", start);
  left = 1;
  for(i=start;i<=(start+HIST_SIZE);i++) {
    if ((Hist[i % HIST_SIZE] == 0) && (left)) {
      new_border.first = (i % HIST_SIZE) * SECTOR_ANGLE;
      left = 0;
    }

    if ((Hist[i % HIST_SIZE] == 1) && (!left)) {
      new_border.second = ((i % HIST_SIZE) - 1) * SECTOR_ANGLE;
      if (new_border.second < 0) {
        new_border.second += 360;
      }
      border.push_back(new_border);
      left = 1;
    }
  }

  //
  // Consider each opening
  //
  for(i=0;i<(int)border.size();i++)
  {
//    printf("BORDER: %d %d\n", border[i].first, border[i].second);
    angle = Delta_Angle(border[i].first, border[i].second);

    if (fabs(angle) < 10)
    {
        // ignore very narrow openings
        continue;
    }

    if (fabs(angle) < 80)
    {
        // narrow opening: aim for the centre

        new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;

        Candidate_Angle.push_back(new_angle);
        Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_NARROW_OPENING));
    }
    else
    {
        // wide opening: consider the centre, and 40deg from each border

        new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;

        Candidate_Angle.push_back(new_angle);
        Candidate_Speed.push_back(Current_Max_Speed);

        new_angle = (float)((border[i].first + 40) % 360);
        Candidate_Angle.push_back(new_angle);
        Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));

        new_angle = (float)(border[i].second - 40);
        if (new_angle < 0)
            new_angle += 360;
        Candidate_Angle.push_back(new_angle);
        Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));

        // See if candidate dir is in this opening
        if ((Delta_Angle(Desired_Angle, Candidate_Angle[Candidate_Angle.size()-2]) < 0) &&
            (Delta_Angle(Desired_Angle, Candidate_Angle[Candidate_Angle.size()-1]) > 0)) {
            Candidate_Angle.push_back(Desired_Angle);
            Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));
        }
    }
  }

  Select_Candidate_Angle();

  return(1);
}

/**
 * Print the cells directions
 */
void VFH_Algorithm::Print_Cells_Dir()
{
  int x, y;

  printf("\nCell Directions:\n");
  printf("****************\n");
  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      printf("%1.1f\t", Cell_Direction[x][y]);
    }
    printf("\n");
  }
}

/**
 * Print the cells magnitude
 */
void VFH_Algorithm::Print_Cells_Mag()
{
  int x, y;

  printf("\nCell Magnitudes:\n");
  printf("****************\n");
  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      printf("%1.1f\t", Cell_Mag[x][y]);
    }
    printf("\n");
  }
}

/**
 * Print the cells distances
 */
void VFH_Algorithm::Print_Cells_Dist()
{
  int x, y;

  printf("\nCell Distances:\n");
  printf("****************\n");
  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      printf("%1.1f\t", Cell_Dist[x][y]);
    }
    printf("\n");
  }
}

/**
 * Print the cells sectors
 */
void VFH_Algorithm::Print_Cells_Sector()
{
  int x, y;
  unsigned int i;

  printf("\nCell Sectors for table 0:\n");
  printf("***************************\n");

  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      for(i=0;i<Cell_Sector[0][x][y].size();i++) {
        if (i < (Cell_Sector[0][x][y].size() -1 )) {
          printf("%d,", Cell_Sector[0][x][y][i]);
        } else {
          printf("%d\t\t", Cell_Sector[0][x][y][i]);
        }
      }
    }
    printf("\n");
  }
}

/**
 * Print the cells enlargement angles
 */
void VFH_Algorithm::Print_Cells_Enlargement_Angle()
{
  int x, y;

  printf("\nEnlargement Angles:\n");
  printf("****************\n");
  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      printf("%1.1f\t", Cell_Enlarge[x][y]);
    }
    printf("\n");
  }
}

/**
 * Print the histogram
 */
void VFH_Algorithm::Print_Hist()
{
  int x;
  printf("Histogram:\n");
  printf("****************\n");

  for(x=0;x<=(HIST_SIZE/2);x++) {
    printf("%d,%1.1f\n", (x * SECTOR_ANGLE), Hist[x]);
  }
  printf("\n\n");
}

/**
 * Calcualte the cells magnitude
 * @param laser_ranges laser (or sonar) readings
 * @param speed robot speed
 * @return 1
 */
int VFH_Algorithm::Calculate_Cells_Mag( double laser_ranges[361][2], int speed )
{
  int x, y;

  float safeSpeed = (float) Get_Safety_Dist(speed);
  float r = ROBOT_RADIUS +  safeSpeed;

//printf("Laser Ranges\n");
//printf("************\n");
//for(x=0;x<=360;x++) {
//printf("%d: %f %f\n", x, laser_ranges[x][0], r);
//}


  // AB: This is a bit dodgy...  Makes it possible to miss really skinny obstacles, since if the
  //     resolution of the cells is finer than the resolution of laser_ranges, some ranges might be missed.
  //     Rather than looping over the cells, should perhaps loop over the laser_ranges.

  // Only deal with the cells in front of the robot, since we can't sense behind.
  for(x=0;x<WINDOW_DIAMETER;x++)
  {
      for(y=0;y<(int)ceil(WINDOW_DIAMETER/2.0);y++)
      {
//    	  printf("Cell %d,%d: Cell_Dist is %f, range is %f i: %d (minimum is %f)\n",
//    	                      x,
//    	                      y,
//    	                      Cell_Dist[x][y] + CELL_WIDTH / 2.0,
//    	                      laser_ranges[(int)rint(Cell_Direction[x][y] * 2.0)][0],
//    	                      (int)rint(Cell_Direction[x][y] * 2.0),
//    	                      r);
    	  // controllo se il laser passa attraverso la cella
          if ((Cell_Dist[x][y] + CELL_WIDTH / 2.0) >
              laser_ranges[(int)rint(Cell_Direction[x][y] * 2.0)][0])
          {
              if ( Cell_Dist[x][y] < r && !(x==CENTER_X && y==CENTER_Y) )
              {
//                   printf("Cell %d,%d: Cell_Dist is %f, range is %f (minimum is %f): too close...\n",
//                          x,
//                          y,
//                          Cell_Dist[x][y] + CELL_WIDTH / 2.0,
//                          laser_ranges[(int)rint(Cell_Direction[x][y] * 2.0)][0],
//                          r);

//                   printf("ROBOT_RADIUS %f, Get_Safety_Dist(speed) %f\n", ROBOT_RADIUS, safeSpeed);

                  // Damn, something got inside our safety_distance...
                  // Short-circuit this process.
                  return(0);
              }
              else
              {
            	  // cella piena quindi:
            	  // assegno alla cella il peso che dipende dalla distanza
                  Cell_Mag[x][y] = Cell_Base_Mag[x][y];
              }
          } else {
        	  // è vuota perchè il laser ci passa oltre!!!!
              Cell_Mag[x][y] = 0.0;
          }
      }
  }

  return(1);
}

/**
 * Build the primary polar histogram
 * @param laser_ranges laser (or sonar) readings
 * @param speed robot speed
 * @return 1
 */
int VFH_Algorithm::Build_Primary_Polar_Histogram( double laser_ranges[361][2], int speed )
{
  int x, y;
  unsigned int i;
  // index into the vector of Cell_Sector tables
  int speed_index = Get_Speed_Index( speed );

//  printf("Build_Primary_Polar_Histogram: speed_index %d %d\n", speed_index, HIST_SIZE);

  for(x=0;x<HIST_SIZE;x++) {
    OriginHist[x] = 0;
  }

  if ( Calculate_Cells_Mag( laser_ranges, speed ) == 0 )
  {
      // set Hist to all blocked
      for(x=0;x<HIST_SIZE;x++) {
          OriginHist[x] = 1;
      }
      return 0;
  }

//  Print_Cells_Dist();
//  Print_Cells_Dir();
//  Print_Cells_Mag();
//  Print_Cells_Sector();
//  Print_Cells_Enlargement_Angle();

  // Only have to go through the cells in front.
  for(y=0;y<=(int)ceil(WINDOW_DIAMETER/2.0);y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      for(i=0;i<Cell_Sector[speed_index][x][y].size();i++) {
        OriginHist[Cell_Sector[speed_index][x][y][i]] += Cell_Mag[x][y];
      }
    }
  }

  return(1);
}

/**
 * Build the binary polar histogram
 * @param speed robot speed
 * @return 1
 */
int VFH_Algorithm::Build_Binary_Polar_Histogram( int speed )
{
  int x;

  for(x=0;x<HIST_SIZE;x++) {
      if (OriginHist[x] > Get_Binary_Hist_High(speed)) {
      Hist[x] = 1.0;
      } else if (OriginHist[x] < Get_Binary_Hist_Low(speed)) {
      Hist[x] = 0.0;
    } else {
      Hist[x] = Last_Binary_Hist[x];
    }
  }

  for(x=0;x<HIST_SIZE;x++) {
    Last_Binary_Hist[x] = Hist[x];
  }

  return(1);
}

//
// This function also sets Blocked_Circle_Radius.
//
/**
 * Build the masked polar histogram
 * @param speed robot speed
 * @return 1
 */
int VFH_Algorithm::Build_Masked_Polar_Histogram(int speed)
{
  int x, y;
  float center_x_right, center_x_left, center_y, dist_r, dist_l;
  float angle_ahead, phi_left, phi_right, angle;

  // center_x_[left|right] is the centre of the circles on either side that
  // are blocked due to the robot's dynamics.  Units are in cells, in the robot's
  // local coordinate system (+y is forward).
  center_x_right = CENTER_X + (Min_Turning_Radius[speed] / (float)CELL_WIDTH);
  center_x_left = CENTER_X - (Min_Turning_Radius[speed] / (float)CELL_WIDTH);
  center_y = CENTER_Y;

  angle_ahead = 90;
  phi_left  = 180;
  phi_right = 0;

  Blocked_Circle_Radius = Min_Turning_Radius[speed] + ROBOT_RADIUS + Get_Safety_Dist(speed);

  //
  // This loop fixes phi_left and phi_right so that they go through the inside-most
  // occupied cells inside the left/right circles.  These circles are centred at the
  // left/right centres of rotation, and are of radius Blocked_Circle_Radius.
  //
  // We have to go between phi_left and phi_right, due to our minimum turning radius.
  //

  //
  // Only loop through the cells in front of us.
  //
  for(y=0;y<(int)ceil(WINDOW_DIAMETER/2.0);y++)
  {
    for(x=0;x<WINDOW_DIAMETER;x++)
    {
        if (Cell_Mag[x][y] == 0)
            continue;

        if ((Delta_Angle(Cell_Direction[x][y], angle_ahead) > 0) &&
            (Delta_Angle(Cell_Direction[x][y], phi_right) <= 0))
        {
            // The cell is between phi_right and angle_ahead

            dist_r = hypot(center_x_right - x, center_y - y) * CELL_WIDTH;
            if (dist_r < Blocked_Circle_Radius)
            {
                phi_right = Cell_Direction[x][y];
            }
        }
        else if ((Delta_Angle(Cell_Direction[x][y], angle_ahead) <= 0) &&
                 (Delta_Angle(Cell_Direction[x][y], phi_left) > 0))
        {
            // The cell is between phi_left and angle_ahead

            dist_l = hypot(center_x_left - x, center_y - y) * CELL_WIDTH;
            if (dist_l < Blocked_Circle_Radius)
            {
                phi_left = Cell_Direction[x][y];
            }
        }
    }
  }

  //
  // Mask out everything outside phi_left and phi_right
  //
  for(x=0;x<HIST_SIZE;x++)
  {
      angle = x * SECTOR_ANGLE;
      if ((Hist[x] == 0) && (((Delta_Angle((float)angle, phi_right) <= 0) &&
                              (Delta_Angle((float)angle, angle_ahead) >= 0)) ||
                             ((Delta_Angle((float)angle, phi_left) >= 0) &&
                              (Delta_Angle((float)angle, angle_ahead) <= 0))))
      {
          Hist[x] = 0;
      }
      else
      {
          Hist[x] = 1;
      }
  }

  return(1);
}

/**
 * Set the motion commands
 * @param speed the desire speed
 * @param turnrate the desire turn rate
 * @param actual_speed the current speed
 * @return 1
 */
int VFH_Algorithm::Set_Motion( int &speed, int &turnrate, int actual_speed )
{
  // This happens if all directions blocked, so just spin in place
  if (speed <= 0)
  {
    //printf("stop\n");
      turnrate = GetMaxTurnrate( actual_speed );
      speed = 0;
  }
  else
  {
//    printf("Picked %f\n", Picked_Angle);
    if ((Picked_Angle > 270) && (Picked_Angle < 360)) {
        turnrate = -1 * GetMaxTurnrate( actual_speed );
    } else if ((Picked_Angle < 270) && (Picked_Angle > 180)) {
      turnrate = GetMaxTurnrate( actual_speed );
    } else {
      turnrate = (int)rint(((float)(Picked_Angle - 90) / 75.0) * GetMaxTurnrate( actual_speed ));
//    	turnrate = (int)rint(((float)(Picked_Angle) / 75.0) * GetMaxTurnrate( actual_speed ));
//      printf("GetMaxTurnrate( actual_speed ): %d, Picked_Angle: %f, turnrate %d\n",
//    		  GetMaxTurnrate( actual_speed ),
//    		  Picked_Angle,
//    		  turnrate);

      if (turnrate > GetMaxTurnrate( actual_speed )) {
        turnrate = GetMaxTurnrate( actual_speed );
      } else if (turnrate < (-1 * GetMaxTurnrate( actual_speed ))) {
        turnrate = -1 * GetMaxTurnrate( actual_speed );
      }

//      if (abs(turnrate) > (0.9 * GetMaxTurnrate( actual_speed ))) {
//        speed = 0;
//      }
    }
  }

//  speed and turnrate have been set for the calling function -- return.

  return(1);
}
