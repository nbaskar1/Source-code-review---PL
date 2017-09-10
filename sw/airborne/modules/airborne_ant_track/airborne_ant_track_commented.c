/*
 * Determines antenna pan angle.
 *
 * project:        Paparazzi
 * description:    Determines antenna pan angle from
 *                 plane's and home's positions and plane's heading angle.
 *                 Software might be optimized
 *                 by removing multiplications with 0, it is left this
 *                 way for better understandabilty and changeability.
 *
 * authors:        Arnold Schroeter, Martin Mueller, Chris Efstathiou
 *
 *
 *
 *
 */

// If the value USE_AIRBORNE_ANT_TRACKING is defined and has a value of 1 then the file is run else the entire execution lines are skipped.
#if defined(USE_AIRBORNE_ANT_TRACKING) && USE_AIRBORNE_ANT_TRACKING == 1

// Includes necessary header files for the different variables or functions mentioned in the program.
#include <math.h>
#include <inttypes.h>
#include "inter_mcu.h"
#include "subsystems/navigation/common_nav.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "state.h"
#include "airborne_ant_track.h"

// Defining a structure VECTOR to declare the vector directions in different axes
typedef struct {
  float fx;  // X axis
  float fy;  // Y axis
  float fz;  // Z axis
} VECTOR;

// Defining a structure MATRIX containing 3*3 elements
typedef struct {
  float fx1; float fx2; float fx3;  // Elements : 1x1, 1x2, 1x3 
  float fy1; float fy2; float fy3;  // Elements : 2x1, 2x2, 2x3
  float fz1; float fz2; float fz3;  // Elements : 3x1, 3x2, 3x3
} MATRIX;

float   airborne_ant_pan;
static bool ant_pan_positive = 0;

void ant_point(void);

// Declaring a function "vSubtractVectors" 
// Alters the first parameter vector variable in the statement when the function is called
static void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC);

// Declaring a function "vMultiplyMatrixByVector"
// Alters the first parameter vector variable in the statement when the function is called
static void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC);

/*******************************************************************
; function name:   vSubtractVectors
; description:     subtracts two vectors a = b - c
; parameters:      Vector A, Vector B, Vector C
;*******************************************************************/
static void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC)
{
   svA->fx = svB.fx - svC.fx;  // Vector component subtraction of X-axis
  svA->fy = svB.fy - svC.fy;  // Vector component subtraction of Y-axis
  svA->fz = svB.fz - svC.fz;  // Vector component subtraction of Z-axis
}

/*******************************************************************
; function name:   vMultiplyMatrixByVector
; description:     multiplies matrix by vector svA = smB * svC
; parameters:      Vector A, MATRIX B, VECTOR C
;*******************************************************************/
static void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC)
{
  svA->fx = smB.fx1 * svC.fx  +  smB.fx2 * svC.fy  +  smB.fx3 * svC.fz;
  svA->fy = smB.fy1 * svC.fx  +  smB.fy2 * svC.fy  +  smB.fy3 * svC.fz;
  svA->fz = smB.fz1 * svC.fx  +  smB.fz2 * svC.fy  +  smB.fz3 * svC.fz;
}

// Antenna Initial point definition
void airborne_ant_point_init(void)
{

  return;
}

// Function to calculate the antenna point position
void airborne_ant_point_periodic(void)
{
  float airborne_ant_pan_servo = 0;
  
  /*
     Declaring static vectors "svPlanePosition", "Home_Position", "Home_PositionForPlane" & "Home_PositionForPlane2"
     which represents the vector plane position, the initial home position, difference between the home and plane position &
     the final rotated output position respectively
  */
  static VECTOR svPlanePosition,
         Home_Position,
         Home_PositionForPlane,
         Home_PositionForPlane2;
  
  // Rotation Matrix Declration
  static MATRIX smRotation;
  
  /* Get values of East, North and Up coordinates and assign North value as "svPlanePosition.fx" (X axis to the Plane Position)
     Get values of East, North and Up coordinates and assign East value as "svPlanePosition.fy" (Y axis to the Plane Position)
     Get values of North, East, Altitude and Zone number and assign Altitude value as "svPlanePosition.fz" (Z axis to the Plane Position) 
     "stateGetPositionEnu_f" is a structure variable defined under the structure EnuCoor_f, defined in pprz_geodectic_float.h
     "stateGetPositionUtm_f" is a structure variable defined under the structure UtmCoor_f, defined in pprz_geodectic_float.h
     "pprz_geodectic_float.h" is imported into state.h file which is in turn imported here. 
  */
  svPlanePosition.fx = stateGetPositionEnu_f()->y;  
  svPlanePosition.fy = stateGetPositionEnu_f()->x;  
  svPlanePosition.fz = stateGetPositionUtm_f()->alt;   

  /* Get values of East, North and Up coordinates of the initial Waypoint and assign North value as "Home_Position.fx" (X axis to the Home Position)
     Get values of East, North and Up coordinates of the initial Waypoint and assign East value as "Home_Position.fy" (Y axis to the Home Position)
     Get values of North, East, Altitude and Zone number of the initial Waypoint and assign Altitude value as "Home_Position.fz" (Z axis to the Home Position)
     "WaypointY" calls the function waypoint_get_y that returns the North value of ENU coordinates as a float value to  "Home_Position.fx" 
     "WaypointX" calls the function waypoint_get_y that returns the East value of ENU coordinates as a float value to  "Home_Position.fy"
     "waypoints" gets the altitude value in float from the structure point defined in common_nav.h under navigation folder.
     "WaypointY", "WaypointX" are defined under waypoints.c
  */
  Home_Position.fx = WaypointY(WP_HOME);
  Home_Position.fy = WaypointX(WP_HOME);
  Home_Position.fz = waypoints[WP_HOME].a;

  /* distance between plane and object */
  // Home_PositionForPlane = Home_Position - svPlanePosition (Vector substraction through call by reference)
  vSubtractVectors(&Home_PositionForPlane, Home_Position, svPlanePosition);

  
  /* yaw */
  /* "stateGetHorizontalSpeedDir_f" is defined in state.h which returns the float value of the direction of horizontal ground speed.
     "cosf" function computes the cosine value of horizontal ground speed and return as float values.
     "sinf" function computes the sine value of horizontal ground speed and return as float values. */
  smRotation.fx1 = cosf(stateGetHorizontalSpeedDir_f());
  smRotation.fx2 = sinf(stateGetHorizontalSpeedDir_f());
  smRotation.fx3 = 0.;
  smRotation.fy1 = -smRotation.fx2;-
  smRotation.fy2 = smRotation.fx1;
  smRotation.fy3 = 0.;
  smRotation.fz1 = 0.;
  smRotation.fz2 = 0.;
  smRotation.fz3 = 1.;

  // Home_PositionForPlane2 = smRotation * Home_PositionForPlane (Multiplication of matrix by vector through call by reference)
  vMultiplyMatrixByVector(&Home_PositionForPlane2, smRotation, Home_PositionForPlane);


  /*
   * This is for one axis pan antenna mechanisms. The default is to
   * circle clockwise so view is right. The pan servo neutral makes
   * the antenna look to the right with 0˚ given, 90˚ is to the back and
   * -90˚ is to the front.
   *
   *
   *
   *   plane front
   *
   *                  90˚
                      ^
   *                  I
   *             135˚ I  45˚
   *                \ I /
   *                 \I/
   *       180˚-------I------- 0˚
   *                 /I\
   *                / I \
   *            -135˚ I  -45˚
   *                  I
   *                -90
   *             plane back
   *
   *
   */

  /* fPan =   0˚  -> antenna looks along the wing
             90˚  -> antenna looks in flight direction
            -90˚  -> antenna looks backwards
  */
  /* fixed to the plane*/
  // Computes the angle between X-axis and vector(Home_PositionForPlane2.fx, Home_PositionForPlane2.fy) in radians
  airborne_ant_pan = (float)(atan2(Home_PositionForPlane2.fx, (Home_PositionForPlane2.fy)));

  // I need to avoid oscillations around the 180 degree mark.
  /* 
     Checks for the condition "0 < airborne_ant_pan <= 175" and sets "airborne_pan_positive" to 1 
     Checks for the condition "-175 <= airborne_ant_pan < 0" and sets "airborne_pan_positive" to 0 
  */   
  if (airborne_ant_pan > 0 && airborne_ant_pan <= RadOfDeg(175)) { ant_pan_positive = 1; }
  if (airborne_ant_pan < 0 && airborne_ant_pan >= RadOfDeg(-175)) { ant_pan_positive = 0; }
  
  // To avoid the oscillations around 180 degree mark the "airborne_ant_pan" is set to -180 if it satisfies the above condition.
  if (airborne_ant_pan > RadOfDeg(175) && ant_pan_positive == 0) {
    airborne_ant_pan = RadOfDeg(-180);
  
  } 
  // To avoid the oscillations around 180 degree mark the "airborne_ant_pan" is set to 180 if it satisfies the above condition.
  else if (airborne_ant_pan < RadOfDeg(-175) && ant_pan_positive) {
    airborne_ant_pan = RadOfDeg(180);
    ant_pan_positive = 0;
  }
  

#ifdef ANT_PAN_NEUTRAL
  /* 
     Computes "airborne_ant_pan" based on the ANT_PAN_NEUTRAL
     MAX_PPRZ = 9600; MIN_PPRZ = -9600; (Defined under paparazzi.h) 
  */ 
  airborne_ant_pan = airborne_ant_pan - RadOfDeg(ANT_PAN_NEUTRAL);
  if (airborne_ant_pan > 0) {
    airborne_ant_pan_servo = MAX_PPRZ * (airborne_ant_pan / (RadOfDeg(ANT_PAN_MAX - ANT_PAN_NEUTRAL)));
  } else {
    airborne_ant_pan_servo = MIN_PPRZ * (airborne_ant_pan / (RadOfDeg(ANT_PAN_MIN - ANT_PAN_NEUTRAL)));
  }
#endif
  /* 
      Limits the "airborne_ant_pan_servo" between MAX_PPRZ and MIN_PPRZ
      (TRIM_PPRZ defined under paparazzi.h)
  */
  airborne_ant_pan_servo = TRIM_PPRZ(airborne_ant_pan_servo);

#ifdef COMMAND_ANT_PAN
  /*
      Sets the Auto Pilot command based on the "airborne_ant_pan_servo"
      "imcu_set_command" is defined under the inter_mcu.h which computes PPRZ_MUTEX_LOCK and PPRZ_MUTEX_UNLOCK 
      (Defined under pprz_mutex.h) 
  */
  imcu_set_command(COMMAND_ANT_PAN, airborne_ant_pan_servo);
#endif


  return;
}

#endif
