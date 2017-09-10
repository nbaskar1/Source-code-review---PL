/*
 * Copyright (C) 2009-2012 Gautier Hattenberger <gautier.hattenberger@enac.fr>,
 *                    Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/cam_control/rotorcraft_cam.c
 * Camera control module for rotorcraft.
 *
 * The camera is controled by the heading of the vehicle for pan
 * and can be controlled by a servo for tilt if defined.
 *
 * Four modes:
 *  - NONE: no control
 *  - MANUAL: the servo position is set with PWM
 *  - HEADING: the servo position and the heading of the rotorcraft are set with angles
 *  - WP: the camera is tracking a waypoint (Default: CAM)
 *
 * If ROTORCRAFT_CAM_SWITCH_GPIO is defined, this gpio is set/cleared to switch the power
 * of the camera on in normal modes and disable it when in NONE mode.
 * On boards with CAM_SWITCH, ROTORCRAFT_CAM_SWITCH_GPIO can be defined to CAM_SWITCH_GPIO.
 */

#include "modules/cam_control/rotorcraft_cam.h"

#include "subsystems/actuators.h"
#include "state.h"
#include "firmwares/rotorcraft/navigation.h"
#include "std.h"

#include "subsystems/datalink/telemetry.h"


/** Gpio output to turn camera power power on.
 * Control whether to set or clear the ROTORCRAFT_CAM_SWITCH_GPIO to turn on the camera power.
 * Should be defined to either gpio_set (default) or gpio_clear.
 * Not used if ROTORCRAFT_CAM_SWITCH_GPIO is not defined.
 */
#ifndef ROTORCRAFT_CAM_ON
#define ROTORCRAFT_CAM_ON gpio_set
#endif

/** Gpio output to turn camera power power off.
 * Control whether to set or clear the ROTORCRAFT_CAM_SWITCH_GPIO to turn off the camera power.
 * Should be defined to either gpio_set or gpio_clear (default).
 * Not used if ROTORCRAFT_CAM_SWITCH_GPIO is not defined.
 */
#ifndef ROTORCRAFT_CAM_OFF
#define ROTORCRAFT_CAM_OFF gpio_clear
#endif

uint8_t rotorcraft_cam_mode;

#define _SERVO_PARAM(_s,_p) SERVO_ ## _s ## _ ## _p
#define SERVO_PARAM(_s,_p) _SERVO_PARAM(_s,_p)

// Tilt definition
int16_t rotorcraft_cam_tilt;
int16_t rotorcraft_cam_tilt_pwm;
#if ROTORCRAFT_CAM_USE_TILT

/* 
   Compute the Rotor craft camera value of Neutral, Minimum or Maximum positions based on 
   Rotor Craft Tilt Servo angle and the corresponding Neutral, Minimum and Maximum values
*/
#define ROTORCRAFT_CAM_TILT_NEUTRAL SERVO_PARAM(ROTORCRAFT_CAM_TILT_SERVO, NEUTRAL)
#define ROTORCRAFT_CAM_TILT_MIN SERVO_PARAM(ROTORCRAFT_CAM_TILT_SERVO, MIN)
#define ROTORCRAFT_CAM_TILT_MAX SERVO_PARAM(ROTORCRAFT_CAM_TILT_SERVO, MAX)

/*
  Compute the Difference in tilt by calculating the range between Minimum and Maximum Rotor Cam tilt value
  CT_MIN is defined as the minimum value between the Camera Minimum and Maximum tilt angle
  CT_MAX is defined as the maximum value between the Camera Minimum and Maximum tilt angle
*/
#define D_TILT (ROTORCRAFT_CAM_TILT_MAX - ROTORCRAFT_CAM_TILT_MIN)
#define CT_MIN Min(CAM_TA_MIN, CAM_TA_MAX)
#define CT_MAX Max(CAM_TA_MIN, CAM_TA_MAX)
#endif

// Pan definition
/* 
   Defining the Maximum and the Minimum for the Rotorcraft Camera pan angle 
   ROTORCRAFT_CAM_PAN_MIN => 0 
   ROTORCRAFT_CAM_PAN_MAX => INT32_ANGLE_2_PI => ANGLE_BFP_OF_REAL(2 * pi) => BFP_REAL(2 * pi * 2 ^ 12)
   ANGLE_BFP_OF_REAL and BFP_REAL are defined under paparazzi_algebra_int.h
*/ 
int16_t rotorcraft_cam_pan;
#define ROTORCRAFT_CAM_PAN_MIN 0
#define ROTORCRAFT_CAM_PAN_MAX INT32_ANGLE_2_PI

static void send_cam(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROTORCRAFT_CAM(trans, dev, AC_ID,
                               &rotorcraft_cam_tilt, &rotorcraft_cam_pan);
}
  
// Switches the camera ON/ OFF depending on the MODE input
void rotorcraft_cam_set_mode(uint8_t mode)
{
  rotorcraft_cam_mode = mode;
// ROTORCRAFT_CAM_SWITCH_GPIO is set/ cleared based on rotorcraft_cam_mode   
#ifdef ROTORCRAFT_CAM_SWITCH_GPIO
  if (rotorcraft_cam_mode == ROTORCRAFT_CAM_MODE_NONE) {
    ROTORCRAFT_CAM_OFF(ROTORCRAFT_CAM_SWITCH_GPIO);
  } else {
    ROTORCRAFT_CAM_ON(ROTORCRAFT_CAM_SWITCH_GPIO);
  }
#endif
}
// Rotorcraft Camera is switched ON/ OFF based on ROTORCRAFT_CAM_SWITCH_GPIO
void rotorcraft_cam_init(void)
{ 
#ifdef ROTORCRAFT_CAM_SWITCH_GPIO
  gpio_setup_output(ROTORCRAFT_CAM_SWITCH_GPIO);
#endif
   
// The mode of the Rotorcraft Camera is set based on the value of ROTORCRAFT_CAM_DEFAULT_MODE
  rotorcraft_cam_set_mode(ROTORCRAFT_CAM_DEFAULT_MODE);
   
//  If ROTORCRAFT_CAM_USE_TILT is set, then the Actuator will be set, based on SERVO and the Camera Tilt Angle Neutral Value   
#if ROTORCRAFT_CAM_USE_TILT
  rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_NEUTRAL;
  ActuatorSet(ROTORCRAFT_CAM_TILT_SERVO, rotorcraft_cam_tilt_pwm);
// If the above condition is not satisfied then 1500 is set to rotorcraft_cam_tilt_pwm    
#else  
  rotorcraft_cam_tilt_pwm = 1500;
#endif
   
// Camera tilt angle and Camera Pan angle are set to 0
  rotorcraft_cam_tilt = 0;
  rotorcraft_cam_pan = 0;
   
/*
  Periodic Telemetry messages are registered based on the number of messages and the call back from the register 
  The function register_periodic_telemetry is defined in telemetry.c
*/
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_CAM, send_cam);
}

void rotorcraft_cam_periodic(void)
{
 // Computing rotorcraft_cam_tilt_pwm based on the Rotor Craft Camera mode 
  switch (rotorcraft_cam_mode) {
    //  Mode => None ; rotorcraft_cam_tilt_pwm is set to Camera Tilt Angle Neutral value  
    case ROTORCRAFT_CAM_MODE_NONE:
#if ROTORCRAFT_CAM_USE_TILT
      rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_NEUTRAL;
#endif
    /*  
      rotorcraft_cam_pan is set to psi value based on orientation of the Eulers value
      stateGetNedToBodyEulers_i is defined in state.h    
    */   
#if ROTORCRAFT_CAM_USE_PAN
      rotorcraft_cam_pan = stateGetNedToBodyEulers_i()->psi;
#endif
      break;
    //  Mode => Manual
    case ROTORCRAFT_CAM_MODE_MANUAL:  
      // nothing to do here, just apply tilt pwm at the end
      break;
    //  Mode => Heading    
    case ROTORCRAFT_CAM_MODE_HEADING:
#if ROTORCRAFT_CAM_USE_TILT_ANGLES
    /*  
        The camera tilt angle, CT_MIN and CT_MAX  is bound to the UAV 
        rotorcraft_cam_tilt_pwm is calculated based on the pre-processors defined in the top of this file
    */   
      Bound(rotorcraft_cam_tilt, CT_MIN, CT_MAX);
      rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_MIN + D_TILT * (rotorcraft_cam_tilt - CAM_TA_MIN) /
                                (CAM_TA_MAX - CAM_TA_MIN);
#endif
 /*
    INT32_COURSE_NORMALIZE will normalise the value between the range 0 - INT32_ANGLE_2_PI
    INT32_ANGLE_2_PI = 2 * pi * (1 << 12); INT32_COURSE_NORMALIZE is defined in pprz_algebra_int.h
 */    
#if ROTORCRAFT_CAM_USE_PAN
      INT32_COURSE_NORMALIZE(rotorcraft_cam_pan);
      //  Rotor Craft Camera Pan Angle is set to the Navigation Heading defined in file navigation.c      
      nav_heading = rotorcraft_cam_pan;
#endif
      break;
    // Mode => WP
    case ROTORCRAFT_CAM_MODE_WP:
#ifdef ROTORCRAFT_CAM_TRACK_WP
      {
         
        /* 
           Defining a structure variable diff which contains two 32 bit integer values as a vector
           Int32Vect2 is defined in pprz_algebra_int.h
        */ 
        struct Int32Vect2 diff;
         
        /* 
           Compute the Vector difference between Waypoints(East, North and Altitude defined in common_nav.h) &
           stateGetPositionEnu_i(East, North and Up Directions defined as a integer in state.h) and the result
           is stored in the variable diff
        */
        VECT2_DIFF(diff, waypoints[ROTORCRAFT_CAM_TRACK_WP], *stateGetPositionEnu_i());
        
        // The values of diff variable is right shifted by INT32_POS_FRAC => 8 (Function is defined under pprz_algebra_int.h)
        INT32_VECT2_RSHIFT(diff, diff, INT32_POS_FRAC);
         
        /* 
           Computing the fixed point arithmetic using int32_atan2 based on the x and y values of diff variable.
           int32_atan2 is defined under pprz_trig_int.c
           Rotor Craft Camera Pan angle is set to Navigation Heading defined in file navigation.c
        */ 
        rotorcraft_cam_pan = int32_atan2(diff.x, diff.y);
        nav_heading = rotorcraft_cam_pan;
#if ROTORCRAFT_CAM_USE_TILT_ANGLES
        int32_t dist, height;
        /* 
           Distance => Vector value of diff variable; 
           height => (Waypoints(Altitude) - ENU Coordinates(Altitude)) right shifted by 8
        */
        dist = INT32_VECT2_NORM(diff);
        height = (waypoints[ROTORCRAFT_CAM_TRACK_WP].z - stateGetPositionEnu_i()->z) >> INT32_POS_FRAC;
        rotorcraft_cam_tilt = int32_atan2(height, dist);
        /*  
          The camera tilt angle, CAM_TA_MIN and CAM_TA_MAX  is bound to the UAV 
          rotorcraft_cam_tilt_pwm is calculated based on the pre-processors defined in the top of this file
        */ 
        Bound(rotorcraft_cam_tilt, CAM_TA_MIN, CAM_TA_MAX);
        rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_MIN + D_TILT * (rotorcraft_cam_tilt - CAM_TA_MIN) /
                                  (CAM_TA_MAX - CAM_TA_MIN);
#endif
      }
#endif
      break;
    default:
      break;
  }
#if ROTORCRAFT_CAM_USE_TILT
  // If ROTORCRAFT_CAM_USE_TILT is set, then the Actuator will be set, based on SERVO and the Camera Tilt PWM Value
  ActuatorSet(ROTORCRAFT_CAM_TILT_SERVO, rotorcraft_cam_tilt_pwm);
#endif
}
