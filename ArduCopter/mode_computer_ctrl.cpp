#include "Copter.h"

/*
 Init and run calls for computer controlled flight mode.
This mode allows a computer to command attitude and direct thrust targets
(as opposed to climb rate). Useful for writing high-level outer loops.

Nimbus Lab.
 -- aj / Dec 21, 2018.
 */
#define COMPUTER_ATTITUDE_TIMEOUT_MS 250
struct {
    // attitude angles are in centidegrees
    // @TODO: can clean up and use quaternions, this only helps readability
    uint32_t update_time_ms;
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
    float yaw_rate_cmd;
    float collective_cmd;
    bool use_yaw_rate;
} static computer_cmd_state;

// function for Mavlink to set the targets. Check GCSMavlink file.
void Copter::ModeComputer::set_targets( const Quaternion &q, float collective,
                                       bool use_yaw_rate, float yaw_rate_rads )
{
  // copy over the values into the "global" struct declared in this file
  // @TODO: can avoid all this by calling attitude_control's input_quaternion(..)
  // @TODO: check how to control yaw rate, if using input_quaternion().
  q.to_euler( computer_cmd_state.roll_cmd, computer_cmd_state.pitch_cmd,
              computer_cmd_state.yaw_cmd );
  computer_cmd_state.roll_cmd = ToDeg( computer_cmd_state.roll_cmd ) * 100.0f;
  computer_cmd_state.pitch_cmd = ToDeg( computer_cmd_state.pitch_cmd ) * 100.0f;
  computer_cmd_state.collective_cmd = collective;
  // intentionally prohibit yaw control - we only want yaw rate:
  // computer_cmd_state.yaw_cmd = ToDeg( computer_cmd_state.yaw_cmd ) * 100.0f;
  computer_cmd_state.yaw_rate_cmd = ToDeg( yaw_rate_rads ) * 100.0f;
  computer_cmd_state.use_yaw_rate = use_yaw_rate;
  computer_cmd_state.update_time_ms = millis();
}                                       

// init function is only called once when mode is switched
bool Copter::ModeComputer::init( bool ignore_checks )
{
  // allow attitude commands to be accepted
  // Mavlink should have already called set_targets function for this mode
  // before init is ever called. This prevents an undefined update_time_ms.
  
  // @TODO: perform checks to enter this state 
  
  // initialise variables, possibly to reset the attitude contoller
  // @TODO: consider using attitude_control->set_attitude_target_to_current_attitude()
  
  // prevent mode switch being considered as an ""update""
  //computer_cmd_state.update_time_ms = millis();
  computer_cmd_state.roll_cmd = ahrs.roll_sensor;
  computer_cmd_state.pitch_cmd = ahrs.pitch_sensor;
  computer_cmd_state.yaw_cmd = ahrs.yaw_sensor;
  
  // @TODO: consider using attitude_control->get_throttle_in()
  // typical f450s seem to ~hover at thrust=0.16 with a 3S battery. Initialise
  // thrust to a slightly lower value ( better than 0 at least :) ).
  computer_cmd_state.collective_cmd = 0.12;
  computer_cmd_state.yaw_rate_cmd = 0.0;
  computer_cmd_state.use_yaw_rate = true;
  
  // return true if no problems so far.
  return true;
}

// run function is called at whatever low-level mode control frequency is (400?)
void Copter::ModeComputer::run()
{
  // defer to another run function .. because, I'll think about it later
  // -- can add case/switch here if needed.
  ModeComputer::computer_control_run();
}

void Copter::ModeComputer::computer_control_run()
{
  if( !motors->armed() || !ap.auto_armed || !motors->get_interlock() ||
      (ap.land_complete) )
  {
    #if FRAME_CONFIG == HELI_FRAME
      attitude_control->set_yaw_target_to_current_heading();
    #endif
    zero_throttle_and_relax_ac();
    pos_control->relax_alt_hold_controllers(0.0f);
    return;
  }
  // constrain desired lean angles
  float roll_in = computer_cmd_state.roll_cmd;
  float pitch_in = computer_cmd_state.pitch_cmd;
  float total_in = norm(roll_in, pitch_in);
  float angle_max = attitude_control->lean_angle_max();
  if( total_in > angle_max )
  {
    float ratio = angle_max / total_in;
    roll_in *= ratio;
    pitch_in *= ratio;
  }

  // wrap yaw. Note that yaw command is not enabled in this mode
  // float yaw_in = wrap_180_cd( computer_cmd_state.yaw_cmd) ;
  float yaw_rate_in = wrap_180_cd( computer_cmd_state.yaw_rate_cmd );
  
  /*
    Copter has automatic "hover learning", that continuously updates the
    throttle stick position required to hover in flight. This allows for scaling
    the thrust value such that 0.5 is close to hovering *regardless of battery*.
    While this may seem enticing, note that this behaviour will deviate from the
    common double-integrator model (and make it closer to a single integrator).
    `get_pilot_desired_throttle` does this remapping. Do not use this 'feature'
    without rigourous testing of the outer control loop. Following 2 lines.
    
    // remap thrust to make 0.5 the hover value
    uint16_t collective_in = (computer_cmd_state.collective_cmd * 1000);
    float collective_in_scaled = get_pilot_desired_throttle( collective_in );
  */
  
  // use throttle value directly from computer
  float collective_in = computer_cmd_state.collective_cmd;
  
  // check for timeout - set lean angles and climb rate to zero if no updates
  uint32_t tnow = millis();
  if( tnow - computer_cmd_state.update_time_ms > COMPUTER_ATTITUDE_TIMEOUT_MS )
  {
    roll_in = 0.0f;
    pitch_in = 0.0f;
    float climb_rate_cms = 0.0f;
    yaw_rate_in = 0.0f;
    pos_control->set_alt_target_from_climb_rate_ff(climb_rate_cms, G_Dt, false);
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in);
    return;
  }
  
  motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

  // call attitude controller
  attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in);

  // Controlling yaw is not allowed in this mode:
  //attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);
  
  // do thrust control, do not compensate for tilt, and apply standard filter len.
  // "g" is a variable inside Mode class, of type Parameters. Fantastic choice of
  // a name for a variable to grep. :)
  attitude_control->set_throttle_out( collective_in, false, g.throttle_filt );
}

