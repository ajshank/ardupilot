#include "Copter.h"

/*
 Init and run calls for computer controlled flight mode.
 Sadly, the code follows a structure similar to the original
 god-forsaken structure found in the arducopter code - which I find needlessly
 obfuscated and oftentimes downright unwelcoming.
 Nimbus Lab.
 -- aj / Dec 21, 2018.
 */
#define COMPUTER_ATTITUDE_TIMEOUT_MS 500
struct {
    // pretty sure that angles are in centidegrees
    uint32_t update_time_ms;
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
    float yaw_rate_cmd;
    float collective_cmd;
    bool use_yaw_rate;
} static computer_cmd_state;

// function for Mavlink to set the targets
void Copter::ModeComputer::set_targets( const Quaternion &q, float collective,
                                       bool use_yaw_rate, float yaw_rate_rads )
{
  // copy over the values into the "global" struct declared in this file
  q.to_euler( computer_cmd_state.roll_cmd, computer_cmd_state.pitch_cmd,
              computer_cmd_state.yaw_cmd );
  computer_cmd_state.roll_cmd = ToDeg( computer_cmd_state.roll_cmd ) * 100.0f;
  computer_cmd_state.pitch_cmd = ToDeg( computer_cmd_state.pitch_cmd ) * 100.0f;
  computer_cmd_state.collective_cmd = collective;
  computer_cmd_state.yaw_cmd = ToDeg( computer_cmd_state.yaw_cmd ) * 100.0f;
  computer_cmd_state.yaw_rate_cmd = ToDeg( yaw_rate_rads ) * 100.0f;
  computer_cmd_state.use_yaw_rate = use_yaw_rate;
  computer_cmd_state.update_time_ms = millis();
}                                       

// init function is only called once when mode is switched
bool Copter::ModeComputer::init( bool ignore_checks )
{
  // allow attitude commands to be accepted
  
  // perform checks to enter this state 
  
  // initialise variables, possibly to reset the attitude contoller
  // @TODO: consider using attitude_control->set_attitude_target_to_current_attitude()
  computer_cmd_state.update_time_ms = millis();
  computer_cmd_state.roll_cmd = ahrs.roll_sensor;
  computer_cmd_state.pitch_cmd = ahrs.pitch_sensor;
  computer_cmd_state.yaw_cmd = ahrs.yaw_sensor;
  // @TODO: consider using attitude_control->get_throttle_in()
  computer_cmd_state.collective_cmd = computer_cmd_state.collective_cmd;
  computer_cmd_state.yaw_rate_cmd = computer_cmd_state.yaw_rate_cmd; // @TODO
  computer_cmd_state.use_yaw_rate = true;
  
  // return true if no problems so far.
  return true;
}

// run function is called at whatever low-level mode control frequency is (100?)
void Copter::ModeComputer::run()
{
  // defer to another run function .. because, I'll think about it later
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

  // wrap yaw request
  float yaw_in = wrap_180_cd( computer_cmd_state.yaw_cmd) ;
  float yaw_rate_in = wrap_180_cd( computer_cmd_state.yaw_rate_cmd );
  
  float collective_in = computer_cmd_state.collective_cmd;
  // @TODO: check for thrust range
  
  // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
  uint32_t tnow = millis();
  if( tnow - computer_cmd_state.update_time_ms > COMPUTER_ATTITUDE_TIMEOUT_MS )
  {
    roll_in = 0.0f;
    pitch_in = 0.0f;
    //climb_rate_cms = 0.0f;
    yaw_rate_in = 0.0f;
  }
  
   motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

  // call attitude controller
  if( computer_cmd_state.use_yaw_rate )
  {
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in);
  }
  else
  {
    attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);
  }
  
  // do thrust control, do not compensate for tilt, and apply standard filter len
  // "g" is a variable inside Mode class, of type Parameters. Fantastic choice of
  // a name, I tell you! Try grepping for that.
  attitude_control->set_throttle_out( collective_in, false, g.throttle_filt );
}

