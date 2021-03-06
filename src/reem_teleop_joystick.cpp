/*
 * A joystick tele-operation controller for the REEM robot.
 * Based on pr2_teleop_general by E. Gil Jones
 */

/*
 * Copyright (c) 2013, David Butterworth, PAL Robotics S.L.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include "reem_teleop_general/reem_teleop_commander.h"

enum JoystickLayoutMode
{
    LAYOUT_NONE,
    LAYOUT_BODY,
    LAYOUT_HEAD,
    LAYOUT_RIGHT_ARM,
    LAYOUT_LEFT_ARM,
    LAYOUT_BOTH_ARMS
};

static const unsigned int BODY_LAYOUT_BUTTON = 4; // 10 on PS3
static const unsigned int RIGHT_ARM_LAYOUT_BUTTON = 7; // 9 on PS3
static const unsigned int LEFT_ARM_LAYOUT_BUTTON = 6; // 8 on PS3
static const unsigned int HEAD_LAYOUT_BUTTON = 5; // 11  on PS3

// Toggle head mode, right button down
static const unsigned int HEAD_MODE_TOGGLE_BUTTON = 1;  // On PR2 with PS3, button 4 was left button up
//static const unsigned int LASER_TOGGLE_BUTTON = 7;
//static const unsigned int PROSILICA_POLL_BUTTON = 6;
static const unsigned int OPEN_GRIPPER_BUTTON = 15;
static const unsigned int CLOSE_GRIPPER_BUTTON = 13;

// Toggle arm mode, right button down
static const unsigned int ARM_MODE_TOGGLE_BUTTON = 1;  // On PR2 with PS3, button 4 was left button up

// ???
static const unsigned int LEFT_AXIS_NUMBER = 1;
static const unsigned int RIGHT_AXIS_NUMBER = 1;

// Put base movement on left joystick for Reem
static const unsigned int VX_AXIS = 1; // 3 = left joystick, up-down
//static const unsigned int VY_AXIS = 2; // 2 = left joystick, left-right
static const unsigned int VW_AXIS = 0; // 0 = left joystick, left-right

static const unsigned int HEAD_PAN_AXIS = 0; // left joystick, left-right
static const unsigned int HEAD_TILT_AXIS = 3; // right joystick, up-down

// for arm IK control
static const unsigned int ARM_X_AXIS = 3;
static const unsigned int ARM_Y_AXIS = 2;
static const unsigned int ARM_Z_AXIS = 1;
// static const unsigned int ARM_YAW_AXIS = 0; // un-used
static const unsigned int WRIST_CLOCKWISE_BUTTON = 0; // right buttons, left
static const unsigned int WRIST_COUNTER_BUTTON = 2; // right buttons, right

// for arm joint control
static const unsigned int ARM_JOINT_1_AXIS = 1; // Shoulder Pitch
static const unsigned int ARM_JOINT_2_AXIS = 0; // Shoulder Roll
static const unsigned int ARM_JOINT_3_AXIS = 2; // Shoulder Yaw
static const unsigned int ARM_JOINT_4_AXIS = 3; // Elbow Pitch
static const unsigned int ARM_JOINT_5_BUTTON1 = 0; // Elbow Yaw
static const unsigned int ARM_JOINT_5_BUTTON2 = 1; // Elbow Yaw
static const unsigned int ARM_JOINT_6_AXIS = 4; // Wrist Roll
static const unsigned int ARM_JOINT_7_AXIS = 5; // Wrist Yaw
static const unsigned int HAND_OPEN_BUTTON = 3; // Grasp off
static const unsigned int HAND_CLOSE_BUTTON = 2; // Grasp on

//static const unsigned int ARM_UNTUCK_BUTTON = 7;
//static const unsigned int ARM_TUCK_BUTTON = 5;

//static const unsigned int MOVE_TO_WALK_ALONG_BUTTON = 0;
//static const unsigned int SET_WALK_ALONG_BUTTON = 3;

static const ros::Duration DOUBLE_TAP_TIMEOUT(.25);

class ReemTeleopGeneralJoystick
{
public:
    ReemTeleopGeneralJoystick(bool deadman_no_publish = false)
    {
        gc = NULL;
    }

    void init()
    {
        des_pan_pos_ = des_tilt_pos_ = 0;
        vel_val_pan_ = vel_val_tilt_ = 0;

        des_torso_pos_ = 0;

        des_torso_vel_ = 0.0;

        ros::NodeHandle n_local("~");

        // Head pan/tilt parameters
        // NEED TO CHECK THESE JOINT LIMITS
        n_local.param("max_pan", max_pan_, 2.7);
        n_local.param("min_tilt", min_tilt_, -0.4);
        n_local.param("max_tilt", max_tilt_, 3.4); // 1.4 for PR2


        n_local.param("tilt_scale", tilt_scale_, .5);
        n_local.param("pan_scale", pan_scale_, 1.2);

        n_local.param("arm_joint_1_scale", arm_joint_1_scale_, 1.2);
        n_local.param("arm_joint_2_scale", arm_joint_2_scale_, 1.2);
        n_local.param("arm_joint_3_scale", arm_joint_3_scale_, 1.2);
        n_local.param("arm_joint_4_scale", arm_joint_4_scale_, 1.2);
        n_local.param("arm_joint_5_scale", arm_joint_5_scale_, 1.2); // button is 0 or 1, so needs a higher scaling factor
        n_local.param("arm_joint_6_scale", arm_joint_6_scale_, 1.2); // axis is 0 or 1, so needs a higher scaling factor
        n_local.param("arm_joint_7_scale", arm_joint_7_scale_, 1.5); // axis is 0 or 1, so needs a higher scaling factor

        // FIX THESE LIMITS
        n_local.param("arm_joint_1_min", arm_joint_1_min_, -1.75);
        n_local.param("arm_joint_1_max", arm_joint_1_max_, 1.75);

        n_local.param("arm_joint_2_min", arm_joint_2_min_, -1.75);
        n_local.param("arm_joint_2_max", arm_joint_2_max_, 1.75);

        n_local.param("arm_joint_3_min", arm_joint_3_min_, -1.75);
        n_local.param("arm_joint_3_max", arm_joint_3_max_, 1.75);

        n_local.param("arm_joint_4_min", arm_joint_4_min_, -1.75);
        n_local.param("arm_joint_4_max", arm_joint_4_max_, 1.75);

        n_local.param("arm_joint_5_min", arm_joint_5_min_, -1.75);
        n_local.param("arm_joint_5_max", arm_joint_5_max_, 1.75);

        n_local.param("arm_joint_6_min", arm_joint_6_min_, -1.75);
        n_local.param("arm_joint_6_max", arm_joint_6_max_, 1.75);

        n_local.param("arm_joint_7_min", arm_joint_7_min_, -1.75);
        n_local.param("arm_joint_7_max", arm_joint_7_max_, 1.75);

        //n_local.param("hand_left_thumb_joint_scale", hand_left_thumb_joint_scale_, 1.0);
        //n_local.param("hand_left_index_1_joint_scale", hand_left_index_1_joint_scale_, 1.0);
        //n_local.param("hand_left_index_2_joint_scale", hand_left_index_2_joint_scale_, 1.0);
        //n_local.param("hand_left_index_3_joint_scale", hand_left_index_3_joint_scale_, 1.0);
        //n_local.param("hand_left_middle_1_joint_scale", hand_left_middle_1_joint_scale_, 1.0);
        //n_local.param("hand_left_middle_2_joint_scale", hand_left_middle_2_joint_scale_, 1.0);
        //n_local.param("hand_left_middle_3_joint_scale", hand_left_middle_3_joint_scale_, 1.0);
        n_local.param("fingers_scale", fingers_scale_, 1.0);
        n_local.param("thumb_scale", thumb_scale_, 7.0);

        n_local.param("hand_left_thumb_joint_min", hand_left_thumb_joint_min_, 0.0);
        n_local.param("hand_left_thumb_joint_max", hand_left_thumb_joint_max_, 2.5);

        n_local.param("hand_left_index_1_joint_min", hand_left_index_1_joint_min_, 0.0);
        n_local.param("hand_left_index_1_joint_max", hand_left_index_1_joint_max_, 2.5);

        n_local.param("hand_left_index_2_joint_min", hand_left_index_2_joint_min_, 0.0);
        n_local.param("hand_left_index_2_joint_max", hand_left_index_2_joint_max_, 2.5);

        n_local.param("hand_left_index_3_joint_min", hand_left_index_3_joint_min_, 0.0);
        n_local.param("hand_left_index_3_joint_max", hand_left_index_3_joint_max_, 2.5);

        n_local.param("hand_left_middle_1_joint_min", hand_left_middle_1_joint_min_, 0.0);
        n_local.param("hand_left_middle_1_joint_max", hand_left_middle_1_joint_max_, 2.5);

        n_local.param("hand_left_middle_2_joint_min", hand_left_middle_2_joint_min_, 0.0);
        n_local.param("hand_left_middle_2_joint_max", hand_left_middle_2_joint_max_, 2.5);

        n_local.param("hand_left_middle_3_joint_min", hand_left_middle_3_joint_min_, 0.0);
        n_local.param("hand_left_middle_3_joint_max", hand_left_middle_3_joint_max_, 2.5);

        //n_local.param("_scale", _scale_, 1.0);
        //n_local.param("_min", _min_, 0.0);
        //n_local.param("_max", _max_, 2.5);

        n_local.param("torso_step", torso_step_, 0.05);
        n_local.param("min_torso", min_torso_, 0.0);
        n_local.param("max_torso", max_torso_, 0.3);

        n_local.param("vx_scale", vx_scale_, 1.2);
        //n_local.param("vy_scale", vy_scale_, 0.6);
        n_local.param("vw_scale", vw_scale_, 10.0); // needs much larger scaling factor, for rotation

        n_local.param("arm_x_scale", arm_x_scale_, 5.15); // .15
        n_local.param("arm_y_scale", arm_y_scale_, 5.15); // .15
        n_local.param("arm_z_scale", arm_z_scale_, 5.15); // .15

        n_local.param("wrist_velocity",wrist_velocity_, 0.7); // 4.5

        //n_local.param("walk_along_x_speed_scale", walk_along_x_speed_scale_, 9.0);
        //n_local.param("walk_along_y_speed_scale", walk_along_y_speed_scale_, 9.0);
        //n_local.param("walk_along_w_speed_scale", walk_along_w_speed_scale_, 9.0);
        //n_local.param("walk_along_thresh", walk_along_thresh_, .015);
        //n_local.param("walk_along_x_dist_max", walk_along_x_dist_max_, .5);
        //n_local.param("walk_along_y_dist_max", walk_along_y_dist_max_, .5);

        //n_local.param("prosilica_namespace", prosilica_namespace_, std::string("prosilica_polled"));

        //bool control_prosilica;
        //n_local.param("control_prosilica", control_prosilica, true);

        bool control_body;
        n_local.param("control_body", control_body, true);

        bool control_larm;
        n_local.param("control_larm", control_larm, true);

        bool control_rarm;
        n_local.param("control_rarm", control_rarm, true);

        bool control_head;
        n_local.param("control_head", control_head, true);

        std::string arm_controller_name;
        n_local.param("arm_controller_name", arm_controller_name,std::string(""));

        //ROS_DEBUG("tilt scale: %.3f rad\n", tilt_scale_);
        //ROS_DEBUG("pan scale: %.3f rad\n", pan_scale_);

        ROS_INFO("Initializing general commander");

        if(arm_controller_name.empty())
        {
            gc = new GeneralCommander(control_body,
                                      control_head,
                                      control_rarm,
                                      control_larm
                                      //,control_prosilica
                                     );
        }
        else
        {
            gc = new GeneralCommander(control_body,
                                      control_head,
                                      control_rarm,
                                      control_larm,
                                      //control_prosilica,
                                      arm_controller_name
                                     );
        }
        first_callback_ = true;

        head_init_ = false;
        torso_init_ = false;

        left_arm_init_ = false;
        right_arm_init_ = false;

        //walk_along_init_waiting_ = false;
        //set_walk_along_mode_ = false;

        joy_sub_ = n_.subscribe("joy", 10, &ReemTeleopGeneralJoystick::joy_cb, this);
    }

    ~ReemTeleopGeneralJoystick()
    {
        if(gc != NULL)
        {
            delete gc;
        }
    }

    bool buttonOkAndOn(unsigned int buttonNum, const sensor_msgs::Joy::ConstPtr& joy_msg) const
    {
        if(buttonNum >= joy_msg->buttons.size()) return false;
        return(joy_msg->buttons[buttonNum]);
    }

    bool axisOk(unsigned int axisNum, const sensor_msgs::Joy::ConstPtr& joy_msg) const
    {
        return (axisNum < joy_msg->axes.size());
    }

    bool sameValueAsLast(unsigned int button,
                         const sensor_msgs::Joy::ConstPtr& new_msg,
                         const sensor_msgs::Joy::ConstPtr& old_msg)
    {
        return (buttonOkAndOn(button, new_msg) == buttonOkAndOn(button, old_msg));
    }

    void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        if(first_callback_)
        {
            last_joy_ = joy_msg;
            first_callback_ = false;
        }

        JoystickLayoutMode layout;

        if(buttonOkAndOn(BODY_LAYOUT_BUTTON, joy_msg))
        {
            layout = LAYOUT_BODY;
            //ROS_INFO("LAYOUT_BODY");
        }
        else if (buttonOkAndOn(RIGHT_ARM_LAYOUT_BUTTON, joy_msg) && buttonOkAndOn(LEFT_ARM_LAYOUT_BUTTON, joy_msg))
        {
            layout = LAYOUT_BOTH_ARMS;
            //ROS_INFO("LAYOUT_BOTH_ARMS");
        }
        else if (buttonOkAndOn(RIGHT_ARM_LAYOUT_BUTTON,joy_msg))
        {
            layout = LAYOUT_RIGHT_ARM;
            //ROS_INFO("LAYOUT_RIGHT_ARM");
        }
        else if (buttonOkAndOn(LEFT_ARM_LAYOUT_BUTTON, joy_msg))
        {
            layout = LAYOUT_LEFT_ARM;
            //ROS_INFO("LAYOUT_LEFT_ARM");
        }
        else if (buttonOkAndOn(HEAD_LAYOUT_BUTTON, joy_msg))
        {
            layout = LAYOUT_HEAD;
            //ROS_INFO("LAYOUT_HEAD");
        }
        else
        {
            layout = LAYOUT_NONE;
            //ROS_INFO("LAYOUT_NONE");
        }

        //bool setting_walk_along_this_cycle_ = false;

        /*
            if(layout == LAYOUT_HEAD) {
              if(buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
                ros::Time now = ros::Time::now();
                if(now-last_walk_along_time_ < DOUBLE_TAP_TIMEOUT) {
                  //only matters if this is off
                  if(!gc->isWalkAlongOk()) {
                    set_walk_along_mode_ = true;
                    setting_walk_along_this_cycle_ = true;
                  }
                }
                if(gc->isWalkAlongOk()) {
                  gc->turnOffWalkAlong();
                  ROS_INFO("Turning off walk along");
                } else {
                  last_walk_along_time_ = now;
                }
              }
            }

            bool in_walk_along = false;
            if(gc->isWalkAlongOk()) {
              if(buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        	gc->turnOffWalkAlong();
        	ROS_INFO("Turning off walk along");
              } else {
        	vel_val_pan_ = 0.0;
        	vel_val_tilt_ = 0.0;
        	des_torso_vel_ = 0.0;
        	des_vx_ = 0.0;
        	des_vy_ = 0.0;
        	des_vw_ = 0.0;
        	des_right_wrist_vel_ = 0.0;
        	right_arm_vx_ = 0.0;
        	right_arm_vy_ = 0.0;
        	right_arm_vz_ = 0.0;
        	des_left_wrist_vel_ = 0.0;
        	left_arm_vx_ = 0.0;
        	left_arm_vy_ = 0.0;
        	left_arm_vz_ = 0.0;
        	in_walk_along = true;
              }
            }

            //we must be moving the arms into the mode
            if(!in_walk_along && layout == LAYOUT_HEAD && set_walk_along_mode_) {
              if(buttonOkAndOn(SET_WALK_ALONG_BUTTON, joy_msg)
        	 && !sameValueAsLast(SET_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
                gc->sendHeadSequence(GeneralCommander::HEAD_SHAKE);
              }
              if(!setting_walk_along_this_cycle_ && buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
                set_walk_along_mode_ = false;
                ROS_INFO("No longer waiting for set");
              }
            }

            if(!in_walk_along && layout == LAYOUT_HEAD && walk_along_init_waiting_) {
              if(buttonOkAndOn(SET_WALK_ALONG_BUTTON, joy_msg)
                 && !sameValueAsLast(SET_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
                bool ok = gc->initWalkAlong();
                if(!ok) {
                  gc->sendHeadSequence(GeneralCommander::HEAD_SHAKE);
                } else {
                  gc->sendHeadSequence(GeneralCommander::HEAD_NOD);
                  ROS_INFO("Should be in walk along");
                  walk_along_init_waiting_ = false;
                }
              }
              if(buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
                walk_along_init_waiting_ = false;
                ROS_INFO("No longer waiting for init");
              }
            }
        */

        /*
            if(layout == LAYOUT_RIGHT_ARM || layout == LAYOUT_LEFT_ARM || layout == LAYOUT_BOTH_ARMS) {

              if(buttonOkAndOn(CLOSE_GRIPPER_BUTTON, joy_msg)
        	 && !sameValueAsLast(CLOSE_GRIPPER_BUTTON, joy_msg, last_joy_)) {
                if(layout == LAYOUT_RIGHT_ARM) {
                  gc->sendGripperCommand(GeneralCommander::ARMS_RIGHT, false);
                } else if(layout == LAYOUT_LEFT_ARM) {
                  gc->sendGripperCommand(GeneralCommander::ARMS_LEFT, false);
                } else {
                  gc->sendGripperCommand(GeneralCommander::ARMS_BOTH, false);
                }
              }
              if(buttonOkAndOn(OPEN_GRIPPER_BUTTON, joy_msg)
        	 && !sameValueAsLast(OPEN_GRIPPER_BUTTON, joy_msg, last_joy_)) {
                if(layout == LAYOUT_RIGHT_ARM) {
                  gc->sendGripperCommand(GeneralCommander::ARMS_RIGHT, true);
                } else if(layout == LAYOUT_LEFT_ARM) {
                  gc->sendGripperCommand(GeneralCommander::ARMS_LEFT, true);
                } else {
                  gc->sendGripperCommand(GeneralCommander::ARMS_BOTH, true);
                }
              }
            }
        */

        //if(!in_walk_along && layout == LAYOUT_HEAD) {
        if(layout == LAYOUT_HEAD)
        {

            //if(buttonOkAndOn(PROJECTOR_TOGGLE_BUTTON, joy_msg)
            // && !sameValueAsLast(PROJECTOR_TOGGLE_BUTTON,joy_msg, last_joy_)) {
            //proj_toggle_com_ = !proj_toggle_com_;
            //gc->sendProjectorStartStop(proj_toggle_com_);
            //}

            //if(buttonOkAndOn(PROSILICA_POLL_BUTTON, joy_msg)
            // && !sameValueAsLast(PROSILICA_POLL_BUTTON,joy_msg, last_joy_)) {
            //gc->requestProsilicaImage(prosilica_namespace_);
            //}

            if(buttonOkAndOn(HEAD_MODE_TOGGLE_BUTTON, joy_msg)
                    && !sameValueAsLast(HEAD_MODE_TOGGLE_BUTTON, joy_msg, last_joy_))
            {

                // Only one single head mode at the moment

                //if(gc->getHeadMode() == GeneralCommander::HEAD_JOYSTICK) {
                //  ROS_DEBUG("Head mode to left");
                //  gc->setHeadMode(GeneralCommander::HEAD_TRACK_LEFT_HAND);
                //} else if(gc->getHeadMode() == GeneralCommander::HEAD_TRACK_LEFT_HAND) {
                //  gc->setHeadMode(GeneralCommander::HEAD_TRACK_RIGHT_HAND);
                //  ROS_DEBUG("Head mode to right");
                //} else if(gc->getHeadMode() == GeneralCommander::HEAD_TRACK_RIGHT_HAND){
                //  gc->setHeadMode(GeneralCommander::HEAD_MANNEQUIN);
                //  ROS_DEBUG("Head mode to mannequin");
                //} else {
                //  ROS_DEBUG("Head mode to joystick");
                //  head_init_ = false;
                //  gc->setHeadMode(GeneralCommander::HEAD_JOYSTICK);
                //}
            }

            if(gc->getHeadMode() == GeneralCommander::HEAD_JOYSTICK)
            {
                //ROS_INFO("Mode: HEAD_JOYSTICK ");
            }

            //if(buttonOkAndOn(LASER_TOGGLE_BUTTON, joy_msg)
            // && !sameValueAsLast(LASER_TOGGLE_BUTTON, joy_msg, last_joy_)) {
            //if(gc->getLaserMode() == GeneralCommander::LASER_TILT_OFF) {
            //  gc->setLaserMode(GeneralCommander::LASER_TILT_SLOW);
            //} else if(gc->getLaserMode() == GeneralCommander::LASER_TILT_SLOW) {
            //  gc->setLaserMode(GeneralCommander::LASER_TILT_FAST);
            //} else {
            //  gc->setLaserMode(GeneralCommander::LASER_TILT_OFF);
            //}
            //}

            if(axisOk(HEAD_PAN_AXIS, joy_msg))
            {
                vel_val_pan_ = joy_msg->axes[HEAD_PAN_AXIS] * pan_scale_;
            }

            if(axisOk(HEAD_TILT_AXIS, joy_msg))
            {
                vel_val_tilt_ = joy_msg->axes[HEAD_TILT_AXIS] * tilt_scale_;
            }
        }
        else
        {
            vel_val_pan_ = 0.0;
            vel_val_tilt_ = 0.0;
        }

        //if(!in_walk_along && layout == LAYOUT_BODY) {
        if(layout == LAYOUT_BODY)
        {
            //bool down = buttonOkAndOn(TORSO_DOWN_BUTTON, joy_msg);
            //bool up = buttonOkAndOn(TORSO_UP_BUTTON, joy_msg);

            //ROS_INFO_STREAM("Down is " << down);
            //ROS_INFO_STREAM("Up is " << up);

            //if(down && !up) {
            //  des_torso_vel_ = -torso_step_;
            //} else if(!down && up) {
            //  des_torso_vel_ = torso_step_;
            //} else {
            //	//ROS_INFO_STREAM("Setting des vel to 0.0");
            //  des_torso_vel_ = 0.0;
            //}
            if(axisOk(VX_AXIS, joy_msg))
            {
                des_vx_ = joy_msg->axes[VX_AXIS]*vx_scale_;
            }
            else
            {
                des_vx_ = 0.0;
            }
            // Reem has no lateral movement
            //if(axisOk(VY_AXIS, joy_msg)) {
            //  des_vy_ = joy_msg->axes[VY_AXIS]*vy_scale_;
            //} else {
            //  des_vy_ = 0.0;
            //}
            if(axisOk(VW_AXIS, joy_msg))
            {
                des_vw_ = joy_msg->axes[VW_AXIS]*vw_scale_;
            }
            else
            {
                des_vw_ = 0.0;
            }
        }
        else
        {
            //des_torso_vel_ = 0.0;
            des_vx_ = 0.0;
            //des_vy_ = 0.0;
            des_vw_ = 0.0;
        }

        /*
            if(layout == LAYOUT_RIGHT_ARM) {
              if(buttonOkAndOn(ARM_MODE_TOGGLE_BUTTON, joy_msg) && !sameValueAsLast(ARM_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)) {
        	if(in_walk_along) {
        	  gc->turnOffWalkAlong();
                  ROS_INFO("Turning off walk along");
        	}
                if(gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_POSITION_CONTROL) {
                  gc->setArmMode(GeneralCommander::ARMS_RIGHT,GeneralCommander::ARM_MANNEQUIN_MODE);
                } else if (gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_MANNEQUIN_MODE) {
                  gc->setArmMode(GeneralCommander::ARMS_RIGHT,GeneralCommander::ARM_NO_CONTROLLER);
                } else {
                  gc->setArmMode(GeneralCommander::ARMS_RIGHT,GeneralCommander::ARM_POSITION_CONTROL);
                }
              }

              if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_TUCK_BUTTON, joy_msg, last_joy_)) {
                if(in_walk_along) {
        	  gc->turnOffWalkAlong();
                  ROS_INFO("Turning off walk along");
        	}
                gc->tuckArms(GeneralCommander::ARMS_RIGHT);
              } else if(buttonOkAndOn(ARM_UNTUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_UNTUCK_BUTTON, joy_msg, last_joy_)) {
                if(in_walk_along) {
        	  gc->turnOffWalkAlong();
                  ROS_INFO("Turning off walk along");
        	}
                gc->untuckArms(GeneralCommander::ARMS_RIGHT);
              }

              if(!in_walk_along) {

                bool lookAnalog = false;
                bool rotClock = buttonOkAndOn(WRIST_CLOCKWISE_BUTTON, joy_msg);
                bool rotCounter = buttonOkAndOn(WRIST_COUNTER_BUTTON, joy_msg);
                if(rotClock && !rotCounter) {
                  des_right_wrist_vel_ = wrist_velocity_;
                } else if(!rotClock && rotCounter) {
                  des_right_wrist_vel_ = -wrist_velocity_;
                } else {
                  des_right_wrist_vel_ = 0.0;
                  lookAnalog = true;
                }

                if(lookAnalog) {
                  //look at analog sticks if we aren't supposed to wrist rotate
                  if(axisOk(ARM_X_AXIS, joy_msg)) {
                    right_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
                  } else {
                    right_arm_vx_ = 0.0;
                  }
                  if(axisOk(ARM_Y_AXIS, joy_msg)) {
                    right_arm_vy_ = joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
                  } else {
                    right_arm_vy_ = 0.0;
                  }
                  if(axisOk(ARM_Z_AXIS, joy_msg)) {
                    right_arm_vz_ = joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
                  } else {
                    right_arm_vz_ = 0.0;
                  }
                  //ROS_INFO_STREAM("Setting vx " << right_arm_vx_ << " " << right_arm_vy_ << " " << right_arm_vz_);
                } else {
                  right_arm_vx_ = 0.0;
                  right_arm_vy_ = 0.0;
                  right_arm_vz_ = 0.0;
                }
              }
            } else if (layout != LAYOUT_BOTH_ARMS) {
              des_right_wrist_vel_ = 0.0;
              right_arm_vx_ = 0.0;
              right_arm_vy_ = 0.0;
              right_arm_vz_ = 0.0;
            }
        */

        if(layout == LAYOUT_LEFT_ARM)
        {
            if(buttonOkAndOn(ARM_MODE_TOGGLE_BUTTON, joy_msg) && !sameValueAsLast(ARM_MODE_TOGGLE_BUTTON, joy_msg, last_joy_))
            {

                //if(in_walk_along) {
                //  gc->turnOffWalkAlong();
                //  ROS_INFO("Turning off walk along");
                //}

                // Toggle controller modes
                if(gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_JOINT_CONTROL)
                {
                    //if(gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_POSITION_CONTROL) {
                    //gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_MANNEQUIN_MODE);
                    gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_CARTESIAN_CONTROL);

                    //} else if (gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_MANNEQUIN_MODE) {
                    //  gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_NO_CONTROLLER);

                }
                else if (gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_CARTESIAN_CONTROL)
                {
                    gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_NO_CONTROLLER);

                }
                else
                {
                    gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_JOINT_CONTROL);
                }
            }

            //if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_TUCK_BUTTON, joy_msg, last_joy_)) {
            //  if(in_walk_along) {
            //  gc->turnOffWalkAlong();
            //  ROS_INFO("Turning off walk along");
            //}
            //
            //gc->tuckArms(GeneralCommander::ARMS_LEFT);
            //} else if(buttonOkAndOn(ARM_UNTUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_UNTUCK_BUTTON, joy_msg, last_joy_)) {
            //  if(in_walk_along) {
            //  gc->turnOffWalkAlong();
            //  ROS_INFO("Turning off walk along");
            //}
            // gc->untuckArms(GeneralCommander::ARMS_LEFT);
            //}

            if(gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_JOINT_CONTROL)
            {
                // ROS_INFO("Arm Mode: ARM_JOINT_CONTROL ");

                if(axisOk(ARM_JOINT_1_AXIS, joy_msg))
                {
                    left_arm_joint_1_cmd_velocity_ = joy_msg->axes[ARM_JOINT_1_AXIS] * arm_joint_1_scale_;
                    // printf("got joint1  cmd vel %f /n", left_arm_joint_1_cmd_velocity_  );
                }
                if(axisOk(ARM_JOINT_2_AXIS, joy_msg))
                {
                    left_arm_joint_2_cmd_velocity_ = joy_msg->axes[ARM_JOINT_2_AXIS] * arm_joint_2_scale_;
                }
                if(axisOk(ARM_JOINT_3_AXIS, joy_msg))
                {
                    left_arm_joint_3_cmd_velocity_ = joy_msg->axes[ARM_JOINT_3_AXIS] * arm_joint_3_scale_;
                }
                if(axisOk(ARM_JOINT_4_AXIS, joy_msg))
                {
                    left_arm_joint_4_cmd_velocity_ = joy_msg->axes[ARM_JOINT_4_AXIS] * arm_joint_4_scale_;
                }

                if(buttonOkAndOn(ARM_JOINT_5_BUTTON1, joy_msg))
                {
                    left_arm_joint_5_cmd_velocity_ = 1.0 * arm_joint_5_scale_;
                }
                else if(buttonOkAndOn(ARM_JOINT_5_BUTTON2, joy_msg))
                {
                    left_arm_joint_5_cmd_velocity_ = -1.0 * arm_joint_5_scale_;
                }
                else
                {
                    left_arm_joint_5_cmd_velocity_ = 0.0;
                }

                /*
                      if(axisOk(ARM_JOINT_5_AXIS, joy_msg))
                      {
                        left_arm_joint_5_cmd_velocity_ = joy_msg->axes[ARM_JOINT_5_AXIS] * arm_joint_5_scale_;
                      }
                */
                if(axisOk(ARM_JOINT_6_AXIS, joy_msg))
                {
                    left_arm_joint_6_cmd_velocity_ = joy_msg->axes[ARM_JOINT_6_AXIS] * arm_joint_6_scale_;
                }
                if(axisOk(ARM_JOINT_7_AXIS, joy_msg))
                {
                    left_arm_joint_7_cmd_velocity_ = joy_msg->axes[ARM_JOINT_7_AXIS] * arm_joint_7_scale_;
                }

                if(buttonOkAndOn(HAND_OPEN_BUTTON, joy_msg))
                {
                    hand_left_thumb_joint_cmd_velocity_ = -1.0 * thumb_scale_;
                    hand_left_index_1_joint_cmd_velocity_ = -1.0 * fingers_scale_;
                    hand_left_index_2_joint_cmd_velocity_ = -1.0 * fingers_scale_;
                    hand_left_index_3_joint_cmd_velocity_ = -1.0 * fingers_scale_;
                    hand_left_middle_1_joint_cmd_velocity_ = -1.0 * fingers_scale_;
                    hand_left_middle_2_joint_cmd_velocity_ = -1.0 * fingers_scale_;
                    hand_left_middle_3_joint_cmd_velocity_ = -1.0 * fingers_scale_;
                }
                else if(buttonOkAndOn(HAND_CLOSE_BUTTON, joy_msg))
                {
                    hand_left_thumb_joint_cmd_velocity_ = 1.0 * thumb_scale_;
                    hand_left_index_1_joint_cmd_velocity_ = 1.0 * fingers_scale_;
                    hand_left_index_2_joint_cmd_velocity_ = 1.0 * fingers_scale_;
                    hand_left_index_3_joint_cmd_velocity_ = 1.0 * fingers_scale_;
                    hand_left_middle_1_joint_cmd_velocity_ = 1.0 * fingers_scale_;
                    hand_left_middle_2_joint_cmd_velocity_ = 1.0 * fingers_scale_;
                    hand_left_middle_3_joint_cmd_velocity_ = 1.0 * fingers_scale_;
                }
                else
                {
                    hand_left_thumb_joint_cmd_velocity_ = 0.0;
                    hand_left_index_1_joint_cmd_velocity_ = 0.0;
                    hand_left_index_2_joint_cmd_velocity_ = 0.0;
                    hand_left_index_3_joint_cmd_velocity_ = 0.0;
                    hand_left_middle_1_joint_cmd_velocity_ = 0.0;
                    hand_left_middle_2_joint_cmd_velocity_ = 0.0;
                    hand_left_middle_3_joint_cmd_velocity_ = 0.0;
                }

            }
            else if(gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_CARTESIAN_CONTROL)
            {
                // ROS_INFO("Arm Mode: ARM_CARTESIAN_CONTROL ");

                //if(!in_walk_along) {
                bool lookAnalog = false;
                bool rotClock = buttonOkAndOn(WRIST_CLOCKWISE_BUTTON, joy_msg);
                bool rotCounter = buttonOkAndOn(WRIST_COUNTER_BUTTON, joy_msg);
                if(rotClock && !rotCounter)
                {
                    des_left_wrist_vel_ = wrist_velocity_;
                }
                else if(!rotClock && rotCounter)
                {
                    des_left_wrist_vel_ = -wrist_velocity_;
                }
                else
                {
                    des_left_wrist_vel_ = 0.0;
                    lookAnalog = true;
                }

                if(lookAnalog)
                {
                    //look at analog sticks if we aren't supposed to wrist rotate
                    if(axisOk(ARM_X_AXIS, joy_msg))
                    {
                        left_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
                    }
                    else
                    {
                        left_arm_vx_ = 0.0;
                    }
                    if(axisOk(ARM_Y_AXIS, joy_msg))
                    {
                        left_arm_vy_ = joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
                    }
                    else
                    {
                        left_arm_vy_ = 0.0;
                    }
                    if(axisOk(ARM_Z_AXIS, joy_msg))
                    {
                        left_arm_vz_ = joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
                    }
                    else
                    {
                        left_arm_vz_ = 0.0;
                    }
                    //ROS_INFO_STREAM("Setting  vx: " << left_arm_vx_ << "  vy: " << left_arm_vy_ << "  vz: " << left_arm_vz_);
                }
                else
                {
                    left_arm_vx_ = 0.0;
                    left_arm_vy_ = 0.0;
                    left_arm_vz_ = 0.0;
                }
                //} // walk along

            }
            else
            {
                // defaults for ARM_JOINT_CONTROL
                left_arm_joint_1_cmd_velocity_ = 0.0;
                left_arm_joint_2_cmd_velocity_ = 0.0;
                left_arm_joint_3_cmd_velocity_ = 0.0;
                left_arm_joint_4_cmd_velocity_ = 0.0;
                left_arm_joint_5_cmd_velocity_ = 0.0;
                left_arm_joint_6_cmd_velocity_ = 0.0;
                left_arm_joint_7_cmd_velocity_ = 0.0;

            }

        }
        else if (layout != LAYOUT_BOTH_ARMS)
        {
            des_left_wrist_vel_ = 0.0;
            left_arm_vx_ = 0.0;
            left_arm_vy_ = 0.0;
            left_arm_vz_ = 0.0;
        }

        /*
            if(layout == LAYOUT_BOTH_ARMS) {
              if(buttonOkAndOn(ARM_MODE_TOGGLE_BUTTON, joy_msg) && !sameValueAsLast(ARM_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)) {
                GeneralCommander::ArmControlMode toSend;
        	if(in_walk_along) {
                  gc->turnOffWalkAlong();
                  ROS_INFO("Turning off walk along");
                }
                if(gc->getArmMode(GeneralCommander::ARMS_RIGHT) != gc->getArmMode(GeneralCommander::ARMS_RIGHT)) {
                  toSend = GeneralCommander::ARM_POSITION_CONTROL;
                } else if(gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_POSITION_CONTROL) {
                  toSend = GeneralCommander::ARM_MANNEQUIN_MODE;
                } else if (gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_MANNEQUIN_MODE) {
                  toSend = GeneralCommander::ARM_NO_CONTROLLER;
                } else {
                  toSend = GeneralCommander::ARM_POSITION_CONTROL;
                }
                gc->setArmMode(GeneralCommander::ARMS_BOTH, toSend);
              }

              if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_TUCK_BUTTON, joy_msg, last_joy_)) {
                if(in_walk_along) {
                  gc->turnOffWalkAlong();
                  ROS_INFO("Turning off walk along");
                }
                gc->tuckArms(GeneralCommander::ARMS_BOTH);
              } else if(buttonOkAndOn(ARM_UNTUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_UNTUCK_BUTTON, joy_msg, last_joy_)) {
                if(in_walk_along) {
                  gc->turnOffWalkAlong();
                  ROS_INFO("Turning off walk along");
                }
                gc->untuckArms(GeneralCommander::ARMS_BOTH);
              }

              if(!in_walk_along) {
                bool lookAnalog = false;
                bool rotClock = buttonOkAndOn(WRIST_CLOCKWISE_BUTTON, joy_msg);
                bool rotCounter = buttonOkAndOn(WRIST_COUNTER_BUTTON, joy_msg);
                if(rotClock && !rotCounter) {
                  des_left_wrist_vel_ = wrist_velocity_;
                  des_right_wrist_vel_ = wrist_velocity_;
                } else if(!rotClock && rotCounter) {
                  des_left_wrist_vel_ = -wrist_velocity_;
                  des_right_wrist_vel_ = -wrist_velocity_;
                } else {
                  des_left_wrist_vel_ = 0.0;
                  des_right_wrist_vel_ = 0.0;
                  lookAnalog = true;
                }

                if(lookAnalog) {
                  //look at analog sticks if we aren't supposed to wrist rotate
                  if(axisOk(ARM_X_AXIS, joy_msg)) {
                    left_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
                    right_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
                  } else {
                    left_arm_vx_ = 0.0;
                    right_arm_vz_ = 0.0;
                  }
                  if(axisOk(ARM_Y_AXIS, joy_msg)) {
                    left_arm_vy_ = joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
                    right_arm_vy_ = joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
                  } else {
                    left_arm_vy_ = 0.0;
                    right_arm_vz_ = 0.0;
                  }
                  if(axisOk(ARM_Z_AXIS, joy_msg)) {
                    left_arm_vz_ = joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
                    right_arm_vz_ = joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
                  } else {
                    left_arm_vz_ = 0.0;
                    right_arm_vz_ = 0.0;
                  }
                //ROS_INFO_STREAM("Setting vx " << left_arm_vx_ << " " << left_arm_vy_ << " " << left_arm_vz_);
                } else {
                  left_arm_vx_ = 0.0;
                  left_arm_vy_ = 0.0;
                  left_arm_vz_ = 0.0;
                  right_arm_vx_ = 0.0;
                  right_arm_vy_ = 0.0;
                  right_arm_vz_ = 0.0;
                }
              }
            } else if (layout != LAYOUT_RIGHT_ARM && layout != LAYOUT_LEFT_ARM) {
              des_right_wrist_vel_ = 0.0;
              des_left_wrist_vel_ = 0.0;
              left_arm_vx_ = 0.0;
              left_arm_vy_ = 0.0;
              left_arm_vz_ = 0.0;
              right_arm_vx_ = 0.0;
              right_arm_vy_ = 0.0;
              right_arm_vz_ = 0.0;
            }

        */

        joy_deadman_ = ros::Time::now();
        last_joy_ = joy_msg;
    }

    /*
      bool convertCurrentVelToDesiredTorsoPos(double hz) {
        if(!torso_init_)  {
          double cur_torso_pos = 0.0;
          bool ok = gc->getJointPosition("torso_lift_joint", cur_torso_pos);
          if(!ok) return false;
          des_torso_pos_ = cur_torso_pos;
          torso_init_ = true;
        }
        double dt = 1.0/double(hz);
        double horizon = dt*5.0;
        double cur_torso_pos = 0.0;
        gc->getJointPosition("torso_lift_joint", cur_torso_pos);
        des_torso_pos_ = cur_torso_pos+des_torso_vel_ * horizon;
        des_torso_pos_ = std::min(des_torso_pos_, max_torso_);
        des_torso_pos_ = std::max(des_torso_pos_, min_torso_);
        return true;
      }
    */

    bool convertCurrentVelToDesiredHeadPos(double hz)
    {

        // First time we call this, get the current position of joints, so we dont have to re-home the robot all the time!!
        if(!head_init_)
        {
            //ROS_INFO("in not headinit");
            double cur_pan_pos = 0.0;
            double cur_tilt_pos = 0.0;
            // ADD COMPILER DEFINER FOR HEAD JOINT NAMES,  theyre used in other code file too
            bool ok;
            ok = gc->getJointPosition("head_1_joint", cur_pan_pos); // for PR2 was  head_pan_joint
            if(!ok) return false;
            //printf("get pan position OK, position = %f ", cur_pan_pos );

            ok = gc->getJointPosition("head_2_joint", cur_tilt_pos); // for PR2 was head_tilt_joint
            if(!ok) return false;
            //printf("get tilt position OK, position = %f ", cur_tilt_pos );

            des_pan_pos_ = cur_pan_pos;
            des_tilt_pos_ = cur_tilt_pos;

            head_init_ = true;
        }
        if(fabs(vel_val_pan_) > .0001)
        {
            des_pan_pos_ = des_pan_pos_ + vel_val_pan_*(1.0/hz);
            des_pan_pos_ = std::min(des_pan_pos_, max_pan_);
            des_pan_pos_ = std::max(des_pan_pos_, -max_pan_);
            //ROS_INFO("4");
        }
        if(fabs(vel_val_tilt_) > .0001)
        {
            des_tilt_pos_ = des_tilt_pos_ - vel_val_tilt_*(1.0/hz);
            des_tilt_pos_ = std::min(des_tilt_pos_, max_tilt_);
            des_tilt_pos_ = std::max(des_tilt_pos_, min_tilt_);
        }
        //ROS_INFO("5");
        //    ROS_INFO_STREAM("Des pan pos " << des_pan_pos_ << " tilt " << des_tilt_pos_);
        return true;
    }

    bool convertCurrentVelToDesiredLeftArmJointPos(double hz)
    {
        if(!left_arm_init_)
        {
            //double cur_pan_pos = 0.0;
            //double cur_tilt_pos = 0.0;

            bool ok;

            double left_arm_joint_1_cur_pos = 0.0;
            double left_arm_joint_2_cur_pos = 0.0;
            double left_arm_joint_3_cur_pos = 0.0;
            double left_arm_joint_4_cur_pos = 0.0;
            double left_arm_joint_5_cur_pos = 0.0;
            double left_arm_joint_6_cur_pos = 0.0;
            double left_arm_joint_7_cur_pos = 0.0;
            ok = gc->getJointPosition("arm_left_1_joint", left_arm_joint_1_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("arm_left_2_joint", left_arm_joint_2_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("arm_left_3_joint", left_arm_joint_3_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("arm_left_4_joint", left_arm_joint_4_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("arm_left_5_joint", left_arm_joint_5_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("arm_left_6_joint", left_arm_joint_6_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("arm_left_7_joint", left_arm_joint_7_cur_pos);
            if(!ok) return false;
            left_arm_joint_1_des_position_ = left_arm_joint_1_cur_pos;
            left_arm_joint_2_des_position_ = left_arm_joint_2_cur_pos;
            left_arm_joint_3_des_position_ = left_arm_joint_3_cur_pos;
            left_arm_joint_4_des_position_ = left_arm_joint_4_cur_pos;
            left_arm_joint_5_des_position_ = left_arm_joint_5_cur_pos;
            left_arm_joint_6_des_position_ = left_arm_joint_6_cur_pos;
            left_arm_joint_7_des_position_ = left_arm_joint_7_cur_pos;

            // SHOULD RENAME TO LEFT HAND INDEX JOINT 1
            double hand_left_thumb_joint_cur_pos = 0.0;
            double hand_left_index_1_joint_cur_pos = 0.0;
            double hand_left_index_2_joint_cur_pos = 0.0;
            double hand_left_index_3_joint_cur_pos = 0.0;
            double hand_left_middle_1_joint_cur_pos = 0.0;
            double hand_left_middle_2_joint_cur_pos = 0.0;
            double hand_left_middle_3_joint_cur_pos = 0.0;
            ok = gc->getJointPosition("hand_left_thumb_joint", hand_left_thumb_joint_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("hand_left_index_1_joint", hand_left_index_1_joint_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("hand_left_index_2_joint", hand_left_index_2_joint_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("hand_left_index_3_joint", hand_left_index_3_joint_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("hand_left_middle_1_joint", hand_left_middle_1_joint_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("hand_left_middle_2_joint", hand_left_middle_2_joint_cur_pos);
            if(!ok) return false;
            ok = gc->getJointPosition("hand_left_middle_3_joint", hand_left_middle_3_joint_cur_pos);
            if(!ok) return false;
            double hand_left_thumb_joint_des_position_ = hand_left_thumb_joint_cur_pos;
            double hand_left_index_1_joint_des_position_ = hand_left_index_1_joint_cur_pos;
            double hand_left_index_2_joint_des_position_ = hand_left_index_2_joint_cur_pos;
            double hand_left_index_3_joint_des_position_ = hand_left_index_3_joint_cur_pos;
            double hand_left_middle_1_joint_des_position_ = hand_left_middle_1_joint_cur_pos;
            double hand_left_middle_2_joint_des_position_ = hand_left_middle_2_joint_cur_pos;
            double hand_left_middle_3_joint_des_position_ = hand_left_middle_3_joint_cur_pos;

            left_arm_init_ = true;
        }
        // ADD BACK LIMIT CHECKING
        /*
        if(fabs(vel_val_pan_) > .0001) {
          des_pan_pos_ = des_pan_pos_ + vel_val_pan_*(1.0/hz);
          des_pan_pos_ = std::min(des_pan_pos_, max_pan_);
          des_pan_pos_ = std::max(des_pan_pos_, -max_pan_);
          //ROS_INFO("4");
        }
        */

        if(fabs(left_arm_joint_1_cmd_velocity_) > .0001)
        {
            left_arm_joint_1_des_position_ = left_arm_joint_1_des_position_ + left_arm_joint_1_cmd_velocity_*(1.0/hz);
            left_arm_joint_1_des_position_ = std::min(left_arm_joint_1_des_position_, arm_joint_1_max_);
            left_arm_joint_1_des_position_ = std::max(left_arm_joint_1_des_position_, arm_joint_1_min_);
        }
        if(fabs(left_arm_joint_2_cmd_velocity_) > .0001)
        {
            left_arm_joint_2_des_position_ = left_arm_joint_2_des_position_ + left_arm_joint_2_cmd_velocity_*(1.0/hz);
            left_arm_joint_2_des_position_ = std::min(left_arm_joint_2_des_position_, arm_joint_2_max_);
            left_arm_joint_2_des_position_ = std::max(left_arm_joint_2_des_position_, arm_joint_2_min_);
        }
        if(fabs(left_arm_joint_3_cmd_velocity_) > .0001)
        {
            left_arm_joint_3_des_position_ = left_arm_joint_3_des_position_ + left_arm_joint_3_cmd_velocity_*(1.0/hz);
            left_arm_joint_3_des_position_ = std::min(left_arm_joint_3_des_position_, arm_joint_3_max_);
            left_arm_joint_3_des_position_ = std::max(left_arm_joint_3_des_position_, arm_joint_3_min_);
        }
        if(fabs(left_arm_joint_4_cmd_velocity_) > .0001)
        {
            left_arm_joint_4_des_position_ = left_arm_joint_4_des_position_ + left_arm_joint_4_cmd_velocity_*(1.0/hz);
            left_arm_joint_4_des_position_ = std::min(left_arm_joint_4_des_position_, arm_joint_4_max_);
            left_arm_joint_4_des_position_ = std::max(left_arm_joint_4_des_position_, arm_joint_4_min_);
        }
        if(fabs(left_arm_joint_5_cmd_velocity_) > .0001)
        {
            left_arm_joint_5_des_position_ = left_arm_joint_5_des_position_ + left_arm_joint_5_cmd_velocity_*(1.0/hz);
            left_arm_joint_5_des_position_ = std::min(left_arm_joint_5_des_position_, arm_joint_5_max_);
            left_arm_joint_5_des_position_ = std::max(left_arm_joint_5_des_position_, arm_joint_5_min_);
        }
        if(fabs(left_arm_joint_6_cmd_velocity_) > .0001)
        {
            left_arm_joint_6_des_position_ = left_arm_joint_6_des_position_ + left_arm_joint_6_cmd_velocity_*(1.0/hz);
            left_arm_joint_6_des_position_ = std::min(left_arm_joint_6_des_position_, arm_joint_6_max_);
            left_arm_joint_6_des_position_ = std::max(left_arm_joint_6_des_position_, arm_joint_6_min_);
        }
        if(fabs(left_arm_joint_7_cmd_velocity_) > .0001)
        {
            left_arm_joint_7_des_position_ = left_arm_joint_7_des_position_ + left_arm_joint_7_cmd_velocity_*(1.0/hz);
            left_arm_joint_7_des_position_ = std::min(left_arm_joint_7_des_position_, arm_joint_7_max_);
            left_arm_joint_7_des_position_ = std::max(left_arm_joint_7_des_position_, arm_joint_7_min_);
        }



        if(fabs(hand_left_thumb_joint_cmd_velocity_) > .0001)
        {
            hand_left_thumb_joint_des_position_ = hand_left_thumb_joint_des_position_ + hand_left_thumb_joint_cmd_velocity_*(1.0/hz);
            hand_left_thumb_joint_des_position_ = std::min(hand_left_thumb_joint_des_position_, hand_left_thumb_joint_max_);
            hand_left_thumb_joint_des_position_ = std::max(hand_left_thumb_joint_des_position_, hand_left_thumb_joint_min_);
        }
        if(fabs(hand_left_index_1_joint_cmd_velocity_) > .0001)
        {
            hand_left_index_1_joint_des_position_ = hand_left_index_1_joint_des_position_ + hand_left_index_1_joint_cmd_velocity_*(1.0/hz);
            hand_left_index_1_joint_des_position_ = std::min(hand_left_index_1_joint_des_position_, hand_left_index_1_joint_max_);
            hand_left_index_1_joint_des_position_ = std::max(hand_left_index_1_joint_des_position_, hand_left_index_1_joint_min_);
        }
        if(fabs(hand_left_index_2_joint_cmd_velocity_) > .0001)
        {
            hand_left_index_2_joint_des_position_ = hand_left_index_2_joint_des_position_ + hand_left_index_2_joint_cmd_velocity_*(1.0/hz);
            hand_left_index_2_joint_des_position_ = std::min(hand_left_index_2_joint_des_position_, hand_left_index_2_joint_max_);
            hand_left_index_2_joint_des_position_ = std::max(hand_left_index_2_joint_des_position_, hand_left_index_2_joint_min_);
        }
        if(fabs(hand_left_index_3_joint_cmd_velocity_) > .0001)
        {
            hand_left_index_3_joint_des_position_ = hand_left_index_3_joint_des_position_ + hand_left_index_3_joint_cmd_velocity_*(1.0/hz);
            hand_left_index_3_joint_des_position_ = std::min(hand_left_index_3_joint_des_position_, hand_left_index_3_joint_max_);
            hand_left_index_3_joint_des_position_ = std::max(hand_left_index_3_joint_des_position_, hand_left_index_3_joint_min_);
        }
        if(fabs(hand_left_middle_1_joint_cmd_velocity_) > .0001)
        {
            hand_left_middle_1_joint_des_position_ = hand_left_middle_1_joint_des_position_ + hand_left_middle_1_joint_cmd_velocity_*(1.0/hz);
            hand_left_middle_1_joint_des_position_ = std::min(hand_left_middle_1_joint_des_position_, hand_left_middle_1_joint_max_);
            hand_left_middle_1_joint_des_position_ = std::max(hand_left_middle_1_joint_des_position_, hand_left_middle_1_joint_min_);
        }
        if(fabs(hand_left_middle_2_joint_cmd_velocity_) > .0001)
        {
            hand_left_middle_2_joint_des_position_ = hand_left_middle_2_joint_des_position_ + hand_left_middle_2_joint_cmd_velocity_*(1.0/hz);
            hand_left_middle_2_joint_des_position_ = std::min(hand_left_middle_2_joint_des_position_, hand_left_middle_2_joint_max_);
            hand_left_middle_2_joint_des_position_ = std::max(hand_left_middle_2_joint_des_position_, hand_left_middle_2_joint_min_);
        }
        if(fabs(hand_left_middle_3_joint_cmd_velocity_) > .0001)
        {
            hand_left_middle_3_joint_des_position_ = hand_left_middle_3_joint_des_position_ + hand_left_middle_3_joint_cmd_velocity_*(1.0/hz);
            hand_left_middle_3_joint_des_position_ = std::min(hand_left_middle_3_joint_des_position_, hand_left_middle_3_joint_max_);
            hand_left_middle_3_joint_des_position_ = std::max(hand_left_middle_3_joint_des_position_, hand_left_middle_3_joint_min_);
        }

        //ROS_INFO("5");
        /*
              ROS_INFO_STREAM("Desired angles:   joint1: " << left_arm_joint_1_des_position_ <<
                                              "  joint2: " << left_arm_joint_2_des_position_ <<
                                              "  joint3: " << left_arm_joint_3_des_position_ <<
                                              "  joint4: " << left_arm_joint_4_des_position_ <<
                                              "  joint5: " << left_arm_joint_5_des_position_ <<
                                              "  joint6: " << left_arm_joint_6_des_position_ <<
                                              "  joint7: " << left_arm_joint_7_des_position_
                                              );
        */

        return true;
    }

public:
    // head  SHOULD BETTER NAME THESE
    double max_pan_, max_tilt_, min_tilt_;
    int axis_pan_, axis_tilt_;
    double pan_scale_, tilt_scale_;

    double des_pan_pos_;
    double des_tilt_pos_;

    double vel_val_pan_;
    double vel_val_tilt_;

    // base
    double des_vx_;
    //double des_vy_;
    double des_vw_;

    double vx_scale_;
    //double vy_scale_;
    double vw_scale_;

    double arm_x_scale_;
    double arm_y_scale_;
    double arm_z_scale_;

    double right_arm_vx_;
    double right_arm_vy_;
    double right_arm_vz_;

    double left_arm_vx_;
    double left_arm_vy_;
    double left_arm_vz_;

    // for arm joint control
    double left_arm_joint_1_cmd_velocity_;
    double left_arm_joint_2_cmd_velocity_;
    double left_arm_joint_3_cmd_velocity_;
    double left_arm_joint_4_cmd_velocity_;
    double left_arm_joint_5_cmd_velocity_;
    double left_arm_joint_6_cmd_velocity_;
    double left_arm_joint_7_cmd_velocity_;

    double left_arm_joint_1_des_position_;
    double left_arm_joint_2_des_position_;
    double left_arm_joint_3_des_position_;
    double left_arm_joint_4_des_position_;
    double left_arm_joint_5_des_position_;
    double left_arm_joint_6_des_position_;
    double left_arm_joint_7_des_position_;

    double arm_joint_1_scale_;
    double arm_joint_2_scale_;
    double arm_joint_3_scale_;
    double arm_joint_4_scale_;
    double arm_joint_5_scale_;
    double arm_joint_6_scale_;
    double arm_joint_7_scale_;

    double arm_joint_1_min_;
    double arm_joint_1_max_;
    double arm_joint_2_min_;
    double arm_joint_2_max_;
    double arm_joint_3_min_;
    double arm_joint_3_max_;
    double arm_joint_4_min_;
    double arm_joint_4_max_;
    double arm_joint_5_min_;
    double arm_joint_5_max_;
    double arm_joint_6_min_;
    double arm_joint_6_max_;
    double arm_joint_7_min_;
    double arm_joint_7_max_;

    double hand_left_thumb_joint_cmd_velocity_;
    double hand_left_index_1_joint_cmd_velocity_;
    double hand_left_index_2_joint_cmd_velocity_;
    double hand_left_index_3_joint_cmd_velocity_;
    double hand_left_middle_1_joint_cmd_velocity_;
    double hand_left_middle_2_joint_cmd_velocity_;
    double hand_left_middle_3_joint_cmd_velocity_;

    double hand_left_thumb_joint_des_position_;
    double hand_left_index_1_joint_des_position_;
    double hand_left_index_2_joint_des_position_;
    double hand_left_index_3_joint_des_position_;
    double hand_left_middle_1_joint_des_position_;
    double hand_left_middle_2_joint_des_position_;
    double hand_left_middle_3_joint_des_position_;

    double hand_left_thumb_joint_min_;
    double hand_left_thumb_joint_max_;

    double hand_left_index_1_joint_min_;
    double hand_left_index_1_joint_max_;

    double hand_left_index_2_joint_min_;
    double hand_left_index_2_joint_max_;

    double hand_left_index_3_joint_min_;
    double hand_left_index_3_joint_max_;

    double hand_left_middle_1_joint_min_;
    double hand_left_middle_1_joint_max_;

    double hand_left_middle_2_joint_min_;
    double hand_left_middle_2_joint_max_;

    double hand_left_middle_3_joint_min_;
    double hand_left_middle_3_joint_max_;

    double fingers_scale_;
    double thumb_scale_;

    bool left_arm_init_;
    bool right_arm_init_;
    bool head_init_;
    bool torso_init_;

    double req_torso_vel_;
    double req_torso_pos_;

    double des_torso_pos_;
    double des_torso_vel_;
    double torso_step_;
    double min_torso_;
    double max_torso_;

    double wrist_velocity_;
    double des_right_wrist_vel_;
    double des_left_wrist_vel_;

    double walk_along_x_speed_scale_;
    double walk_along_y_speed_scale_;
    double walk_along_w_speed_scale_;
    double walk_along_thresh_;
    double walk_along_x_dist_max_;
    double walk_along_y_dist_max_;

    bool walk_along_init_waiting_;
    bool set_walk_along_mode_;

    std::string prosilica_namespace_;

    bool proj_toggle_com_;

    int projector_toggle_button_;
    int tilt_toggle_button_;
    int switch_head_control_mode_button_;

    GeneralCommander* gc;

    ros::Time joy_deadman_;

    sensor_msgs::JoyConstPtr last_joy_;
    bool first_callback_;

    ros::NodeHandle n_;
    ros::Subscriber joy_sub_;

    ros::Time last_projector_toggle_;
    ros::Time last_laser_toggle_;
    ros::Time last_head_toggle_;

    ros::Time last_walk_along_time_;

};

static const double FastHz = 100;
static const double SlowHz = 20;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reem_telep_general_joystick", ros::init_options::NoSigintHandler);
    //signal(SIGINT,quit);

    //boost::thread spin_thread(boost::bind(&spin_function));

    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();

    ReemTeleopGeneralJoystick generaljoy;
    generaljoy.init();

    ros::Rate pub_rate(FastHz);

    //  unsigned int counter_limit = (unsigned int)(FastHz/SlowHz); // 100/20
    unsigned int counter_limit = 20;

    unsigned int counter = 0;

    ros::Time beforeCall = ros::Time::now();
    ros::Time afterCall = ros::Time::now();
    while (ros::ok())
    {
        //ROS_INFO_STREAM("Time since last " << (ros::Time::now()-beforeCall).toSec());
        beforeCall = ros::Time::now();

        //if(!generaljoy.gc->isWalkAlongOk() && !generaljoy.set_walk_along_mode_ && !generaljoy.walk_along_init_waiting_) {
        if(generaljoy.convertCurrentVelToDesiredHeadPos(FastHz))
        {
            generaljoy.gc->sendHeadCommand(generaljoy.des_pan_pos_, generaljoy.des_tilt_pos_);
        }
        //generaljoy.gc->sendHeadTrackCommand();
        //generaljoy.gc->sendBaseCommand(generaljoy.des_vx_, generaljoy.des_vy_, generaljoy.des_vw_);

        // For Reem, vy is always zero, no lateral movement
        generaljoy.gc->sendBaseCommand(generaljoy.des_vx_, 0.0, generaljoy.des_vw_);
        //}

        if(generaljoy.convertCurrentVelToDesiredLeftArmJointPos(FastHz))
        {
            generaljoy.gc->sendArmPositionCommands(generaljoy.left_arm_joint_1_des_position_,
                                                   generaljoy.left_arm_joint_2_des_position_,
                                                   generaljoy.left_arm_joint_3_des_position_,
                                                   generaljoy.left_arm_joint_4_des_position_,
                                                   generaljoy.left_arm_joint_5_des_position_,
                                                   generaljoy.left_arm_joint_6_des_position_,
                                                   generaljoy.left_arm_joint_7_des_position_
                                                   ,
                                                   generaljoy.hand_left_thumb_joint_des_position_,
                                                   generaljoy.hand_left_index_1_joint_des_position_,
                                                   generaljoy.hand_left_index_2_joint_des_position_,
                                                   generaljoy.hand_left_index_3_joint_des_position_,
                                                   generaljoy.hand_left_middle_1_joint_des_position_,
                                                   generaljoy.hand_left_middle_2_joint_des_position_,
                                                   generaljoy.hand_left_middle_3_joint_des_position_
                                                  );
        }

        if((counter % counter_limit) == 0)
        {
            counter = 0;

            generaljoy.gc->updateCurrentWristPositions();
            generaljoy.gc->sendWristVelCommands(generaljoy.des_right_wrist_vel_, generaljoy.des_left_wrist_vel_, SlowHz);

            generaljoy.gc->sendArmVelCommands(generaljoy.right_arm_vx_, generaljoy.right_arm_vy_, generaljoy.right_arm_vz_, 0.0,
                                              generaljoy.left_arm_vx_, generaljoy.left_arm_vy_, generaljoy.left_arm_vz_, 0.0,
                                              SlowHz);

            //if(generaljoy.set_walk_along_mode_) {
            //  bool ok = generaljoy.gc->moveToWalkAlongArmPose();
            //if we didn't get a select while moving the arms
            //  if(ok && generaljoy.set_walk_along_mode_) {
            //    ROS_INFO("Arms in walk position");
            //    generaljoy.walk_along_init_waiting_ = true;
            //  } else {
            //    ROS_INFO("Arms not in walk position");
            //  }
            //  generaljoy.set_walk_along_mode_ = false;
            //}

            //if(generaljoy.gc->isWalkAlongOk()) {

            //  generaljoy.gc->sendWalkAlongCommand(generaljoy.walk_along_thresh_,
            //                                      generaljoy.walk_along_x_dist_max_,
            //                                      generaljoy.walk_along_x_speed_scale_,
            //                                      generaljoy.walk_along_y_dist_max_,
            //                                      generaljoy.walk_along_y_speed_scale_,
            //                                      generaljoy.walk_along_w_speed_scale_);
            //} else {
            //  if(generaljoy.convertCurrentVelToDesiredTorsoPos(SlowHz)) {
            //    generaljoy.gc->sendTorsoCommand(generaljoy.des_torso_pos_, generaljoy.des_torso_vel_);
            //  }

            /*
                    generaljoy.gc->updateCurrentWristPositions();
                    generaljoy.gc->sendWristVelCommands(generaljoy.des_right_wrist_vel_, generaljoy.des_left_wrist_vel_, SlowHz);

                    generaljoy.gc->sendArmVelCommands(generaljoy.right_arm_vx_, generaljoy.right_arm_vy_, generaljoy.right_arm_vz_, 0.0,
                                                      generaljoy.left_arm_vx_, generaljoy.left_arm_vy_, generaljoy.left_arm_vz_, 0.0,
                                                      SlowHz);

            */

            //}
        }

        //ROS_INFO_STREAM("Everything took " << (afterCall-beforeCall).toSec());
        counter++;
        pub_rate.sleep();
    }

    ros::shutdown();
    spinner.stop();

    return 0;
}
