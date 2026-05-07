#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <urdf/model.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>

//#include "omni_msgs/OmniState.h"
#include <pthread.h>

float prev_time;
int calibrationStyle;

static const hduVector3Dd nominalBaseTorque(15000.0, 15000.0, 15000.0); //mNm

struct OmniState {
  hduVector3Dd position;  //3x1 vector of position
  hduVector3Dd velocity;  //3x1 vector of velocity
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  //hduVector3Dd joints;
  hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
    hduVector3Dd jointTorque;
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool close_gripper;
  hduVector3Dd lock_pos;
  double units_ratio;
  bool is_torque_mode = true;
  pthread_mutex_t force_lock;
};

class PhantomROS {

public:
  ros::NodeHandle n;
  ros::Publisher button_pub;
  ros::Publisher state_publisher;
  ros::Publisher position_publisher;
  ros::Publisher speed_publisher;
  ros::Publisher joint_publisher;
  ros::Subscriber haptic_sub;
  std::string omni_name, ref_frame, units;
  std::string force_topic;
  bool is_torque_mode;

  OmniState *state;

  void init(OmniState *s) {
    bool is_torque_mode;
    ros::param::param(std::string("~omni_name"), omni_name, std::string("/geomagic"));
    ros::param::param(std::string("~reference_frame"), ref_frame, std::string("/map"));
    ros::param::param(std::string("~units"), units, std::string("mm"));
    ros::param::param(std::string("~torque_mode"), is_torque_mode, true);
    ros::param::param(std::string("~force_topic"), force_topic, std::string("/arm/force_feedback"));

    //Subscribe to NAME/force_feedback
    std::ostringstream stream3;
    //stream3 << "/arm/force_feedback";
    stream3 << force_topic;
    std::string force_feedback_topic = std::string(stream3.str());
    haptic_sub = n.subscribe(force_feedback_topic.c_str(), 1, &PhantomROS::force_callback, this);

    //Publish on NAME/position
    std::ostringstream stream4;
    stream4 << omni_name << "/position";
    std::string position_topic_name = std::string(stream4.str());
    position_publisher = n.advertise<geometry_msgs::Vector3>(position_topic_name.c_str(), 1);
    
    //Publish on NAME/speed
    std::ostringstream stream2;
    stream2 << omni_name << "/speed";
    std::string speed_topic_name = std::string(stream2.str());
    speed_publisher = n.advertise<geometry_msgs::Vector3>(speed_topic_name.c_str(), 1);

    //Publish on NAME/joint_states
    std::ostringstream stream5;
    stream5 << omni_name << "/joint_states";
    std::string joint_topic_name = std::string(stream5.str());
    joint_publisher = n.advertise<sensor_msgs::JointState>(joint_topic_name.c_str(), 1);

    // Publish button state on NAME/button.
    std::ostringstream button_topic;
    button_topic << omni_name << "/button";
    button_pub = n.advertise<std_msgs::Int32MultiArray>(button_topic.str(), 1);

    state = s;
    if (pthread_mutex_init(&state->force_lock, NULL) != 0) {
      ROS_FATAL("[Geomagic] pthread_mutex_init(force_lock) failed");
    }
    state->is_torque_mode = is_torque_mode;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->force = zeros;
    state->jointTorque = zeros;
    state->position = zeros;
    for (int i = 0; i < 7; ++i) {
      state->thetas[i] = 0.f;
    }
    state->velocity = zeros;
    state->inp_vel1 = zeros;  //3x1 history of velocity
    state->inp_vel2 = zeros;  //3x1 history of velocity
    state->inp_vel3 = zeros;  //3x1 history of velocity
    state->out_vel1 = zeros;  //3x1 history of velocity
    state->out_vel2 = zeros;  //3x1 history of velocity
    state->out_vel3 = zeros;  //3x1 history of velocity
    state->pos_hist1 = zeros; //3x1 history of position
    state->pos_hist2 = zeros; //3x1 history of position
    state->lock = false;
    state->close_gripper = false;
    state->lock_pos = zeros;
    if (!units.compare("mm"))
      state->units_ratio = 1.0;
    else if (!units.compare("cm"))
      state->units_ratio = 10.0;
    else if (!units.compare("dm"))
      state->units_ratio = 100.0;
    else if (!units.compare("m"))
      state->units_ratio = 1000.0;
    else
    {
      state->units_ratio = 1.0;
      ROS_WARN("[Geomagic] Unknown units [%s] unsing [mm]", units.c_str());
      units = "mm";
    }
    ROS_INFO("[Geomagic] Geomagic position given in [%s], ratio [%.1f]", units.c_str(), state->units_ratio);
  }

  /*******************************************************************************
   ROS node callback.
   *******************************************************************************/
//void force_callback(const geometry_msgs::Vector3& omnifeed) {
  void force_callback(const sensor_msgs::ChannelFloat32& omnifeed) {
    ////////////////////Some people might not like this extra damping, but it
    ////////////////////helps to stabilize the overall force feedback. It isn't
    ////////////////////like we are getting direct impedance matching from the
    ////////////////////omni anyway

    if (omnifeed.values.size() < 3)
      return;
    pthread_mutex_lock(&state->force_lock);
    if (!state->is_torque_mode) {
      state->force[0] = omnifeed.values[0];
      state->force[1] = omnifeed.values[1];
      state->force[2] = omnifeed.values[2];
    } else {
      state->jointTorque[0] = omnifeed.values[0];
      state->jointTorque[1] = omnifeed.values[1];
      state->jointTorque[2] = omnifeed.values[2];
    }
    pthread_mutex_unlock(&state->force_lock);
  }

  void publish_omni_state() {
    // Build the state msg

    // Publish the JointState msg
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.velocity.resize(6);
    joint_state.effort.resize(6);
    joint_state.name[0] = "waist";
    joint_state.position[0] = -state->thetas[1]; // *180.0/M_PI;
    joint_state.velocity[0] = state->velocity[0];
    joint_state.name[1] = "shoulder";
    joint_state.position[1] = state->thetas[2]; // *180.0/M_PI;
    joint_state.velocity[1] = state->velocity[1];
    joint_state.name[2] = "elbow";
    joint_state.position[2] = state->thetas[3]; // *180.0/M_PI;
    //joint_state.position[2] = (state->thetas[3] - M_PI/2.0);// *180.0/M_PI;
    joint_state.velocity[2] = state->velocity[2];
    joint_state.name[3] = "yaw";
    //joint_state.position[3] = -state->thetas[4] + M_PI;
    joint_state.position[3] = -state->thetas[4]; // *180.0/M_PI;
    joint_state.velocity[3] = state->velocity[3];
    joint_state.name[4] = "pitch";
    //joint_state.position[4] = -state->thetas[5] - 3*M_PI/4;
    joint_state.position[4] = state->thetas[5] - M_PI/8.0;
    joint_state.velocity[4] = state->velocity[4];
    joint_state.name[5] = "roll";
    //joint_state.position[5] = -state->thetas[6] - M_PI;
    joint_state.position[5] = state->thetas[6];
    joint_state.velocity[5] = state->velocity[5];
    joint_publisher.publish(joint_state);

    // Build the position msg
    geometry_msgs::Vector3 position_msg;
    position_msg.x = state->position[0];
    position_msg.y = state->position[1];
    position_msg.z = state->position[2];
    position_publisher.publish(position_msg);

    // Build the speed msg
    geometry_msgs::Vector3 speed_msg;
    speed_msg.x = state->velocity[0];
    speed_msg.y = state->velocity[1];
    speed_msg.z = state->velocity[2];
    speed_publisher.publish(speed_msg);

    // Build button msg
    std_msgs::Int32MultiArray button_msg;
    button_msg.data.clear();
    button_msg.data.push_back(state->buttons[0]);  //cinza escuro
    button_msg.data.push_back(state->buttons[1]);  //cinza claro
    if (state->buttons[0] != state->buttons_prev[0] || state->buttons[1] != state->buttons_prev[1]){
      button_pub.publish(button_msg);
    }
    state->buttons_prev[0] = state->buttons[0];
    state->buttons_prev[1] = state->buttons[1];
    }
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) 
{
  const HDdouble kTorqueInfluence = 3.14; /* radians */
  const HDdouble kStylusTorqueConstant = 500; /* torque spring constant (mN.m/radian)*/

  OmniState *omni_state = static_cast<OmniState *>(pUserData);
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    ROS_DEBUG("[Geomagic] Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
  }
  hdBeginFrame(hdGetCurrentDevice());
  // Get transform and angles
  hduMatrix transform;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hduVector3Dd joints;
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, joints);
  // Do not return early here: return -1 is not a valid HDCallbackCode and skips hdEndFrame(),
  // which stops the scheduler / freezes /geomagic/joint_states. Real faults are handled after hdEndFrame.

  //ROS_INFO("Joints: %.2f, %.2f, %.2f",joints[0],joints[1],joints[2]);

  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  hduVector3Dd position;
  hdGetDoublev(HD_CURRENT_POSITION, position);
  hduVector3Dd speed;
  hdGetDoublev(HD_CURRENT_VELOCITY, speed);

  omni_state->position = position;
  omni_state->position /= omni_state->units_ratio;
  omni_state->velocity = speed;
  omni_state->velocity /= omni_state->units_ratio;

  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                             1.0,  0.0, 0.0, 0.0,
                             0.0,  0.0, 1.0, 0.0,
                             0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  omni_state->rot = hduQuaternion(rotation_offset * rotation);
  // Velocity estimation
  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
      + omni_state->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
  omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
      + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
          - 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;
  
    
   /* if (hduVecMagnitude(jointAngleOfTwist) < kTorqueInfluence)
    {
        /* >  T = k * r  < We calculate torque by assuming a torsional spring at each joint. 
           T: Torque in Milli Newton . meter (mN.m)
           k: Torque Spring Constant (mN.m / radian)
           r: Angle of twist from fulcrum point (radians)*/

      /*  hduVecScale(jointTorque, jointAngleOfTwist, kJointTorqueConstant);
    }*/
  hduVector3Dd jointTorque(0., 0., 0.);
  hduVector3Dd feedback(0., 0., 0.);
  pthread_mutex_lock(&omni_state->force_lock);
  if (!omni_state->is_torque_mode) {
    // Swap axes to match stylus frame (historical Omni convention).
    feedback[0] = omni_state->force[1] / 10.0;
    feedback[1] = omni_state->force[2] / 10.0;
    feedback[2] = omni_state->force[0] / 10.0;
  } else {
    jointTorque[0] = omni_state->jointTorque[0];
    jointTorque[1] = omni_state->jointTorque[1];
    jointTorque[2] = omni_state->jointTorque[2];
  }
  pthread_mutex_unlock(&omni_state->force_lock);

// Clamp the base torques to the nominal values.
  for (int i = 0; i < 3; ++i)
  {
    if (jointTorque[i] > nominalBaseTorque[i])
      jointTorque[i] = nominalBaseTorque[i];
    else if (jointTorque[i] < -nominalBaseTorque[i])
      jointTorque[i] = -nominalBaseTorque[i];
  }

  for (int i = 0; i < 3; ++i)
  {
    if (!std::isfinite(feedback[i]))
      feedback[i] = 0.0;
    if (!std::isfinite(jointTorque[i]))
      jointTorque[i] = 0.0;
  }

  //Notice that we are changing Y <---> Z (copied above under lock).
  if (!omni_state->is_torque_mode)
    hdSetDoublev(HD_CURRENT_FORCE, feedback);
  else
    hdSetDoublev(HD_CURRENT_JOINT_TORQUE, jointTorque);
  //hdSetDoublev(HD_CURRENT_JOINT_TORQUE, feedback); // using torque instead off force
  
  //Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0; //grey
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0; //white
  //ROS_INFO("Botao 0: %d;  Botao 1: %d", omni_state->buttons[0], omni_state->buttons[1]);
  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "[Geomagic] Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = { 0., joints[0], joints[1],
      joints[2] - joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];
  return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("[Geomagic] HD_CALIBRATION_ENCODER_RESET..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("[Geomagic] HD_CALIBRATION_INKWELL..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    ROS_INFO("[Geomagic] HD_CALIBRATION_AUTO..");
  }

  if(hdCheckCalibration() == HD_CALIBRATION_OK)
    ROS_INFO("[Geomagic] Already calibrated");    
  else  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
          if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) 
            ROS_INFO("[Geomagic] Please place the device into the inkwell for calibration");
          else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
                 if (calibrationStyle == HD_CALIBRATION_INKWELL) {
                   do {
                     hdUpdateCalibration(calibrationStyle);
                     ROS_INFO("[Geomagic] Calibrating.. (keep stylus in inkwell)");
                     if (HD_DEVICE_ERROR(error = hdGetError())) {
                       hduPrintError(stderr, &error, "[Geomagic] Inkwell calibration failed.");
                       break;
                     }
                   } while (hdCheckCalibration() != HD_CALIBRATION_OK);
                   ROS_INFO("[Geomagic] Calibration complete.");
                 }
               }
               else ROS_FATAL("[Geomagic] Unknown calibration status");
        usleep(1e6); 
        }
}

void *ros_publish(void *ptr) {
  PhantomROS *omni_ros = (PhantomROS *) ptr;
  int publish_rate;
  ros::param::param(std::string("~publish_rate"), publish_rate, 50);
  ROS_INFO("[Geomagic] Publishing Geomagic state at [%d] Hz", publish_rate);
  ros::Rate loop_rate(publish_rate);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok()) {
    omni_ros->publish_omni_state();
    loop_rate.sleep();
  }
  return NULL;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "geomagic_node");

  OmniState state;
  PhantomROS omni_ros;
  omni_ros.init(&state);

  HDErrorInfo error;
  HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR("[Geomagic] Failed to initialize haptic device");
    pthread_mutex_destroy(&state.force_lock);
    return -1;
  }

  ROS_INFO("[Geomagic] Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  HHD_Auto_Calibration();

  hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR("[Geomagic] Failed to start the scheduler");
    hdDisableDevice(hHD);
    pthread_mutex_destroy(&state.force_lock);
    return -1;
  }

  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void*)&omni_ros);
  pthread_join(publish_thread, NULL);

  ROS_INFO("[Geomagic] Ending Session....");
  hdStopScheduler();
  pthread_mutex_destroy(&state.force_lock);
  hdDisableDevice(hHD);

  return 0;
}
