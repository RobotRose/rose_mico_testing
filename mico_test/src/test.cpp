#include <mico_test/test.hpp>

namespace mico_test {

MicoTest::MicoTest(std::string name, ros::NodeHandle n)
  : n_ (n)
  , name_ (name)
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);

  // initialize random seed
  srand (time(NULL));

  ROS_INFO("Trying to initialize API...");
  InitAPI();
  ROS_INFO("Api initialized.");
  ros::Duration(1.0).sleep();
  ROS_INFO("Starting control API...");
  StartControlAPI();
  ROS_INFO("Control API started...");
  ros::Duration(3.0).sleep();
  ROS_INFO("Stopping control API...");
  StopControlAPI();
  ROS_INFO("Control API stopped.");

  // Initialize arm
  ROS_INFO("Homing arm...");
  MoveHome();
  ROS_INFO("Done.");
  ROS_INFO("Initializing fingers...");
  InitFingers();
  ROS_INFO("Done.");
  SetFrameType(0); //set end effector to move with respect to the fixed frame

  StartControlAPI();

  // Initialize variables
  current_control_mode_ = 0;
  SetCartesianControl();

  send_angular_mode_timer_   = n.createTimer(ros::Duration(1/SWITCH_ANGULAR_MODE_TIME), boost::bind(&MicoTest::setAngularMode, this));
  send_cartesian_mode_timer_ = n.createTimer(ros::Duration(1/SWITCH_CARTESIAN_MODE_TIME), boost::bind(&MicoTest::setCartesianMode, this));
  show_info_timer_           = n.createTimer(ros::Duration(1/SHOW_INFO_TIME), boost::bind(&MicoTest::showInfo, this));
}

MicoTest::~MicoTest()
{
  StopControlAPI();
  CloseAPI();
}

void MicoTest::setCartesianMode()
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);

  SetCartesianControl();
  current_control_mode_ = 0;

  armPositionControl();
}

void MicoTest::setAngularMode()
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);

  SetAngularControl();
  current_control_mode_ = 1;

  fingerPositionControl(rand() % 6400, rand() % 6400, 0);
}

void MicoTest::showInfo()
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);

  int kinova_control_type;
  GetControlType(kinova_control_type); // 1 is angular control, 0 is cartesian control

  if ( current_control_mode_ == kinova_control_type )
    ROS_INFO("Control mode: %d, Kinova control mode: %d", current_control_mode_, kinova_control_type);
  else
    ROS_ERROR("[MISMATCH] Control mode: %d, Kinova control mode: %d", current_control_mode_, kinova_control_type);
}

void MicoTest::armPositionControl()
{
  //take control of the arm
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();

    SetCartesianControl();
  }

  TrajectoryPoint jacoPoint;
  jacoPoint.InitStruct();

  // Populate arm command
  jacoPoint.Position.Type = CARTESIAN_VELOCITY;

  jacoPoint.Position.CartesianPosition.X      = 0.0;
  jacoPoint.Position.CartesianPosition.Y      = 0.0;
  jacoPoint.Position.CartesianPosition.Z      = 0.1;
  jacoPoint.Position.CartesianPosition.ThetaX = 0.0;
  jacoPoint.Position.CartesianPosition.ThetaY = 0.0;
  jacoPoint.Position.CartesianPosition.ThetaZ = 0.0;

  jacoPoint.Position.HandMode = HAND_NOMOVEMENT;

  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    SendBasicTrajectory(jacoPoint);

    // Send the command repeatedly for ~1/60th of a second
    //(this is sometimes necessary for velocity commands to work correctly)
    ros::Rate rate(600);
    for (int i = 0; i < 10; i++)
    {
      SendBasicTrajectory(jacoPoint);
      rate.sleep();
    }
  }
}

void MicoTest::fingerPositionControl(float f1, float f2, float f3)
{
  f1 = std::max(f1, .02f);
  f2 = std::max(f2, .02f);
  f3 = std::max(f3, .02f);

  TrajectoryPoint jacoPoint;
  jacoPoint.InitStruct();
  jacoPoint.Position.Type                = ANGULAR_VELOCITY;
  jacoPoint.Position.Actuators.Actuator1 = 0.0;
  jacoPoint.Position.Actuators.Actuator2 = 0.0;
  jacoPoint.Position.Actuators.Actuator3 = 0.0;
  jacoPoint.Position.Actuators.Actuator4 = 0.0;
  jacoPoint.Position.Actuators.Actuator5 = 0.0;
  jacoPoint.Position.Actuators.Actuator6 = 0.0;
  jacoPoint.Position.HandMode            = VELOCITY_MODE;

  bool goal_reached = false;

  AngularPosition position_data;
  float error[3];
  float prevTotalError;
  float counter = 0; //check if error is unchanging, this likely means a finger is blocked by something so the controller should terminate
  std::vector<float> errorFinger1;
  std::vector<float> errorFinger2;
  std::vector<float> errorFinger3;
  errorFinger1.resize(10);
  errorFinger2.resize(10);
  errorFinger3.resize(10);
  for (unsigned int i = 0; i < errorFinger1.size(); i++)
  {
    errorFinger1[i] = 0.0;
    errorFinger2[i] = 0.0;
    errorFinger3[i] = 0.0;
  }
  ros::Rate rate(600);
  while (!goal_reached)
  {
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);

      //get current finger position
      GetAngularPosition(position_data);
      error[0] = f1 - position_data.Fingers.Finger1;
      error[1] = f2 - position_data.Fingers.Finger2;
      error[2] = 0.0;

      float totalError = fabs(error[0]) + fabs(error[1]) + fabs(error[2]);
      if (totalError == prevTotalError)
        counter++;
      else
        counter = 0;

      prevTotalError = totalError;

      if (totalError < 0.166 || counter > 40)
      {
        goal_reached = true;
        jacoPoint.Position.Fingers.Finger1 = 0.0;
        jacoPoint.Position.Fingers.Finger2 = 0.0;
        jacoPoint.Position.Fingers.Finger3 = 0.0;
      }
      else
      {
        float errorSum[3] = {0};
        for (unsigned int i = 0; i < errorFinger1.size(); i ++)
        {
          errorSum[0] += errorFinger1[i];
          errorSum[1] += errorFinger2[i];
          errorSum[2] += errorFinger3[i];
        }
        jacoPoint.Position.Fingers.Finger1 = std::max(std::min(KP_F*error[0] + KV_F*(error[0] - errorFinger1.front()) + KI_F*errorSum[0], 3000.0), -3000.0);
        jacoPoint.Position.Fingers.Finger2 = std::max(std::min(KP_F*error[1] + KV_F*(error[1] - errorFinger2.front()) + KI_F*errorSum[1], 3000.0), -3000.0);
        jacoPoint.Position.Fingers.Finger3 = std::max(std::min(KP_F*error[2] + KV_F*(error[2] - errorFinger3.front()) + KI_F*errorSum[2], 3000.0), -3000.0);

        errorFinger1.insert(errorFinger1.begin(), error[0]);
        errorFinger2.insert(errorFinger2.begin(), error[1]);
        errorFinger3.insert(errorFinger3.begin(), error[2]);

        errorFinger1.resize(10);
        errorFinger2.resize(10);
        errorFinger3.resize(10);
      }
      
      EraseAllTrajectories();
      SendBasicTrajectory(jacoPoint);
    }

    rate.sleep();
  }
}

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mico_test");

  ros::NodeHandle n;

  mico_test::MicoTest test("mico_test", n);
  ros::spin();
}
