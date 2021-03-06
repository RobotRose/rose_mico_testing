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
  
  ros::Rate rate(MAX_WAIT_TIME);
  rate.sleep();
  
  SetCartesianControl();
  current_control_mode_ = 0;

  armPositionControl();
}

void MicoTest::setAngularMode()
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);

  ros::Rate rate(MAX_WAIT_TIME);
  rate.sleep();

  SetAngularControl();
  current_control_mode_ = 1;

  float f1 = rand() % 6400;
  float f2 = rand() % 6400;

  fingerPositionControl(f1, f2, 0);
}

void MicoTest::showInfo()
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);

  ros::Rate rate(MAX_WAIT_TIME);
  rate.sleep();

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
  // ROS_INFO("f1: %f, f2: %f", f1, f2);

  TrajectoryPoint jacoPoint;
  AngularPosition angular_position;
  GetAngularPosition(angular_position);

  jacoPoint.InitStruct();
  jacoPoint.Position.Type                = ANGULAR_POSITION;
  jacoPoint.Position.Actuators.Actuator1 = angular_position.Actuators.Actuator1;
  jacoPoint.Position.Actuators.Actuator2 = angular_position.Actuators.Actuator2;
  jacoPoint.Position.Actuators.Actuator3 = angular_position.Actuators.Actuator3;
  jacoPoint.Position.Actuators.Actuator4 = angular_position.Actuators.Actuator4;
  jacoPoint.Position.Actuators.Actuator5 = angular_position.Actuators.Actuator5;
  jacoPoint.Position.Actuators.Actuator6 = angular_position.Actuators.Actuator6;

  jacoPoint.Position.HandMode            = POSITION_MODE;
  jacoPoint.Position.Fingers.Finger1     = f1;
  jacoPoint.Position.Fingers.Finger2     = f2;
  jacoPoint.Position.Fingers.Finger3     = 0.0;

  EraseAllTrajectories();
  SendBasicTrajectory(jacoPoint);

  bool goal_reached      = false;
  bool converging        = true;
  float prev_error       = std::numeric_limits<float>::max();
  int not_converging_cnt = 0;

  // Check goal reached
  ros::Rate rate(10);
  while ( not goal_reached && not_converging_cnt < 5 )
  {
    GetAngularPosition(angular_position);

    float total_error = fabs(angular_position.Fingers.Finger1 - f1) + fabs(angular_position.Fingers.Finger2 - f2);
    goal_reached      = (total_error < 0.166);
    converging        = (total_error < prev_error);
    prev_error        = total_error;

    if ( not converging )
      not_converging_cnt++;
    else
      not_converging_cnt = 0;

    rate.sleep();
  }

  return;
}

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mico_test");

  ros::NodeHandle n;

  mico_test::MicoTest test("mico_test", n);
  ros::spin();
}
