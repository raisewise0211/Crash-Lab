/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
#include <ros/ros.h>
#include <motor_test/motor_node.h>
#include <fstream>
#include <cmath>
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/motor_test/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)                                                                                                                                
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PWM_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Control_cycle = atof(line.substr(found+2).c_str()); break;
      case 4: Acceleration_ratio = atof(line.substr(found+2).c_str()); break;
      case 5: Wheel_radius = atof(line.substr(found+2).c_str()); break;
      case 6: Robot_radius = atof(line.substr(found+2).c_str()); break;
      case 7: Encoder_resolution = atof(line.substr(found+2).c_str()); break;
          //case :  = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}
int Motor_Setup(void)
{
  pinum=pigpio_start(NULL, NULL);
  
  if(pinum<0)
  {
    ROS_INFO("Setup failed");
    ROS_INFO("pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_DIR, PI_OUTPUT);
  set_mode(pinum, motor2_DIR, PI_OUTPUT);
  set_mode(pinum, motor1_PWM, PI_OUTPUT);
  set_mode(pinum, motor2_PWM, PI_OUTPUT);
  set_mode(pinum, motor1_ENA, PI_INPUT);
  set_mode(pinum, motor1_ENB, PI_INPUT);
  set_mode(pinum, motor2_ENA, PI_INPUT);
  set_mode(pinum, motor2_ENB, PI_INPUT);

  gpio_write(pinum, motor1_DIR, PI_LOW);
  gpio_write(pinum, motor2_DIR, PI_LOW);

  set_PWM_range(pinum, motor1_PWM, PWM_range);
  set_PWM_range(pinum, motor2_PWM, PWM_range);
  set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
  set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);

  set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

  current_PWM1 = 0;
  current_PWM2 = 0;

  current_Direction1 = true;
  current_Direction2 = true;

  acceleration = PWM_limit/(Acceleration_ratio);

  ROS_INFO("Setup Fin");
  return 0;
}
void Interrupt_Setting(void)
{
    callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
}
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1A ++;
  else EncoderCounter1A --;
  EncoderSpeedCounter1 ++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1B ++;
  else EncoderCounter1B --;
  EncoderSpeedCounter1 ++;
}
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2A --;
  else EncoderCounter2A ++;
  EncoderSpeedCounter2 ++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2B --;
  else EncoderCounter2B ++;
  EncoderSpeedCounter2 ++;
}
int Motor1_Encoder_Sum()
{
  EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
  return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
  EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
  return EncoderCounter2;
}
void Init_Encoder(void)
{
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;
  EncoderCounter1A = 0;
  EncoderCounter1B = 0;
  EncoderCounter2A = 0;
  EncoderCounter2B = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void)
{
  Text_Input();
  Motor_Setup();
  Init_Encoder();
  Interrupt_Setting();

  Wheel_round = 2*M_PI*Wheel_radius;
  Robot_round = 2*M_PI*Robot_radius;

  switch_direction = true;
  Theta_Distance_Flag = 0;

  ROS_INFO("PWM_range %d", PWM_range);
  ROS_INFO("PWM_frequency %d", PWM_frequency);
  ROS_INFO("PWM_limit %d", PWM_limit);
  ROS_INFO("Control_cycle %f", Control_cycle);
  ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
  ROS_INFO("Initialize Complete");

  printf("\033[2J");  
}

void Motor_Controller(int motor_num, bool direction, int pwm)
{
  int local_PWM = Limit_Function(pwm);

  if(motor_num == 1)
  {
    if(direction == true)
    {
      gpio_write(pinum, motor1_DIR, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = true;
      linear_vel1 = ((2*57.5*M_PI*RPM_Value1)/60);
    }
    else if(direction == false)
    {
      gpio_write(pinum, motor1_DIR, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = false;
      linear_vel1 = -((2*57.5*M_PI*RPM_Value1)/60);
    }
  }
  
  else if(motor_num == 2)
  {
   if(direction == true)
   {
     gpio_write(pinum, motor2_DIR, PI_LOW);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = true;
     linear_vel2 = -((2*57.5*M_PI*RPM_Value2)/60);
   }
   else if(direction == false)
   {
     gpio_write(pinum, motor2_DIR, PI_HIGH);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = false;
     linear_vel2 = ((2*57.5*M_PI*RPM_Value2)/60);
   }
  }
}
void Accel_Controller(int motor_num, bool direction, int desired_pwm)
{
  bool local_current_direction;
  int local_PWM;
  int local_current_PWM;

  if(motor_num == 1)
  {
    local_current_direction = current_Direction1;
    local_current_PWM = current_PWM1;
  }
  else if(motor_num == 2)
  {
    local_current_direction = current_Direction2;
    local_current_PWM = current_PWM2;
  }

  if(direction == local_current_direction)
  {
    if(desired_pwm > local_current_PWM)
    {
      local_PWM = local_current_PWM + acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else if(desired_pwm < local_current_PWM)
    {
      local_PWM = local_current_PWM - acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
  else
  {
	  if(desired_pwm >= 0)
	  {
      local_PWM = local_current_PWM - acceleration;
      if(local_PWM <= 0)
      {
        local_PWM = 0;
        Motor_Controller(motor_num, direction, local_PWM);
      }
      else Motor_Controller(motor_num, local_current_direction, local_PWM);
	  }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
int Limit_Function(int pwm)
{
  int output;
  if (pwm > PWM_limit*2)
  {
    output = PWM_limit;
    ROS_WARN("PWM too fast!!!");
  }
  else if(pwm > PWM_limit)output = PWM_limit;
  else if(pwm < 0)
  {
	output = 0;
    ROS_WARN("trash value!!!");
  }
  else output = pwm;
  return output; 
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
double PidContoller(double goal, double curr, double dt, pid *pid_data, pid_param *pid_paramdata, int error_rat)
{
  double error = goal - curr;
  ROS_INFO(" goal : %f, curr: %f, dt: %f", goal,curr,dt);
  ROS_INFO(" error : %f", error);
  if (fabs(error) < error_rat)
    error = 0;

  pid_data->p_out = pid_paramdata->kP * error;
  double p_data = pid_data->p_out ;

  pid_data->integrator += (error * pid_paramdata->kI) * dt;
  pid_data->integrator = constrain(pid_data->integrator, -pid_paramdata->Imax, pid_paramdata->Imax);
  double i_data = pid_data->integrator;

  double filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  pid_data->derivative = (goal - pid_data->last_input) / dt;
  pid_data->derivative = pid_data->lastderivative + (dt / (filter + dt)) * (pid_data->derivative - pid_data->lastderivative);
  pid_data->last_input = goal;
  pid_data->lastderivative = pid_data->derivative;
  double d_data = pid_paramdata->kD * pid_data->derivative;
  d_data = constrain(d_data, -pid_paramdata->Dmax, pid_paramdata->Dmax);

  double output = p_data + i_data + d_data;
  pid_data->output = output;

  return pid_data->output;
}

void RPM_Calculator()
{
  RPM_Value1 = (EncoderSpeedCounter1*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter1 = 0;
  RPM_Value2 = (EncoderSpeedCounter2*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter2 = 0;
}
void carcul_packet()
{
 linear = (linear_vel1 + linear_vel2)/2;
 angular =(linear_vel2 - linear_vel1)/Robot_radius*2;
 odom_l = linear_vel2/(Control_cycle*Wheel_radius);
 odom_r = linear_vel1/(Control_cycle*Wheel_radius);
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
  ROS_INFO("i heard :[%f]", msg->linear.x);
  // vx=msg->linear.x
  // vz=msg->angular.z
  // M1=0
  // Mz=0
  // // 
  // Motor_Controller(1, false, M1);
  // Motor_Controller(2, false, M2);

}

void Motor_View()
{
  RPM_Calculator();
	carcul_packet();
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
	printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
	printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value1, RPM_Value2);
	printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
  printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
	printf("Acc  :%10.0d\n", acceleration);
	printf("\n");
	printf("linear vel1: %10.0f || linear vel2: %10.0f\n",linear_vel1, linear_vel2); 
  printf("liner_vel: %10.0f || angular_vel: %10.0f\n",linear, angular); 
  printf("odom_l: %10.0f || odom_r: %10.0f\n",odom_l, odom_r);
}

int main(int argc, char** argv)
{
  ROS_INFO("ASDF");
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  ros::Publisher packet_pub = nh.advertise<motor_test::MotorPacket>("motor_packet",1, false);
  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, cmd_vel_callback);
  Initialize();
  ros::Rate loop_rate(Control_cycle);
  while(ros::ok())
  {
    // Motor_Controller(1, false, 100);
    // Motor_Controller(2, false, 100);
    Accel_Controller(1, true, 50);
    Accel_Controller(2, true, 50);
    //Switch_Turn_Example(100, 100);
    //Theta_Distance(180,100,30,110);
    Motor_View();
    ros::spinOnce();
    loop_rate.sleep();
  }
  Motor_Controller(1, true, 0);
  Motor_Controller(2, false, 0);
  return 0;
}