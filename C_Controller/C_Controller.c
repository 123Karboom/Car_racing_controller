#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <webots/supervisor.h>
#define TIME_STEP 8

/*---------------------Định nghĩa---------------------*/

// Signal
#define NOP -1
#define MID 0
#define LEFT 1
#define RIGHT 2
#define FULL_SIGNAL 3
#define BLANK_SIGNAL 4
#define STOP_SIGNAL 5
#define LEFT_TURN_90 6
#define RIGHT_TURN_90 7
#define EXTRA_LEFT 8
#define EXTRA_RIGHT 9
#define MAX_SPEED 12
#define CIRCLE 10
// Các hằng số PID
#define KP 1.9
#define KI 0.01
#define KD 0.001
// Sensors
#define NB_GROUND_SENS 8

// LEDs
#define NB_LEDS 5

/*--------------Khởi tạo thông tin robot--------------*/

// KHÔNG CHỈNH SỬA TIME_STEP !!!
// Khai báo biến cho các sensors
unsigned short threshold[NB_GROUND_SENS] = {300, 300, 300, 300, 300, 300, 300, 300};
unsigned int filted[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Velocities
double left_ratio = 0.0;
double right_ratio = 0.0;

// Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs
WbDeviceTag led[NB_LEDS];

// Motors
WbDeviceTag left_motor, right_motor;
// Biến lưu trữ lỗi và tích phân lỗi
double error = 0.0;
double integral = 0.0;
double previous_error = 0.0;

// Vận tốc MIN , MAX
void constrain(double *value, double min, double max)
{
  if (*value > max)
    *value = max;
  if (*value < min)
    *value = min;
}

/*----------------Phần code code set up---------------*/

/* Hàm đọc giá trị sensors
   KHÔNG ĐƯỢC THIẾU!!!     */

void ReadSensors()
{
  unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < NB_GROUND_SENS; i++)
  {
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    // So sánh giá trị gs_value với threshold -> chuyển đổi sang nhị phân
    if (gs_value[i] < threshold[i])
      filted[i] = 1;
    else
      filted[i] = 0;
  }
}

double DesiredPosition()
{
  // Xác định vị trí mong muốn, ví dụ: giữa đường là 3.5
  return 3.5;
}

// Hàm lấy vị trí thực tế từ cảm biến
double ActualPosition()
{
  // Tính vị trí thực tế từ các giá trị cảm biến
  double position = 0.0;
  double weight_sum = 0.0;
  for (int i = 0; i < NB_GROUND_SENS; i++)
  {
    position += filted[i] * i;
    weight_sum += filted[i];
  }
  if (weight_sum > 0)
  {
    position /= weight_sum;
  }
  return position;
}

int Position()
{
  // Xác định vị trí của xe trên đường
  double actualPos = ActualPosition(); // Lấy vị trí thực tế từ cảm biến
  int position = MID;                  // Mặc định là giữa
  int sensorSum = 0;                   // Tổng các giá trị cảm biến

  // Kiểm tra các trường hợp
  if (actualPos < 3.5)
  {
    position = LEFT;
  }
  else if (actualPos > 4.5)
  {
    position = RIGHT;
  }

  // Kiểm tra mất line
  for (int i = 0; i < NB_GROUND_SENS; i++)
  {
    sensorSum += filted[i];
  }
  if (sensorSum == 0)
  {
    position = BLANK_SIGNAL;
  }

  if (sensorSum == 8)
  {
    position = FULL_SIGNAL;
  }

  // Kiểm tra rẽ trái 90 độ
  if (ActualPosition() < 3.5 && filted[0] == 1 && filted[1] == 1 && filted[2] == 1  && filted[6] == 0 && filted[7] == 0)
  {
    position = LEFT_TURN_90;
  }

  // Kiểm tra rẽ phải 90 độ
  if (ActualPosition() > 4.5 && filted[7] == 1 && filted[6] == 1 && filted[5] == 1 && filted[1] == 0 && filted[0] == 0)
  {
    position = RIGHT_TURN_90;
  }

  if (ActualPosition() > 4.5 && filted[5] == 0 && filted[4] == 0 && filted[3] == 0 && filted[2] == 0 && filted[1] == 0 && filted[6] == 0 && filted[7] == 1)
  {
    position = EXTRA_RIGHT;
  }

  if (ActualPosition() < 3.5 && filted[5] == 0 && filted[4] == 0 && filted[3] == 0 && filted[2] == 0 && filted[1] == 0 && filted[0] == 1)
  {
    position = EXTRA_LEFT;
  }

  if(filted[7] == 0 && filted[5] == 1 && filted[4] == 1 && filted[3] == 1 && filted[2] == 1 && filted[0] == 0)
  {
    position = CIRCLE;
  }

  return position;
}

// Hàm PID điều khiển
void PIDControl()
{
  // Đọc giá trị cảm biến và tính lỗi
  ReadSensors();
  error = DesiredPosition() - ActualPosition();

  // Tính các thành phần PID
  double proportional = error;
  integral += error;
  double derivative = error - previous_error;

  // Áp dụng công thức PID
  double control = KP * proportional + KI * integral + KD * derivative;

  // Giới hạn giá trị điều khiển
  if (control > MAX_SPEED)
  {
    control = MAX_SPEED;
  }
  else if (control < -MAX_SPEED)
  {
    control = -MAX_SPEED;
  }

  // Đặt tốc độ cho bánh trái và bánh phải
  left_ratio = 3.0 - control;
  right_ratio = 3.0 + control;

  // Lưu trữ giá trị lỗi hiện tại
  previous_error = error;
}

// Trả về giá trị tốc độ hai bánh đi khi thẳng
void GoStraight()
{
  left_ratio = 4.0;
  right_ratio = 4.0;
}

// Trả về giá trị tốc độ hai bánh đi khi dừng
void Stop()
{
  left_ratio = 0.0;
  right_ratio = 0.0;
}

void LeftTurn()
{
  left_ratio = 1.0;
  right_ratio = 2.5;
}

void RightTurn()
{
  left_ratio = 2.5;
  right_ratio = 1.0;
}

void LeftTurn90()
{
  left_ratio = 0.0;
  right_ratio = 2.0;
}

void RightTurn90()
{
  left_ratio = 2.0;
  right_ratio = 0.0;
}

/*---------------------Main loop---------------------*/

int main()
{
  /*------------------Khởi động robot------------------*/

  /* Khởi động robot
     KHÔNG ĐƯỢC BỎ!!! */
  wb_robot_init();

  // Khởi động camera
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 64);

  // Khởi động sensors
  char name[20];
  for (int i = 0; i < NB_GROUND_SENS; i++)
  {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  // Khởi động LEDs
  for (int i = 0; i < NB_LEDS; i++)
  {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name);
    wb_led_set(led[i], 1);
  }

  // Khởi động Motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Chương trình sẽ được lặp lại vô tận trong hàm while
  // Cần sử dụng hàm Position() trong hàm loop() của bạn
  // để xác định cách điều khiển xe dựa trên vị trí trên đường
  // Nga ba
  bool cross_left = 0;  // Cờ báo hiệu rẽ trái
  bool cross_right = 0; // Cờ báo hiệu rẽ phải
  bool cross_left_90 = false;
  bool cross_right_90 = false;
  bool in_circle = false;
  bool out_circle = false;
  int pre_position = -1;
  while (wb_robot_step(TIME_STEP) != -1)
  {
    ReadSensors();

    // In giá trị của cảm biến ra màn hình
    printf("\n\t\tPosition : 0b");
    for (int i = 0; i < 8; i++)
    {
      printf("%u", filted[i]);
    }
    // Sử dụng hàm Position() để xác định vị trí của xe
    int position = Position();
    // printf("\n\t\tPosition : %d", position);
    // printf("\n\t\tActualPosition : %f", ActualPosition());
    printf("\n\t\tCross left : %d", cross_left);
    printf("\n\t\tCross right : %d", cross_right);
    printf("\n\t\tCross left 90 : %d", cross_left_90);
    printf("\n\t\tCross right 90 : %d", cross_right_90);

    // Điều khiển xe dựa trên vị trí của xe
    if (cross_left_90)
    {
      if (position == EXTRA_LEFT)
      {
        cross_left_90 = false;
        cross_right = false;
        cross_left = false;
        Stop();
        printf("\n\t\tCross Left 90");
      }
      else
      {
        LeftTurn90();
        printf("\n\t\tCross Left 90");
      }
    }
    else if (cross_right_90)
    {
      if (position == EXTRA_RIGHT)
      {
        cross_right_90 = false;
        cross_right = false;
        cross_left = false;
        Stop();
        printf("\n\t\tCross Right 90");
      }
      else
      {
        RightTurn90();
        printf("\n\t\tCross Right 90");
      }
    }
    else if (out_circle)
    {
      if(position == EXTRA_RIGHT)
      {
        out_circle = false;
        in_circle = false;
        Stop();
        printf("\n\t\tOut circle success");
      }
      else 
      {
        RightTurn90();
        printf("\n\t\tOut circle");
      }
    }
    else if (position == MID)
    {
      pre_position = MID;
      GoStraight();
      printf("\n\t\tGo straight");
    }
    else if (position == LEFT || position == EXTRA_LEFT)
    {
      PIDControl();
      printf("\n\t\tPID");
    }
    else if (position == RIGHT || position == EXTRA_RIGHT)
    {
      PIDControl();
      printf("\n\t\tPID");
    }
    else if (position == LEFT_TURN_90)
    {
      // Điều khiển xe rẽ trái 90 độ
      cross_left = true;
      cross_right = false;
      pre_position = LEFT_TURN_90;
      printf("\n\t\tLeft 90");
    }
    else if (position == RIGHT_TURN_90)
    {
      // Điều khiển xe rẽ phải 90 độ
      cross_right = true;
      cross_left = false;
      pre_position = RIGHT_TURN_90;
      printf("\n\t\tRight 90");
    }
    else if (position == RIGHT_TURN_90 && in_circle)
    {
      out_circle = true;
      prinft("\n\t\tOut circle");
    }
    else if (position == CIRCLE)
    {
      out_circle = false;
      in_circle = true;
      printf("\n\t\tCircle");
    }
    else if (position == BLANK_SIGNAL && pre_position == LEFT_TURN_90)
    {
      cross_right = false;
      cross_left = false;
      LeftTurn90();
      printf("\n\t\tLeft 90");
    }
    else if (position == BLANK_SIGNAL && pre_position == RIGHT_TURN_90)
    {
      cross_right = false;
      cross_left = false;
      RightTurn90();
      printf("\n\t\tRight 90");
    }
    else if (position == BLANK_SIGNAL && pre_position == MID)
    {
      GoStraight();
      printf("\n\t\tGo straight lost line");
    }
    else if (position == BLANK_SIGNAL && in_circle)
    {
      RightTurn90();
      printf("\n\t\tRight 90 in circle");
    }
    else if (position == FULL_SIGNAL && cross_left)
    {
      cross_left_90 = true;
      Stop();
      printf("\n\t\tCross Left 90");
    }
    else if (position == FULL_SIGNAL && cross_right)
    {
      cross_right_90 = true;
      Stop();
      printf("\n\t\tCross Right 90");
    }

    // Đặt tốc độ cho bánh trái và bánh phải dựa trên PID
    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
  }

  wb_robot_cleanup();
  return 0;
}
