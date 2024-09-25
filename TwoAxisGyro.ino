#include <Wire.h>
#include <Servo.h>

// Constants
const int MPU_ADDRESS = 0x68;  // I2C address of MPU6050
const int GYRO_CONFIG_REGISTER = 0x1B;
const int ACCEL_CONFIG_REGISTER = 0x1C;
const int POWER_MGMT_REGISTER = 0x6B;
const int GYRO_XOUT_REGISTER = 0x43;
const int ACCEL_XOUT_REGISTER = 0x3B;

const float GYRO_SENSITIVITY = 32.8;  // Sensitivity for +/-1000 degrees/sec
const float ACCEL_SENSITIVITY = 4096.0;  // Sensitivity for +/-8g

const int CALIBRATION_SAMPLES = 200;  // Number of samples for calibration
const float COMPLEMENTARY_FILTER_ALPHA = 0.98;  // Alpha value for complementary filter

const int SERVO_CENTER_X = 45;  // Offset for X-axis servo
const int SERVO_CENTER_Y = 90;  // Offset for Y-axis servo
const int SERVO_PIN_1 = 4;  // Pin for first servo (X-axis)
const int SERVO_PIN_2 = 7;  // Pin for second servo (Y-axis)
const int DEFAULT_SERVO_POSITION = 90;  // Default servo position
const int SERIAL_BAUD_RATE = 9600;

const int SERVO_UPDATE_THRESHOLD = 2;  // Minimum change in angle to trigger servo update

// Class to handle sensor initialization and readings
class Mpu6050 {
  private:
    float gyro_raw_x, gyro_raw_y;
    float gyro_angle_x, gyro_angle_y;
    float gyro_raw_error_x, gyro_raw_error_y;
    float acc_raw_x, acc_raw_y, acc_raw_z;
    float acc_angle_x, acc_angle_y;
    float acc_angle_error_x, acc_angle_error_y;
    int gyro_error = 0, acc_error = 0;
    
  public:
    Mpu6050() {}

    void initialize() {
      Wire.begin();
      
      // mpu6050 setup
      Wire.beginTransmission(MPU_ADDRESS);
      Wire.write(POWER_MGMT_REGISTER);  // Power management register
      Wire.write(0x00);  // Wake up mpu6050
      Wire.endTransmission(true);
      
      // Configure gyro sensitivity
      Wire.beginTransmission(MPU_ADDRESS);
      Wire.write(GYRO_CONFIG_REGISTER);  // Gyro config
      Wire.write(0x10);  // +/-1000 degrees/sec
      Wire.endTransmission(true);
      
      // Configure accelerometer sensitivity
      Wire.beginTransmission(MPU_ADDRESS);
      Wire.write(ACCEL_CONFIG_REGISTER);  // Accel config
      Wire.write(0x10);  // +/-8g
      Wire.endTransmission(true);
      
      calibrate_sensors();
    }

    void calibrate_sensors() {
      // Calibrate accelerometer and gyroscope for error correction
      if (acc_error == 0) {
        for (int a = 0; a < CALIBRATION_SAMPLES; a++) {
          read_accelerometer();
          acc_angle_error_x += (atan(acc_raw_y / sqrt(pow(acc_raw_x, 2) + pow(acc_raw_z, 2))) * RAD_TO_DEG);
          acc_angle_error_y += (atan(-acc_raw_x / sqrt(pow(acc_raw_y, 2) + pow(acc_raw_z, 2))) * RAD_TO_DEG);
        }
        acc_angle_error_x /= CALIBRATION_SAMPLES;
        acc_angle_error_y /= CALIBRATION_SAMPLES;
        acc_error = 1;
      }

      if (gyro_error == 0) {
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
          read_gyroscope();
          gyro_raw_error_x += (gyro_raw_x / GYRO_SENSITIVITY);
          gyro_raw_error_y += (gyro_raw_y / GYRO_SENSITIVITY);
        }
        gyro_raw_error_x /= CALIBRATION_SAMPLES;
        gyro_raw_error_y /= CALIBRATION_SAMPLES;
        gyro_error = 1;
      }
    }

    void read_accelerometer() {
      // Reading accelerometer data
      Wire.beginTransmission(MPU_ADDRESS);
      Wire.write(ACCEL_XOUT_REGISTER);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDRESS, 6, true);
      acc_raw_x = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;
      acc_raw_y = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;
      acc_raw_z = (Wire.read() << 8 | Wire.read()) / ACCEL_SENSITIVITY;
    }

    void read_gyroscope() {
      // Reading gyroscope data
      Wire.beginTransmission(MPU_ADDRESS);
      Wire.write(GYRO_XOUT_REGISTER);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDRESS, 4, true);
      gyro_raw_x = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
      gyro_raw_y = (Wire.read() << 8 | Wire.read()) / GYRO_SENSITIVITY;
      gyro_raw_x -= gyro_raw_error_x;
      gyro_raw_y -= gyro_raw_error_y;
    }

    float calculate_gyro_angle(float elapsed_time, float& angle_x, float& angle_y) {
      // Use the elapsed time and gyroscope data to calculate angle
      gyro_angle_x = gyro_raw_x * elapsed_time;
      gyro_angle_y = gyro_raw_y * elapsed_time;
      angle_x += gyro_angle_x;
      angle_y += gyro_angle_y;
    }

    void calculate_acc_angle(float& angle_x, float& angle_y) {
      // Calculate angles using the accelerometer data
      acc_angle_x = (atan(acc_raw_y / sqrt(pow(acc_raw_x, 2) + pow(acc_raw_z, 2))) * RAD_TO_DEG) - acc_angle_error_x;
      acc_angle_y = (atan(-acc_raw_x / sqrt(pow(acc_raw_y, 2) + pow(acc_raw_z, 2))) * RAD_TO_DEG) - acc_angle_error_y;
      angle_x = acc_angle_x;
      angle_y = acc_angle_y;
    }
};

// Class to handle servos
class ServoController {
  private:
    Servo servo_1, servo_2;
    int pin_1, pin_2;
    
  public:
    ServoController(int pin_1, int pin_2) : pin_1(pin_1), pin_2(pin_2) {}

    void initialize() {
      servo_1.attach(pin_1);
      servo_2.attach(pin_2);
      servo_1.write(DEFAULT_SERVO_POSITION);  // Set to default position
      servo_2.write(DEFAULT_SERVO_POSITION);
    }

    void update_servos(float angle_x, float angle_y, int offset_x, int offset_y) {
      servo_1.write(angle_x + SERVO_CENTER_X + offset_x);
      servo_2.write(angle_y + SERVO_CENTER_Y + offset_y);
    }
};

// Global variables for time tracking
float elapsed_time, time_prev, time_now;
float total_angle_x, total_angle_y;
Mpu6050 mpu;
ServoController servos(SERVO_PIN_1, SERVO_PIN_2);

// Offsets
int offset_x = 0;
int offset_y = 0;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  mpu.initialize();
  servos.initialize();
  time_prev = millis();
}

// Variables to track previous servo positions
int prev_servo_x_pos = -1;
int prev_servo_y_pos = -1;

void loop() {
  // Time
  time_now = millis();
  elapsed_time = (time_now - time_prev) / 1000.0;
  time_prev = time_now;

  // Check for serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read input from serial until newline
    parse_offsets(input);  // Parse and apply offsets
  }

  // Reading sensors
  mpu.read_gyroscope();
  mpu.calculate_gyro_angle(elapsed_time, total_angle_x, total_angle_y);

  mpu.read_accelerometer();
  float acc_angle_x, acc_angle_y;
  mpu.calculate_acc_angle(acc_angle_x, acc_angle_y);

  // Complementary filter
  total_angle_x = COMPLEMENTARY_FILTER_ALPHA * total_angle_x + (1 - COMPLEMENTARY_FILTER_ALPHA) * acc_angle_x;
  total_angle_y = COMPLEMENTARY_FILTER_ALPHA * total_angle_y + (1 - COMPLEMENTARY_FILTER_ALPHA) * acc_angle_y;

  // Calculate new servo positions (rounded to integer)
  int servo_x_pos = round(total_angle_x + SERVO_CENTER_X);
  int servo_y_pos = round(total_angle_y + SERVO_CENTER_Y);
  
  // Check if the positions have changed by a significant amount to avoid flooding serial monitor with jitter moves
  if (abs(servo_x_pos - prev_servo_x_pos) > SERVO_UPDATE_THRESHOLD || abs(servo_y_pos - prev_servo_y_pos) > SERVO_UPDATE_THRESHOLD) {
    // Update servos with new angles and offsets
    servos.update_servos(total_angle_x, total_angle_y, offset_x, offset_y);

    // Output positional data to Serial Monitor
    Serial.print("Gyro Angle X: ");
    Serial.print(total_angle_x);
    Serial.print(" | Acc Angle X: ");
    Serial.print(acc_angle_x);
    Serial.print(" | Total X Degrees: ");
    Serial.print(total_angle_x);
    Serial.print(" | Gyro Angle Y: ");
    Serial.print(total_angle_y);
    Serial.print(" | Acc Angle Y: ");
    Serial.print(acc_angle_y);
    Serial.print(" | Total Y Degrees: ");
    Serial.println(total_angle_y);

    // Output servo moves to Serial Monitor
    Serial.print("Servo 1 (X-Axis) moved to: ");
    Serial.print(servo_x_pos + offset_x);
    Serial.print(" degrees | Servo 2 (Y-Axis) moved to: ");
    Serial.print(servo_y_pos + offset_y);
    Serial.println(" degrees");

    // Update previous positions
    prev_servo_x_pos = servo_x_pos;
    prev_servo_y_pos = servo_y_pos;
  }

  delay(10);
}

// Function to parse offsets from the serial input
void parse_offsets(String input) {
  input.trim();  // Remove whitespace
    int comma_index = input.indexOf(',');
    if (comma_index > 0) {
      String x_str = input.substring(0, comma_index);
      String y_str = input.substring(comma_index + 1);
      offset_x = x_str.toInt();
      offset_y = y_str.toInt();
      Serial.print("New offsets set - X: ");
      Serial.print(offset_x);
      Serial.print(", Y: ");
      Serial.println(offset_y);
  
      // Update servos with new offsets
      servos.update_servos(total_angle_x, total_angle_y, offset_x, offset_y);
    }

}
