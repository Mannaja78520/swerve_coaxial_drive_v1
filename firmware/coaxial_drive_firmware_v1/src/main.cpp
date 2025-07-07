#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>

#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_AS5600.h>
#include <Utilize.h>
#include <TCA9548A.h>

#if defined(ESP32)
    #include <esp32_Encoder.h>    
    #include <ESP32Servo.h>
#else
    #include<Servo.h>
    #include <Encoder.h>
#endif



#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

//------------------------------ < Define > -------------------------------------//

rcl_publisher_t debug_move_wheel_motor_publisher;
rcl_publisher_t debug_move_wheel_encoder_publisher;
rcl_publisher_t debug_move_wheel_angle_publisher;

rcl_subscription_t move_wheel_motor_subscriber;
rcl_subscription_t movement_mode_subscriber;
rcl_subscription_t wheel_angle_subscriber;

std_msgs__msg__String movement_mode_msg;

geometry_msgs__msg__Twist debug_wheel_motor_msg;
geometry_msgs__msg__Twist debug_wheel_encoder_msg;
geometry_msgs__msg__Twist debug_wheel_angle_msg;
geometry_msgs__msg__Twist moveMotor_msg;
geometry_msgs__msg__Twist wheel_angle_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;
static unsigned long last_pub = 0;
static int disconnect_count = 0;

String movement_mode = "mps"; 
float motor1_target = 0.0;
float motor2_target = 0.0;
float motor3_target = 0.0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

Adafruit_AS5600 as5600;  // One for each sensor
PIDF pidf_wheel[3] = {
    PIDF(Servo_Motor_MinSpeed, Servo_Motor_MaxSpeed, Servo_Motor_KP, Servo_Motor_KI, Servo_Motor_I_Min, Servo_Motor_I_Max, Servo_Motor_KD, Servo_Motor_KF, Servo_Motor_ERROR_TOLERANCE),
    PIDF(Servo_Motor_MinSpeed, Servo_Motor_MaxSpeed, Servo_Motor_KP, Servo_Motor_KI, Servo_Motor_I_Min, Servo_Motor_I_Max, Servo_Motor_KD, Servo_Motor_KF, Servo_Motor_ERROR_TOLERANCE),
    PIDF(Servo_Motor_MinSpeed, Servo_Motor_MaxSpeed, Servo_Motor_KP, Servo_Motor_KI, Servo_Motor_I_Min, Servo_Motor_I_Max, Servo_Motor_KD, Servo_Motor_KF, Servo_Motor_ERROR_TOLERANCE),
};

float servo_zero_point[3] = {CONTINUTE_SERVO1_ZERO_POINT, CONTINUTE_SERVO2_ZERO_POINT, CONTINUTE_SERVO3_ZERO_POINT}; // Zero point for each servo

Servo servo_wheel[3];
const int servoPins[3] = {CONTINUTE_SERVO1_PIN, CONTINUTE_SERVO2_PIN, CONTINUTE_SERVO3_PIN};

PIDF motor1_pidf(PWM_Max, PWM_Min, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
PIDF motor2_pidf(PWM_Max, PWM_Min, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
PIDF motor3_pidf(PWM_Max, PWM_Min, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);

// Move motor
Controller motor1(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BRAKE, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Controller motor3(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_BRAKE, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);


// Move Encoder
#if defined(ESP32)
    esp32_Encoder Encoder1(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV, MOTOR1_ENCODER_RATIO);
    esp32_Encoder Encoder2(MOTOR2_ENCODER_PIN_A, MOTOR2_ENCODER_PIN_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV, MOTOR2_ENCODER_RATIO);
    esp32_Encoder Encoder3(MOTOR3_ENCODER_PIN_A, MOTOR3_ENCODER_PIN_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV, MOTOR3_ENCODER_RATIO);
#else
    Encoder Encoder1(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV, MOTOR1_ENCODER_RATIO);
    Encoder Encoder2(MOTOR2_ENCODER_PIN_A, MOTOR2_ENCODER_PIN_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV, MOTOR2_ENCODER_RATIO);
    Encoder Encoder3(MOTOR3_ENCODER_PIN_A, MOTOR3_ENCODER_PIN_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV, MOTOR3_ENCODER_RATIO);
#endif

TCA9548A tca;
//------------------------------ < Fuction Prototype > ------------------------------//

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void flashLED(unsigned int);
struct timespec getTime();

void publishData();
void getEncoderData();
void Move();
void MoveRPM();
void RotageWheel();
void MovePower(float, float, float);
//------------------------------ < Main > -------------------------------------//

void setup()
{

    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    #if defined(ESP32)
        tca.begin();

        for (int i = 0; i < AS5600_COUNT; i++) {

            if (!tca.selectChannel(FIRST_TCA_CHANNEL + i)) {
                Serial.print("Failed to select TCA channel ");
                Serial.println(FIRST_TCA_CHANNEL + i);
                continue;
            }

            if (!as5600.begin()) {
                Serial.print("AS5600 ");
                Serial.print(i);
                Serial.println(" not found!");
                continue;
            }

            // You can configure each individually if needed
            as5600.setPowerMode(AS5600_POWER_MODE_NOM);
            as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
            as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
            as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
            as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
            as5600.setZPosition(0);
            as5600.setMPosition(4095);
            as5600.setMaxAngle(4095);
        }

        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);

        for (int i = 0; i < 3; i++) {
            servo_wheel[i].setPeriodHertz(50);
            servo_wheel[i].attach(servoPins[i], 500, 2500);
            servo_wheel[i].write(90);
        }
        // Serial.println("Initializing micro-ROS transport...");
        
        #endif
    #ifdef MICROROS_WIFI
        IPAddress agent_ip(AGENT_IP);
        uint16_t agent_port = AGENT_PORT;
        set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip, agent_port);
    #else
        set_microros_serial_transports(Serial);
    #endif
}

void loop()
{
    #ifdef MICROROS_WIFI
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.begin((char*)SSID, (char*)SSID_PW);
            delay(500);
        }
    #endif

    // EXECUTE_EVERY_N_MS(1000, digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)););
    
    switch (state)
    {
    case WAITING_AGENT:
        // EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        #if defined(ESP32)
            EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        #else
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        #endif
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        #if defined(ESP32)
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        #else
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        #endif
        if (state == AGENT_CONNECTED)
        {
            #if defined(ESP32)
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(300));
            #else
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            #endif
        }
        break;
    case AGENT_DISCONNECTED:
        MovePower(0, 0, 0);
        destroyEntities();
        disconnect_count = 0;
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//

void MovePower(float Motor1Speed, float Motor2Speed, float Motor3Speed)
{
    motor1.spin(Motor1Speed);
    motor2.spin(Motor2Speed);
    motor3.spin(Motor3Speed);
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        getEncoderData();
        if (movement_mode == "rpm"){
        }
        else if (movement_mode == "manual"){
        MoveRPM();
            Move();
        }
        RotageWheel();
        publishData();
        // if (millis() - last_pub > 200) { // Every 200ms
        //     last_pub = millis();
        // }
    }
}

void wheelMoveCallback(const void *msgin)
{
    prev_cmd_time = millis();
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    motor1_target = msg->linear.x;
    motor2_target = msg->linear.y;
    motor3_target = msg->linear.z;
    // motor1.spin(motor1_target);
}

void movementModeCallback(const void *msgin)
{
    prev_cmd_time = millis();
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    movement_mode = String(msg->data.data);  // Copy it to global for later use
    // const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    // movement_mode = String(msg->data.data);
    // Example: Print mode (optional)
    // Serial.print("Movement Mode: ");
    // Serial.println(msg->data.data);
}

void wheelAngleCallback(const void *msgin)
{
    prev_cmd_time = millis();
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    wheel_angle_msg = *msg;
    // const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    // Copy the wheel angles to global variable
    // wheel_angle_msg = *msg;
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // create node
    RCCHECK(rclc_node_init_default(&node, "coaxial_swerve_basemove_hardware", "", &support));

    // Pub
    RCCHECK(rclc_publisher_init_best_effort(
        &debug_move_wheel_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/wheel/motor_speed"));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_move_wheel_encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/wheel/encoder_rpm"));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_move_wheel_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/wheel/angle"));

    // Sub

    RCCHECK(rclc_subscription_init_default(
        &move_wheel_motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/wheel/motor_speed"));

    RCCHECK(rclc_subscription_init_default(
        &movement_mode_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/movement_mode"));

    RCCHECK(rclc_subscription_init_default(
        &wheel_angle_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/wheel/angle"));

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 40;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &move_wheel_motor_subscriber,
        &moveMotor_msg,
        &wheelMoveCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &movement_mode_subscriber,
        &movement_mode_msg,
        &movementModeCallback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &wheel_angle_subscriber,
        &wheel_angle_msg,
        &wheelAngleCallback,
        ON_NEW_DATA));

    // synchronize time with the agent
    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_move_wheel_motor_publisher, &node);
    rcl_publisher_fini(&debug_move_wheel_encoder_publisher, &node);
    rcl_subscription_fini(&move_wheel_motor_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void Move()
{
    MovePower(motor1_target, motor2_target, motor3_target);
}

void MoveRPM()
{
    // Convert the linear.x, linear.y, and linear.z to RPM for each motor
    // Set the RPM for each motor
    float motor1Speed = motor1_pidf.compute(motor1_target, debug_wheel_encoder_msg.linear.x);
    float motor2Speed = motor2_pidf.compute(motor2_target, debug_wheel_encoder_msg.linear.y);
    float motor3Speed = motor3_pidf.compute(motor3_target, debug_wheel_encoder_msg.linear.z);
    MovePower(motor1Speed, motor2Speed, motor3Speed);
}

void RotageWheel()
{
    float targetAngles[3] = {
        WrapDegs(wheel_angle_msg.linear.x),
        WrapDegs(wheel_angle_msg.linear.y),
        WrapDegs(wheel_angle_msg.linear.z)
    };

    for (int i = 0; i < AS5600_COUNT; i++) {
        int tca_channel = i + FIRST_TCA_CHANNEL;
        tca.selectChannel(tca_channel);
        uint16_t raw = as5600.getAngle();
        if (raw == 0xFFFF) continue;
        float currentAngle = WrapDegs(WrapDegs(raw * 360.0 / 4096.0) + servo_zero_point[i]);
        float error = WrapDegs(targetAngles[i] - currentAngle);
        float output = pidf_wheel[i].compute_with_error(error);
        float pulse = 1500 + output;
        
        pulse = constrain(pulse, 500, 2500);
        servo_wheel[i].writeMicroseconds(pulse);
        if (i == 0) debug_wheel_angle_msg.linear.x = currentAngle;
        else if (i == 1) debug_wheel_angle_msg.linear.y = currentAngle;
        else if (i == 2) debug_wheel_angle_msg.linear.z = currentAngle;

    }
}

void getEncoderData()
{
    // Get encoder data
    debug_wheel_encoder_msg.linear.x = Encoder1.getRPM();
    debug_wheel_encoder_msg.linear.y = Encoder2.getRPM();
    debug_wheel_encoder_msg.linear.z = Encoder3.getRPM();

}

void publishData()
{
    debug_wheel_motor_msg.linear.x = moveMotor_msg.linear.x;
    debug_wheel_motor_msg.linear.y = moveMotor_msg.linear.y;
    debug_wheel_motor_msg.linear.z = moveMotor_msg.linear.z;
    struct timespec time_stamp = getTime();
    rcl_publish(&debug_move_wheel_motor_publisher, &debug_wheel_motor_msg, NULL);
    rcl_publish(&debug_move_wheel_encoder_publisher, &debug_wheel_encoder_msg, NULL);
    rcl_publish(&debug_move_wheel_angle_publisher, &debug_wheel_angle_msg, NULL);
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    while (true)
    {
        flashLED(3);
    }
}

void flashLED(unsigned int n_times)
{
    // for (int i = 0; i < n_times; i++)
    // {
        // digitalWrite(LED_BUILTIN, HIGH);
        // delay(100);
        // digitalWrite(LED_BUILTIN, LOW);
        // delay(100);
    // }
    delay(1000);
}