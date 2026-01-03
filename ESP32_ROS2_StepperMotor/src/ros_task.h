#ifndef ROS_TASK_H
#define ROS_TASK_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

// --- Global ROS Objects ---
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg_pub;
std_msgs__msg__Float32 msg_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_ros; // Rename to avoid conflict with hw_timer_t

// --- Shared Variables (from your Motor Task) ---
extern volatile long targetPos;
extern volatile long currentPos; // Assuming you have this from the AS5600

// --- Callback: When ROS sends a command ---
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

  // Convert ROS float (Degree) to Encoder Counts
  // Input: Degrees (0-360...)
  // Encoder: 4096 counts per rev
  Serial.print("ROS Cmd Received: ");
  Serial.println(msg->data);
  targetPos = (long)(msg->data * (4096.0 / 360.0));
}

// --- Callback: Scheduled Telemetry Publish ---
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer != NULL) {
    // Convert Encoder Counts to Degrees for ROS
    msg_pub.data = (float)currentPos * (360.0 / 4096.0);
    rcl_publish(&publisher, &msg_pub, NULL);
  }
}

// --- The RTOS Task ---
void TaskMicroROS(void *pvParameters) {
  // 1. Setup Transport (WiFi)
  Serial.println("Connecting to WiFi...");
  IPAddress agent_ip(192, 168, 11, 47); // HOST IP
  size_t agent_port = 8888;
  char ssid[] = "LeoHome";
  char psk[] = "0922365286";

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  delay(2000); // Give WiFi time to settle
  Serial.println("Connected to WiFi!");

  allocator = rcl_get_default_allocator();

  // 2. Initialize Micro-ROS
  // Loop until agent is found
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Connecting to Agent...");
    delay(1000);
  }

  // 3. Create Node
  rclc_node_init_default(&node, "esp32_rail_cart", "", &support);

  // 4. Create Publisher (Encoder Feedback)
  rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor/encoder");

  // 5. Create Subscriber (Target Command)
  rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor/target_pos");

  // 6. Create Timer (Publish at 20Hz)
  const unsigned int timer_timeout = 50; // ms
  rclc_timer_init_default(&timer_ros, &support, RCL_MS_TO_NS(timer_timeout),
                          timer_callback);

  // 7. Create Executor
  rclc_executor_init(&executor, &support.context, 2,
                     &allocator); // Handles 2 handles (Sub + Timer)
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub,
                                 &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer_ros);

  Serial.println("Micro-ROS Agent Connected!");

  // 8. Spin Loop
  while (true) {
    rclc_executor_spin_some(
        &executor, RCL_MS_TO_NS(10000)); // Process messages, wait up to 10ms
    vTaskDelay(10 / portTICK_PERIOD_MS); // Yield to other tasks
  }
}

#endif
