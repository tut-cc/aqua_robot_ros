# [V]
float32 battery

# [G]
geometry_msgs/Vector3 accel

# [deg/s]
geometry_msgs/Vector3 angular_velocity

# [deg]
float32 pitch
float32 roll
float32 yaw

# emergency_modeがtrueであれば、モータは全て停止し、ESCから定期的にブザー音が鳴る
bool emergency_mode

aqua_robot_messages/MotorVelocity motor_velocity
