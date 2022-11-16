#Onboard data from the tello drone
#MVO(Monocular Visual Odometry)(Not very dependable) data:
#velocity:
float32 vel_x
float32 vel_y
float32 vel_z
#position:
float32 pos_x
float32 pos_y
float32 pos_z

#IMU data:
#Accelerometer:
float32 acc_x
float32 acc_y
float32 acc_z
#Gyroscope
float32 gyro_x
float32 gyro_y
float32 gyro_z
#Quaternion
float32 q0
float32 q1
float32 q2
float32 q3
#Speed
float32 vg_x
float32 vg_y
float32 vg_z

#Flight data:
int16 battery_percentage
float32 height
float32 ground_speed
float32 east_speed
float32 north_speed