FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/fmMsgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/fmMsgs/msg/__init__.py"
  "../src/fmMsgs/msg/_rtq.py"
  "../src/fmMsgs/msg/_engine_rpm.py"
  "../src/fmMsgs/msg/_xyz_r_t.py"
  "../src/fmMsgs/msg/_adc.py"
  "../src/fmMsgs/msg/_gpgga.py"
  "../src/fmMsgs/msg/_accelerometer.py"
  "../src/fmMsgs/msg/_claas_row_cam.py"
  "../src/fmMsgs/msg/_nmea.py"
  "../src/fmMsgs/msg/_lidar_safety_zone.py"
  "../src/fmMsgs/msg/_steering_angle_cmd.py"
  "../src/fmMsgs/msg/_magnetometer.py"
  "../src/fmMsgs/msg/_gyroscope.py"
  "../src/fmMsgs/msg/_positions.py"
  "../src/fmMsgs/msg/_row.py"
  "../src/fmMsgs/msg/_gps_state.py"
  "../src/fmMsgs/msg/_rtq_command.py"
  "../src/fmMsgs/msg/_serial.py"
  "../src/fmMsgs/msg/_rtq_lamp_command.py"
  "../src/fmMsgs/msg/_encoder.py"
  "../src/fmMsgs/msg/_motor_status.py"
  "../src/fmMsgs/msg/_joint_conf.py"
  "../src/fmMsgs/msg/_can.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
