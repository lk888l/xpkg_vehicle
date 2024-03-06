##ROS driver for vehicle

How to use xpkg_XXX(XXX is your device) ros driver
=====================================================================
        1) Clone this ros package folder(xpkg_XXX) to your catkin's workspace src folder
        2) Run catkin_make to build
        3) Add path of setup.bash to ~/.bashrc like: echo "source ~/workspace/devel/setup.bash">> ~/.bashrc	source ~/.bashrc
        4) Change parameters in xnode_XXX_test.launch(xpkg_XXX/launch/xnode_XXX_test.launch)
        6) prepare xpkg_comm and Roslaunch xnode_comm.launch(xpkg_comm/launch/xnode_comm.launch)
        7) Roslaunch xnode_XXX_test.launch(xpkg_XXX/launch/xnode_XXX_test.launch)
        8) Use topic /xpkg_XXX/ctrl_json to control sensor function

Note
=====================================================================
        1) When device connect successfully,ros will print out:"xnode_XXX: device online"
        2) When device enable successfully,ros will print out:"xnode_XXX: device enable"
        3) When device ready,ros will print out:"xnode_XXX: device init finish"
        4) Don't Change parameter of "ini_path"
        5) This node will publish message "/xtopic_XXX/device_state_json" and "/xtopic_comm/com_send_xstd"
        6) Must run with xpkg_comm

Special note: please use include/LIB_JSON/ArduinoJson.h,view < https://arduinojson.org > for details

Mode
=====================================================================
        1) Standby :        (0) Recrive no order with motors locked
        2) Remoter :        (1) Use remoter for control
        3) CAN :            (2) Use CAN for control
        4) Free :           (3) Recrive no order with motors unlocked

Parameter
=====================================================================
        1) rate_x/y/z/az :      (0.0 to 1.0) rate of speed x,y,z and angular z
        2) show_loc :           (false or true) show location calculated from odom or speed
        3) show_path :          (false or true) show path in rviz
        4) calc_speed :         (false or true) use speed or odom calculate mode
        5) mode_can_lock :      (false or true)if auto lock on CAN mode
        6) ini_path :           DON'T CHANGE
        7) pub_tf :             (false or true)publish tf message

json out list(/xtopic_vehicle/device_state_json)
=====================================================================
        1) (unsigned char) device_type :            type of vehicle
        2) (unsigned char) device_num :             number of vehicle
        3) (bool) work_state :                      0=fine 1=error
        4) (bool) beep_state :                      0=off 1=on
        5) (bool) brake_state :                     0=off 1=on
        6) (unsigned char) work_mode :              0=standby 1=remote 2=CAN 3=free
        7) (bool) rotate_mode :                     0=angle vel 1=angle(for Ackermann model)
        8) (bool) spec_func_state :                 0=off 1=on, cheak datasheet for special function details
        9) (float) battery_vol :                    (V) voltage of battery
        10) (unsigned char) err_motor :             motor error code(check manual for details)
        11) (unsigned char) err_driver :            driver error code(check manual for details)
        12) (unsigned char) err_driver_offline :    driver offline code(check manual for details)
        13) (unsigned char) err_power :             power error code(check manual for details)
        14) (bool) err_bat_down :                   (0=fine 1=error) battery shutdown (<5%)
        15) (bool) err_bat_low :                    (0=fine 1=error) battery voltage low(<15%)
        16) (bool) err_ctrl_lost :                  (0=fine 1=error) remote controller lost
        17) (bool) err_bump :                       (0=fine 1=error) bumper touch
        18) (bool) err_stop :                       (0=fine 1=error) emergency stop pressed
        19) (double) speed_x :                      (m/s) speed of x axis
        20) (double) speed_y :                      (m/s) speed of y axis
        21) (double) speed_r :                      (rad/s) speed of rotate axis
        22) (double) odom_left_main :               (m) odom of left front wheel
        23) (double) odom_right_main :              (m) speed of right front wheel
        24) (double) odom_left_second :             (m) speed of left rear wheel
        25) (double) odmo_right_second :            (m) speed of right rear wheel

json in list(/xtopic_vehicle/ctrl_json)
=====================================================================
        1) (bool) beep_state :               0=off 1=on
        2) (bool) brake_state :              0=off 1=on
        3) (bool) spec_func_state :          0=off 1=on, cheak datasheet for special function details
        4) (unsigned char) work_mode :       0=standby 1=remote 2=CAN 3=free
        5) (bool) err_clear :                0=off 1=on,clear all error status
