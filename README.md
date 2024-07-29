# ROS-QT-Drone-App 

Boilerplate PyQt5 GUI code for PX4-ROS Research

GUI Displays:
- IMU data
- Local pos
- Local velocity
- Global pos
- Voltage
- Flight Time

Default Controls:
- MavROS State Machine (Arm, Disarm, Takeoff, Land)
- Mode switcher between Offboard and POSCTL
- Position setpoints

Everytime you make changes to the GUI in Qt Designer, generate the python file with  
```pyuic5 -x drone_app.ui -o GUI_drone_app.py ```
