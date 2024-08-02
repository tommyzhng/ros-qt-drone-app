import rospy
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtWidgets import QMessageBox
import common
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from mavros_msgs.srv import CommandLong, SetMode
from mavros_msgs.msg import State, ActuatorControl
from geometry_msgs.msg import PoseStamped
import json
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class SingleDroneRosNode(QObject):
    ## define signals
    update_data = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.data_struct = common.CommonData()
        # define subscribers
        self.imu_sub = rospy.Subscriber('mavros/imu/data', Imu, callback=self.imu_sub)
        self.pos_global_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, callback=self.pos_global_sub)
        self.odom_local_sub = rospy.Subscriber('mavros/local_position/odom', Odometry, callback=self.odom_local_sub)
        self.bat_sub = rospy.Subscriber('mavros/battery', BatteryState, callback=self.bat_sub)
        self.mavros_status_sub = rospy.Subscriber('mavros/state', State, callback=self.mavros_status_sub)

        # define publishers / services
        self.arming_service = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
        self.land_service = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
        self.set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # vrs state machine publishers
        self.coords_pub = rospy.Publisher('/vrs_failsafe/setpoint_position', PoseStamped, queue_size=1)
        self.vel_pub = rospy.Publisher('/vrs_failsafe/setpoint_drop_vel', Float32, queue_size=1)
        self.state_pub = rospy.Publisher('/vrs_failsafe/state', String, queue_size=1)
        self.tilt_angle_pub = rospy.Publisher('vrs_failsafe/gui_servo_setpoint', Float32, queue_size=1)
        # other
        self.rate = rospy.Rate(15)

        # read config file
        with open('config/config.json') as f:
            geofence = json.load(f)
            self.config = [0, 0, 0]
            self.config[0] = geofence['x']
            self.config[1] = geofence['y']
            self.config[2] = geofence['z']
        
    ### define signal connections to / from gui ###
    def connect_update_gui(self, callback):
        self.update_data.connect(callback)

    ### define callback functions from ros topics ###
    def imu_sub(self, msg): 
        self.data_struct.update_imu(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)    
    def pos_global_sub(self, msg):
        self.data_struct.update_global_pos(msg.latitude, msg.longitude, msg.altitude)
    def odom_local_sub(self, msg):
        self.data_struct.update_local_pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self.data_struct.update_vel(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)    
    def bat_sub(self, msg):
        self.data_struct.update_bat(msg.percentage, msg.voltage)
    def mavros_status_sub(self, msg):
        self.data_struct.update_state(msg.connected, msg.armed, msg.manual_input, msg.mode, msg.header.stamp.secs)

    ## define publisher functions###
    def publish_coordinates(self, x, y, z, yaw):
        pos_targ = PoseStamped()
        pos_targ.header.stamp = rospy.Time.now()
        pos_targ.pose.position.x = x
        pos_targ.pose.position.y = y
        pos_targ.pose.position.z = z
        pos_targ.pose.orientation.z = yaw
        self.coords_pub.publish(pos_targ)
        self.state_pub.publish(String("posSetpoint"))

    def publish_drop_velocity(self, vel):
        self.vel_pub.publish(vel)
        self.state_pub.publish(String("dropVelSetpoint"))

    def publish_servo_setpoint(self, angle):
        self.tilt_angle_pub.publish(angle)
        self.state_pub.publish(String("posSetpoint"))

    # main loop of ros node
    def run(self):
        while not rospy.is_shutdown():
            try:
                self.update_data.emit(0)
                self.rate.sleep()
            except rospy.ROSInterruptException:
                print("ROS Shutdown Requested")
                break

class SingleDroneRosThread:
    def __init__(self, ui):
        super().__init__()
        self.ros_object = SingleDroneRosNode()
        self.thread = QThread()

        # setup signals
        self.ui = ui
        self.set_ros_callbacks()

        # move and start thread
        self.ros_object.moveToThread(self.thread)
        self.lock = self.ros_object.data_struct.lock
        self.thread.started.connect(self.ros_object.run)

    def start(self):
        self.thread.start()
        self.switch_flight_mode("POSCTL")

    # define the signal-slot combination of ros and pyqt GUI
    def set_ros_callbacks(self):
        # feedbacks from ros
        self.ros_object.connect_update_gui(self.update_gui_data)

        # callbacks from GUI
        self.ui.SendPositionUAV.clicked.connect(self.send_coordinates)
        self.ui.GetCurrentPositionUAV.clicked.connect(self.get_coordinates)

        self.ui.ARM.clicked.connect(lambda: self.send_arming_request(True, 0))
        self.ui.DISARM.clicked.connect(lambda: self.send_arming_request(False, 0))
        self.ui.Land.clicked.connect(lambda: self.send_land_request())
        self.ui.EmergencyStop.clicked.connect(lambda: self.send_arming_request(False, 21196))

        self.ui.OFFBOARD.clicked.connect(lambda: self.switch_flight_mode("OFFBOARD"))
        self.ui.POSCTL.clicked.connect(lambda: self.switch_flight_mode("POSCTL"))
        self.ui.HOLD.clicked.connect(lambda: self.switch_flight_mode("AUTO.LOITER"))

        # vrs testing
        self.ui.btnSetHeight.clicked.connect(lambda: self.send_set_height_request(float(self.ui.tbDropHeight.text())))
        self.ui.btnDropVelocity.clicked.connect(lambda: self.send_drop_velocity(-float(self.ui.tbDropVelocity.text())))
        self.ui.btnFreefall.clicked.connect(lambda: self.ros_object.state_pub.publish(String("freefall")))
        self.ui.btnTiltAngle.clicked.connect(lambda: self.send_tilt_angle(float(self.ui.tbTiltAngle.text())))

    # update GUI data
    def update_gui_data(self):
        if not self.lock.tryLock():
            print("SingleDroneRosThread: lock failed")
            return
        # store to local variables for fast lock release
        self.imu_msg = self.ros_object.data_struct.current_imu
        global_pos_msg = self.ros_object.data_struct.current_global_pos
        self.local_pos_msg = self.ros_object.data_struct.current_local_pos
        vel_msg = self.ros_object.data_struct.current_vel
        bat_msg = self.ros_object.data_struct.current_battery_status
        state_msg = self.ros_object.data_struct.current_state
        self.lock.unlock()

        # accelerometer data
        self.ui.X_DISP.display("{:.2f}".format(self.imu_msg.roll, 2))
        self.ui.Y_DISP.display("{:.2f}".format(self.imu_msg.pitch, 2))
        self.ui.Z_DISP.display("{:.2f}".format(self.imu_msg.yaw, 2))

        # global & local position data
        self.ui.LatGPS_DISP.display("{:.2f}".format(global_pos_msg.latitude, 2))
        self.ui.LongGPS_DISP.display("{:.2f}".format(global_pos_msg.longitude, 2))
        self.ui.AltGPS_DISP.display("{:.2f}".format(global_pos_msg.altitude, 2))
        self.ui.RelX_DISP.display("{:.2f}".format(self.local_pos_msg.x, 2))
        self.ui.RelY_DISP.display("{:.2f}".format(self.local_pos_msg.y, 2))
        self.ui.AGL_DISP.display("{:.2f}".format(self.local_pos_msg.z, 2))

        # velocity data
        self.ui.U_Vel_DISP.display("{:.2f}".format(vel_msg.x, 2))
        self.ui.V_Vel_DISP.display("{:.2f}".format(vel_msg.y, 2))
        self.ui.W_Vel_DISP.display("{:.2f}".format(vel_msg.z, 2))

        # state updates
        self.ui.StateARM.setText("Armed" if state_msg.armed else "Disarmed")
        self.ui.StateARM.setStyleSheet("color: red" if state_msg.armed else "color: green")
        self.ui.StateConnected.setText("Connected" if state_msg.connected else "Disconnected")
        self.ui.StateConnected.setStyleSheet("color: green" if state_msg.connected else "color: red")
        self.ui.StateMode.setText(state_msg.mode)

        # misc data
        if bat_msg: # takes long to initialize
            if self.ui.BatInd.isTextVisible() == False:
                self.ui.BatInd.setTextVisible(True)
            self.ui.BatInd.setValue(float(bat_msg.percentage)*100)
            self.ui.VOLT_DISP.display("{:.2f}".format(bat_msg.voltage, 2))

        # update seconds
        if not hasattr(self, 'seconds'):
            self.armed_seconds = 0
        if not hasattr(self, 'last_time'):
            self.last_time = state_msg.seconds
        if state_msg.armed:
            self.armed_seconds = state_msg.seconds - self.last_time         # time since armed
            self.ui.Sec_DISP.display("{}".format(self.armed_seconds, 1))
        else:                                                               # reset time if disarmed
            self.last_time = state_msg.seconds
            self.armed_seconds = 0
        # update minutes
        if self.armed_seconds >= 60:                                        # update minutes if seconds > 60 and reset seconds
            self.ui.Min_DISP.display("{}".format(int(self.ui.Min_DISP.value() + 1), 1))
            self.last_time = state_msg.seconds

    def get_coordinates(self):
        # get current relative position
        self.ui.XPositionUAV.setText("{:.2f}".format(self.local_pos_msg.x, 2))
        self.ui.YPositionUAV.setText("{:.2f}".format(self.local_pos_msg.y, 2))
        self.ui.ZPositionUAV.setText("{:.2f}".format(self.local_pos_msg.z, 2))
        self.ui.YAWUAV.setText("{:.2f}".format(self.imu_msg.yaw, 2))

    ### define publish / service functions to ros topics ###
    def send_arming_request(self, arm, param2):
        response = self.ros_object.arming_service(command=400, confirmation=0, param1 = arm, param2 = param2)
        print(response)
        return response

    def send_land_request(self):
        response = self.ros_object.land_service(command=21, confirmation=0, param1 = 0, param7 = 0)
        print(response)

    def switch_flight_mode(self, mode):
        response = self.ros_object.set_mode_service(custom_mode=mode)
        print(response)

    def send_set_height_request(self, req_altitude):
        arm_response = self.send_arming_request(True, 0)
        # if armed takeoff
        if arm_response.result == 0:
            self.ros_object.publish_coordinates(self.local_pos_msg.x, self.local_pos_msg.y, req_altitude, (90-self.imu_msg.yaw))
            print(f"Drop request sent at {req_altitude} meters")

    def send_drop_velocity(self, vel):
        self.ros_object.publish_drop_velocity(vel)

    def send_tilt_angle(self, angle):
        self.ros_object.publish_servo_setpoint(angle)

    def send_coordinates(self):
        # if text is inalid, warn user
        try :
            x = float(self.ui.XPositionUAV.text())
            y = float(self.ui.YPositionUAV.text())
            z = float(self.ui.ZPositionUAV.text())
            yaw = (90 - float(self.ui.YAWUAV.text()))
        except ValueError:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Invalid input, make sure values are numbers")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return
        # if values are not within 5 meters of current position warn user
        if abs(x) > int(self.ros_object.config[0]) or abs(y) > int(self.ros_object.config[1]) or abs(z) > int(self.ros_object.config[2]) or z <= 0:
            ## pop up dialog 
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(f"Position is not within geofence: {self.ros_object.config}")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return

        self.ros_object.publish_coordinates(x, y, z, yaw)
