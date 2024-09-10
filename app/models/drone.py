from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationGlobal, Command
from pymavlink import mavutil
import time
import math
import requests
from fastapi import HTTPException
from app.models.schemas import TakeoffInfo, ChangeModeInfo, GotoLocationInfo, WaypointList

class Drone:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.vehicle = None
        self.message = []

    def connect(self):
        try:
            connection_string = self.connection_string
            connection_type = connection_string.split(":")[0]
            if connection_type == "tcp" or connection_type == "direct_tcp":
                self.vehicle = connect(connection_string, wait_ready=True)
            elif connection_type == "direct":
                print("=============")
                print(connection_string.split(":")[1])
                print("=============")
                self.vehicle = connect(connection_string.split(":")[1], baud=921600, wait_ready=True)


            #self.vehicle = connect(connection_string, wait_ready=True)
            #오렌지파이용 연결명령
            #self.vehicle = connect('/dev/serial/by-id/usb-ArduPilot_KakuteH7v2_2C0036000151313131393134-if00', baud=115200, wait_ready=True)

            @self.vehicle.on_message('*')
            def listener(_, name, message):
                if name == 'MISSION_ACK' or name == 'MISSION_ITEM_REACHED' :
                    new_message = {"name": name, "message": str(message)}
                    self.message.append(new_message)

            # @self.vehicle.on_message('MISSION_ITEM_REACHED')
            # def mission_item_callback(_, name, message):
            #     new_message = {"name": name, "message": str(message)}
            #     self.message.append(new_message)

            # @self.vehicle.on_message('AUTOPILOT_VERSION')
            # def mission_item_callback(_, name, message):
            #     new_message = {"name": name, "message": str(message)}
            #     self.message.append(new_message)

            if self.vehicle.location.global_frame.alt is None:
                lat = float(self.vehicle.location.global_frame.lat)
                lon = float(self.vehicle.location.global_frame.lon)
                try:
                    alt = self.get_elevation(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)
                except:
                    alt = 0
                new_home_location = LocationGlobal(lat, lon, alt)
                self.vehicle.home_location = new_home_location
            return {"status": "Connected", "details": str(self.vehicle)}
        except APIException as e:
            return {"status": "Failed", "details": str(e)}

    def disconnect(self):
        if self.vehicle is not None:
            self.vehicle.close()
            self.vehicle = None
            return {"status": "Disconnected"}
        else:
            return {"status": "No active connection"}

    def drone_status(self):
        if self.vehicle is not None:
            current_location = self.vehicle.location.global_relative_frame
            attitude = self.vehicle.attitude
            return {
                "GPS": str(self.vehicle.gps_0),
                "Battery": str(self.vehicle.battery),
                "ekf_ok": self.vehicle.ekf_ok,
                "Heartbeat": str(self.vehicle.last_heartbeat),
                "Mode": str(self.vehicle.mode.name),
                "AirSpeed": str(self.vehicle.airspeed),
                "GroundSpeed": str(self.vehicle.groundspeed),
                "Home": str(self.vehicle.home_location),
                "Lat": current_location.lat,
                "Lng": current_location.lon,
                "Alt": current_location.alt,
                "SL_Alt": self.vehicle.location.global_frame.alt,
                "Roll": attitude.roll,
                "Pitch": attitude.pitch,
                "Yaw": math.degrees(attitude.yaw),
                "Armed": self.vehicle.armed
            }
        else:
            raise HTTPException(status_code=400, detail="No active drone connection")

    def change_mode(self, change_mode_info: ChangeModeInfo):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")
        # try:
        #     new_mode = VehicleMode(change_mode_info.new_mode)
        #     self.vehicle.mode = new_mode
        #     while not self.vehicle.mode.name == change_mode_info.new_mode:
        #         time.sleep(1)
        #     return {"status": "모드 변경됨", "new_mode": change_mode_info.new_mode}
        # except APIException as e:
        #     raise HTTPException(status_code=500, detail=str(e))
        
        try:
            new_mode = VehicleMode(change_mode_info.new_mode)
            self.vehicle.mode = new_mode
            start_time = time.time()
            timeout = 5  # 최대 대기 시간 (초)
            while not self.vehicle.mode.name == change_mode_info.new_mode:
                if time.time() - start_time > timeout:
                    raise HTTPException(status_code=500, detail="Failed to change mode to GUIDED within the timeout period.")
                time.sleep(1)
            if self.vehicle.mode.name == "LOITER":
                self.vehicle.channels.overrides['3'] = 1500
            if self.vehicle.mode.name == "GUIDED":
                self.send_ned_velocity(0, -1, 0, 1)
            return {"status": "모드 변경됨", "new_mode": change_mode_info.new_mode}
        except APIException as e:
            raise HTTPException(status_code=500, detail=str(e))



    def land(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")
        try:
            new_mode = VehicleMode("LAND")
            self.vehicle.mode = new_mode
            while not self.vehicle.mode.name == "LAND":
                time.sleep(1)
            return {"status": "착륙이 시작되었습니다."}
        except APIException as e:
            raise HTTPException(status_code=500, detail=str(e))

    def arm_drone(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="No active drone connection")
        if self.vehicle.mode != 'GUIDED':
            self.vehicle.mode = VehicleMode("GUIDED")
            while not self.vehicle.mode.name == "GUIDED":
                time.sleep(1)
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)
        return {"status": "Armed"}

    def takeoff(self, takeoff_info: TakeoffInfo):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")
        if not self.vehicle.armed:
            raise HTTPException(status_code=400, detail="드론이 시동되지 않았습니다. 먼저 시동을 걸어주세요.")
        if self.vehicle.mode.name != "GUIDED":
            raise HTTPException(status_code=400, detail="드론이 GUIDED 모드가 아닙니다.")
        target_altitude = float(takeoff_info.target_altitude)
        self.vehicle.simple_takeoff(target_altitude)
        return {"status": "start takeoff"}

    def current_location(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")
        current_location = self.vehicle.location.global_relative_frame
        attitude = self.vehicle.attitude
        return {
            "latitude": current_location.lat,
            "longitude": current_location.lon,
            "altitude": current_location.alt,
            "sea_level_altitude": self.vehicle.location.global_frame.alt,
            "roll": attitude.roll,
            "pitch": attitude.pitch,
            "yaw": math.degrees(attitude.yaw),
            "armed": self.vehicle.armed
        }

    def goto_location(self, goto_location_info: GotoLocationInfo):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")
        if self.vehicle.mode.name != "GUIDED":
            try:
                self.vehicle.mode = VehicleMode("GUIDED")
                start_time = time.time()
                timeout = 5  # 최대 대기 시간 (초)
                while not self.vehicle.mode.name == "GUIDED":
                    if time.time() - start_time > timeout:
                        raise HTTPException(status_code=500, detail="Failed to change mode to GUIDED within the timeout period.")
                    time.sleep(1)
            except APIException as e:
                raise HTTPException(status_code=500, detail=str(e))
        self.vehicle.groundspeed = 15
        if goto_location_info.method == "relative":
            target_location = LocationGlobalRelative(goto_location_info.latitude, goto_location_info.longitude, goto_location_info.altitude)
        else:
            target_location = LocationGlobal(goto_location_info.latitude, goto_location_info.longitude, goto_location_info.altitude)
        self.vehicle.simple_goto(target_location)
        return {"status": "이동 시작", "target_location": goto_location_info}

    def upload_mission(self, waypoint_list: WaypointList):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")
        cmds = self.vehicle.commands
        cmds.clear()
        while len(cmds) > 0:
            time.sleep(0.5)
        for waypoint in waypoint_list.waypoints:
            command = self.get_command(waypoint.command)
            altitudeType = self.get_altitude_type(waypoint.altitudeType)
            cmds.add(Command(0, 0, 0, altitudeType, command, 0, 0, waypoint.delay, waypoint.radius, 0, 0, waypoint.latitude, waypoint.longitude, waypoint.altitude))
        cmds.upload()
        return {"status": "success"}

    def download_mission(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="No active drone connection")
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        missionlist = []
        for cmd in cmds:
            command_data = {
                "seq": cmd.seq,
                "frame": cmd.frame,
                "command": cmd.command,
                "current": cmd.current,
                "autocontinue": cmd.autocontinue,
                "param1": cmd.param1,
                "param2": cmd.param2,
                "param3": cmd.param3,
                "param4": cmd.param4,
                "x": cmd.x,
                "y": cmd.y,
                "z": cmd.z
            }
            missionlist.append(command_data)
        return {"mission": missionlist}

    def disarm_drone(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="No active drone connection")
        if self.vehicle.armed:
            self.vehicle.armed = False
            while self.vehicle.armed:
                time.sleep(1)
            return {"status": "기체 시동을 해제하였습니다."}
        else:
            return {"status": "기체의 시동이 이미 해제되어 있습니다."}

    def get_elevation(self, lat, lon):
        query = f"https://api.open-elevation.com/api/v1/lookup?locations={lat},{lon}"
        response = requests.get(query).json()
        elevation = response['results'][0]['elevation']
        return elevation

    def get_command(self, command_name):
        command_dict = {
            "waypoint": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "takeoff": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            "land": mavutil.mavlink.MAV_CMD_NAV_LAND
        }
        return command_dict.get(command_name, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)

    def get_altitude_type(self, altitude_type_name):
        altitude_type_dict = {
            "relative": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "terrain": mavutil.mavlink.MAV_FRAME_GLOBAL,
            "absolute": mavutil.mavlink.MAV_FRAME_GLOBAL
        }
        return altitude_type_dict.get(altitude_type_name, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def common_callback(self, vehicle, name, message):
        if name == 'MISSION_ACK':
            new_message = {"name": name, "message": str(message)}
            self.message.append(new_message)

    def download_message(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="No active drone connection")
        return {"message": self.message}
    
    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        드론에 NED 좌표계를 기준으로 속도 명령을 보냅니다.
        :param velocity_x: 북쪽으로의 속도 (m/s)
        :param velocity_y: 동쪽으로의 속도 (m/s)
        :param velocity_z: 아래로의 속도 (m/s)
        :param duration: 명령을 지속할 시간 (s)
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # 타임스탬프
            0, 0,  # 타깃 시스템, 타깃 컴포넌트
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 좌표계
            0b0000111111000111,  # 제어할 항목
            0, 0, 0,  # x, y, z 위치
            velocity_x, velocity_y, velocity_z,  # x, y, z 속도
            0, 0, 0,  # x, y, z 가속도 (사용 안 함)
            0, 0)  # yaw, yaw_rate (사용 안 함)

        for x in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)
