from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationGlobal
from typing import List
from dronekit import Command
from pymavlink import mavutil
import time
import math
import requests

app = FastAPI()

# CORS 미들웨어 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 모든 오리진 허용
    allow_credentials=True,
    allow_methods=["*"],  # 모든 메소드 허용
    allow_headers=["*"],  # 모든 헤더 허용
)

class ConnectionInfo(BaseModel):
    connection_string: str
    drone_id: str  # 드론을 구분하는 고유 ID

class TakeoffInfo(BaseModel):
    drone_id: str
    target_altitude: str

class ChangeModeInfo(BaseModel):
    drone_id: str
    new_mode: str

class GotoLocationInfo(BaseModel):
    drone_id: str
    latitude: float
    longitude: float
    altitude: float
    method: str

class Waypoint(BaseModel):
    command: str
    altitudeType: str
    delay: int
    radius: int
    latitude: float
    longitude: float
    altitude: float

class WaypointList(BaseModel):
    drone_id: str
    waypoints: List[Waypoint]

class Drone:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.vehicle = None

    def connect(self):
        try:
            self.vehicle = connect(self.connection_string, wait_ready=True)

            if (self.vehicle.location.global_frame.alt==None):
                lat = float(self.vehicle.location.global_frame.lat)
                lon = float(self.vehicle.location.global_frame.lon)
                alt = self.get_elevation(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)
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
                "Lat": current_location.lat,  # 위도
                "Lng": current_location.lon,  # 경도
                "Alt": current_location.alt,  # 상대 고도
                "SL_Alt": self.vehicle.location.global_frame.alt,  # 해수면 고도
                "Roll": attitude.roll,  # 롤
                "Pitch": attitude.pitch,  # 피치
                "Yaw": math.degrees(attitude.yaw),  # 요
                "Armed": self.vehicle.armed  # 시동여부
            }
        else:
            raise HTTPException(status_code=400, detail="No active drone connection")
        
    def change_mode(self, change_mode_info: ChangeModeInfo):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

        try:
            new_mode = VehicleMode(change_mode_info.new_mode)
            self.vehicle.mode = new_mode

            # 모드가 변경될 때까지 기다림
            while not self.vehicle.mode.name == change_mode_info.new_mode:
                print("모드 변경을 기다리는 중...")
                time.sleep(1)

            return {"status": "모드 변경됨", "new_mode": change_mode_info.new_mode}
        except APIException as e:
            raise HTTPException(status_code=500, detail=str(e))
        
    def land(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

        try:
            new_mode = VehicleMode("LAND")
            self.vehicle.mode = new_mode

            # 모드가 변경될 때까지 기다림
            while not self.vehicle.mode.name == "LAND":
                print("모드 변경을 기다리는 중...")
                time.sleep(1)

            return {"status": "착륙이 시작되었습니다."}
        except APIException as e:
            raise HTTPException(status_code=500, detail=str(e))
        
    
        
    def arm_drone(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="No active drone connection")

        if self.vehicle.mode != 'GUIDED':
            #raise HTTPException(status_code=400, detail="Drone is not in GUIDED mode")
            self.vehicle.mode = VehicleMode("GUIDED")

            # 모드가 변경될 때까지 기다림
            while not self.vehicle.mode.name == "GUIDED":
                print("모드 변경을 기다리는 중...")
                time.sleep(1)

        self.vehicle.armed = True

        # 시동이 걸릴 때까지 기다림
        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

        return {"status": "Armed"}
    
    def takeoff(self, takeoff_info: TakeoffInfo):
        if self.vehicle is None:
            print("No active drone connection")
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

        if not self.vehicle.armed:
            print("Drone is not armed")
            raise HTTPException(status_code=400, detail="드론이 시동되지 않았습니다. 먼저 시동을 걸어주세요.")

        if self.vehicle.mode.name != "GUIDED":
            print("Drone is not in GUIDED mode")
            raise HTTPException(status_code=400, detail="드론이 GUIDED 모드가 아닙니다.")

        print("start takeoff...")
        target_altitude = float(takeoff_info.target_altitude)
        self.vehicle.simple_takeoff(target_altitude)  # 이륙 명령

        return {"status": "start takeoff"}
    
    def current_location(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

        # 드론의 현재 위치와 고도 정보를 가져옴
        current_location = self.vehicle.location.global_relative_frame
        attitude = self.vehicle.attitude
        return {
            "latitude": current_location.lat,  # 위도
            "longitude": current_location.lon,  # 경도
            "altitude": current_location.alt,  # 상대 고도
            "sea_level_altitude": self.vehicle.location.global_frame.alt,  # 해수면 고도
            "roll": attitude.roll,  # 롤
            "pitch": attitude.pitch,  # 피치
            "yaw": math.degrees(attitude.yaw),  # 요
            "armed": math.degrees(self.vehicle.armed)  # 시동여부
        }
    
    def goto_location(self, goto_location_info: GotoLocationInfo):
        print(goto_location_info)

        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

        if self.vehicle.mode.name != "GUIDED":
            raise HTTPException(status_code=400, detail="드론이 GUIDED 모드가 아닙니다.")
        
        self.vehicle.groundspeed = 10;

        

        # LocationGlobal 객체를 사용하여 해발 고도 기반으로 위치 설정
        #current_location = self.vehicle.location.global_relative_frame
        if goto_location_info.method == "relative":
            target_location = LocationGlobalRelative(goto_location_info.latitude, goto_location_info.longitude, goto_location_info.altitude)
        else:
            target_location = LocationGlobal(goto_location_info.latitude, goto_location_info.longitude, goto_location_info.altitude)


        # 드론에게 목적지로 이동하도록 명령
        self.vehicle.simple_goto(target_location)

        return {"status": "이동 시작", "target_location": goto_location_info}
    
    def upload_mission(self, waypoint_list: WaypointList):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

        cmds = self.vehicle.commands
        cmds.clear()
        while len(cmds) > 0:
            time.sleep(0.5)


        # 받은 웨이포인트 리스트를 기체의 웨이포인트 리스트에 추가
        for waypoint in waypoint_list.waypoints:

            if waypoint.command == "waypoint":
                command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            elif waypoint.command == "takeoff":
                command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            elif waypoint.command == "land":
                command = mavutil.mavlink.MAV_CMD_NAV_LAND
            

            if waypoint.altitudeType == "relative":
                altitudeType = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            elif waypoint.altitudeType == "terrain ":
                altitudeType = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            elif waypoint.altitudeType == "absolute":
                altitudeType = mavutil.mavlink.MAV_FRAME_GLOBAL


            cmds.add(Command(0, 0, 0, altitudeType, command, 0, 0, waypoint.delay, waypoint.radius, 0, 0, waypoint.latitude, waypoint.longitude, waypoint.altitude))
            if cmds.count < 1:
                cmds.add(Command(0, 0, 0, altitudeType, command, 0, 0, waypoint.delay, waypoint.radius, 0, 0, waypoint.latitude, waypoint.longitude, waypoint.altitude))

            #cmds.add(Command(0, 0, 0, altitudeType, command, 0, 0, 1.0, 1.0, 0, 0, waypoint.latitude, waypoint.longitude, 10.0))
            #Command(target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z)
            #target_system: 명령을 수신할 시스템의 ID입니다. 일반적으로 드론에 명령을 보낼 때는 0을 사용하여 모든 시스템이 이 명령을 수신할 수 있게 합니다.
            #target_component: 명령을 수신할 컴포넌트의 ID입니다. 컴포넌트는 시스템 내의 개별 요소를 나타냅니다. 대부분의 경우 0을 사용하여 주요 컴포넌트가 명령을 처리하도록 합니다.
            #seq: 명령 시퀀스 번호입니다. 미션 내에서 명령의 순서를 정의합니다. 보통 이 값은 Command 객체를 commands 목록에 추가할 때 자동으로 관리됩니다.
            #frame: 명령의 좌표계를 정의합니다. 예를 들어, MAV_FRAME_GLOBAL_RELATIVE_ALT는 지구를 기준으로 한 좌표계에서 상대 고도를 사용함을 의미합니다.
            #command: 실행할 명령의 유형을 나타내는 MAVLink 명령 ID입니다. 예를 들어, MAV_CMD_NAV_WAYPOINT는 경로점(waypoint) 이동 명령을 나타냅니다.
            #current: 현재 명령이 실행 중인지 여부를 나타냅니다. 0이면 아니오, 1이면 예입니다. 일반적으로 처음 명령을 추가할 때는 0을 사용합니다.
            #autocontinue: 이 명령 실행 후 다음 명령으로 자동으로 넘어갈지 여부를 나타냅니다. 0은 아니오, 1은 예입니다.
            #param1 ~ param4: 명령에 전달되는 특정 파라미터입니다. 이 파라미터들의 의미는 command에 따라 다릅니다. 예를 들어, MAV_CMD_NAV_WAYPOINT의 경우 param1은 대기 시간, param2 ~ param4는 사용되지 않습니다.
            #x, y, z: 명령의 위치를 나타냅니다. 일반적으로 x와 y는 위도와 경도를, z는 고도를 의미합니다. 위치 기반 명령에만 해당합니다.

        cmds.upload()  # 기체에 웨이포인트 리스트 전송
        return {"status": "success"}

    # 기체에서 mission 읽어오는 함수
    def download_mission(self):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="No active drone connection")

        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()

        missionlist = []
        for cmd in cmds:
            # cmd 객체의 필요한 속성만 추출하여 딕셔너리로 저장
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
            # 비활성화가 완료될 때까지 기다림
            while self.vehicle.armed:
                print("Disarming...")
                time.sleep(1)
            return {"status": "기체 시동을 해제하였습니다."}
        else:
            return {"status": "기체의 시동이 이미 해제되어 있습니다."}
        
    def get_elevation(self, lat, lon):
        query = f"https://api.open-elevation.com/api/v1/lookup?locations={lat},{lon}"
        response = requests.get(query).json()  # API 호출
        # 결과에서 해발 고도 추출
        elevation = response['results'][0]['elevation']
        return elevation

    


# 드론 인스턴스를 관리하는 딕셔너리
drones = {}

@app.post("/connect_drone")
async def connect_drone(connection_info: ConnectionInfo):
    drone_id = connection_info.drone_id
    if drone_id in drones:
        return {"status": "Already Connected", "details": str(drones[drone_id].vehicle)}
    
    drone = Drone(connection_info.connection_string)
    connection_result = drone.connect()
    drones[drone_id] = drone
    return connection_result

@app.get("/disconnect_drone/{drone_id}")
async def disconnect_drone(drone_id: str):
    if drone_id in drones:
        disconnect_result = drones[drone_id].disconnect()
        del drones[drone_id]  # 드론 인스턴스 삭제
        return disconnect_result
    else:
        return {"status": "Drone not found"}

# 나머지 엔드포인트들... 각 엔드포인트는 drone_id를 매개변수로 받아 해당 드론 인스턴스를 사용
@app.get("/drone_status/{drone_id}")
async def drone_status(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].drone_status()
        return result
    else:
        return {"status": "Drone not found"}


    
@app.get("/arm_drone/{drone_id}")
async def arm_drone(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].arm_drone()
        return result
    else:
        return {"status": "Drone not found"}
    
@app.get("/disarm_drone/{drone_id}")
async def disarm_drone(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].disarm_drone()
        return result
    else:
        return {"status": "드론을 확인할수 없습니다."}

@app.post("/takeoff")
async def takeoff(takeoff_info: TakeoffInfo):
    if takeoff_info.drone_id in drones:
        result = drones[takeoff_info.drone_id].takeoff(takeoff_info)
        return result
    else:
        return {"status": "Drone not found"}
    
@app.post("/change_mode")
async def change_mode(change_mode_info : ChangeModeInfo):
    if change_mode_info.drone_id in drones:
        result = drones[change_mode_info.drone_id].change_mode(change_mode_info)
        return result
    else:
        return {"status": "Drone not found"}

@app.get("/land/{drone_id}")
async def land(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].land()
        return result
    else:
        return {"status": "Drone not found"}


@app.get("/current_location/{drone_id}")
async def current_location(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].current_location()
        return result
    else:
        return {"status": "Drone not found"}


@app.post("/goto_location")
async def goto_location(goto_location_info: GotoLocationInfo):
    print(goto_location_info)
    if goto_location_info.drone_id in drones:
        result = drones[goto_location_info.drone_id].goto_location(goto_location_info)
        return result
    else:
        return {"status": "Drone not found"}

@app.post("/upload_mission")
async def upload_mission(waypoint_list: WaypointList):
    if waypoint_list.drone_id in drones:
        result = drones[waypoint_list.drone_id].upload_mission(waypoint_list)
        return result
    else:
        return {"status": "Drone not found"}
    
@app.get("/download_mission/{drone_id}")
async def download_mission(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].download_mission()
        return result
    else:
        return {"status": "Drone not found"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)