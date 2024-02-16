from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationGlobal
from typing import List
from dronekit import Command
from pymavlink import mavutil
import time
import math

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

class GotoLocation(BaseModel):
    latitude: float
    longitude: float
    altitude: float  # 해발 고도

class GotoLocationInfo(BaseModel):
    drone_id: str
    latitude: float
    longitude: float
    altitude: float  # 해발 고도

class Waypoint(BaseModel):
    latitude: float
    longitude: float
    altitude: float

class WaypointList(BaseModel):
    waypoints: List[Waypoint]

class Drone:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.vehicle = None

    def connect(self):
        try:
            self.vehicle = connect(self.connection_string, wait_ready=True)
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
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

        if self.vehicle.mode.name != "GUIDED":
            raise HTTPException(status_code=400, detail="드론이 GUIDED 모드가 아닙니다.")
        
        self.vehicle.groundspeed = 10;

        # LocationGlobal 객체를 사용하여 해발 고도 기반으로 위치 설정
        current_location = self.vehicle.location.global_relative_frame
        target_location = LocationGlobal(goto_location_info.latitude, goto_location_info.longitude, goto_location_info.altitude + (self.vehicle.location.global_frame.alt - current_location.alt))

        # 드론에게 목적지로 이동하도록 명령
        self.vehicle.simple_goto(target_location)

        return {"status": "이동 시작", "target_location": goto_location_info}
    
    def upload_mission(self, waypoint_list: WaypointList):
        if self.vehicle is None:
            raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

        cmds = self.vehicle.commands
        cmds.clear()
        time.sleep(2)

        # 시작점을 웨이포인트로 추가
        home = self.vehicle.location.global_relative_frame
        #cmds.add(Command(0, 0, 0, 3, 22, 0, 0, home.lat, home.lon, home.alt, True, 0, 0, 0, 0))

        # 받은 웨이포인트 리스트를 기체의 웨이포인트 리스트에 추가
        for waypoint in waypoint_list.waypoints:
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.latitude, waypoint.longitude, waypoint.altitude))
            #Command(target_system=0, target_component=0, seq=0, frame=3, command=16, current=0, autocontinue=0, param1=0, param2=0, param3=0, param4=0, x=waypoint.latitude, y=waypoint.longitude, z=waypoint.altitude)

        cmds.upload()  # 기체에 웨이포인트 리스트 전송
        return {"status": "웨이포인트 업로드 완료"}
    
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
    if goto_location_info.drone_id in drones:
        result = drones[goto_location_info.drone_id].goto_location(goto_location_info)
        return result
    else:
        return {"status": "Drone not found"}

@app.post("/upload_mission")
async def upload_mission(drone_id: str, waypoint_list: WaypointList):
    if drone_id in drones:
        result = drones[drone_id].upload_mission(waypoint_list)
        return result
    else:
        return {"status": "Drone not found"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)