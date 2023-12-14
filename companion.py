from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationGlobal
from typing import List
from dronekit import Command
import time
import math

app = FastAPI()

# CORS 미들웨어 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 특정 오리진을 허용하거나 ["*"]로 모든 오리진 허용
    allow_credentials=True,
    allow_methods=["*"],  # 특정 HTTP 메소드를 허용하거나 ["*"]로 모든 메소드 허용
    allow_headers=["*"],  # 특정 HTTP 헤더를 허용하거나 ["*"]로 모든 헤더 허용
)

class ConnectionInfo(BaseModel):
    connection_string: str

class ModeChangeRequest(BaseModel):
    new_mode: str    

class ModeTargetAltitude(BaseModel):
    target_altitude: str

class GotoLocation(BaseModel):
    latitude: float
    longitude: float
    altitude: float  # 해발 고도

class Waypoint(BaseModel):
    latitude: float
    longitude: float
    altitude: float

class WaypointList(BaseModel):
    waypoints: List[Waypoint]

global vehicle
vehicle = None

@app.post("/connect_drone")
async def connect_drone(connection_info: ConnectionInfo):
    global vehicle
    # 이미 연결된 드론이 있는지 확인
    if vehicle is not None:
        return {"status": "Already Connected", "details": str(vehicle)}

    try:
        vehicle = connect(connection_info.connection_string, wait_ready=True)
        return {"status": "Connected", "details": str(vehicle)}
    except APIException as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/disconnect_drone")
async def disconnect_drone():
    global vehicle
    if vehicle is not None:
        vehicle.close()
        vehicle = None
        return {"status": "Disconnected"}
    else:
        return {"status": "No active connection"}

@app.get("/drone_status")
async def drone_status():
    if vehicle is not None:
        print(vehicle.home_location)
        return {
            "GPS": str(vehicle.gps_0),
            "Battery": str(vehicle.battery),
            "Heartbeat": str(vehicle.last_heartbeat),
            "Mode": str(vehicle.mode.name),
            "Speed": str(vehicle.groundspeed),
            "Home": str(vehicle.home_location)
        }
    else:
        raise HTTPException(status_code=400, detail="No active drone connection")

@app.post("/change_mode")
async def change_mode(mode_request: ModeChangeRequest):
    global vehicle
    if vehicle is None:
        raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

    try:
        new_mode = VehicleMode(mode_request.new_mode)
        vehicle.mode = new_mode

        # 모드가 변경될 때까지 기다림
        while not vehicle.mode.name == mode_request.new_mode:
            print("모드 변경을 기다리는 중...")
            time.sleep(1)

        return {"status": "모드 변경됨", "new_mode": mode_request.new_mode}
    except APIException as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/arm_drone")
async def arm_drone():
    global vehicle
    if vehicle is None:
        raise HTTPException(status_code=400, detail="No active drone connection")

    if vehicle.mode != 'GUIDED':
        #raise HTTPException(status_code=400, detail="Drone is not in GUIDED mode")
        vehicle.mode = VehicleMode("GUIDED")

        # 모드가 변경될 때까지 기다림
        while not vehicle.mode.name == "GUIDED":
            print("모드 변경을 기다리는 중...")
            time.sleep(1)

    vehicle.armed = True

    # 시동이 걸릴 때까지 기다림
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    return {"status": "Armed"}

@app.post("/takeoff")
async def takeoff(takeof_info: ModeTargetAltitude):
    global vehicle
    if vehicle is None:
        raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

    if not vehicle.armed:
        raise HTTPException(status_code=400, detail="드론이 시동되지 않았습니다. 먼저 시동을 걸어주세요.")

    if vehicle.mode.name != "GUIDED":
        raise HTTPException(status_code=400, detail="드론이 GUIDED 모드가 아닙니다.")

    print("이륙 중...")
    target_altitude = float(takeof_info.target_altitude)
    vehicle.simple_takeoff(target_altitude)  # 이륙 명령

    return {"status": "이륙 시작"}

    # 목표 고도에 도달할 때까지 기다림
    # while True:
    #     print("현재 고도:", vehicle.location.global_relative_frame.alt)
    #     if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
    #         print("목표 고도 도달")
    #         break
    #     time.sleep(1)

    # return {"status": "이륙 완료", "current_altitude": vehicle.location.global_relative_frame.alt}

@app.get("/current_location")
async def current_location():
    global vehicle
    if vehicle is None:
        raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

    # 드론의 현재 위치와 고도 정보를 가져옴
    current_location = vehicle.location.global_relative_frame
    attitude = vehicle.attitude
    return {
        "latitude": current_location.lat,  # 위도
        "longitude": current_location.lon,  # 경도
        "altitude": current_location.alt,  # 상대 고도
        "sea_level_altitude": vehicle.location.global_frame.alt,  # 해수면 고도
        "roll": attitude.roll,  # 롤
        "pitch": attitude.pitch,  # 피치
        "yaw": math.degrees(attitude.yaw),  # 요
        "armed": math.degrees(vehicle.armed)  # 시동여부
    }

@app.post("/goto_location")
async def goto_location(location: GotoLocation):
    global vehicle
    if vehicle is None:
        raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")

    if vehicle.mode.name != "GUIDED":
        raise HTTPException(status_code=400, detail="드론이 GUIDED 모드가 아닙니다.")
    
    vehicle.groundspeed = 10;

    # LocationGlobal 객체를 사용하여 해발 고도 기반으로 위치 설정
    current_location = vehicle.location.global_relative_frame
    target_location = LocationGlobal(location.latitude, location.longitude, location.altitude + (vehicle.location.global_frame.alt - current_location.alt))

    # 드론에게 목적지로 이동하도록 명령
    vehicle.simple_goto(target_location)

    
    return {"status": "이동 시작", "target_location": location}

@app.post("/upload_mission")
async def upload_mission(waypoint_list: WaypointList):
    global vehicle
    if vehicle is None:
        raise HTTPException(status_code=400, detail="활성 드론 연결이 없습니다.")
    
    print("업로드 시작")

    cmds = vehicle.commands
    cmds.clear()

    # 시작점을 웨이포인트로 추가
    home = vehicle.location.global_relative_frame
    cmds.add(Command(0, 0, 0, 3, 22, 0, 0, home.lat, home.lon, home.alt, True, 0, 0, 0, 0))

    # 받은 웨이포인트 리스트를 기체의 웨이포인트 리스트에 추가
    for waypoint in waypoint_list.waypoints:
        cmds.add(Command(0, 0, 0, 3, 16, 0, 0, waypoint.latitude, waypoint.longitude, waypoint.altitude, True, 0, 0, 0, 0))
        print (waypoint.latitude, waypoint.longitude, waypoint.altitude)
        #Command(target_system=0, target_component=0, seq=0, frame=3, command=16, current=0, autocontinue=0, param1=0, param2=0, param3=0, param4=0, x=waypoint.latitude, y=waypoint.longitude, z=waypoint.altitude)

    cmds.upload()  # 기체에 웨이포인트 리스트 전송
    return {"status": "웨이포인트 업로드 완료"}