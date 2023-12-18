from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from mavsdk import System
from mavsdk.mission import MissionPlan, MissionItem
from typing import List
import asyncio

app = FastAPI()

# CORS 미들웨어 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
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
    altitude: float

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
    if vehicle is not None:
        return {"status": "이미 연결됨"}

    vehicle = System()
    await vehicle.connect(system_address=connection_info.connection_string)
    return {"status": "연결됨"}

@app.get("/disconnect_drone")
async def disconnect_drone():
    global vehicle
    if vehicle is not None:
        await vehicle.close()
        vehicle = None
        return {"status": "연결 해제됨"}
    else:
        return {"status": "활성 연결 없음"}

@app.get("/drone_status")
async def drone_status():
    if vehicle is not None:
        battery = await vehicle.telemetry.battery()
        return {
            "배터리": str(battery.remaining_percent),
            # 다른 텔레메트리 데이터 추가
        }
    else:
        raise HTTPException(status_code=400, detail="활성 드론 연결 없음")

# MAVSDK는 모드 변경에 대한 직접적인 API를 제공하지 않으므로 이 부분은 제외합니다.

@app.post("/arm_drone")
async def arm_drone():
    global vehicle
    if vehicle is None:
        raise HTTPException(status_code=400, detail="활성 드론 연결 없음")

    await vehicle.action.arm()
    return {"status": "시동 걸림"}

@app.post("/takeoff")
async def takeoff(takeof_info: ModeTargetAltitude):
    global vehicle
    if vehicle is None:
        raise HTTPException(status_code=400, detail="활성 드론 연결 없음")

    target_altitude = float(takeof_info.target_altitude)
    await vehicle.action.takeoff(target_altitude)
    return {"status": "이륙 시작"}

@app.post("/upload_mission")
async def upload_mission(waypoint_list: WaypointList):
    global vehicle
    if vehicle is None:
        raise HTTPException(status_code=400, detail="활성 드론 연결 없음")

    mission_items = []
    for waypoint in waypoint_list.waypoints:
        mission_items.append(MissionItem(waypoint.latitude, waypoint.longitude, waypoint.altitude, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan')))
    
    mission_plan = MissionPlan(mission_items)
    await vehicle.mission.set_return_to_launch_after_mission(True)
    await vehicle.mission.upload_mission(mission_plan)
    return {"status": "미션 업로드됨"}

# MAVSDK는 간단한 이동 명령을 위한 직접적인 API를 제공하지 않으므로 이 부분은 제외합니다.

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
