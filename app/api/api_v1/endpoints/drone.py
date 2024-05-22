from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List
from app.models.schemas import ConnectionInfo, TakeoffInfo, ChangeModeInfo, GotoLocationInfo, WaypointList
from app.models.drone import Drone

router = APIRouter()

# 드론 인스턴스를 관리하는 딕셔너리
drones = {}

@router.post("/connect_drone")
async def connect_drone(connection_info: ConnectionInfo):
    drone_id = connection_info.drone_id
    if drone_id in drones:
        return {"status": "Already Connected", "details": str(drones[drone_id].vehicle)}
    
    drone = Drone(connection_info.connection_string)
    connection_result = drone.connect()
    drones[drone_id] = drone
    return connection_result

@router.get("/disconnect_drone/{drone_id}")
async def disconnect_drone(drone_id: str):
    if drone_id in drones:
        disconnect_result = drones[drone_id].disconnect()
        del drones[drone_id]
        return disconnect_result
    else:
        return {"status": "Drone not found"}

@router.get("/drone_status/{drone_id}")
async def drone_status(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].drone_status()
        return result
    else:
        return {"status": "Drone not found"}

@router.get("/arm_drone/{drone_id}")
async def arm_drone(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].arm_drone()
        return result
    else:
        return {"status": "Drone not found"}

@router.get("/disarm_drone/{drone_id}")
async def disarm_drone(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].disarm_drone()
        return result
    else:
        return {"status": "Drone not found"}

@router.post("/takeoff")
async def takeoff(takeoff_info: TakeoffInfo):
    if takeoff_info.drone_id in drones:
        result = drones[takeoff_info.drone_id].takeoff(takeoff_info)
        return result
    else:
        return {"status": "Drone not found"}

@router.post("/arm_and_takeoff")
async def arm_and_takeoff(takeoff_info: TakeoffInfo):
    if takeoff_info.drone_id in drones:
        result = drones[takeoff_info.drone_id].arm_drone()
        if result["status"] == "Armed":
            result = drones[takeoff_info.drone_id].takeoff(takeoff_info)
        return result
    else:
        return {"status": "Drone not found"}

@router.post("/change_mode")
async def change_mode(change_mode_info: ChangeModeInfo):
    if change_mode_info.drone_id in drones:
        result = drones[change_mode_info.drone_id].change_mode(change_mode_info)
        return result
    else:
        return {"status": "Drone not found"}

@router.get("/land/{drone_id}")
async def land(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].land()
        return result
    else:
        return {"status": "Drone not found"}

@router.get("/current_location/{drone_id}")
async def current_location(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].current_location()
        return result
    else:
        return {"status": "Drone not found"}

@router.post("/goto_location")
async def goto_location(goto_location_info: GotoLocationInfo):
    if goto_location_info.drone_id in drones:
        result = drones[goto_location_info.drone_id].goto_location(goto_location_info)
        return result
    else:
        return {"status": "Drone not found"}

@router.post("/upload_mission")
async def upload_mission(waypoint_list: WaypointList):
    if waypoint_list.drone_id in drones:
        result = drones[waypoint_list.drone_id].upload_mission(waypoint_list)
        return result
    else:
        return {"status": "Drone not found"}

@router.get("/download_mission/{drone_id}")
async def download_mission(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].download_mission()
        return result
    else:
        return {"status": "Drone not found"}

@router.get("/download_message/{drone_id}")
async def download_message(drone_id: str):
    if drone_id in drones:
        result = drones[drone_id].download_message()
        return result
    else:
        return {"status": "Drone not found"}
