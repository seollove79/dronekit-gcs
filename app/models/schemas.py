from pydantic import BaseModel
from typing import List

class ConnectionInfo(BaseModel):
    connection_string: str
    drone_id: str

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
