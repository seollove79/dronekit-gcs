from pydantic import BaseModel
from typing import List

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
