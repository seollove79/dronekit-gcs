from fastapi import APIRouter
from app.api.api_v1.endpoints import drone

api_router = APIRouter()
api_router.include_router(drone.router, prefix="/drones", tags=["drones"])
