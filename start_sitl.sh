#!/bin/bash

# 첫 번째 드론 시작
dronekit-sitl copter --instance 0 --home=37.520995,126.621259,0,0 &

# 두 번째 드론 시작
dronekit-sitl copter --instance 1 --home=37.520995,126.621259,0,0 &

# 세 번째 드론 시작 (오타를 수정하였고, 인스턴스 번호를 2로 변경하였습니다. 필요에 따라 조정하세요.)
dronekit-sitl copter --instance 2 --home=37.520995,126.621259,0,0 &

