# capstone_s2_ws

## AMR

## Manipulator

cd ~/capstone_s2_ws/src/realsense-ros
git fetch --tags
git checkout 4.56.4

# 워크스페이스에서 realsense 관련 캐시를 확실히 제거
cd ~/capstone_s2_ws
rm -rf build/realsense2_camera build/realsense2_* install/realsense2_* log

# 다시 빌드
colcon build --packages-up-to realsense2_camera \
  --cmake-args -DCMAKE_BUILD_TYPE=Release