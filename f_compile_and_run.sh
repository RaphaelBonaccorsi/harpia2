clear

echo "Building harpia_msgs:"
cd src/harpia_msgs
if ! colcon build; then
  echo "Error building harpia_msgs"
  exit 1
fi
cd ../..
source src/harpia_msgs/install/setup.bash
echo "Builded harpia_msgs successfully"


echo "Building main project:"
# if ! colcon build --symlink-install; then
if ! colcon build; then
  echo "Error building main project"
  exit 1
fi
source install/setup.bash
echo "Builded main project successfully"

chmod +x install/route_executor2/share/route_executor2/solver/OPTIC/generate_plan.sh install/route_executor2/share/route_executor2/solver/OPTIC/optic-clp

echo "Running project:"
ros2 launch route_executor2 patrol_example_launch.py