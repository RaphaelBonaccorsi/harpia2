clear
# Primeiro comando
if colcon build --symlink-install; then
  # Executar esses comandos apenas se o primeiro não der erro
  source install/setup.bash
  ros2 launch route_executor2 patrol_example_launch.py
else
  echo "Erro durante a build"
fi