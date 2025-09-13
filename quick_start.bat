@echo off

echo ROS2 Autonomous Navigation Demo - Quick Start
echo =============================================

if "%ROS_DISTRO%"=="" (
    echo ❌ ROS2 not found. Please source ROS2 first:
    echo    call C:\opt\ros\humble\setup.bat
    pause
    exit /b 1
)

echo ROS2 %ROS_DISTRO% detected

echo Building workspace...
colcon build --symlink-install

if %errorlevel% neq 0 (
    echo ❌ Build failed. Please check the error messages above.
    pause
    exit /b 1
)

echo Build successful

echo Sourcing workspace...
call install\setup.bat

echo Launching complete demo...
echo  - Gazebo simulation with warehouse world
echo  - Husky robot with LiDAR
echo  - Navigation stack (perception, planning, control)
echo  - RViz visualization
echo.
echo The robot will navigate from (0,0) to (5,3)
echo Press Ctrl+C to stop the demo
echo.

ros2 launch path_ros2 demo.launch.py

pause