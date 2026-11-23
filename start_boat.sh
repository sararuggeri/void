#!/bin/bash
trap 'kill $(jobs -p)' EXIT

export GZ_SIM_RESOURCE_PATH=$HOME/marine_ws/models:$GZ_SIM_RESOURCE_PATH

source /opt/ros/jazzy/setup.bash
source ~/marine_ws/install/setup.bash

echo "Avvio simulazione WAM-V..."
ros2 launch vrx_gz competition.launch.py urdf:=$(dirname "$(realpath "$0")")/my_boat.urdf headless:=False locked:=false &

echo "Attendo 10 secondi il caricamento dell'oceano..."
sleep 10

echo "Spawn del bagnante su percorso lungo quasi lineare con curva (z = -0.65, velocita' normale ~1.1 m/s)..."
ros2 run ros_gz_sim create -world sydney_regatta -name bagnante_obliquo \
  -string '<sdf version="1.6">
    <actor name="bagnante_obliquo">
      <skin>
        <filename>model://person_walking/meshes/walking.dae</filename>
      </skin>
      <animation name="walking">
        <filename>model://person_walking/meshes/walking.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <waypoint>
            <time>0.0</time>
            <pose>-523.50 170.50 -0.65 0 0 0.95</pose>
          </waypoint>
          <waypoint>
            <time>5.0</time>
            <pose>-520.20 174.90 -0.65 0 0 0.95</pose>
          </waypoint>
          <waypoint>
            <time>10.0</time>
            <pose>-516.80 179.30 -0.65 0 0 0.45</pose>
          </waypoint>
          <waypoint>
            <time>15.0</time>
            <pose>-512.00 181.60 -0.65 0 0 0.15</pose>
          </waypoint>
          <waypoint>
            <time>21.0</time>
            <pose>-516.80 179.30 -0.65 0 0 -2.65</pose>
          </waypoint>
          <waypoint>
            <time>27.0</time>
            <pose>-523.50 170.50 -0.65 0 0 -2.15</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>
  </sdf>' &

echo "Avvio ponti di comunicazione..."
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
ros2 run ros_gz_image image_bridge /wamv/sensors/cameras/front_left_camera_sensor/image_raw &
ros2 run ros_gz_image image_bridge /wamv/sensors/cameras/front_right_camera_sensor/image_raw &
ros2 run ros_gz_bridge parameter_bridge /wamv/sonar_servo_cmd@std_msgs/msg/Float64@gz.msgs.Double &
ros2 run ros_gz_bridge parameter_bridge /wamv/sensors/sonar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
ros2 run ros_gz_bridge parameter_bridge /wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double &
ros2 run ros_gz_bridge parameter_bridge /wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double &

wait
