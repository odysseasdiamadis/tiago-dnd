Source env vars
```sh
# One script to source them all
source /src/tiago_ws/source.bash
```
Or
```sh
source /tiago_public_ws/devel/setup.bash
source /src/tiago_ws/devel/setup.bash
```

Guarda dalla telecamera di tiago
```sh
rosrun rqt_image_view rqt_image_view
```

Lancia la scena "ricerca giocatori"
(per avviarla con ros bisogna aver fatto il source sopra)
```sh
rosrun tiago_actions search_players
```

```sh
python3 src/tiago_ws/src/tiago_actions/src/scripts/search_players.py
```

Gazebo (cambia l'ultimo parametro seguendo i nomi dei file .world nella root del progetto per cambiare mondo)
```sh
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-hey5 world:=dnd_3players
```


Compile Catkin
```sh
catkin build --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

Control tiago with arrow keys
```sh
rosrun key_teleop key_teleop.py
```

Non mi ricordo, roba inutile
```sh
rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory "
  joint_names: ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
  points:
  - positions: [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {secs: 0, nsecs: 500}"

```