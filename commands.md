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
 
Check what Tiago sees from its camera
```sh
rosrun rqt_image_view rqt_image_view
```

Start scene "search_players", with face scanning and player modeling
(per avviarla con ros bisogna aver fatto il source sopra)
```sh
rosrun tiago_actions search_players
```
then
```sh
python3 src/tiago_ws/src/tiago_actions/src/scripts/search_players.py
```

Start Gazebo (to change scenario, use any of the .world files as world:= param)
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