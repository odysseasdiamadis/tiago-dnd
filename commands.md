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

Launch gazebo
```sh
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-hey5 world:=dnd
```

Compile Catkin
```sh
catkin build --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

Person detection
```sh
roslaunch pal_person_detector_opencv detector.launch image:=/xtion/rgb/image_raw
```

Face detector
```sh
roslaunch pal_face_detector_opencv detector.launch image:=/xtion/rgb/image_raw
```

See from tiago's eyes!
```sh
rosrun rqt_image_view rqt_image_view
```

Control tiago with arrow keys
```sh
rosrun key_teleop key_teleop.py
```