Source env vars
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

Face detection (not working? should publish on /pal/faces/debug or soemthing)
```sh
roslaunch pal_face_detector_opencv detector.launch image:=/xtion/rgb/image_raw
```

See from tiago's eyes!
```sh
rosrun rqt_image_view rqt_image_view
```