# Intro and goal of the project
The aim of this project is to reproduce a Dungeons and Dragons session with Tiago as a Dungeon Master.

# Requirements
All requirements to run scripts are defined in:
- config/tiago_env.yaml (for conda, to use outside of docker)
- config/tiago_requirements.txt (inside docker, used to create a venv env)
The script source ./src/install-deps.sh installs libraries and sources the venv env in the current window (if inside docker)


## How to start the simulation
In a terminal from the root of this repo, run:
sudo xhost +local:docker 
sudo ./start_docker.sh
Then inside the docker environment opened:
cd ..
source /src/tiago_ws/source.bash        # init dependancies and start venv env
catkin build --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5       # build catkind dependancies for ros, ONLY needed once to create ros folders, then when it's created no need to enven if you restart the docker
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-hey5 world:=dnd_3players      # start gazebo with the 3 player world
python3 src/tiago_ws/src/tiago_actions/src/scripts/search_players.py    # start simulation test script

To work or send ros commands on Tiago, run ./join.sh in another terminal window. 