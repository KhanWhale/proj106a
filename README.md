# proj106a

## Running the system

* `cd ros_ws`
* `source devel/setup.bash`
* `roslaunch drone_106a drone.launch`

## CV Pipeline

* How to setup/use virtual environment with Google MediaPipe dependencies:
  * `cd old-code/vision`
  * `source setup.sh` only the first time
  * `source activate.sh` any time after that

* How to run the hand detection program:
  * `cd vision`
  * `source activate.sh`
  * `python3 hand-detection.py [camera-input-source-num]`

