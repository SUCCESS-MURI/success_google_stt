# success_google_stt
License - MIT  
Maintainer - zhi.tan@ri.cmu.edu  

ROS Speeh-To-Text module. The system uses Google Cloud Speech API to do speech recognition from the default microphone. You will
first need to enable the STT using the ROS service call at `/success_google_stt/toggl_stt`.
The service calls takes a `std_srvs/SetBool` service message. Once the STT is enabled, it will start publishing on the topc, `/success_google_stt/stt` with the message type: `success_ros_msgs/Speech.msg`


### Dependencies & Installation
1. Setup a Google Service Account with Google Speech enabled and save the credentials. By default the credentials are saved as `~/Keys/google_service_account.json`. If you want to store it at a different location, change the arg `google_credential_path` in the launch file
2. Create a virtual environment to save the python3 packages listed in `requirement.txt`. By default, we use python 3.6 and the virtual environment called `ros` is saved at `~/Dev/envs/ros`. You can change these values in the launch file. Following are the list of steps to set it up on our machines:
    1. `mkdir -p Dev/envs`
    2. `sudo apt install python3-venv`
    3. `python3 -m venv ~/Dev/envs/ros`
    4. `source ~/Dev/envs/ros/bin/activate`
    5. be at the current folder and `pip install -r requirement.txt`
3. Note for `pyalsaaudio`, you going to need to install alsaaudio first, `sudo apt install libasound2-dev`.


### Run
1. Launch the launch file `roslaunch success_google_stt google_stt.launch`
    * `roslaunch success_google_stt google_stt.launch method='mic'` to directly use the microphone.
2. An example of the code and test is given in [scripts/test_stt.py](scripts/test_stt.py)

### Parameters
* `success_google_stt/method` - You can either select `mic` or `topic`. `mic`

### Changelog
* 2019/02 - updated launch file & instructions.
* 2018/12 - Moved to SUCCESS_MURI
* 2018/09 - Added ability to toggle STT on and off.