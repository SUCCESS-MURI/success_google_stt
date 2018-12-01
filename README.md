# success_google_stt
License - MIT  
Maintainer - zhi.tan@ri.cmu.edu  

ROS Speeh-To-Text module. The system uses Google Cloud Speech API to do speech recognition from the default microphone. You will
first need to enable the STT using the ROS service call at `/success_google_stt/toggl_stt`.
The service calls takes a `std_srvs/SetBool` service message. Once the STT is enabled, it will start publishing on the topc, `/success_google_stt/stt` with the message type: `success_ros_msgs/Speech.msg`


### Dependencies
Make sure you the packages listed in `requirement.txt`. To install them, do `pip install -r requirement.txt`

Note for `pyalsaaudio`, you going to need to install alsaaudio first, `sudo apt install libasound2-dev`.

### Run
1. First change `google_stt.launch` such that the `GOOGLE_APPLICATION_CREDENTIALS` points to your credential and `PYTHONPATH` to the correct python environment. You can easily find the python site-package path with `python -m site`.
2. Launch the launch file `roslaunch success_google_stt google_stt.launch`
    * `roslaunch success_google_stt google_stt.launch method='mic'` to directly use the microphone.
3. An example of the code and test is given in [scripts/test_stt.py](scripts/test_stt.py)

### Parameters
* `success_google_stt/method` - You can either select `mic` or `topic`. `mic`

### Changelog
* 2018/12 - Moved to SUCCESS_MURI
* 2018/09 - Added ability to toggle STT on and off.