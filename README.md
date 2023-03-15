# 🤖 CMPUT 412: Exercise 3 - Computer Vision for Robotics 🤖

Implementation of detecting and using apriltags for localization. Visualization in rviz and lane following.

## Run on your Duckiebot
1. Run the LED demo to get the led_pattern service started:
  * ```shell 
    dts duckiebot demo --demo_name led_emitter_node --duckiebot_name $BOT --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8
    ```   
2. Build it:
  * ```shell 
    dts devel build -f -H MY_ROBOT.local
    ```   
3. Run it:
  * ```shell 
    dts devel run -H MY_ROBOT.local
    ```   

## References 🫡
- [LED node](https://github.com/anna-ssi/duckiebot/blob/50d0b24eab13eb32d92fa83273a05564ca4dd8ef/assignment2/src/led_node.py)
- [img undistortion](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [dt-apriltags](https://github.com/duckietown/lib-dt-apriltags/blob/master/test/test.py)
- [apriltag labelling](https://pyimagesearch.com/2020/11/02/apriltag-with-python/)
- [frame transform](https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/tf2_ros/buffer.py)
- [yaml load](https://docs.duckietown.org/daffy/duckietown-classical-robotics/out/cra_perception.html)
- [dead reckoning](https://github.com/wagonhelm/cmput412_exercise3)
- [LED demo](https://github.com/duckietown/dt-core/blob/6d8e99a5849737f86cab72b04fd2b449528226be/packages/led_emitter/src/led_emitter_node.py#L254)
- [transformations](https://docs.ros.org/en/jade/api/tf/html/python/transformations.html)
- [broadcast transforms](https://github.com/ros/geometry2/tree/noetic-devel/tf2_ros/src/tf2_ros)
- [lane following](https://medium.com/@mrhwick/simple-lane-detection-with-opencv-bfeb6ae54ec)
- [lane following](https://youtu.be/rVBVqVmHtfc)

## Made using: [this template](https://github.com/wagonhelm/cmput412_exercise3)
