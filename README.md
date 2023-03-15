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
- [LED demo](https://github.com/duckietown/dt-core/blob/6d8e99a5849737f86cab72b04fd2b449528226be/packages/led_emitter/src/led_emitter_node.py#L254)
- [lane following](https://eclass.srv.ualberta.ca/course/view.php?id=85053)

## Made using: [this template](https://github.com/XZPshaw/CMPUT412503_exercise4)
