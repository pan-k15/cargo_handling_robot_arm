# ROS2 Pick-and-Sort Vision Pipeline
ติดกล้องบนแขน  → YOLOv8 classify วัตถุตามสี/รูปร่าง → MoveIt2 หยิบแล้วจัดเรียงลงถัง — demo ที่ผสม vision + manipulation


## Instruction
1. create a new world called object.sdf
2. create a new launch file object_world_sim.launch.py
    - use robot_2.urdf.xacro
3. setup YOLO in robot_vision package
    - topic name /object_class result of object classification
4. use moveit_config_2 to manipulate the robot
