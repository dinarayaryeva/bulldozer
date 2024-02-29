https://github.com/anybotics/grid_map

## CAN Protocol Package
This package is responsible for two main functionalities: 
1. Sending control commands to Kamaz truck
2. Reading and reporting the current state of the truck (steering wheel, velocity, throttle)

The package is composed of each of which is responsible for the one of the previous tasks.

### CAN_controller Node: 
This node is responsible for receiving 2 types of control messages which are ```AckermannDrive``` and ```CarlaEgoVehicleControl```. Then is converts the recieved message into a series of $64$ bits which make the CAN control message with ```ID = 0x18FDD6FE```.
The code is built according to the [documentation](https://docs.google.com/spreadsheets/d/1KXqOUA4eCI4RWAR-ILVy--vwGB4SF82LraIuOGL-RjU/edit#gid=0) provided from the hardware team.

### state_reader Node:
This node is responsible for receiving the CAN control message with ```ID = 0x18FF02F1``` or ```ID = 0x18FDD6F1``` and parsing the 64 bits to produce the steering wheel, velocity, and throttle values.

### How to use it:
This package already depends on the ```carla-msgs``` package as you can see in the ```package.xml``` 
```bash
ros2 launch vehicle_can_controller vehicle_can_controller.launch.py 
```

## Docker Image
To build the dockerfile you need to clone this repo to your local PC and change line $42$ with the directory of the package on your local PC.
Also you need to make run.sh as executable 
```bash
chmod +x run.sh 
docker build . -t vehiclecancontroller:latest
```
Once it is built you can run it, and it will automatically launch both nodes 
```bash
 docker run -it --rm --ipc=host --net host  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/home/auto/.Xauthority --name vehiclecancontroller vehiclecancontroller:latest
```