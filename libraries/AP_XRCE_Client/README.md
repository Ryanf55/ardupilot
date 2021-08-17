# Testing with DDS/micro-Ros

## Testing with a UART

Run the simulator with the following command (assuming we are using ttyUSB0 for Ardupilot):
```sim_vehicle.py -D  --console --enable-xrce-dds -A "--uartC=uart:/dev/ttyUSB0"``` 

Then set the following parameters : 

- set **SERIAL1_BAUD = 115**
- set **SERIAL1_PROTOCOL = 41** 

Now set the XRCE parameters:

- set **XRCE_TYPE = 0** (default) for a DDS Agent
- set **XRCE_TYPE = 1** for a micro-ROS Agent

- set **XRCE_TOPIC**( based on the topic you want Ardupilot to publish)

## Starting DDS Agent

Follow the steps to use the DDS agent

- Install DDS Agent (as described here)

  - https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-agent-standalone

- In a new terminal, run the following command :

  - ```cd /usr/local/bin && MicroXRCEAgent serial -b 115200 -D /dev/ttyUSB2``` (assuming we are using ttyUSB2 for Ardupilot)
or,
  - ```cd /usr/local/bin && MicroXRCEAgent tcp4 -p 2019``` (assuming we are using the port 2019)

For more information ,one can take a look here - https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html#agent-cli 
## Starting with microROS Agent

Follow the steps to use the microROS Agent

- Install ROS Foxy (as described here)

  - https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

- Install and run the microROS agent (as descibed here)

  - https://micro.ros.org/docs/tutorials/core/first_application_linux/

- Run microROS agent with the following command

  - ```ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/ttyUSB2 (assuming we are using ttyUSB2 for Ardupilot)```

## Tutorial

### Setting the ROS2-workspace 
If you have installed the microROS agent and ROS-2 Foxy

- Source the ros2 installation
  - ```source /opt/ros/foxy/setup.bash```

- Create a ros2 workspace

  - ```mkdir -p ~/dev_ws/src```

- You can install a simple ros2 package from here : https://github.com/arshPratap/ROS2-test-example.git

- ```cd ~/dev_ws/src```
- clone the above repo
- Build the package
  - In the root of dev_ws, run:
   ```colcon build --packages-select AP_Ros2_Int```
### Run the microROS agent

- Run the microROS agent as described above

- ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/ttyUSBX (replace X with the correct port number)

### Run the Ardupilot simulator

- As described above , run the Ardupilot simulator and make sure the following parameters have the corresponding values

  - **SERIAL1_BAUD 115**
  - **SERIAL1_PROTOCOL 41**
  - **XRCE_TYPE  1**
  - **XRCE_TOPIC 7**

- Once the above values are set ,run the simulator again as :

  - ```sim_vehicle.py --enable-xrce-dds -A "--uartC=uart:/dev/ttyUSB2:115200" --console```

If everything has been setup correctly , you will see the session established message on the agent.
### Reading ROS2 data

- Open a new terminal
- Source ROS2 installation
  - ```source /opt/ros/foxy/setup.bash```
- Go to the ROS2 workspace created earlier
- Source the package installation (make sure you are in the root of the dev_ws folder)
  - ```. install/setup.bash```

- Then run the following commands
  - ```ros2 node list```
    - you should see the following name in the node list(if everything is setup correctly)
    - */Ardupilot_XRCE_Client*

  - ```ros2 topic list```
    - you should see the following name in the topic list(if everything is setup correctly)
    - /AP_ROS2_Int8

  - ```ros2 run AP_Ros2_Int listener```

    - you should see the ROS2 subscriber listening to the 8bit Int being sent from Ardupilot
