# Testing with DDS/micro-Ros

## Architecture

Ardupilot contains the XRCE Client library, which can run as SITL. Then, the DDS application runs a ROS2 node, an EProsima Integration Service, and the MicroXRCE Agent. The two systems communicate over serial, which is the only supported protocol in Ardupilot MicroXCE DDS at this time.

```mermaid
---
title: Hardware Serial Port Loopback
---
graph LR

  subgraph Linux Computer

    subgraph Ardupilot SITL
      veh[sim_vehicle.py] <--> xrceClient[EProsima XRCE Client]
      xrceClient <--> port1[devUSB1]
    end

    subgraph DDS Application
      ros[ROS2 Node] <--> is[EProsima Integration Service]
      is <--> xrceAgent[MicroXRCE Agent]
      xrceAgent <--> port2[devUSB2]
    end

    port1 <--> port2

  end
```

## Parameters for XRCE DDS

| Name | Description |
| - | - |
| SERIAL1_BAUD | The serial baud rate for XRCE DDS |
| SERIAL1_PROTOCOL | Set this to 45 to use XRCE DDS on the serial port |
| XRCE+TYPE | Set this to 0 to connect to the native XRCE Agent and 1 for MicroROS |


## Testing with a UART

On Linux, first create a virtual serial port for use with SITL like [this](https://stackoverflow.com/questions/52187/virtual-serial-port-for-linux)

```
sudo apt-get update
sudo apt-get install socat
```

Then, start a virtual serial port with socat. Take note of the two `/dev/pts/*` ports. If yours are different, substitute as needed.
```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
>>> 2023/02/21 05:26:06 socat[334] N PTY is /dev/pts/1
>>> 2023/02/21 05:26:06 socat[334] N PTY is /dev/pts/2
>>> 2023/02/21 05:26:06 socat[334] N starting data transfer loop with FDs [5,5] and [7,7]
```

Set up your [SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
Run the simulator with the following command (assuming we are using /dev/pts/1 for Ardupilot SITL). Take note how two parameters need adjusting from default to use XRCE DDS.
```
# Select your favorite vehicle type
cd ArduCopter
# Wipe params till you see "AP: ArduPilot Ready"
sim_vehicle.py -w

# Set params
param set SERIAL1_BAUD 115
# See libraries/AP_SerialManager/AP_SerialManager.h AP_SerialManager SerialProtocol_DDS_XRCE
param set SERIAL1_PROTOCOL 45
```

# Start the sim now with the new params
```
sim_vehicle.py -D  --console --enable-xrce-dds -A "--uartC=uart:/dev/pts/1"
```

If desired, set the XRCE parameters:

- set **XRCE_TYPE = 0** (default) for a DDS Agent
- set **XRCE_TYPE = 1** for a micro-ROS Agent


For example:
```
param set XRCE_TYPE 1
```
Then, restart the simulator if you changed XRCE_TYPE.

## MicroXRCE

The section below applies to MicroXRCE only 

## Starting MicroXRCEAgent

Follow the steps to use the MicroXRCE agent

- Install MicroXRCE Agent (as described here), using the stable `master` branch

  - https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-agent-standalone

- In a new terminal, run the following command :

  - ```cd /usr/local/bin && MicroXRCEAgent serial -b 115200 -D /dev/pts/2``` (assuming we are using pts/2 for Ardupilot)

For more information, one can take a look here - https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html#agent-cli

### Starting the Integration Service

- Install dependencies for the Integration Service and build it with `colcon`

  - https://integration-service.docs.eprosima.com/en/latest/installation_manual/installation.html#installation

TODO add to this once EProsima fixes build errors on Ubuntu 22.04.

## Starting with microROS Agent

Follow the steps to use the microROS Agent

- Install ROS Humble (as described here)

  - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

- Install and run the microROS agent (as descibed here). Make sure to use the `humble` branch.

- https://micro.ros.org/docs/tutorials/core/first_application_linux/

TODO issue a PR to update MicroROS docs from foxy to something else. 

Follow the instructions for the following:

* Do "Installing ROS 2 and the micro-ROS build system"
  * Skip the docker run command, build it locally instead
* Skip "Creating a new firmware workspace"
* Skip "Building the firmware"
* Do "Creating the micro-ROS agent"

- Run microROS agent with the following command

  - ```ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/pts/2 # (assuming we are using tty/pts/2 for Ardupilot)```

## Tutorial

### Using the ROS2 CLI to Read Ardupilot Data

If you have installed the microROS agent and ROS-2 Humble

- Source the ros2 installation
  - ```source /opt/ros/humble/setup.bash```

- If SITL is running alongise MicroROS Agent, you should be able to see the agent here and view the data output.


```
$ ros2 node list
/Ardupilot_DDS_XRCE_Client

$ ros2 topic list  -v
Published topics:
 * /ROS2_Time [builtin_interfaces/msg/Time] 1 publisher
 * /parameter_events [rcl_interfaces/msg/ParameterEvent] 1 publisher
 * /rosout [rcl_interfaces/msg/Log] 1 publisher

Subscribed topics:

$ ros2 topic hz /ROS2_Time
average rate: 4.484
        min: 0.000s max: 2.659s std dev: 0.73445s window: 12

$ ros2 topic echo /ROS2_Time 
sec: 152
nanosec: 0
---
```

### Writing a minimal ROS2 application

Use the previous workspace if you want, add in the ROS2 messages supported by Ardupilot.

TODO change the URL once it's merged
```
cd microros_ws/src
git clone git@github.com:Ryanf55/ardupilot_ros2.git
cd ../..
colcon build
source install/setup.bash
ros2 run ap_custom_msg_subscribers num_listener
```

You should see output similar to below:
```
[INFO] [1677196375.960439643] [ap_num_subscriber]: From AP, Data:'359'
[INFO] [1677196375.960594857] [ap_num_subscriber]: From AP, Data:'359'
[INFO] [1677196375.960613172] [ap_num_subscriber]: From AP, Data:'359'
```

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
  - **SERIAL1_PROTOCOL 45**
  - **XRCE_TYPE  1**
  - **XRCE_TOPIC 7**

- Once the above values are set ,run the simulator again as :

  - ```sim_vehicle.py --enable-xrce-dds -A "--uartC=uart:/dev/ttyUSB2:115200" --console```

If everything has been setup correctly , you will see the session established message on the agent.

### Reading ROS2 data

- Open a new terminal
- Source ROS2 installation
  - ```source /opt/ros/humble/setup.bash```
- Go to the ROS2 workspace created earlier
- Source the package installation (make sure you are in the root of the dev_ws folder)
  - ```source install/setup.bash```

- Then run the following commands
  - ```ros2 node list```
    - you should see the following name in the node list(if everything is setup correctly)
    - */Ardupilot_XRCE_Client*

  - ```ros2 topic list```
    - you should see the following name in the topic list(if everything is setup correctly)
    - /AP_ROS2_Int8

  - ```ros2 run AP_Ros2_Int listener```

    - you should see the ROS2 subscriber listening to the 8bit Int being sent from Ardupilot
