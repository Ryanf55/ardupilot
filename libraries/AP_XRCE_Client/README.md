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

## Installing Build Dependencies

While DDS support in Ardupilot is mostly through git submodules, another tool needs to be available on your system: Micro XRCE DDS Gen.

1. Go to a directory on your system to clone the repo (perhaps next to `ardupilot`)
1. Install java
  ```console
  sudo apt install java
  ````
1. Follow instructions [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-micro-xrce-dds-gen-tool) to install the generator, but use `develop` branch instead of `master` (for now).
  ```console
  git clone -b develop --recurse-submodules https://github.com/eProsima/Micro-XRCE-DDS-Gen.git
  cd Micro-XRCE-DDS-Gen
  ./gradlew assemble
  ```

1. Add the generator directory to $PATH, like [so](https://github.com/eProsima/Micro-XRCE-DDS-docs/issues/83). 
1. Test it
  ```console
  cd /path/to/ardupilot
  microxrceddsgen -version
  # openjdk version "11.0.18" 2023-01-17
  # OpenJDK Runtime Environment (build 11.0.18+10-post-Ubuntu-0ubuntu122.04)
  # OpenJDK 64-Bit Server VM (build 11.0.18+10-post-Ubuntu-0ubuntu122.04, mixed mode, sharing)
  # microxrceddsgen version: 1.0.0beta2
  ```

> :warning: **If you have installed FastDDS or FastDDSGen globally on your system**:
eProsima's libraries and the packaging system in Ardupilot are not determistic in this scenario.
You may experience the wrong version of a library brought in, or runtime segfaults.
For now, avoid having simultaneous local and global installs.
If you followed the [global install](https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#global-installation)
section, you should remove it and switch to local install.

## Parameters for XRCE DDS

| Name | Description |
| - | - |
| SERIAL1_BAUD | The serial baud rate for XRCE DDS |
| SERIAL1_PROTOCOL | Set this to 45 to use XRCE DDS on the serial port |


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

## Starting with microROS Agent

Follow the steps to use the microROS Agent

- Install ROS Humble (as described here)

  - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

- Install and run the microROS agent (as descibed here). Make sure to use the `humble` branch.

- https://micro.ros.org/docs/tutorials/core/first_application_linux/

Until this [PR](https://github.com/micro-ROS/micro-ROS.github.io) is merged, ignore the notes about `foxy`. It works on `humble`.

Follow the instructions for the following:

* Do "Installing ROS 2 and the micro-ROS build system"
  * Skip the docker run command, build it locally instead
* Skip "Creating a new firmware workspace"
* Skip "Building the firmware"
* Do "Creating the micro-ROS agent"
* Source your ROS workspace

- Run microROS agent with the following command

```bash
cd ardupilot/libraries/AP_XRCE_Client
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/pts/2  -r dds_xrce_profile.xml # (assuming we are using tty/pts/2 for Ardupilot)
```

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


## MicroXRCE (Only if you aren't using ROS2 compatible topics)

The section below applies to MicroXRCE only 

## Starting MicroXRCEAgent

Follow the steps to use the MicroXRCE agent

- Install MicroXRCE Agent (as described here), using the `develop` branch

  - https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-agent-standalone

- In a new terminal, run the following command :

  - ```cd /usr/local/bin && MicroXRCEAgent serial -b 115200 -D /dev/pts/2``` (assuming we are using pts/2 for Ardupilot)

For more information, one can take a look here - https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html#agent-cli

### Starting the Integration Service

- Install dependencies for the Integration Service and build it with `colcon`

  - https://integration-service.docs.eprosima.com/en/latest/installation_manual/installation.html#installation

TODO add to this once EProsima fixes build errors on Ubuntu 22.04.

### Writing a minimal ROS2 application for custom messages

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

## TODO List

* Handle agent connection regardless of boot order and handle restarts of either side
* Determine why the publisher queus up a few messages and the update() is not updated 
  * Tridge suspects it's from the pts, here's some debug steps we tried
  * ```
    strace -ttT -o trace.out -p $(pidof arducopter)
    lsof -n | grep "pts/8"
    cat trace.out | grep "11" | less
    ```
  * If we can reproduce on hardware, then we can drill down
  * Instead of pts, can do two serial ports
* Do ping pong test, get time optimzed over variable length, then do larger packet

## Adding DDS messages to Ardupilot

Unlike the use of ROS 2 `.msg` files, since Ardupilot supports native DDS, the message files follow [OMG IDL DDS v4.2](https://www.omg.org/spec/IDL/4.2/PDF).
This package is intended to work with any `.idl` file complying with those extensions, with some limitations. 

1. IDL files from ROS 2 will need all `module` keywords stripped out. 

To get a new IDL file from ROS2, follow this process:
```console
cd ardupilot
source /opt/ros/humble/setup.bash
# Find the IDL file
find /opt/ros/$ROS_DISTRO -type f -wholename \*builtin_interfaces/msg/Time.idl
# Create the directory in the source tree if it doens't exist
mkdir -p libraries/AP_XRCE_Client/Idl/builtin_interfaces/msg/
# Copy the IDL
cp -r /opt/ros/humble/share/builtin_interfaces/msg/Time.idl libraries/AP_XRCE_Client/Idl/builtin_interfaces/msg/
# Create an output directory to test it
mkdir -p /tmp/xrce_out
# Run the generator
microxrceddsgen -replace -d /tmp/xrce_out libraries/AP_XRCE_Client/Idl/builtin_interfaces/msg/Time.idl
# cat /tmp/xrce_out/
```
