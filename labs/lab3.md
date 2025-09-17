---
title: Lab 3
subtitle: Representing States, System Parameterization, and Services
layout: page
show_sidebar: true
---

# Abstractions to manage complexity

In this lab, we will work on three types of abstractions that we use in robotics to help us manage system complexity.
First, we will keep working on separating functionality into ROS nodes. We will then further separate code within a node based on a system's natural discrete states. Such discrete states are commonly found in robotics and often managed through finite state machines. For instance, the [2001 Mars Odyssey spacecraft](https://mars.nasa.gov/odyssey/), NASA's longest-lasting spacecraft to Mars, consists of multiple stages (states). At each of these states, the spaceship is performing unique functionality, and under certain events, it will transition from one state to the next. The figure below visualizes the [changes in state](https://mars.nasa.gov/odyssey/mission/timeline/mtlaunch/launchsequence/) the Odyssey went through after launch.

Second, we will work on generalizing the applicability of robot systems by parameterizing their functionality. Abstracting parameters from the code and placing them in a more accessible place is common in software engineering. By making parameters configurable during deployment, the system functionality can be tailored without modifying the code.
Last, we will work on abstracting functionality that must be provided synchronously by defining our own services.


<div class="columns is-centered">
    <div class="column is-centered is-12">
        <figure class="image">
        <img src="../images/lab3/rocket.gif">
        </figure>
    </div>
</div>

---

# Learning Objectives

At the end of this lab, you should understand:
* How to add nodes to an existing system
* How to keep rich logs
* How to track and implement states with a finite-state automaton
* How to use ROS messages
* How to use ROS parameters
* How to use ROS services

---

# Improving the Drone Control Software

{% include notification.html
message="Terminology: In robotics, the word **state** is often overloaded. The most general interpretation is that a state represents a snapshot of the system, that is, a set of variable-value pairs representing everything in the system. A more particular definition of  **state** that we will use in a later lab refers mostly to the physical state, including sensed or estimated variables such as the system location or velocity. A third definition, one that we are using in this lab, refers to discrete **states** that can often be associated with certain functionality or certain interpretation of events."
icon="false"
status="is-success" %}

<p> </p>

In Lab 2, we implemented a keyboard manager that published desired positions to be consumed by our drone. We now want to improve on that implementation by providing a couple of extra features:

1. Tracking mission state: By tracking our mission state, we are able to process commands according to the specific mission state we are in.
2. Geofencing: We want to enforce the drone only operates inside a predefined space for the safety of the drone and anyone using it.

To address these issues, we will explicitly define a new node as follows:

<div class="columns is-centered">
    <div class="column is-centered is-6">
        <figure class="image">
        <img src="../images/lab3/goal.png">
        </figure>
    </div>
</div>

## Create the "Geofence and Mission" Node

The first step in improving the drone's control software will be to create the node. We will start the implementation of this node by only considering the first objective:

* Track the mission state of the drone

To start pull the latest code ***inside of a new Docker terminal***.
```bash
# Change to lab directory
cd ~/csci_420_robotics_labs/
# Clone the code
git pull
```

You should see a new workspace `lab3_ws`. This workspace will have the keyboard node and keyboard manager already implemented for you.

To track the drone's mission state, we are going to need to create a new node in the `simple_control` package. Create a new node in the `simple_control` package in the `lab3_ws` workspace called `geofence_and_mission.py`.

Once you've created the new node, you must update the `~/csci_420_robotics_labs/lab3_ws/src/simple_control/setup.py` file to tell ROS to include the new node in the build. Find the dictionary of entry points and edit it as shown below:

Old:
```python3
    entry_points={
        'console_scripts': [
            'keyboard_manager = simple_control.keyboard_manager:main'
        ],
    },
```

New:
```python3
    entry_points={
        'console_scripts': [
            'keyboard_manager = simple_control.keyboard_manager:main',
            'geofence_and_mission = simple_control.geofence_and_mission:main'
        ],
    },
```

{% include notification.html
message="Note: remember to give the node execution permissions using the chmod command."
icon="false"
status="is-success" %}

<p> </p>

The software of a robot operation can be complex. One way to manage this complexity is to decouple the functionality based on the system into discrete states and then organize the system as a Finite State Automaton (FSA). An FSA is a mathematical computation model that can be in exactly one of a finite number of states at any given time, and where the system can make well-defined transitions from one state to another in response to inputs or events.
Using an FSA we will design the `geofence_and_mission.py` node as follows


<div class="columns is-centered">
    <div class="column is-centered">
        <figure class="image">
        <img src="../images/lab3/statesafety.png">
        </figure>
    </div>
</div>



The `geofence_and_mission.py` node will consume a position that is published by the keyboard manager. If we are in state 1, the hovering state, and a new position is set then we will transition into state 3, the moving state. The moving state will move the quadrotor to the requested position. Once the drone reaches the required position, the FSA will transition back to state 1, the hovering state, and wait for the next input. There are 4 main design considerations to take into account:

1. Keep track of the drone's mission state.
2. Encode state transitions.
3. In lab 2, our keyboard manager integrated directly with the drone on the topic `/uav/input/position`. We will now need the keyboard manager to publish positions instead (the desired input to the FSA). This allows us to separate the logic into different components as discussed previously.
4. Need a way to tell when quadrotor has reached the desired position so that it can transition from back to state 1.

To implement each of these points, we will do the following:

**Fulfilling 1)** We will implement a state machine. We can do that in many ways (e.g., nested switches, table mapping, state pattern), but for simplicity and given the number of states, we will use a sequence of nested predicates and start by declaring an enumeration that represents the possible states. An enumeration is a set of symbolic names (members) bound to unique, constant values.


{% include notification.html message="Note: We are not yet using the `VERIFYING` state. This will be used later in the lab." %}

<p> </p>

```python
# A class to keep track of the quadrotors state
class DroneState(Enum):
    HOVERING = 1
    VERIFYING = 2
    MOVING = 3
```

**Fulfilling 2)** We will break the code up into a set of code functions. Each of the functions will encode the drone behavior within a state. We can then track the mission states using a class variable and call the correct function based on the state. For example, if the quadrotor is in the `MOVING` state, then the `processMoving()` function will be called.

```python
# Check if the drone is in a moving state
if self.state == DroneState.MOVING:
    self.processMoving()
# If we are hovering then accept keyboard commands
elif self.state == DroneState.HOVERING:
    self.processHovering()
```

**Fulfilling 3)** We will subscribe to the `/keyboardmanager/position` topic using a subscriber so that we can start setting up the `geofence_and_mission` logic using this data. (Later in the lab we will adjust the keyboard manager so that it publishes the correct messages)

```python
self.keyboard_sub = self.create_subscription(Vector3, '/keyboardmanager/position', self.getKeyboardCommand, 1)
```

**Fulfilling 4)** We will need a way to get the drone's current position (we will do this after Checkpoint 1) to transition back from `MOVING` to `HOVERING`. Assuming we will have that information, we can implement a simple check to see how far away the quadrotor is from the desired goal. If the quadrotor is inside an acceptance-range, then the machine can transition back into a hovering state:

```python
dx = self.goal_cmd.x - self.drone_position.x
dy = self.goal_cmd.y - self.drone_position.y
dz = self.goal_cmd.z - self.drone_position.z
# Euclidean distance
distance_to_goal = np.sqrt(np.power(dx, 2) + np.power(dy, 2) + np.power(dz, 2))

# If goal is reached transition to hovering
if distance_to_goal < self.acceptance_range:
    self.state = DroneState.HOVERING
    ...
```

## The Code for the Geofence and Mission node

Putting it all together: copy and paste the following code into the `geofence_and_mission.py` node. You will notice that we are using the previous points in the code and a few others that we discuss next.

```python
#!/usr/bin/env python
import copy
from enum import Enum
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3, PoseStamped, Point



# A class to keep track of the quadrotors state
class DroneState(Enum):
    HOVERING = 1
    VERIFYING = 2
    MOVING = 3


# Create a class which takes in a position and verifies it, keeps track of the state and publishes commands to a drone
class GeofenceAndMission(Node):

    # Node initialization
    def __init__(self):
        super().__init__('GeofenceAndMission')
        # Create the publisher and subscriber
        self.position_pub = self.create_publisher(Vector3, '/uav/input/position', 1)
        self.keyboard_sub = self.create_subscription(Vector3, '/keyboardmanager/position',self.getKeyboardCommand, 1)

        # TO BE COMPLETED FOR CHECKPOINT 2
        # TODO: Add a position_sub that subscribes to the drones pose

        # TO BE COMPLETED FOR CHECKPOINT 3
        # TODO: Get the geofence parameters
        # Save the acceptance range
        self.acceptance_range = 0.5
        # Create the drones state as hovering
        self.state = DroneState.HOVERING
        self.get_logger().info("Current State: HOVERING")
        # Create the goal messages we are going to be sending
        self.goal_cmd = Vector3()

        # Create a point message that saves the drones current position
        self.drone_position = Point()

        # Start the drone a little bit off the ground
        self.goal_cmd.z = 3.0
        
        # TO BE COMPLETED FOR CHECKPOINT 5
        # TODO: Create the Toggle_Geofence service object, create the handler, etc.
        self.geofence_on = True  # use this variable later

        # Keeps track of whether the goal  position was changed or not
        self.goal_changed = False

        # Set the timer to call the mainloop of our class
        self.rate = 20
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)


    # Callback for the keyboard manager
    def getKeyboardCommand(self, msg):
        # Save the keyboard command
        if self.state == DroneState.HOVERING:
            self.goal_changed = True
            self.goal_cmd = copy.deepcopy(msg)


    # TO BE COMPLETED FOR CHECKPOINT 2
    # TODO: Add function to receive drone's position messages


    # Converts a position to string for printing
    def goalToString(self, msg):
        pos_str = "(" + str(msg.x)
        pos_str += ", " + str(msg.y)
        pos_str += ", " + str(msg.z) + ")"
        return pos_str


    # TO COMPLETE FOR CHECKPOINT 4
    # TODO: Implement processVerifying
    def processVerifying(self):
        # Check if the new goal is inside the geofence

        # If it is change state to moving
        # If it is not change to hovering
        pass


    # This function is called when we are in the hovering state
    def processHovering(self):
        # Print the requested goal if the position changed
        if self.goal_changed:
            self.get_logger().debug(f"Requested Position: {self.goalToString} ({self.goal_cmd})")
            self.get_logger().info("Current State: MOVING")
            #  TO BE COMPLETED FOR CHECKPOINT 4
            # TODO: Update
            self.state = DroneState.MOVING
            self.goal_changed = False


    # This function is called when we are in the moving state
    def processMoving(self):
        # Compute the distance between requested position and current position
        dx = self.goal_cmd.x - self.drone_position.x
        dy = self.goal_cmd.y - self.drone_position.y
        dz = self.goal_cmd.z - self.drone_position.z

        # Euclidean distance
        distance_to_goal = np.sqrt(np.power(dx, 2) + np.power(dy, 2) + np.power(dz, 2))
        # If goal is reached transition to hovering
        if distance_to_goal < self.acceptance_range:
            self.state = DroneState.HOVERING
            self.get_logger().info("Complete")
            self.get_logger().info("-------------------------------")
            self.get_logger().info("Current State: HOVERING")

    # The main loop of the function
    def mainloop(self):
        # Publish the position
        self.position_pub.publish(self.goal_cmd)

        # Check if the drone is in a moving state
        if self.state == DroneState.MOVING:
            self.processMoving()
        # If we are hovering then accept keyboard commands
        elif self.state == DroneState.HOVERING:
            self.processHovering()
        elif self.state == DroneState.VERIFYING:
            self.processVerifying()

def main():
    rclpy.init()
    try:
        rclpy.spin(GeofenceAndMission())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()
```

## A useful resource to notice: ROS Logging

You will notice that this code no longer uses a standard print command.
Each of the print commands has been replaced with a `self.get_logger().debug("...")` command.
Using a log command in ROS is a good practice.
A print command will print a message to the terminal, with no extra logging information.
While using the terminal is easy enough for small, simple applications, there are many instances where this is impractical.
If there are lots of nodes, or a node that prints lots to the terminal, the messages you want to investigate will be difficult to find.
A `self.get_logger().info()` command will print a message to the terminal, as well as keep that message inside the ROS logs.
That way, you can go back and review your robot's behavior at a later stage or easily compare logs with other people.
ROS logging allows you to create levels of messages so that important messages and less important messages can be distinguished.
The most important messages are called fatal messages. To publish a fatal message, you use the command `.fatal()`.
The lowest level of a message is a debug message which can be logged using `.debug()`.
More on ROS logging can be found on the [ROS Wiki](https://wiki.ros.org/rospy/Overview/Logging).


## Updating the Keyboard Manager

Now let's make the changes to the `keyboard_manager` node to fulfill requirement **#2**. The `keyboard_manager` node needs to change in a few ways:

1. We need to publish to the `/keyboardmanager/position` topic which is of type `Vector3` containing the setpoint command for the drone position. This topic is subscribed to by the `geofence_and_mission` node.
2. It should only publish a message once a position is set instead of continuously publishing any keyboard input. To do this, we will change the `keyboard_manager` only to publish messages when the user hits **enter** on their keyboard.
3. The `keyboard_manager` will display the position to be sent onto the terminal using logging.

To address each of these changes, we will make the following updates to the keyboard_manager.

**Fulfilling 1)** Change what the node publisher sends by updating the publishing statement:

```python
self.position_pub = self.create_publisher(Vector3, '/keyboardmanager/position', 1)
```

**Fulfilling 2)** Change the main loop to only publish messages when the user hits enter:

```python
# If the user presses the ENTER key
if self.key_code == Key.KEY_RETURN:
    # TO BE COMPLETED FOR CHECKPOINT 1
    # TODO : Publish the position (self.pos)
    self.get_logger().info("Sending Position")
```

**Fulfilling 3)** Log positions typed when they change. If the `prev_pos` and `pos` do not match, the `pos` has changed:

```python
# TO BE COMPLETED FOR CHECKPOINT 1
# TODO: Check if the position has changed by comparing it to the current position
# Note you will need to change the if statement below from if False -> if #... TODO
if False:
    self.prev_pos = copy.deepcopy(self.pos)
    self.get_logger().info(f"Keyboard: {self.goalToString(self.pos)}")
```

## The Keyboard Manager Code

When you implement these changes your new `keyboard_manager` should look like this:

```python
#!/usr/bin/env python
import numpy as np
import copy
import rclpy
from rclpy.node import Node
from keyboard_msgs.msg import Key
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3
import time

# Create a class which we will use to take keyboard commands and convert them to a position
class KeyboardManager(Node):
    # On node initialization
    def __init__(self):
        super().__init__('KeyboardManagerNode')
        # Create the publisher and subscriber
        self.position_pub = self.create_publisher(Vector3, '/keyboardmanager/position', 1)
        self.keyboard_sub = self.create_subscription(Key, '/keydown', self.get_key, 1)
        # Create the position message we are going to be sending
        self.pos = Vector3()
        self.prev_pos = Vector3()
        # Create a variable we will use to hold the key code
        self.key_code = -1
        # Give the simulation enough time to start
        time.sleep(10)
        # Call the mainloop of our class
        self.rate = 20
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    # Callback for the keyboard sub
    def get_key(self, msg):
        self.key_code = msg.code

    # Converts a position to string for printing
    def goalToString(self, msg):
        pos_str = "(" + str(msg.x)
        pos_str += ", " + str(msg.y)
        pos_str += ", " + str(msg.z) + ")"
        return pos_str


    def mainloop(self):
        # Check if any key has been pressed
        # Left
        if self.key_code == Key.KEY_LEFT:
            self.pos.x = self.pos.x + np.cos(0)
            # Up
        if self.key_code == Key.KEY_UP:
            self.pos.y = self.pos.y + np.sin(np.pi/2)
            # Right
        if self.key_code == Key.KEY_RIGHT:
            self.pos.x = self.pos.x - np.sin(3*np.pi/2)
            # Down
        if self.key_code == Key.KEY_DOWN:
            self.pos.y = self.pos.y - np.cos(np.pi)


        # Publish the position
        if self.key_code == Key.KEY_RETURN:
            # TO BE COMPLETED FOR CHECKPOINT 1
            # TODO : Publish the position (self.pos)
            self.get_logger().info("Sending Position")

        # TO BE COMPLETED FOR CHECKPOINT 1
        # TODO: Check if the position has changed by comparing it to the current position
        # Note you will need to change the if statement below from if False -> if #... TODO
        if False:
            self.prev_pos = copy.deepcopy(self.pos)
            self.get_logger().info(f"Keyboard: {self.goalToString(self.pos)}")

        # Reset the code
        if self.key_code != -1:
            self.key_code = -1

def main():
    rclpy.init()
    try:
        rclpy.spin(KeyboardManager())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()
```

## Putting it All Together

The last thing we need to do is to change our launch file so that we can run each of the nodes. To do that, add each node to the roslaunch file. The launch file is located in: `~/csci_420_robotics_labs/lab3_ws/src/flightcontroller/launch/fly.launch`

```xml
...
        <!--From Lab 2-->
<node name="keyboard_manager_node" pkg="simple_control" exec="keyboard_manager" output="screen"/>
<node name="keyboard" pkg="keyboard" type="keyboard"/>
        <!-- To be added-->
        ...
```


Remember that you will need to `colcon build` and `source install/setup.bash` as discussed in Lab 2 before launching the program.


---

# Checkpoint 1

Launch the simulator and check that you have correctly created the geofence and mission node as well as changed the keyboard manager to publish the correct data. Your ROS computation graph should look like the one below. Take a screenshot of the ROS computation graph:


<div class="columns is-centered">
    <div class="column is-centered is-12">
        <figure class="image">
        <img src="../images/lab3/check1.png">
        </figure>
    </div>
</div>

Next, try to fly the quadrotor using the keyboard. Change the requested position using the keyboard keys. Once you have selected a position, hit the ENTER key to move the drone.

1. What happens when you hit the enter key? Answer this in terms of the FSA states we implemented
2. What happens when you request the drone to fly to a second position? Answer this in terms of the actual code used in `geofence_and_mission.py`.

---

## Message Types

Let's implement the drone's position tracking. Start by figuring out what topic the drone's position is being published on. To do this, start the simulation and list the available topics:

```bash
ros2 topic list
```
Expected output
```bash
...
/uav/sensors/gps
...
```

The list of available topics should look as shown above. As you become more familiar with robotics in ROS, you will start to work with a set of common message types. This was purposefully done by the ROS developers to try and keep a set of standard messages and thus increase compatibility between different nodes, and allow for faster development.

One of the standard ROS message types is `PoseStamped` which contains both the robot's pose, and information about how that pose was calculated.
A pose represents a robot's position and orientation. 
You will notice a topic called `/uav/sensors/gps`.
Let's identify what this topic is publishing and what message type it is. First, let's see if this is the topic we want. Echo the topic:

```bash
ros2 topic echo /uav/sensors/gps
```
Expected output:
```bash
pose: 
  position: 
    x: -0.01205089835
    y: -0.014817305419
    z: 0.548691896515
  orientation: 
    x: 0.000119791513234
    y: -8.21647300556e-05
    z: -0.000329261478202
    w: 0.999999935243
---
...
```

Nice! Just as we expected, there is a position and an orientation. Let's now try and figure out what type this message is so that we can understand its structure and subscribe to it. To find more information about a topic, you can use another useful command: `ros2 topic info`. This command prints all the information for a given topic. Run the command:

```bash
ros2 topic info /uav/sensors/gps 
```

Expected output:
```bash
Type: geometry_msgs/msg/PoseStamped
Publisher count: 1
Subscription count: 4
```

We can see that the message `/uav/sensors/gps` is of message type `geometry_msgs/PoseStamped`. To get more information about the specific message structure we can use another useful command `ros2 interface show`. Run the command:
```bash
ros2 interface show geometry_msgs/msg/PoseStamped
```
Expected output:
```bash
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id
Pose pose
        Point position
                float64 x
                float64 y
                float64 z
        Quaternion orientation
                float64 x 0
                float64 y 0
                float64 z 0
                float64 w 1
```

{% include notification.html
message="More information about this message type (and others) can also be found on the [ROS Docs](https://docs.ros.org/en/ros2_packages/kilted/api/geometry_msgs/msg/PoseStamped.html)"
icon="false"
status="is-success" %}

<p> </p>

Using the output of the `ros2 interface show` command for reference, we can see that it is a standard ROS message that consists of a header as well as a pose. Below is a diagram of the message:

<div class="columns is-centered">
    <div class="column is-centered is-10">
        <figure class="image">
        <img src="../images/lab3/structure.png">
        </figure>
    </div>
</div>

You will see that a `Pose` consists of a position and an orientation. Interestingly the position is of type `Point`, which is also declared in `geometry_msgs`. We find that the message type `Point` consists of three floats `x`, `y`, and, `z`. This method of identifying common messages can be repeated. In general, it is better to use standard ROS messages, as it allows faster development times and standardizes the messages sent on topics for easier integration between different projects. It is analogous to how cellphones all use standard communication standards (within a country at least). If every company designed their own, keeping track of which phone comm to use would be a nightmare.

{% include notification.html
message="Note: There are ways to create custom message types. You can find more information on this in the extra section. If possible, standard message usage is recommended. "
icon="false"
status="is-success" %}

## What's in a message type?
Above we saw that the `PoseStamped` message type used the `Point` [message type](https://docs.ros.org/en/kilted/p/geometry_msgs/msg/Point.html) which consists of 3 fields `x, y, z`. However, if we look at the list of available message types, we see that there is also a `Vector3` [message type](https://docs.ros.org/en/kilted/p/geometry_msgs/msg/Vector3.html) which has the same three fields `x, y, z`. The fields contained by these two message types are exactly the same! Why would we want to have two message types for the same information?

Looking at the comments in the documentation below, we can see why: they represent different things in the physical world.
`Point` represents a location in 3D, whereas `Vector3` represents a **direction**.
The ROS developers understood these were two common use cases for `x, y, z` data, and so created different message types to represent them.
By using different message types to represent these different physical comments, it not only helps make your code more readable, maintainable, and portable, but also can allow for automatic type checking and let different libraries automatically know how to interpret the data.

The ROS library has many common types to represent many common physical quantities. In addition to trying to use an existing type when possible, you should always try to use a type that represents the physical quantity that you need to send.

<div class="columns is-centered">
    <div class="column is-centered is-10">
        <figure class="image">
        <img src="../images/lab3/point_and_vector.png">
        </figure>
    </div>
</div>


# Adding Position Tracking to Enable the Transition from Moving to Hovering
Using what we learned with message types, let's adapt our `geofence_and_mission.py` node to subscribe to the `Pose` topic published by the simulator. Doing this will allow us to monitor when the drone reaches a goal and transitions from state `MOVING` back to state `HOVERING`. In the `geofence_and_mission.py` node, write code to fill in the two `#TODO` sections.

First, in the node initialization, subscribe to the drone's pose using the correct topic name, and message type. Direct each of the messages to a callback.

Second, implement the callback for this message. The callback should store the drone's position to the class variable `self.drone_position`. You can access each component of the message using standard dot notation. For instance, if I wanted to get the orientation of the quadrotor, I could use:

```python
msg.pose.orientation
```

Where ``msg`` is passed into the callback as a function parameter.

---
# Checkpoint 2

Launch the simulator and check that you have correctly created the geofence and mission node as well as changed the keyboard manager to publish the correct data. Your ROS computation graph should look as shown below. Take a screenshot of the ROS computation graph:

<div class="columns is-centered">
    <div class="column is-centered is-12">
        <figure class="image">
        <img src="../images/lab3/check2.png">
        </figure>
    </div>
</div>

Next, try to fly the quadrotor using the keyboard. You should see your quadrotor move after you hit the ENTER key.
1. What happens when you request the drone to fly to a second position?
2. Send a message to the drone while it is moving to a target position. What happens? Why?

---


# Abstracting Parameters

Abstracting parameters from the code and placing them in a more accessible place is a common practice in software engineering. Without this, configuring codebases to a particular situation would be extremely labor-intensive. In terms of robotics, think about designing a package that can successfully navigate and control ground robots. In order to keep this package general and applicable to multiple ground vehicles, many parameters of the vehicle need to be abstracted, such as the maximum velocity, the maximum steering angle, the size, the sensor capabilities, etc. A good example of a package that is used in robotic navigation is `move_base`. This package is used in robots such as the PR2, Husky, and Turtlebot all shown below:

<div class="columns is-centered">
    <div class="column is-4">
        <figure class="image is-square">
            <img src="../images/lab3/robot_a.png">
        </figure>
        <p href='http://www.willowgarage.com/pages/pr2/overview' class='has-text-centered'>PR2 Robot</p>
    </div>
    <div class="column is-4">
        <figure class="image is-square">
            <img src="../images/lab3/robot_b.png">
        </figure>
        <p href='https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/' class='has-text-centered'>Husky Robot</p>
    </div>
    <div class="column is-4">
        <figure class="image is-square">
            <img src="../images/lab3/robot_c.png">
        </figure>
        <p href='https://www.turtlebot.com/turtlebot2/' class='has-text-centered'>Turtlebot 2 Robot</p>
    </div>
</div>

Let's learn how to configure our quadrotor to use parameters.


# Adding a Verifying State

To learn to apply parameters, let's start by implementing the verification state to forbid the quadrotor from flying outside of a virtual geofence. Verifying that a waypoint is within a geofence (a virtual cage) is a good practice as it makes sure that you do not accidentally send a waypoint to the quadrotor that causes it to fly away or crash into a known obstacle. In general, most commands sent to a robot that is going to result in the robot performing some action in the real-world should be verified for the safety of both the people around it and itself.

<div class="columns is-centered">
    <div class="column is-centered is-6">
        <figure class="image">
        <img src="../images/lab3/geofence.png">
        </figure>
    </div>
</div>

The first thing we need is to create a geofence area in such a way that it is easy for the user to change the parameters of the geofence. That way, if the drone is deployed into an area of a different size, it can quickly and easily change the geofence size without having to change the code.

## Using Parameters

We can do that in ROS using parameters. The [ROS docs](https://docs.ros.org/en/kilted/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) describe parameters as "A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters".
In other words, our node can reference these parameters to get the geofence area dimensions. The benefit of doing this is we can move setting the parameters into the launch file. That way, if we want to change the geofence area size, all we have to do is change the values in the launch file. This is beneficial as any parameters you want to set get abstracted out of the code, making the code easier for you to manage. It also makes it easier for people using your code as they don't need to understand your implementation to configure the parameters.

You could take this one step further and expose the parameters as command-line arguments, allowing you to pass the parameters in from the command line.

{% include notification.html
message="Extra: Here is more information on [custom message types](https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html), and adding [command-line arguments](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Launch/Using-Substitutions.html) to a launch file "
icon="false"
status="is-success" %}


We will start by querying the parameters for the geofence size and acceptance range from inside the verification node. We will call the verification node the `geofence_and_mission_node`. We will name the geofence area the `geofence`. Add the following code in the node initialization:

```python
# get the acceptance range
acceptance_range_param = '/geofence_and_mission_node/acceptance_range'
self.declare_parameter(acceptance_range_param, 0.5)
self.acceptance_range = self.get_parameter(acceptance_range_param).get_parameter_value().double_value
# get the geofence
geofence_params = '/geofence_and_mission_node/geofence'
self.declare_parameter(f'{geofence_params}/x', 5.0)
self.declare_parameter(f'{geofence_params}/y', 5.0)
self.declare_parameter(f'{geofence_params}/z', 5.0)
geofence_x = self.get_parameter(f'{geofence_params}/x').get_parameter_value().double_value
geofence_y = self.get_parameter(f'{geofence_params}/y').get_parameter_value().double_value
geofence_z = self.get_parameter(f'{geofence_params}/z').get_parameter_value().double_value
self.geofence_x = [-1*geofence_x, geofence_x]
self.geofence_y = [-1*geofence_y, geofence_y]
self.geofence_z = [0, geofence_z]
# Display the parameters
self.get_logger().info(f'Geofence X: {self.geofence_x}')
self.get_logger().info(f'Geofence Y: {self.geofence_y}')
self.get_logger().info(f'Geofence Z: {self.geofence_z}')
self.get_logger().info(f'Acceptance Range: {self.acceptance_range}')
```

Let's spend some time understanding how this code works.

First, we changed the acceptance range to be queried from the parameter server. We also give this parameter a default value of 0.5 (in case the parameter can not be found). Next, when we look at the geofence, we see that it has 3 separate parameters, one for each dimension. These values are then used to create the geofence variables that describe the acceptable area.

Recall that  such values are set in the launch file. To set the parameters in the launch file (`~/csci_420_robotics_labs/lab3_ws/src/flightcontroller/launch/fly.launch`) change it as follows:

```xml
        <node name="geofence_and_mission_node" pkg="simple_control" exec="geofence_and_mission" output="screen">
    <param name="/geofence_and_mission_node/geofence/x" type="float" value="5" />
    <param name="/geofence_and_mission_node/geofence/y" type="float" value="5" />
    <param name="/geofence_and_mission_node/geofence/z" type="float" value="2" />
    <param name="/geofence_and_mission_node/acceptance_range" type="float" value="0.5" />
</node>
```

You will notice that each of the parameters we query are now defined inside the launch file.

--- 

# Checkpoint 3

Check that you can change the geofence parameters by changing the values in the launch file. Change the geofence size to 3, 3, 3, and the acceptance range to 0.25. Run the simulator and show us your output:

**Note:** Do not change the default values inside the code; only change the launch file. This is advantageous in that we can adjust the behavior of the system without rebuilding, recoding, or redeploying the system.

Below is an example of what you would see when you launch the simulator after changing the launch file:

<div class="columns is-centered">
    <div class="column is-centered is-12">
        <figure class="image">
        <img src="../images/lab3/check3.png">
        </figure>
    </div>
</div>

---

## Verifying that Waypoints are within Cage

Next lets adapt our FSA to include a verifying state. This verification state will verify the command position and make sure it is inside the geofence before transitioning to a moving state. The design for the final `geofence_and_mission` node will be as follows:


<div class="columns is-centered">
    <div class="column is-centered is-12">
        <figure class="image">
        <img src="../images/lab3/fsa_diagram.png">
        </figure>
    </div>
</div>


The new design uses an FSA, which works as follows: First when we receive a position message, we transition to a verifying state. The verifying state checks if the published position is inside the geofence. If the new position is outside the geofence, the position is rejected, and the FSA transitions back into the hovering state. If the position is inside the geofence, the position is accepted, and the FSA transitions to the moving state. When the drone reaches the final position, the FSA transitions back to the hovering state, where the next command can be given to the quadrotor.

{% include notification.html message="Notice that inside the main loop you are constantly publishing the `goal_cmd`. You will thus need to make sure that you are not updating the `goal_cmd` until you are certain that it has been verified." %}

<p> </p>

Update the `geofence_and_mission.py` node to implement the extended  FSA.

---

# Checkpoint 4

Show the quadrotor flying inside the geofence area. First, send the drone a waypoint inside the geofence area. Second, send the drone a waypoint outside of the geofence area.

<div class="columns is-centered">
    <div class="column is-centered is-12">
        <figure class="image">
        <img src="../images/lab3/check4.gif">
        </figure>
    </div>
</div>

---

# Using ROS Services

For the last section of this lab we will implement two services that allow us to control our quadrotor's behavior. Services, like topics, are another way to pass data between nodes in ROS. Services differ from topics in two key ways, 1) services are synchronous procedure calls, whereas topics are asynchronous communication channels, and 2) services return a response to each request, whereas topics do not. Each service will have a defined input and a defined output format. The node which **advertises the service** is called a **server**. The node which **calls the service** is called the **client**.

Services are generally used when you need to run functions that require a response. Good scenarios to use a service are:

+ When you want to execute discrete actions on a robot, for instance
    + Turning a sensor on or off
    + Taking a picture with a camera
+ When you want to execute computations on another node

## Implementing a Service

The first service we will be implementing will allow us to "calibrate" the drone's pressure sensor. A pressure sensor returns the atmospheric pressure, which at sea level is approximately 1,013.25 millibars. When invoked, the calibration service zeroes the baseline pressure so that later pressure differentials can be used to compute altitude estimates. The service will return the baseline pressure to the user.

Unlike topics that use a publish-subscribe architecture, services use a request-reply architecture. This interaction is defined by a pair of messages, one for the request and one for the reply. To declare this pair of messages, we need to build a service definition file. These service definition files are declared by us using ROS message types. Our service's request will be a boolean value from the `std_msgs` library that describes whether the pressure sensor should be zeroed. Our service reply is a float value from the `std_msgs` library that contains the current baseline pressure reading.

Unlike topic messages, there are no class libraries for service types. This is because services are dynamic and build upon the existing ROS message types. Due to this, we need to 1) create the service definition file and 2) configure Catkin to build the service definition file. A similar process is done when you want to create custom message types used for topics.

Service definition files are typically put in a directory called `srv`.

+ Start by creating the directory srv inside the `sensor_simulation` package.
+ Inside the srv folder create the service definition file called `Calibrate.srv`

Now, let's add content to the service definition file. Service definition files start with the specification of the request (input), followed by the specification of reply (output). The request and reply are separated using three dashes. Let's add the request and reply for our pressure calibration service:

```
bool zero
---
float64 baseline
```

Remember, there are no standard class libraries for service types, and so rather than using a standard library as we do with messages, we need to configure our workspace to build them for us. To configure the workspace to build our new service files, we need to make changes to the makefiles. The service is being created inside the `sensor_simulator` package. Thus **all changes to the Make files need to be done inside this package**.

Update the `sensor_simulation/CMakeList.txt` to add the following after the other `find_package` lines:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Calibrate.srv"
)
```

We then need to add the additional package dependencies to the `sensor_simulation/package.xml` file. Add the following lines inside this file. These to allow our package to generate service messages.

```
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Let's now check if we have done everything correctly. It is always a **good idea to break long processes like this into smaller steps** to allow you to identify early on if you have made a mistake. Let's compile the workspace and check if we can validate that the service call definition exists. To do this run:

```bash
cd ~/csci_420_robotics_labs/lab3_ws/
colcon build
source install/setup.bash
ros2 interface show sensor_simulation/srv/Calibrate
```

The result should be:
```bash
bool zero
---
float64 baseline
```

If the ros2 interface list can find the service definition `Calibrate.srv` you have configured your make files correctly, and `colcon build` can generate the message files.
ROS automatically generates three type definitions when we add a new service definition file. In general, if you create a service definition file `<name>.srv`, ROS will create the following type definitions:

+ `<name>_Response`
+ `<name>_Request`

Therefore in our case, there are three new types we can reference in Python, namely, `Calibrate`, `Calibrate_Response`, and `Calibrate_Request`. A service that handles our calibration process will take in a `Calibrate_Request` and return a `Calibrate_Response`. Keeping this in the back of our minds, let's move forward and implement the service handler inside the pressure node.

Start by importing the types created by ROS. The pressure node is the **server** only needs to import `Calibrate` since Python can infer the other types. In C++, you would need to directly use all 3 types. Add the following import to `pressure.py`:

```python
from sensor_simulation.srv import Calibrate
```

Next, let's add the service to this node after the node initialization. This will advertise the service under the name `calibrate_pressure`. The service will have input and output as defined by the `calibrate.srv` and will redirect all service requests to the function `self.calibrateFunction`:

```python
self.service = self.create_service(Calibrate, 'calibrate_pressure', self.calibrateFunction)
```

Finally, we can define the server function that will handle the service requests. You will notice that inside the `pressure.py` node, there is a class variable `baseline_value` that defaults to 0. This function will update that value:

```python
# Saves the baseline value
def calibrateFunction(self, request, response):
    # If we want to calibrate
    if request.zero:
        self.baseline_value = self.pressure
    else:
        self.baseline_value = 0
    response.baseline = self.baseline_value
    # Return the new baseline value
    return response
```

Let's again test if we have set the service up correctly. To test whether the service was set up correctly, launch the simulator. The first thing to check is that there are no errors when launching. If there are no errors, proceed to list the available services:

```bash
ros2 service list
>>> ...
>>> /position_controller_node/describe_parameters
>>> /position_controller_node/get_parameter_types
>>> /position_controller_node/get_parameters
>>> /position_controller_node/get_type_description
>>> /position_controller_node/list_parameters
>>> /position_controller_node/set_parameters
>>> /position_controller_node/set_parameters_atomically
>>> ...
```

You will notice that even for this small simulator, there are a lot of services. A quick way to find what you are looking for is to send all the output text to a `grep` command, known as piping to grep. `grep` is a linux program **used for searching for keywords through text**. Let's try this now:

```bash
ros2 service list | grep calibrate
>>> /calibrate_pressure
```

This command will output all services that contain the word calibrate. Keep in mind that piping to grep is not specific to ROS and **so can be used at any time in Linux**. Now that we know that our service is being advertised let's check the type of this service using:

```bash
ros2 service info /calibrate_pressure
>>> Type: sensor_simulation/srv/Calibrate
>>> Clients count: 0
>>> Services count: 1
```

We can see that everything is as expected. We can see the Node which provides the service. We can see what message type the service uses and what the input arguments are. We are now ready to test our service. Open three terminals. In the first terminal, we will build and launch the simulator. In the second terminal, we will echo the pressure topic to see if our calibration was successful. In the third terminal, we will call the calibration service.

### Terminal 1

```bash
source ~/csci_420_robotics_labs/lab3_ws/install/setup.bash
ros2 launch flightcontroller fly.launch
```

### Terminal 2

```bash
source ~/csci_420_robotics_labs/lab3_ws/install/setup.bash
ros2 topic echo /uav/sensors/pressure
```

Expected output:
```bash
>>> data: 1013.24774066
>>> ---
>>> data: 1013.24769358
>>> ---
>>> data: 1013.24773901
>>> ...
```

### Terminal 3

```bash
source ~/csci_420_robotics_labs/lab3_ws/install/setup.bash
ros2 service call /calibrate_pressure sensor_simulation/srv/Calibrate "{zero: True}"
```

Expected output:
```bash
requester: making request: sensor_simulation.srv.Calibrate_Request(zero=True)

response:
sensor_simulation.srv.Calibrate_Response(baseline=1012.8884926210843)
```

If everything was implemented correctly after running the command in terminal 3, you should see that the topic `/uav/sensors/pressure` is now publishing messages that have been zeroed according to the returned baseline.

### Terminal 2

```bash
>>> data: -0.00017854943485
>>> ---
>>> data: -0.000147954746467
>>> ---
>>> data: -0.00016459436074
```

# Implementing your own service

Now use what you have learned to implement a service that will toggle the geofence area created during Checkpoint 2 on and off. Inside the `geofence_and_mission` node in the `simple_control` package, create a service `Toggle_Geofence` that has its inputs and outputs defined in the service definition file `Toggle_Geofence.srv`. This service should take in a **boolean input** parameter called `geofence_on` and **output a boolean** parameter `success`. The service should allow the user to turn the geofence area on or off and should return whether the call was a success or not (**note**: for our implementation it should always succeed in turning the geofence area on or off; however, we could imagine a different implementation that will not let you engage the geofence unless the quadrotor is already inside the geofence).


### Converting from a pure Python package to C++

Note that `simple_control` does not currently have a  `CMakeLists.txt`, yet we had to add lines to that file to generate our service for pressure.
This is because the `simple_control` package was initially defined as a pure Python package. However, the build tools used to create services rely on C++ components, so ROS requires us to convert this to use that build chain instead. To do this we will make several edits:

#### Setting up CMakeLists.txt

Create the corresponding `CMakeLists.txt` and initialize it with the following before making the necessary adjustments:

```
cmake_minimum_required(VERSION 3.14.4)
project(simple_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin and any catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)

# TODO add lines for handling the service

ament_python_install_package(src)

install(PROGRAMS
  src/keyboard_manager.py
  DESTINATION lib/simple_control )

install(PROGRAMS
  src/geofence_and_mission.py
  DESTINATION lib/simple_control )

ament_package()
```

#### Edit package.xml
Replace the `package.xml file as follows`
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>simple_control</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="root@todo.todo">root</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>ament_xmllint</test_depend>
  <test_depend>python3-pytest</test_depend>

  <buildtool_depend>ament_cmake_ros</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <depend>rclcpp</depend>

  <!-- TODO: Add service pieces here-->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### Clean up remaining items
Delete the `setup.cfg` and `setup.py` files. In the `flightcontroller/launch/fly.launch` file, the Python files must now be referenced using their file path, e.g., `keyboard_manager.py` instead of `keybaord_manager`.

---

# Checkpoint 5

Show that you can turn the geofence area on and off. First, set a waypoint outside the geofence area and attempt to fly to that position. The position should be rejected. Then turn the geofence area off using your service `toggle_geofence`. Resend the drone a position outside the geofence area. The drone should now fly to the waypoint.

---

Congratulations, you are done with Lab 3!

---

# Final Check
1. Show the computation graph for checkpoint 1
    1. Try to fly the drone using the keyboard
        1. What happens when you hit the enter key? Answer this in terms of the FSA states we implemented.
        2. What happens when you request the drone to fly to a second position? Answer this in terms of the actual code used in `geofence_and_mission.py`.
2. Show the computation graph for checkpoint 2
    1. Try to fly the drone using the keyboard
        1. What happens when you request the drone to fly to a second position?
        2. Send a message to the drone while it is moving to a target position. What happens? Why?
3. Change the parameters in the launch file to `3,3,3` and `0.25`
    1. Show the output from running the simulation
4. Fly the drone inside your geofence area. Your drone should:
    1. Fly to waypoints inside the geofence area using commands send via the keyboard.
    2. Reject waypoints that are outside the geofence area.
    3. You should be able to explain how you implemented the verification state.
5. Turn the geofence area on or off using the `Toggle_Geofence` service.
    1. Show that after your geofence area is turned off, you can fly to a point outside the geofence area.

---
