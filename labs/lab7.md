---
title: Lab 7
subtitle: Transformations
layout: page
show_sidebar: true
---

# Why do we need transformations?
Robots employ many different coordinate systems (aka frames) to define their location, the location of their sensors and components, or the location of external entities.  Transformations among those frames is necessary for the robot or a robot component to understand how to interpret data that is provided in a different frame.  You may not have realized it, but we have already used many  frames and implicit transformations in our previous labs. For example, we use the world frame when sensing the drone position, attitude, and velocity, to plan a path and implement the control commands to realize that path. Frames and transformations are everywhere, so let's learn how to programmatically structure them in ROS.

# Learning Objectives
At the end of this lab, you should understand:
* When and why we use transforms
* How to  publish a transform in ROS
* How to use a ROS transform listener
* How to use a ROS transform to perform a coordinate transformation
* How to use a dynamic transform in ROS

# Before Starting
Be sure to pull the latest code

# Scenario Overview

In previous labs, we specified the drone's goal position using the command:

```bash
ros2 topic pub /uav/input/goal geometry_msgs/Vector3 '{x: -2, y: -1, z: 2.5}'
```

This command sends the drone to a position that we defined in our world frame. The drone navigated based on the assumption that it was operating in that world frame. Now, let's see what would happen if that assumption does not hold.

For this lab, we will assume you have been hired by an online shopping company that has recently shown interest in delivering packages using drones. You are the lead engineer of the drone delivery system.

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image">
        <img src="../images/lab7/house_drone.png">
        </figure>
    </div>
</div>

**World Frame.** In your first tests, you place the drone at the origin of the world frame (as illustrated in the figure above) and try to deliver packages to different houses. You start by delivering a package to the house on the bottom left of the figure (the one with the package). You find that all you need to do is send a Vector3 message to `/uav/input/goal`  containing the coordinates `[-4,-2]`.


**Tower Frame.**  As you expand your operation, you realize that you need a tower to control your family of drones. You place the tower at the top of a mountain to have longer communication reach. The tower, however, has a frame that is +Y aligned with the magnetic North. The tower then directs the drone to deliver a package to the location corresponding to the same house as before. You send the drone to that location when all of a sudden the drone flights to the right  downwards and crashes into the house just below it.  What happened? When the tower gave the drone the position to go to, it told the drone where to go *in its own frame of reference*. When you use poses that are not in the correct frame of reference, it can have unexpected and dangerous consequences.


<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image">
        <img src="../images/lab7/house_tower.png">
        </figure>
    </div>
</div>

In this lab, a tower node is provided. The tower will publish goal positions (once a drone's path is complete) on the `/tower/goal` topic, using the tower frame. *Your job will be to use ROS transforms to convert the tower's commands into the world frame so that the drone can successfully navigate to the goals positions sent to the drone.*

# ROS Support for Frames and Transformations
ROS supports transformation between frames through the `tf2` package, which provides very [rich functionality](http://wiki.ros.org/tf2), well beyond what is covered in this lab.

ROS also provides several standardized frames. For example, the ENU (East, North, Up) frame which is commonly used in ground robots has its positive x-axis pointing to the vehicle's left, positive y pointing north, and the positive z axis pointing up.  **The frame we will be using in this lab is NED (North, East, Down), which is commonly used in aerial vehicles and has its x pointing north, positive y-axis pointing east, and positive z axis pointing down.**

<div class="columns is-centered">
    <div class="column is-centered is-6">
        <figure class="image">
        <img src="../images/lab7/enu_ned.png">
        </figure>
    </div>
</div>

For the first part of this lab, we will be working with a particular kind of transform that does not change over time, known as a *static transform*. Our transform will aim to transform from the tower frame to the world frame. In contrast, a dynamic transform is one which changes as time goes by. For example, we would need a dynamic transform if our tower was attached to the delivery van; each time the van turned a corner the transform from the vans frame to the drone frame would also need to change. We will explore using a dynamic transform later in the lab.

# Exploring the Coordinate Frames in our Simulator
Let's start by exploring what frames are already available in our simulator. A quick way to do that is using the `view_frames` tool which will depict the system of coordinates used by a robot. Start two terminals and run the following commands:

Terminal 1:

```bash
cd ~/csci_420_robotics_labs/lab7_ws/
colcon build
source ~/csci_420_robotics_labs/lab7_ws/install/setup.bash
ros2 launch flightcontroller fly.launch
```

Terminal 2:
```bash
source ~/csci_420_robotics_labs/lab7_ws/install/setup.bash
cd ~/csci_420_robotics_labs/lab7_ws/
ros2 run tf2_tools view_frames
```


The `view_frames.py` script provided from `tf2_tools` produces a PDF giving information about the current transformations available. In the `~/csci_420_robotics_labs/lab7_ws/` directory you should have a file `frames_<timestamp>.pdf` with a content similar to the image below:

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image">
        <img src="../images/lab7/sample_coords_diagram.png">
        </figure>
    </div>
</div>


The picture above shows us that we have a small tree of transforms, rooted in the frame `world`  with two child frames. Note that in ROS, each frame has a unique `string` identifier.

The first child frame is the `world/ned` frame. As discussed above, drones often use a NED coordinate system, and thus this transform is taking the world frame, which is in ENU, and converting it to NED. This would be useful for, e.g., drones using this coordinate system to fly. This system of coordinates is always fixed.

The second child frame is `uav_imu` which corresponds to the body frame of the drone with its origin at the center of the robot.  Thus, the body frame moves with the drone and it is valid only for a small time interval. For example, if we were to use the drone's camera to determine the location of the an object relative to the drone, the camera would operate in the `uav_imu` frame.

**The important part to notice is that ROS provides a way to define frames, to access them, and to relate them**  (the labels associated with each edge will be explained as we introduce more functionality of the `tf2` package.)

# Defining a transformation in ROS

To support transformations of geometric data, ROS provides the message class `geometry_msgs/TransformStamped`:
<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image">
        <img src="../images/lab7/geom_msg.png">
        </figure>
    </div>
</div>

Using this message, a transformation is a function that transforms coordinates from the frame specified in `header.frame_id` to the coordinates specified in the `child_frame_id`.

Going back to our tree of frames in the `view_frames` output, the left-edge has `header.frame_id` set to `'world'`, and `child_frame_id` to `'world/ned'`.

The `transform` attribute is defined as a message of type `geometry_msgs/Transform`, which consists of  the mathematical transformation between two 3D Cartesian frames  translation followed by a rotation.

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image">
        <img src="../images/lab7/transform_msg.png">
        </figure>
    </div>
</div>

The translation component is similar to a transformation in geometry and adds `translation.x`, `translation.y` and `translation.z` to the parents frames x, y and z coordinates.

A rotation can be specified either using   Euler angles (roll, pitch, and yaw) or a quaternion (a vector in x,y,z, and a rotation w).


{% include notification.html
message="Side note: a problem with Euler angles is that the order in which rotations are applied can lead to different transformations. Thus the use of quaternions is generally preferred. Quaternions also provide many other benefits such as avoiding gimbal lock,  which actually occurred during the Apollo missions. (Check out [this video](https://www.youtube.com/watch?v=OmCzZ-D8Wdk) how a gimbal lock was narrowly avoided on the Apollo 13 mission.)."
icon="false"
status="is-success" %}


# Defining our own Transformation
In this lab, we are aiming to transform the tower reported positions into the corresponding positions in the world frame that our drone understands. The diagram below gives specifics about the location of the tower relative to the origin of our world.

{% include notification.html
message="Important: `tf2` can transform coordinates in both directions across an edge or a sequence of edges in the transform tree. Here, we will calculate the transform from the `world` frame to the `tower` frame. Then, later on we will ask `tf2` to translate points from the `tower` frame into the `world` frame. It will internally calculate the inverse transform for us and perform the transformation."
icon="false"
status="is-primary" %}

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image">
        <img src="../images/lab7/tower_to_drone.png">
        </figure>
    </div>
</div>

**Now, using trigonometry (Transformation lecture -- just for 2D in this case because we are ignoring altitude), installop the transformation function to do just that.**


Remember that the transform, similar to a `geometry_msgs/Transform` message, must include a translation and a rotation.  Use Euler angles for the rotation.  Since we are using the NED convention (x points north, y points east, and z points down), the rotation matrix must be adjusted to:
<div class="columns is-centered">
    <div class="column is-centered is-2">
        <figure class="image">
        <img src="../images/lab7/rotmat.png">
        </figure>
    </div>
</div>
A positive yaw angle is one that moves from the +X axis to the +Y axis. In the case of NED frames this means that positive is clockwise.

# Self-Checkpoint
To test that you have derived the correct transformation, manually transform these inputs using your formulas and check that you get the following output. Do not move on with the lab until your transformation is correct.

* World [0.0, 10.0] -> Tower [-133.6, 85.9]
* World [10.0, 10.0] -> Tower [-124.9, 90.9]
* World [-10.0, 10.0] -> Tower [-142.2, 80.9]
* World [10.0, -10.0] -> Tower [-114.9, 73.6]
* World [-100.0, 100.0] -> Tower [-265.2, 113.9]
* World [-120.0, 80.0] -> Tower [-272.5, 86.5]
* World [-200.0, 100.0] -> Tower [-351.8, 63.9]




Note: These answers have been rounded to the first decimal.

# Broadcasting a Transformation
Let's now use that transformation as part of our system. The architecture of the system we will installop for Checkpoint #1 is shown below. The blue nodes are similar to those in Lab 7. The green node is the new `Tower`  node that is given to you, and the red nodes are what we will need to implement.

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image">
        <img src="../images/lab7/system_diagram.png">
        </figure>
    </div>
</div>

The package `tf2` provides a broadcasting mechanism through which it sets an internal node to send out time stamped transformations to anyone that is listening. The idea is that all components of the system can access and use the transforms provided by other components, and continually broadcasting them makes sense if they are likely to change over time or if they are consumed at different times.

A node that needs to retrieve transformations from the system creates a transform listener. When the node needs to apply a transformation from frame A to frame B, it uses the listener to obtain that transform from the system and then applies it. Observe that a transform listener can be used to retrieve multiple transforms, while a typical ROS subscriber may only receive messages on one topic.

**Broadcasting setup**. As mentioned earlier, in this lab we will initially use a static broadcaster since the transformation from *tower* frame to *world* frame is time-independent. Broadcasting static transforms is so common, that  `tf2` provides a built-in `static_transform_publisher` node that we can add directly to our `fly.launch` file.

```xml
<node pkg="tf2_ros" exec="static_transform_publisher" name="tower_broadcaster" args="TODO"/>
```

`static_transform_publisher` takes as arguments the three components of the translation, the three angles of rotation, the parent frame id, and the child frame id as follows.
```python
 x y z yaw pitch roll frame_id child_frame_id
```


Our job now is to replace the **TODO** in the arguments in the launch file line above with your computed transform. You **must** have the `frame_id` be the `world` frame and the `chile_frame_id` be the `tower` frame


A few things to keep in mind:
* In our context: z, pitch, and roll should be set to zero since we are operating in just 2D.
* `static_transform_publisher` only requires the rotation angle between the frames over x and y (rotation between x and y is the yaw) from which it computes the necessary trigonometry for us. The rotation angle must be specified in radians (not degrees) and again since our frames are in NED,  clockwise is considered a positive rotation.
* We are converting from the `tower` frame to the `world` frame
* Precision is important. While above we rounded to 2 decimal places, consider using full decimal places in the launch file. **2 decimal places is not sufficient**.

Once you are done, ROS offers capabilities to inspect the values sent through the system. To view the transformation you just defined, use the following command **after you start the simulator** and confirm that the translation and rotation match what you expect from your calculations above:

```bash
ros2 run tf2_ros tf2_echo world tower
>>> Translation: [..., ..., ...]
>>> Rotation: in Quaternion [..., ..., ..., ...]
            in RPY (radian) [..., ..., ...]
            in RPY (degree) [..., ..., ...]
```

Now, when we use it in our system, we will ask for the transformation in the other direction. To view this, run:

```bash
ros2 run tf2_ros tf2_echo tower world
>>> Translation: [..., ..., ...]
>>> Rotation: in Quaternion [..., ..., ..., ...]
            in RPY (radian) [..., ..., ...]
            in RPY (degree) [..., ..., ...]
```

# Checkpoint 0
1. How do the transforms in the different directions relate?

# Listening and Transforming
Using `static_transform_publisher` we just set up a node to broadcast the transform from the `tower` frame to the `world` frame. Our drone needs to listen for this transform so that it can transform the goals that it receives in the `tower` frame to the `world` frame.

## Listening Setup
To listen to the broadcasted transforms, we use a `TransformListener`. This can be created as follows:

```python
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# ...
   # in the constructor
   self.tfBuffer = Buffer()
   self.listener = TransformListener(self.tfBuffer, self)
```

Since nodes in ROS run in their own process and their execution order is not enforced, it is possible that the transformation you need is not available when requested, so it is a good practice to enclose the lookup call for the transform in a try block as follows. We will use the  code below in our main loop to always pull the most recent transform. Although this is a static transform and thus we do not expect it to change, asking for the transform every time through the loop allows us to not have to wait for the static transform to become available, and also re-use the same code constructs when we employ dynamic transforms.

```python
  try:
     # lookup_transform arguments are target_frame, source_frame, and time
     t = self.tf_buffer.lookup_transform('world', 'tower', rclpy.time.Time())
  except TransformException as ex:
     self.get_logger().info(f'Could not transform world to tower: {ex}')
     return
```

## Applying the Transform
The above code gave us the variable `transform`. One of the functions provided from `tf2` allows us to apply transforms directly to `Geometry PointStamped` messages to move across frames as shown below:

```python
from tf2_geometry_msgs import do_transform_point
#...
new_point = do_transform_point(point, transform)
```

Where `point` is point in coordinates you want to transform, and `transform` is the transform we just looked up and intend to apply to that point.

## Using the Goals set by the Tower Frame in the Drone Planner
Let's  create a node `tower_to_map.py` that subscribes to `/tower/goal` (which is in the `tower` frame) and publishes a goal for our drone on the topic `/uav/input/goal` (in the `world` frame). The tower waits for about 10 seconds before it broadcasts the first goal. The node will use the transform declared by the `static_transform_publisher` to transform the goal across the frames. To do this, complete the following steps:

**Step 1:** Skeleton code has already been provided in the `simple_control` folder. Open that now.

**Step 2:** Add the node to `fly.launch`.

**Step 3:** Implement the TODOs using what we just learned about trasnforms.


After you have implemented this node, the tower node will publish a new goal after your drone has reached its current goal. Thus, your drone should now successfully navigate the map getting new goals each time it reaches its current goal.

# Checkpoint 1 (Worth 50% of a lab)
1. Explain how you calculated the values for `args` for the `tower_broadcaster` node
2. Run the simulator and show that your drone is capable of reaching 3 different goals sent by the tower.

<div class="columns is-centered">
    <div class="column is-centered is-10">
        <figure class="image">
        <img src="../images/lab7/cp1.gif">
        </figure>
    </div>
</div>

# Bonus Checkpoints, total 30% extra credit on Lab 7
A **PID Controller** is an algorithm that uses the error between a setpoint and a measurement to adjust the system’s output with the goal of maintaining the set point. A PID relies on three terms (P,I,D) to control the system output to minimize the error. PIDs’ broad applicability and popularity in robotics makes them worth their own lab.

<div class="columns is-centered">
    <div class="column is-centered is-8">
    <figure class="image">
        <a href="https://youtu.be/v27xYKdZUzI">
          <img src="../images/lab7/video_titlescreen.png">
        </a>
        <a href="https://youtu.be/v27xYKdZUzI"> OMEGA Engineering: What is a PID Controller and how does it work?</a>
      </figure>
    </div>
</div>

We will be implementing a simple drone that operates in two dimensions and aims to move to a set of waypoints. We will use a PID controller to help the drone move between these waypoints, with one PID controller for each dimension.
The PID controller tells the drone what *velocity* to move in each direction. Even though the error term for the PID is in units of distance, i.e., how far am I from the waypoint in this dimension, the output controls the velocity. This is a common occurrence in robot controls, where the output you control is different from the output you are measuring. Think about driving a car: the car has an accelerator and a brake that each control positive or negative acceleration which we then use to change our position.

In the environment, there is a strong cross-wind in the Y direction, but not in the X direction. We will explore how this impacts the tuning of our PID controller.

For these checkpoints, you will need to implement the basic PID algorithm in the class `velocity_pid.py`. You will then test the PID using `debug_pid.py` as explained in the checkpoints below. ***You may not modify `debug_pid.py`***.

## Bonus Controls Mini Checkpoint 1, 10% extra credit on Lab 7
In the first checkpoint, we will work on the PID controller for the X dimension. Once you have implemented the PID controller in `velocity_pid.py`, you will run tests in the form:

```bash
python debug_pid.py [px, ix, dx]
```

Where `px`, `ix`, and `dx` are the P, I, and D values for the X dimension. When you launch the debugger, a window will appear in the VNC browser that shows the drone trying to navigate from the starting position at the origin, to the first waypoint at `x=5, y=0`.

There is noise in the system, so the drone will not always respond exactly to the command, and will move around slightly even when not commanded. For example, the below image shows the output of `python debug_pid.py [0,0,0]`. With all 0 parameters, the PID controller provides no input and the drone is blown around by the noise without reaching the waypoint.

 <div class="columns is-centered">
    <div class="column is-centered is-6">
        <figure class="image">
        <img src="../images/lab7/pid_x.png">
        </figure>
    </div>
</div>

1. Tune the PID controller so that the drone reliably reaches the waypoint in less than 30 steps. What is the impact of changing the different parameters? Do you need all of the parameters?


## Bonus Controls Mini Checkpoint 2, 20% extra credit on Lab 7
Once the drone reaches the first waypoint, it the program will start trying to send it to the next waypoint, `x=5, y=5`. However, with only a controller for the X dimension, the drone will not be able to reach it. To enable the PID controller for the Y dimension, invoke the debugger as:

```bash
python debug_pid.py "[[px, ix, dx], [py, iy, dy]]"
```

Where the array now contains separate parameters for the X dimension and the Y dimension. The image below shows the result of running `python debug_pid.py "[[0,0,0],[0,0,0]]"`. As noted, there is a strong cross-wind in the Y dimension that constantly blows the drone off course. Here, without any control in the Y dimension, the drone is quickly blown off screen.

 <div class="columns is-centered">
    <div class="column is-centered is-6">
        <figure class="image">
        <img src="../images/lab7/pid_y.png">
        </figure>
    </div>
</div>

2. Tune both sets of PID parameters so that the drone can reach each of the waypoints. How do the parameters for Y differ from X? You may consider adjusting the PID implementation to include more advanced features like [integral reset](https://en.wikipedia.org/wiki/Integral_windup). Below is an example of a PID controller being used to complete all of the waypoints:

 <div class="columns is-centered">
    <div class="column is-centered is-6">
        <figure class="image">
        <img src="../images/lab7/pid_xy_complete.png">
        </figure>
    </div>
</div>

---

Congratulations, you are done with lab 7!

---

# Final Check
0. How do the transforms in the different directions relate?

1. Verify that your drone can visit the points given by the tower
    1. Explain how you calculated the values for `args` for transform
    2. Run the simulator and show that your drone is capable of reaching 3 different goals sent by the tower.

Bonus:
1. Show the PID working solely in the X component to reach the first waypoint.
2. Show the PID working in the X and Y components to navigate to each waypoint. 