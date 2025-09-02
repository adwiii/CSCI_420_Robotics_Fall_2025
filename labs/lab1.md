---
title: Lab 1
subtitle: Setting up the development environment and basic ROS
layout: page
---

# The Robotic Operating System

In this class, we are going to be using the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Introduction)).
ROS is a sophisticated and open source framework of software libraries, simulators, and development tools that help us to quickly and effectively build robot systems.
Why are we learning ROS? Well, ROS has become the de-facto standard for the robotics community and is one of the enablers of the current robotic revolution, so what we learn in this class (a small portion of ROS) will be valuable regardless of what direction your future takes you.
Another reason to use ROS is that you will join an active community that constantly invents new robots (in the hundreds), provides new packages to cover the latest emerging algorithms and paradigms, and give us tons of documentation, answers, and support (thousands of contributors!). Take a look at the wide range of robots that use ROS below (or the [whole list](https://robots.ros.org)):

<div class="columns is-centered">
    <div class="column is-3">
        <figure class="image is-square">
            <img src="../images/lab1/robot_a.jpg">
        </figure>
        <p class='has-text-centered'>Prosthetic Robot</p>
    </div>
    <div class="column is-3">
        <figure class="image is-square">
            <img src="../images/lab1/robot_b.jpg">
        </figure>
        <p class='has-text-centered'>Ground Robot</p>
    </div>
    <div class="column is-3">
        <figure class="image is-square">
            <img src="../images/lab1/robot_c.jpg">
        </figure>
        <p class='has-text-centered'>Aerial Robot</p>
    </div>
    <div class="column is-3">
        <figure class="image is-square">
            <img src="../images/lab1/robot_d.png">
        </figure>
        <p class='has-text-centered'>Underwater Robot</p>
    </div>
</div>
---- 

# Learning Objectives
This lab is structured into two sections. The first shows you how to set up the Docker container that we will be using for all labs in this course (more info about Docker below). This section aims to teach:
* What Docker is and why we use it.
* How to install a Docker container that includes ROS.
* How to run basic ROS commands in the Docker container.

The second part of the lab aims at teaching you the basics of building ROS-based systems and tooling. More specifically it aims to teach you how to:
* Compile ROS code.
* Launch ROS files.
* Use ROS tools.

---- 

# Getting a Machine with ROS
## Why Docker
ROS is mainly developed on the Linux-Ubuntu operating system, and to build the labs for this course you will need to have access to an Ubuntu environment. If you were to work day-to-day in ROS, you would likely have a whole desktop work machine that only runs Ubuntu (and several of the TAs do!), but you may be using another operating system like macOS or Windows and not have the ability to change your system operating system easily.
Docker allows us to solve this problem by creating an isolated Ubuntu environment that we can run *from inside another operating system*. Although we will use Docker to create and configure an Ubuntu environment, Docker can be used for many different operating systems and configurations.  
More specifically, Docker allows us to create [`images`](https://docs.docker.com/glossary/#image), a blueprint that tells Docker exactly how we need the Ubuntu system configured. This is especially useful in this case, because the teaching staff can guarantee that each student's environment is exactly the same; more generally, any time you need to make sure you have a repeatable, consistent environment, Docker can help. Docker then lets us use these images to build [`containers`](https://docs.docker.com/glossary/#container) that we can run, starting the Ubuntu system.
You can think of this as a small virtual machine running inside of your machine. For more information about Docker, refer to their documentation [here](https://docs.docker.com/get-started/), though we will explain the necessary commands you need for the labs as we go.

Thus, to get started you first need to install Docker and set up the provided Docker image that will allow you to start developing in ROS. The set-up process is outlined below:

## Installing Docker:
Docker itself is a program that bridges the gap between the container and your machine.
We will start by installing Docker for your machine. See the below section corresponding to your machine's operating system.
### Linux
Follow the instructions [here](https://docs.docker.com/desktop/install/linux-install/).

### Mac
Follow the instructions [here](https://docs.docker.com/desktop/install/mac-install/). Note that if you are using one of the new M1 or M2 macs, you will need to select the appropriate option `Mac with Apple chip`.

### Windows
Follow the instructions [here](https://docs.docker.com/desktop/install/windows-install/).
Windows has the `Windows Subsystem for Linux` which Docker can use to speed up and more easily run containers. We strongly recommend using WSL2 if your machine supports it (see requirements in the `WSL 2 backend` tab); otherwise, you will need to use the `Hyper-V backend`.

## Verifying your Docker Installation
Once you have Docker installed, you will need to ensure that it is started. You can configure Docker Desktop to start when your computer starts for convenience. However, if you do not want it to run all the time, you will need to remember to start it each time you want to run Docker.

Once Docker has started, verify that it is working correctly by running this small Hello World demo. Open a terminal and run:

```bash
docker run hello-world
```

You should see an output similar to this:

```
Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (amd64)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/
```

## Setup the Lab Docker Image
Open a new terminal on your host machine and enter the following commands to download the Docker image instructions (the [`Dockerfile`](https://docs.docker.com/glossary/#dockerfile)):
```bash
cd ~  # You will need to access this repository every time we work on a lab, so choose a convenient place
git clone https://github.com/adwiii/csci-420-robotics-docker.git
```

Next, clone the class repository and build the container:
```bash
cd ~/csci-420-robotics-docker  # edit the location if you did not clone the repository to ~ during Docker Setup
git clone https://github.com/adwiii/csci_420_robotics_labs.git
docker compose up --build -d
```

The `docker compose up --build -d` command looks in the [`docker compose.yml`](https://github.com/adwiii/csci-420-robotics-docker/blob/main/docker_compose.yml)
file that specifies both how to build the image using the `Dockerfile` and how to run the image to get a working container.
You will generally not need to edit this file, other than the one time specified later in this lab.
You will run the `docker compose up --build -d` command each time you want to start the container to work on labs.

{% include notification.html message="The first time you run this command, it will take a long time to complete, possibly 15-20 minutes or more. Future runs will be much shorter."
status="is-success"
icon="fas fa-exclamation-triangle" %}

During this initial set up, Docker is building the container from the image we provided in the Docker file.
While doing so, it caches the result at each step so that even if you were to edit part of the Dockerfile later, it would not redo parts that had not changed.
We will not need to edit the Docker file during the semester, so after the first run, every other run should take only a few seconds to start.

If successful, the output should look like this:
```
...
 ✔ csci_420_robotics_docker-ros                Built
 ✔ Network csci_420_robotics_docker_x11        Created
 ✔ Container csci_420_robotics_docker-novnc-1  Started
 ✔ Container csci_420_robotics_docker-ros-1    Started
```

This is now running in the background on your computer managed by Docker. You can view the status in Docker Desktop, where it should look like:

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image">
        <img src="../images/lab1/docker_desktop.png">
        </figure>
    </div>
</div>

It is safe to run this command multiple times - running `docker compose up --build -d` while the container is already
running will output the following and not affect the running container.
```
 ✔ csci_420_robotics_docker-ros                Built
 ✔ Container csci_420_robotics_docker-novnc-1  Running
 ✔ Container csci_420_robotics_docker-ros-1    Running
```

{% include notification.html message="Fun fact, this docker environment was adapted from work done by a previous student who took this class! If you find, develop, or discover something interesting we would love to know!"
status="is-success"
icon="fas fa-exclamation-triangle" %}

### Stopping the Docker container
Once you run the above `docker compose up --build -d` command, the Docker container will be running in the background
and we will explore later how to interact with the container. However, if you need to stop the container for any reason
you can do so through Docker Desktop or run:

```bash
cd ~/csci-420-robotics-docker  # edit the location if you did not clone the repository to ~ during Docker Setup
docker compose stop
```

Which will stop the containers and display:

```
[+] Stopping 2/2
 ✔ Container csci_420_robotics_docker-ros-1    Stopped
 ✔ Container csci_420_robotics_docker-novnc-1  Stopped
```

## Accessing a Terminal
You can also use `docker compose` to access a terminal for the container directly from your host machine. In a terminal on your host machine (Terminal 1) enter the following commands to connect to your Docker container and open a terminal:
```bash
cd ~/csci-420-robotics-docker  # edit the location if you did not clone the repository to ~ during Docker Setup
docker compose exec ros bash
```

This will open a terminal inside the Docker container. This is an Ubuntu terminal with ROS installed which we will use for the labs. It will open to a blank terminal line:

```bash
root@ros_docker:~#
```

## Test your Docker Container:
To test that the Docker container and ROS are working properly, we will try to start ROS.
ROS is a type of middle-ware. It isn't a full operating system, but similar to Docker how Docker helped your machine run Ubuntu, ROS helps facilitate how your ROS programs run and communicate.

To do this, enter the following command into Terminal 1:

```bash
ros2 node list -a
```

The terminal should output something similar to the following:

```bash
root@ros_docker:~/csci_420_robotics_labs/lab1_ws# ros2 node list -a
/_ros2cli_daemon_0_a1e0d230bf104e4097e2a23f8a4bfce9
```

---- 
# Checkpoint 1 [Self-Check]

For this checkpoint, you should be able to do the following:
1. Launch the Docker container
2. Run `ros2 node list -a` in the terminal

{% include notification.html
message="Extra: Here is a bit more on the [history](https://www.ros.org/history/) of ROS, and a short interesting [video](https://www.youtube.com/watch?v=Dm7HnQb8n9Y) on who uses ROS."
icon="false"
status="is-success" %}
---- 

# Our First Code in ROS

Robots are complex systems, made of many software and electro-mechanical components, all interacting as the robot senses and acts on the world.
ROS helps us to manage that complexity through abstractions, infrastructure, and tools.
To start getting a sense of what ROS can do for us and how it operates, in this lab, we will build, run, and inspect some simple ROS code to control a simple rocket operating in simulation.

## Rocket Ship

Let's assume we have been given the software used to fly a rocket to the moon. The rocket's software is written using ROS. Our job is to use the ROS tools to compile, launch, and then monitor this rocket on its journey to the moon.

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-5by4">
        <img src="../images/lab1/rocket1.png">
        </figure>
    </div>
</div>

## Downloading the Code
Throughout the semester the teaching staff will be adding the labs to the [lab GitHub repository](https://github.com/adwiii/csci_420_robotics_labs) during the week the lab is assigned.
Before each lab, you need to make sure the lab code is up-to-date. Within Docker, enter the following:

```bash
# Change to labs directory. 
# When you are working in the Docker container, the lab directory will always be ~/csci_420_robotics_labs
cd ~/csci_420_robotics_labs
# Pull the latest code
git pull
```

## Structuring and Building ROS Code

To start, we first need to compile the rocket's ROS code. ROS code is organized into workspaces.
A ROS workspace is a folder where you modify, build, and install ROS code.
Workspaces are one way to address the complexity in the development of large systems
by providing a standard structure where developers and tools know where things are located.
ROS even has community [conventions](https://docs.ros.org/en/kilted/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html) on such structures.

Let's now build the code in the workspace. 
In Terminal 1, run the following command:

```bash
# Change the directory to the ROS workspace
cd ~/csci_420_robotics_labs/lab1_ws/
# Build the code in this directory
colcon build
# Update your environment variables so that ROS knows where your workspace is
source install/setup.bash
```

If you get no errors, you have successfully built your rocket's ROS code.  **When you change your code and want to run it, you should build it first.** If you are in the same terminal, you can simply run `colcon build` again.

Every time you run `colcon build`, ROS generates a new set of files to run. After each build, you must run `source install/setup.bash`; this preps your terminal so that ROS knows where to find all of the files. You must also run this command every time you open a new terminal to run ROS.


Next, let's run our rocket software.
The rocket software consists of three software components. Let's start each component by opening  **three new terminals**, then enter the following:

### Terminal 1:

```bash
cd ~/csci-420-robotics-docker  # edit the location if you did not clone the repository to ~ during Docker Setup
# Connect to the Docker container
docker compose exec ros bash
# cd into the directory for the lab
cd ~/csci_420_robotics_labs/lab1_ws
# Update your environment variables so that ROS knows where your workspace is
source install/setup.bash
# Run the rocket-ship source code
ros2 run rocketship rocket_engine.py
```

The `rocket_engine` software  controlling the power to the rocket engine is now running.  You will see no further messages printed in this terminal.

### Terminal 2:

```bash
cd ~/csci-420-robotics-docker  # edit the location if you did not clone the repository to ~ during Docker Setup
# Connect to the Docker container
docker compose exec ros bash
# cd into the directory for the lab
cd ~/csci_420_robotics_labs/lab1_ws
# Update your environment variables so that ROS knows where your workspace is
source install/setup.bash
# Run the rocket-ship source code
ros2 run rocketship main_controller
```

The `main_controller` which commands the `rocket_engine` to go at certain velocity is now running. The main controller should respond with the following message:

```bash
>>> [INFO] [...]: Rocket Ready for Countdown
```

Now let's get ready to launch the rocket! Start the countdown using Terminal 4.

### Terminal 3:

```bash
cd ~/csci-420-robotics-docker  # edit the location if you did not clone the repository to ~ during Docker Setup
# Connect to the Docker container
docker compose exec ros bash
# cd into the directory for the lab
cd ~/csci_420_robotics_labs/lab1_ws
# Update your environment variables so that ROS knows where your workspace is
source install/setup.bash
# Run the rocket-ship source code
ros2 run rocketship countdown
```

The `countdown` component gives the command to the `main_controller` to initiate the launch sequence.



If everything went correctly in **Terminal 2**, you should see a countdown (after 5 seconds) and subsequent velocity commands as shown below:

```bash
>>> [ INFO] [...]: Countdown: 10s
>>> [ INFO] [...]: Countdown: 9s
>>> ...
>>> [ INFO] [...]: Requested Velocity: 0.076450m/s
>>> [ INFO] [...]: Requested Velocity: 0.077994m/s
>>> ...
```

That's awesome! We just launched the rocketship using ROS code!

You can stop execution of your code using the key-combination `CTRL-C` in each of the terminals.

But let's dive a bit deeper so that we start to understand what is going on behind the scenes. To launch this rocket, we needed three terminals to launch the rocket_engine, the main_controller software, and the countdown software.
Using this information, we can start to understand some of the system organization as shown below:

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-16by9">
        <img src="../images/lab1/rocket2.png">
        </figure>
    </div>
</div>

## Roslaunch

Having to run many different software components is common in robotics, but doing it manually by setting terminals as we have done is not practical, especially when deploying a system requires hundreds of processes to be run in specific ways.  Another way that ROS helps to address system complexity is by making it easier to deploy multiple software components through **launch files**.  Another benefit of a ROS launch file is that it automatically checks if a `roscore` is running, and if not, it starts one.

### Editing files
Let's create a launch file to run all the software components of our rocket. First, we will edit the base `rocket.launch` file provided to you. To do this, open the `csci_420_robotics_labs/lab1_ws/src` directory in your preferred IDE. Docker conveniently allows your Ubuntu machine and your host machine to both access the files directly. This means that you can either edit the files directly in Ubuntu, or we recommend using a full-fledged IDE such as [Visual Studio Code](https://code.visualstudio.com/) or [PyCharm](https://www.jetbrains.com/pycharm/) on your host machine. For PyCharm, you can use the community edition, or students are eligible for a [free Educational License](https://www.jetbrains.com/community/education/#students) that provides access to the full commercial suite for free.

Next, navigate to the `rocketship/launch/` directory and open the `rocket.launch` file. You will find an empty launch file with xml code as shown below:

```xml
<?xml version="1.0"?>
<launch>
   <!-- Add your code here -->
</launch>
```

Let's add each of the different software components into the launch file. ROS calls each software component a node. Add the following lines between `<launch>`and `</launch>`.

```xml
<node pkg="rocketship" exec="rocket_engine.py" name="RocketEngineNode"></node>
<node pkg="rocketship" exec="main_controller" name="MainControllerNode" output="screen"></node>
<node pkg="rocketship" exec="countdown" name="CountDownNode"></node>
```

Take a second to explore what is inside the launch file. By using a launch file, ROS knows to start `roscore` automatically and how to start three software modules (nodes) `rocket_engine`, `main_controller`, and `countdown`. Each of the nodes belongs to the rocketship package (pkg). We want the `main_controller` to output to screen (the terminal). Save the file and go back to your terminal. Let's launch the rocket. Close all terminals and open a new one. To launch the rocket we need to run:

```bash
cd ~/csci-420-robotics-docker  # edit the location if you did not clone the repository to ~ during Docker Setup
# Connect to the Docker container
docker compose exec ros bash
# cd into the directory for the lab
cd ~/csci_420_robotics_labs/lab1_ws
# build the workspace
colcon build
# Update your environment variables so that ROS knows where your workspace is
source install/setup.bash
# Run the countdown code
ros2 launch rocketship rocket.launch
```

This time you should see the rocket state that it is ready for launch. The countdown should automatically begin, and the rocket should launch!

## Using ROS Tools

Next, let's use some of the built-in ROS tools to get a better understanding of how our rocketship works. Let's start by finding what kind of communication is occurring between the software modules in this rocketship.

Leave the rocketship running and open another terminal. In the new terminal, run the following command. (Note we will discuss ROS nodes and topics in more detail in the next lecture)

```bash
cd ~/csci-420-robotics-docker  # edit the location if you did not clone the repository to ~ during Docker Setup
# Connect to the Docker container
docker compose exec ros bash
# List all the communication channels (ROS topics)
ros2 topic list
```

You will see the following output:
```bash
/cmd_vel
/countdown
/launch_abort
#...
```

This shows us the names of all the created communication channels (`topics`) that exist between processes.

The next thing we might want to do is to display the data (`messages`) being sent on any one of these communication channels. Lets, for instance, display the rockets commanded velocity `/cmd_vel`:

```bash
# Display the data on the ROS communication channel (ROS messages)
ros2 topic echo /cmd_vel
```
This will display something similar to the following:
```
data: 1684.0
---
data: 1684.0
...
```

Notice how these numbers match the numbers being printed by the rocketship software:
```
[ INFO] [...]: Requested Velocity: 1684.000000m/s
[ INFO] [...]: Requested Velocity: 1684.000000m/s
```

To stop the stream of messages, press `CTRL-C` in the terminal.

## Using Visual Applications
While the Docker container is running, there is a full-fledged Ubuntu machine running in the background on your machine that so far we have only accessed from the terminal.
To access the desktop, follow this link [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) and click "Connect".
This will connect you to a very simple Ubuntu desktop interface.
We have configured the container to use [VNC](https://en.wikipedia.org/wiki/Virtual_Network_Computing), but instead of using the Internet to connect to a machine, this connects to your Ubuntu container directly.
Any actions you perform in the terminal that open a window will do so in this browser window.

The initial desktop will look like the figure below. Note that the desktop is very small; we have set the desktop size to 800 by 600 in the `docker compose.yml` file. If we set it to something like 1920 by 1080, a common screen resolution, the Ubuntu desktop would have scroll bars because inside the browser is not the full available space of your screen. You can adjust your local `docker compose.yml` to a resolution that works for you. Adjust the following as needed; on my 1920 by 1080 display, I found that 1920 by 900 worked well, but this varies by monitor and window size. Be cognizant that if you have scroll bars, you may not be able to see all of a window, and you may need to scroll to see the taskbar at the bottom.

```yml
  novnc:  
    image: theasp/novnc:latest
    environment:
      - DISPLAY_WIDTH=800  #EDIT 
      - DISPLAY_HEIGHT=600 #EDIT 
```


<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-16by9">
        <img src="../images/lab1/vnc_initial.png">
        </figure>
    </div>
</div>

{% include notification.html message="Additional word of caution. This desktop manager has [Workspaces](https://wiki.archlinux.org/title/fluxbox#Workspaces), a way of grouping different sets of windows together. Note that when you first open the browser it says `Workspace 1` in the lower left corner. *If you scroll the mouse wheel while over the desktop, it will scroll through the workspaces*. Any windows you have open will no longer be visible, as they are on the other workspace. If you find yourself missing a window you expect to be visible, check the taskbar and examine which workspace you are viewing." %}



## RQT Graph

Our rocketship code is reasonably simple, and keeping track of what is going on in our heads is manageable.
However, when you get to work on larger systems, it sometimes helps to **visualize the software modules and their communication** altogether. One way to do that is to use `rqt_graph`. `rqt_graph` provides a GUI plugin for visualizing the ROS computation graph made of topics and nodes.

While your rocketship is running, run the following command in a separate terminal:

```bash
rqt
```

You will be presented with a blank window in your VNC browser (if not open, click [here](http://localhost:8080/vnc.html) and connect). In the window, click `Plugins->Introspection->Node Graph`. It will now show graph with information about how the ROS processes communicate. Note that you may need to zoom in to see it which can be done by scrolling on the mouse wheel.
Change the first dropdown to `Nodes/Topics (all)` and uncheck the **hide deadsinks** and **hide leaf topics** as shown in the figure below, then click the refresh button (the two blue chasing arrows to the left):

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-4by1">
        <img src="../images/lab1/rqt1.png">
        </figure>
    </div>
</div>

This gives us a much clearer picture of the software we are running.
The software modules are the nodes and they are represented as rounded bubbles.
The communication channels are the topics and they are represented as rectangles.


----

# Checkpoint 2

Take a screenshot of the ROS communication graph.

1. How many software modules (nodes) are there?
2. How many communication channels (topics) are there?

----

## RQT Plot

Previously we displayed data on the terminal using `rostopic echo`. For identifying data patterns or trends, sometimes if helps to visualize the data through plots. To do this, we will use the `rqt_plot` tool that is part of `rqt`.

With the rocket running, start `rqt` in a separate Docker container and check your VNC browser:

```bash
rqt
```

To open `rqt_plot` click `Plugins->Visualization->Plot`. You will see output similar to the figure on the left. In the topic edit-box, type in `/cmd_vel/data` and hit the green + button. You will see a blue line, as shown in the right-hand figure. Use the magnifying glass or chart settings to set the scale correctly and the arrows to locate the data being plotted. As shown in the figure below, after a while the rocket reaches a constant cruising speed.

<div class="columns is-centered">
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/plot1.png">
        </figure>
        <p class='has-text-centered' markdown="1">After running the `rqt_plot` command, a GUI similar to this will appear on your screen.</p>
    </div>
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/plot2.png">
        </figure>
        <p class='has-text-centered' markdown="1">Enter the `/cmd_vel/data` topic and hit the green + button. You can use the **magnifying glass to zoom in and out** (right-click and drag to zoom out).</p>
    </div>
</div>

----
# Checkpoint 3

You now have all you need to launch the rocket and plot the data passed between the different software components. Now plot both the command velocity (`/cmd_vel/data`) and the velocity sensor data (type in `/sensor_vel/data` and hit the + button again) using `rqt_plot`. Relaunch the rocket's software so that you can see the velocity from the countdown until it reaches maximum velocity. **Take a screenshot of your graph**.

<div class="columns is-centered">
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/plot3.gif">
        </figure>
        <p class='has-text-centered' markdown="1">The graph is continually updated and should   move as the rocket launches. **This is what you should see.**</p>
    </div>
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/plot4.png">
        </figure>
        <p class='has-text-centered' markdown="1">Take a screenshot, **including the entire GUI.** The graph should look similar to this. **This is what your screenshot should look like.**</p>
    </div>
</div>

Answer the following question using the plot:

1. Why do the sensor readings (red line) not match the command velocity (blue line) exactly? (zoom-in as needed to detect discrepancies)

---- 
# RQT Publisher

The final tool we will inspect in this lab is `rqt_publisher`. `rqt_publisher` provides a GUI plugin for **sending arbitrary ROS messages to a channel** to any node that may be listening to that channel.
We will use this tool to control the abort function that is already implemented in our rocketship. You would have noticed (in the listed topics or in the rqt graph) that one of the communication channels (topics) the rocketship accepts is called `/launch_abort`, but it is not connected to any node so the only way to abort is to directly send a message to the topic.
Let's use `rqt_publisher` to abort one of our launches. While your rocketship is running, open RQT and click `Plugins->Topics->Message Publisher`. You will see a GUI appear, similar to the one below:

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-4by3">
        <img src="../images/lab1/graph1.png">
        </figure>
    </div>
</div>

We were interested in the communication channel (topic) `/launch_abort`.
We are going to send (publish) messages on that communication channel (topic) and try to **abort** the rocketship from launching. To do that, do the following (note the checked checkbox next to the topic in the list):

<div class="columns is-centered">
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/graph2.png">
        </figure>
        <p class='has-text-centered' markdown="1">First: Select `/launch_abort` from the dropdown. Then change the frequency to 5. **Then click the green + button.**</p>
    </div>
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/graph3.png">
        </figure>
        <p class='has-text-centered' markdown="1">Second: Open the message using the side arrows. Then select the checkmark. Selecting the checkmark sends messages.</p>
    </div>
</div>

You will notice that **nothing happens.** Let's investigate why. Open a new Docker container and display the content (ROS messages) of the communication channel (topic) using the following code:

```bash
cd ~/csci-420-robotics-docker  # edit the location if you did not clone the repository to ~ during Docker Setup
# Connect to the Docker container
docker compose exec ros bash
rostopic echo /launch_abort
>>> data: False
>>> ---
>>> data: False
>>> ...
```

This explains why it did nothing! We were sending `/launch_abort` message set to **false**. So we are **telling the robot not to abort!** To send true abort messages, check the checkbox next to the word `False`, or double-click on the word `False` and type `True`. Note that you might have to click the `>` triangle to the left to make that option visible. Once you have done that, you should see the following appear on your rocketship terminal:

```bash
>>> [ INFO] [...]: Abort Failed
>>> [ INFO] [...]: Requested Velocity: 1684.000000m/s
>>> [ INFO] [...]: Requested Velocity: 1684.000000m/s
>>> [ INFO] [...]: Abort Failed
>>> [ INFO] [...]: Requested Velocity: 1684.000000m/s
```

The abort fails because the rocketship is already flying and so you can't abort the launch.

---- 

# Checkpoint 4:

Rerun the abort test before the rocket takes off. **Take a screenshot of the terminal, during an abort**.

1. What does the rocket print to the terminal when you abort before launch?
2. After the successful abort, try and echo the command velocity (`/cmd_vel`). What message do you receive, and does this make sense?

----

# Final Check:

At the end of this lab, you should have the following:

1. [Self-Check] Have installed Docker and the necessary container
    1. Launch the Docker container
    2. Run `ros2 node list -a` in the terminal
2. A screenshot of the ROS communication graph. Use this graph to answer:
    1. How many software modules (nodes) are there?
    2. How many communication channels (topics) are there?
3. A screenshot of the velocity and sensor readings. Using the knowledge learned in class answer:
    1. Why do the sensor and velocity readings not match?
4. A screenshot of the rocketship software during an abort during the countdown. Using the screenshot to answer:
    1. What is printed to screen when you abort?
    2. What is printed to screen when you try and echo /cmd_vel? Does this make sense?

{% include notification.html
message="Extra: Here is more information on understanding [nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes). Here is more information on understanding [topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)."
icon="false"
status="is-success" %}