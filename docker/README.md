# Robot Kinematics and Docker

## Motivation

Not everyone wants to set up ROS on his own machine, or the operating system is simply not supported. Docker can be used to overcome these and many more problems.

## But what about graphical applications like Rviz?

There are multiple solutions to running graphical applications inside a Docker container. In this repository, two solutions are implemented. A VNC server can be used to show the graphical applications. This has been tested with Rviz, and it works really well. However, it is always a bit slower than running applications native. Therefore another technique can be used: using the X11 server from the host machine. It is tested on Debian testing, and there have been no problems so far. It runs just like any other application.

## Okay sounds good, but what do I need?

You need to set up both Docker and Docker-compose. It's not that hard, and many tutorials explain how to install these programs, which can be found on the internet. After you have done that, you need to make sure that you don't have to use ```sudo``` every time. Besides that, to execute the Docker commands, a little Makefile is created to make life a little bit easier. So you need to install ```make``` as well.

## Let's get started

### Compile everything

To compile everything, simply navigate to this directory (after cloning, of course) and type ```make build_app```. You will notice after everything worked out, that the folders ```build``` and ```devel``` are made. Now you are ready to go to the next step!

### Creating the No VNC image

If you are planning to use the VNC server, you have to make this image as well. Make it with ```make build_novnc```.

### Building the Robot Kinematics image

The only thing you have to do to build the image is running ```make build_image```. This will create it for you.

### Run everything

To run the Robot High-Level Interface, AL5D controller, Serial Forwarder, Simulator, World and with the VNC server, run ```make run_backend_cross_platform```. After it is up and running, you can go to https://localhost:9090 to see the simulator. To run it as an X11 application, make sure you have setup ```xhost``` correctly. Open a terminal and type: ```xhost +```. Now run the command ```make run_backend_linux```. 

To run the TUI, simply run ```make run_frontend```. That's it! It may be useful to see the backend log, which can be accessed through running ```make see_log```.

### Notes

If you are lazy and don't want to type that much for building everything, you can just type: ```make build```. To stop the backend, type ```make kill_backend_cross_platform``` or ```make kill_backend_linux```.