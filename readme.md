# Robot Kinematics


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

A supported by ROS operating system (and ROS itself) is necessary to build and run the application.

### Installing


## Running the Al5D package without the Al5D

To see what the Al5D would receive if you connect it, which is really convenient for testing purposes,
you can setup a virtual serial port. There are a few commands which you have to execute and they rely on socat.
If you haven't installed it yet, you can install it by executing: `sudo apt-get install socat` in a terminal.

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

The output should look something like this (ports could be different):

```
2020/02/17 12:19:24 socat[40447] N PTY is /dev/pts/4
2020/02/17 12:19:24 socat[40447] N PTY is /dev/pts/5
2020/02/17 12:19:24 socat[40447] N starting data transfer loop with FDs [5,5] and [7,7]
```

Next, open a terminal and execute: `cat < /dev/pts/4`. The "4" in this example doesn't necessarily have to be a 4. Look at the output of
socat after the first command. In this terminal you will see the output of the Al5D package.

The other port will be your serial port which you have to specify in the config.ini in the config folder. There should be
an entry in the config file which says `serial_port=`. Change the port accordingly.

Now you are ready to start the Al5D program. Navigate to your ROS workspace and execute: `source devel/setup.bash`.
At the moment we have set up a remote ROS master, if you want to make use of it, you should execute:
`export ROS_MASTER_URI=http://142.93.224.106:11311`. Otherwise, start a local ROS master.

After that, execute `rosrun al5d al5d` and you're good to go.

## Built With

* [ROS](http://wiki.ros.org/) - Framework for writing robot software

## Authors

* **Derk Wiegerinck** - [derk1998](https://github.com/derk1998)
* **Rene van Eendenburg** - [RcvanEendenburg](https://github.com/RcvanEendenburg)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.