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

## Application Overview

### Text User Interface (TUI)
The TUI consists of different "menus". To select a menu, you have to type the name of the menu in the console. These menus will be displayed when entering the TUI so no worry about forgetting menu names. After selecting the menu, you will be greeted with a welcome message. Typically, this consists of what commands are supported. At the moment, two types of commands can be entered:

* The single operation: this consists of one word and has no arguments. This will always end up in the same result, since preferred behaviour cannot be specified in the TUI.
* The operation with argument: as the name suggests, this operation supports arguments. At the moment, the argument can consist up to two words, but this can programmatically be extended to the preferred amount of words.

After entering a menu, the TUI will start one thread for handling user input and the main thread will perform scheduled actions. Small non-blocking actions can also be performed in the separate user input thread, but in order to create a pleasant user experience, the actions have to be kept short. 

Not sure how to implement a new mode correctly? See the Programmers Guide on how to create modes.

## Programmers Guide

### Keywords and why it is better to use it
The last thing we want to end up with is having thousands of strings defined to handle user input or handle a config file. Things can go wrong quickly like defining the same command multiple times but slightly different ("exit" or "Exit" for instance). This is where Keywords come in to place. It consists of a mapping between the Keyword enum and a defined string. 

In the TUI for instance, we want to be able to parse user input and once we have a matching string, perform specific behavior. The Keyword enum is implemented in Mode and has two helper functions.

If you want to implement the Keyword mechanism in another component, here are the helper functions you have to define:

```cpp
const std::string& MyClass::keywordToString(Keyword keyword)
{
    auto result = std::find_if(keywordTable.begin(), keywordTable.end(),
                               [keyword](const auto& keywordBinding){return keywordBinding.first == keyword;});
    if(result != keywordTable.end())
        return result->second;
    throw std::runtime_error("Couldn't find the string to keyword");
}

MyClass::Keyword MyClass::stringToKeyword(const std::string& str)
{
    auto result = std::find_if(keywordTable.begin(), keywordTable.end(),
                               [str](const auto& keywordBinding){return keywordBinding.second == str;});
    if(result != keywordTable.end())
        return result->first;
    throw std::runtime_error("Couldn't find the keyword");
}
```

Also remember to create an ```enum class Keyword``` inside ```MyClass```!

### Adding new TUI modes
The class ```Mode``` will start two threads as explained before. These will be set up when ```Mode::start``` is called. Generally speaking, the behavior you want to achieve is by calling your modes start function, the user will be greeted with a welcome message and when he leaves the mode, he will see an exit message.

This behavior is achieved by setting the welcomeMessage and exitMessage in ```Mode``` by calling ```Mode::setWelcomeMessage``` and ```Mode::setExitMessage``` respectively. Since, the scope of a mode can differ per implementation of ```currentMode``` in the class ```Application```, it is safe to set these messages when the start function is called. This means that you have to override the start method, setting your messages and then call ```Mode::start```.

Since the exit of a mode is pure defined by commands which are handled in the mode, there is no exit function in the interface of ```Mode```.

A really basic custom mode will look like this:

```cpp
#include <tui/Mode.h>

class MyMode : public Mode
{
public:
  MyMode() = default;
  ~MyMode() override = default;
  virtual void start() override;
};

void MyMode::start()
{
  setWelcomeMessage("Welcome to my mode!");
  setExitMessage("Exiting my mode...");
  Mode::start();
}

```

Next, make sure you alter the ```Application::run``` method to make the custom mode accessible to users. The run method can look like this:

```cpp
void Application::run()
{
  std::string operation;
  while(true)
  {
    if(!currentMode || !currentMode->isStarted())
    {
      std::cout << "Type 'mymode' to access MyMode." << std::endl;
      std::cout << "'exit' will exit the application." << std::endl;
      std::cin >> operation;
      if (operation == "mymode")
      {
        currentMode = std::move(std::make_unique<MyMode>());
        currentMode->start();
      }
      else if (operation == "exit")
        break;
    }
  }
}

```

Now the only thing left to do is creating commands. A good place to create the commands is the constructor of your mode. Please keep the class ```Mode``` clean! A few examples:

```cpp
// Single operation, action created with std::bind
addSingleOperation(keywordToString(Keyword::Quit), std::bind(&Mode::exitAction, this));

// Single operation, action created with lambda
addSingleOperation(keywordToString(Keyword::Quit), [this](){return exitAction();});

// Operation with argument, action created with std::bind
using namespace std::placeholders
addOperationWithArgument(keywordToString(Keyword::MyCommand), std::bind(&MyMode::handleMyCommand, this, _1));

// Operation with argument, action created with std::bind
using namespace std::placeholders
addOperationWithArgument(keywordToString(Keyword::MyCommand), [this](const std::string& arg){return handleMyCommand(arg);});
```

Remember, as previously stated it is a good tactic to use Keywords here.


## Built With

* [ROS](http://wiki.ros.org/) - Framework for writing robot software

## Authors

* **Derk Wiegerinck** - [derk1998](https://github.com/derk1998)
* **Rene van Eendenburg** - [RcvanEendenburg](https://github.com/RcvanEendenburg)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.