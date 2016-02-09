# Path Follower
A library to move the robot along a path of absolute coordinates. The API is
fully compatible with C, even if the lib's core is written in C++.

## Installation
This library is designed for Raspberry Pi with Raspbian. It depends on the
RobotDriver library.

* Install RobotDriver. Further information on [RobotDriver's github repository](https://github.com/TelecomParistoc/RobotDriver).
* Then clone the repository (`git clone git@github.com:TelecomParistoc/pathFollower.git`)
* cd to the root of the repository and enter `make`
* finally, enter `sudo make install`

## Usage

Don't forget to compile (actually, link) your C/C++ app with `-lpathfollower` option.
Since you need to initialize the driver to use the library, you'll probably need
`-lrobotdriver` too.

To access the functions, include the header with :

```c
#include <pathfollower/pathfollower.h>
```

For more information, see [pathfollower.h](https://github.com/TelecomParistoc/pathFollower/blob/master/pathfollower.h).

## Example

You can find a simple example written in C in [test.c](https://github.com/TelecomParistoc/pathFollower/blob/master/test.c).

To compile it, first install the library, then use `make test`.
To test, run as super user on a Raspberry Pi : `sudo ./bin/test`
