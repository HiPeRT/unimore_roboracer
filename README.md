# Pure pursuit
General-purpose but racing-oriented ROS2 pure pursuit control implementation.

## Prerequisites
- `ackermann-msgs`
- `adx_msgs`
- `adx_curve`
- `libyamlcpp-dev`

## Build
The package can be built in any colcon workspace with
```
colcon build
```

## Configuration and launch
Runtime parameters can be tuned via a YAML file into the `conf/` directory.

Some example launchfiles are also provided into the `launch/` directory.

You can use `ros2 launch` normally to run a launchfile
```
ros2 launch purepursuit <launchfile>
```

## License
**Apache 2.0** - [License](https://opensource.org/licenses/Apache-2.0)

## Authors
* **Federico Gavioli** - [fgavioli](https://git.hipert.unimore.it/fgavioli) - _(Maintainer)_
* **Francesco Gatti**[fgatti](https://git.hipert.unimore.it/fgatti) - _(Original python implementation)_
* **Ayoub Raji** - [araji](https://git.hipert.unimore.it/araji) - _(Original python implementation)_