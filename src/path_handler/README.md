# path_handler

Anything path and trajectory.

## Path Publisher
Publishes a trajectory file (`.trj`, in TUM format) as a `adx_msgs::msg::Plan` or `nav_msgs::msg::Path` topic.

Run with
```bash
ros2 launch path_publisher path_publisher.launch.xml
```

## Path Logger
Logs a trajectory from a specified input topic. Writes to file the resulting trajectory in real-time in TUM format.

Run with
```bash
ros2 launch path_logger path_logger.launch.xml
```

## License
**Apache 2.0** - [License](https://opensource.org/licenses/Apache-2.0)

## Authors
* **Federico Gavioli** - [fgavioli](https://git.hipert.unimore.it/fgavioli) - _(Maintainer)_
* **Michele Guzzinati** - [mguzzinati](https://github.com/mguzzina)
* **Andrea Bernardi** - [andreabernard](https://github.com/andreabernard)
