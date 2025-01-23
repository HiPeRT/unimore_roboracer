# adx_msgs
Set of custom ROS messages for the autonomous driving domain.

## Available messages
| Name | Description | 
| ------ | ----------- |
|  `ArbitratedAckermannDrive`  | A wrapper of the `AckermannDriveStamped` message and control multiplexing logic. |
| `LapTime` | Represents a completed lap. Contains the timestamp and the number of the completed laps. |
| `Obstacle`    | Represents a generic 3D cylindric obstacle. The `Obstacle` is defined by a pose and size. |
| `ObstacleArray` | Array of `Obstacle` messages. |
| `PlanPoint` | Represents the desired state of a vehicle. The `PlanPoint` is defined by a 3D position and speed. |
| `Plan` | Represents the plan of a vehicle. The `Plan` is implemented by a vector of `PlanPoint` messages. |
| `SectorStamped` | Stamped sector identifier. |
| `Track` | Represents a tracked obstacle. Contains its id, length, width and pose. |
| `TrackArray` | Array of `Track` messages.|

## License
**Apache 2.0** - [License](https://opensource.org/licenses/Apache-2.0)

## Authors
* **Federico Gavioli** - [fgavioli](https://github.com/fgavioli) - _(Maintainer)_
* **Michele Guzzinati** - [mguzzina](https://github.com/mguzzina)
* **Andrea Bernardi** - [andreabernard](https://github.com/andreabernard)
* **Antonio Russo** - [russoanto](https://github.com/russoanto)

## Project Managers
* **Paolo Burgio** - [pburgio](https://github.com/pburgio)

## AD Project
This repository is part of the autonomus driving project of the University of Modena and Reggio Emilia, [read more](https://hipert.github.io/ad_site/).