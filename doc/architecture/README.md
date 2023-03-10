# Radon Alcer - Line Follower <!-- omit in toc -->

- [SW Architecture](#sw-architecture)
- [Abbreviations](#abbreviations)
  - [Physical View](#physical-view)
  - [Development View](#development-view)
    - [Layers](#layers)
  - [Logical View](#logical-view)
    - [Application](#application)
      - [Startup](#startup)
      - [LineSensorsCalibration](#linesensorscalibration)
      - [Ready](#ready)
      - [Release Track](#release-track)
      - [Driving](#driving)
      - [Error](#error)
    - [Service](#service)
    - [HAL](#hal)
  - [Process View](#process-view)
- [Issues, Ideas And Bugs](#issues-ideas-and-bugs)
- [License](#license)
- [Contribution](#contribution)

# SW Architecture

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/ViewModels.plantuml)

# Abbreviations

| Abbreviation | Description |
| - | - |
| HAL | Hardware Abstraction Layer |

## Physical View

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/PhysicalView/Deployment.plantuml)

## Development View

### Layers

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/DevelopmentView/Layers.plantuml)

## Logical View

### Application

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/LogicalView/Application.plantuml)

#### Startup

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/LogicalView/StartupState.plantuml)

#### LineSensorsCalibration

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/LogicalView/LineSensorsCalibrationState.plantuml)

#### Ready

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/LogicalView/ReadyState.plantuml)

#### Release Track

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/LogicalView/ReleaseTrackState.plantuml)

#### Driving

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/LogicalView/DrivingState.plantuml)

#### Error

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/LogicalView/ErrorState.plantuml)

### Service

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/LogicalView/Service.plantuml)

### HAL
The hardware abstraction layer (HAL) for the target is shown. For the simulation and the test only the realization part is different and may base on other 3rd party components.

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/LogicalView/HAL.plantuml)

## Process View

![deployment](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/RadonAlcer/master/doc/architecture/uml/ProcessView/SystemStates2.plantuml)

# Issues, Ideas And Bugs
If you have further ideas or you found some bugs, great! Create a [issue](https://github.com/BlueAndi/RadonAlcer/issues) or if you are able and willing to fix it by yourself, clone the repository and create a pull request.

# License
The whole source code is published under the [MIT license](http://choosealicense.com/licenses/mit/).
Consider the different licenses of the used third party libraries too!

# Contribution
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, shall be licensed as above, without any
additional terms or conditions.
