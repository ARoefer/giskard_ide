# Giskard IDE

This repository contains an RVIZ panel that aims to aid the development and testing of [giskard](https://github.com/SemRoCo/giskard_core) motion controllers.

## Using the panel

![Image of the RVIZ panel](https://github.com/ARoefer/giskard_ide/blob/master/docs/img/simpanel.png "Giskard IDE Panel")

### Scenarios

![Image of the scenario tab](https://github.com/ARoefer/giskard_ide/blob/master/docs/img/scenario-tab.png "Scenario Tab")

The workspaces of the IDE are organized in *scenarios*. The scenario tab allows the user to select a name and a location for the current environment to be saved to, or load a scenario from a file. Also a URDF file can be specified, which is automatically loaded and displayed when the scenario is loaded. When the RVIZ view configuration is saved, the scenario is also automatically saved. This is done either to the specified file, or in the *~/.rviz/* folder.
The scenario tab also displays the current topics used for receiving updates about the robot state and 

### Postures

![Image of the posture tab](https://github.com/ARoefer/giskard_ide/blob/master/docs/img/posture-tab.png "Posture Tab")

Scenarios can contain postures for the robot. These are displayed in the posture tab. These postures can be used as starting point for the execution of controllers.
The tab contains options for adding the robot's current posture to the library, adding an empty posture, to set apply the selected posture to the robot, or to remove it from the library. When adding the current posture of the robot, only the joints controlled by the current controller are included.
On the right hand side of the panel, the postures are listed and can be edited manually. However this is limited [Issue #1](https://github.com/ARoefer/giskard_ide/issues/1)

### Controllers

![Image of the controller tab](https://github.com/ARoefer/giskard_ide/blob/master/docs/img/controller-tab.png "Controller Tab")

The controller tab allows the user to select a controller to be used in the scenario. The tab displays the joints being controlled by the loaded controller, as well as an assignment for it's other inputs. The assignments can currently be either constant or dynamic. The dynamic versions are currently always positions, rotations or transforms of TF-frames. More options are planned: [Issue #3](https://github.com/ARoefer/giskard_ide/issues/3)
As this tab can currently not make any changes to the controller, the save button is without function.

### Scenes

![Image of the scene tab](https://github.com/ARoefer/giskard_ide/blob/master/docs/img/scene-tab.png "Scene Tab")

The *scene* of a scenario contains additional objects that can be used for input assignments. These objects are published as interactive markers. An object always has a name, a parent frame, a visual component and transformation to it's parent frame. These can be edited by selecting the object either in the RVIZ viewport, or in the list on the left side of the panel and then adjusting the values on the right side accordingly.


### Running a Controller

![Image of the simulation controls](https://github.com/ARoefer/giskard_ide/blob/master/docs/img/simcontrols.png "Simulation Controls")

Once the controller is loaded and the inputs are specified, the execution of the controller can be started by clicking the *Play* button at the bottom of the IDE's panel. The execution can be paused by clicking that button a second time. To reset the robot to it's original posture, the *Reset* button can be clicked. The posture is being specified by the combo box labeled *Initial Posture*.

### Reload Behavior

The panel automatically reloads a controller when the file is changed on disk. If the previous controller was being executed, the robot is reset to the initial posture and the execution continues. 

# About The Code

The panel is structured as a model-view system. All UI components transmit their updates to the `IScenario` interface. The implementation of this interface contains all the application's logic. The state of the application is saved in the `SScenarioContext` class.