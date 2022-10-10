# hk_msmrc
ROS packes used to interface with the Navion installed in the Multi-Scale Medical Roboics Center in Hongkong

## Launch files

### navion2.launch

Interfaces with Navion without GUI. The control of the field is performed with a PS4 controller.

## RViz plugins

#### Usage

The user can use the following controls from the panel:

* Go grom READY to RUN state and start pushing currents: click on "Start" (calls the service `navion/start_running` with req. `True`)
* Go from RUN and READY state and stop pushing currents: click on "Stop" (calls the service `navion/start_running` with req. `False`)
* Recover from an error state: click on "Acknolwedge error" (calls the service `navion/reset_system`)
* Change the logging modulus: set the modulus in the spin box and click "Set log mod." (calls the service `navion/set_logging_modulus` with the provided modulus)

#### Topics

##### Subscribed

System report

* **`navion/system_report`** ([nav_controller/systemreport_navion2])

##### Service calls

Start and stop the system

* **`navion/start_running`** ([std_srvs::SetBool])

Ackowledge fault state to reset system

* **`navion/reset_system`** ([std_srvs::Trigger])

Set the modulus for the log

* **`navion/set_logging_modulus`** ([nav_controller::msgSetLogMod])
