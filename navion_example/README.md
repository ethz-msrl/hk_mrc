# Controller Mapping Package


This package allow to control the magnetic field and insertion/retraction with a controller. YAML files can be used to modify joystick mapping.

## Pair controller
=======
## Controller pairing
* Add new device in **System Settings/All Settings/Bluetooth/"+"**
* Connect controller via MINI USB cable
* Select "Navigation Controller" > PIN options: "Do not pair" > Next 
* Message should read: "Successfully set up new device 'Navigation Controller'"
* Remove MINI USB connection and press "PS button" on the controller. The red LED will stop blinking when connected

## Installations
Install the ROS joy package and follow the instructions: [ROS joy tutorial](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick "http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick")

## Controller mapping
#### Controller mapping tools
There are many tools that allow to check or change the mapping of the controller. A convienent tool is **jstest-gtk**. To install:

```
sudo apt-get install jstest-gtk
```

#### Controller configuration
* Check if the buttons and axes are mapped correctly
* The assignation of the buttons and axes can be changed in the configuration file: **config/ps_navi_controller.yaml**
* The launch file **launch/ps_navi_controller.launch** reads in the configuration file **config/ps_navi_controller.yaml**. Make sure the path is correct.

```
<!-- Load controller paramter file -->
<rosparam command="load" file="$(find ps_navi_controller)/config/ps_navi_controller.yaml" />
```

#### Functionalities
=======
#### Modes
* Instant mode: Changes in magnetic field and advancement are directly published to e-MNS. Activated by pressing button **O**. Default.
* Planning mode: Changes in magnetic field and advancement are only published to e-MNS when button **L3** is pressed. Activated by pressing button **X**.

#### Rotation mapping

* 0: Intrinsic: rotations about the X and Y axes within a frame ```local``` moving with the field vector
* 1: Spherical: rotations are the elevation and azimuth angle about the fix global frame
* 2: Extrinsic: rotations about the X and Y axes within the fix global frame


#### Control mapping
The following functionalities are implemented:
* axis 0: Magnetic field **B** rotation about first axis
* axis 1: Magnetic field **B** rotation about secon axis
* button up: Magnetic field magnitude |**B**| increase
* button down: Magnetic field magnitude |**B**| decrease
* button x: Switch to planning mode
* button o: Swithch to instant mode
* button L1: Advancer unit advancing
* button L2: Advancer unit retracting
* button L3: Magnetic field magnitude |**B**| set to zero

## Launches
```
roslaunch ps_navi_controller ps_navi_controller.launch
```



## Known issues

#### Controller taking over mouse

The controller can cause random mouse movements as soon as it has been paired. This problem has occured with the PS Navigator Controller.
Go to folder

```
/etc/X11/
```

Create a folder in that directory named "xorg.conf.d" (if not already there). In that newly created folder create a new file called "51-joystick.conf". The file should include the following lines

```
Section "InputClass"
        Identifier "joystick catchall"
        MatchIsJoystick "on"
        MatchDevicePath "/dev/input/event*"
        Driver "joystick"
        Option "StartKeysEnabled" "False"       #Disable mouse
        Option "StartMouseEnabled" "False"      #support
EndSection
```

You can install a software if you cannot open/edit the files as administrator

```
sudo apt install nautilus-admin
```

If necessary, the software can be reloaded with

```
sudo nautilus -q
```
More information can be found here: https://wiki.archlinux.org/index.php/Gamepad#Disable_joystick_from_controlling_mouse

Restart computer