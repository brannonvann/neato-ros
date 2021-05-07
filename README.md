# neato-ros

Turn your neato vacuum into a ROS robot using a Raspberry Pi. This project contains the code and instructions to run ROS the Raspberry Pi, setup ROS, and install the ROS node to control the Neato robotic vacuum. 

The code includes the ROS Neato node that communicates to a Neato vacuum using the USB port and the [neato-driver-python](https://github.com/brannonvann/neato-driver-python). 

## Sources

Several of the tasks below were researched and found on various sites. I sourced those references. Thank you all for your contributions.

## Setup

This setup makes use of two computers; A Raspberry Pi and a computer with a UI-based Operating System such as a laptop. In my case I leveraged a Raspberry Pi 3 B+ (and a Pi 4 B on another) and a laptop running Ubuntu Desktop 20.04. The Raspberry Pi is the ROS master and interacts with the Neato vacuum via USB. 

The second computer is used to control navigation of the robot through rviz. If you choose to use a Raspbery Pi 4 4gb, you should be able to install Ubuntu Desktop 20.04 on it and skip the ROS setup on second computer all together. You could follow this guide but might have to make a few alerations to handle the differences. Also, ROS on the secondary computer would not be necessary if you don't wish to create a map and have the robot navigate to locations. You could just drive it around with a controller or add additional ROS packages (or your own) to do other things. More detail is available in the [Secondary Computer Setup](#secondary-computer-setup)

### Raspberry Pi Setup

1.  Acquire a Raspberry Pi. I used a Model 3 B+ or 4 B however I believe these instructions will apply to other models as well.

1.  Download and Install Ubuntu Server 20.04.2 LTS 64bit using Raspberry Pi Imager

1.  Update Network Configuration

    1.  Re-insert the SD card into computer and modify `network_config` file with wifi network configuration. Make sure to uncomment etho and wifi lines. Also surround the network name and password with double quote marks (").

1.  Insert SD card into Raspberry Pi and power up the Pi.

1.  Login using SSH and set the hostname. The default user and password are both `ubuntu`.

    My DHCP server assigned the ip `192.168.2.167` - Most can find this by logging into their network router's interface. Please subsitute your Raspberry Pi's IP with this one when following these instructions.

        ssh ubuntu@192.168.2.167

    1.  update password, if needed:

            sudo passwd ubuntu

    1.  Update hostname(neato1)

        I set my hostname to `neato1`. If you decide to use a different hostname, make the appropiate subsitutions when following these instructions.

            sudo hostnamectl set-hostname neato1
            sudo shutdown -r now

1.  Setup key share for password

    NOTE: ubuntu does not require the use of a .local to resolve the hostname

        ssh-copy-id -i "$HOME/.ssh/id_rsa.pub" "ubuntu@neato1"

1.  Optional: At this point, I connect to the robot using VS Code and set a unique color for the sidebar so I can tell it apart from others. This is VS Code's remote development and can be read about here: <https://code.visualstudio.com/docs/remote/remote-overview>.

    code /home/ubuntu/.vscode-server/data/Machine/settings.json

Adjust your colors as you wish. Here is an example setting file.

```json
{
  "remote.autoForwardPortsSource": "output",
  "workbench.colorCustomizations": {
    "activityBar.background": "#163972",
    "activityBar.foreground": "#fff"
  }
}
```

1.  Optional: Install and enable Network Manager - This is needed for more stable wifi

[netplan](https://netplan.io/reference/) is the tool that works with networkd or NetworkManager to help setup networks on Ubuntu 20.04.

The default service for the server install of Ubuntu is networkd. I believe it is NetworkManager for Ubuntu Desktop. Networkd seemed to be unable to reconnect after wifi drops and I wanted to install network-manager.

    sudo apt update
    sudo apt-get install network-manager -y

Update this file: If this file is not available open the file located in `/etc/netplan`

    sudo nano /etc/netplan/50-cloud-init.yaml

Add th `renderer: NetworkManager` under network like the example below:

```yaml
#...comments were above here
network:
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  version: 2
  wifis:
    wlan0:
#...More below here
```

Check your configuration:

    sudo netplan try

If successful, restart the pi. If not, adjust and "try" again.

    sudo shutdown -r now

If there are still a problem with wifi not reconnecting this might be a solution although the script provided in this link will need to be modified to use the services available in Ubuntu 20.04: <https://www.raspberrypi.org/forums/viewtopic.php?p=1077741#p1077741>

1.  Set Timezone (adjust the timezone setting for your timezone)

        sudo timedatectl set-timezone America/New_York

1.  Optional: Setup File Samba Fileshare

    As tagged, this is optional. You can use ssh to move files as needed. 

    Reference: https://discourse.ubuntu.com/t/install-and-configure-samba/13948

    1.  Update ubuntu and install samba

            sudo apt update
            sudo apt install samba -y

    1.  Configure Samba

            sudo nano /etc/samba/smb.conf

        and uncomment these lines and change them to these values

            [homes]
            comment = Home Directories
            browseable = yes

            ...

            read only = no

    1.  Restart Samba

            sudo service smbd restart

    1.  Set password for ubuntu user via Samba

            sudo smbpasswd -a ubuntu

### Install ROS

#### ROS Install

Reference: http://wiki.ros.org/Installation/Ubuntu

1.  Setup sources

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

1.  Setup Keys

        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

1.  Installation

        sudo apt update
        sudo apt install ros-noetic-ros-base -y

1.  Source ROS

        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
        source ~/.bashrc

1.  Add ROS Packages

        sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

1.  Initialize rosdep

        sudo rosdep init
        rosdep update

#### ROS Workspace Setup

Create Catkin Workspace for Robot

    source /opt/ros/noetic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc

#### Setup ROS for Neato

1.  Clone this package with the neato-driver-python submodule to the Raspberry Pi.

        cd ~/catkin_ws/src && git clone --recurse-submodules https://github.com/brannonvann/neato-ros.git

1.  Install Packages

        sudo apt install ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-urdf-tutorial ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-teleop-twist-joy ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-move-base ros-$ROS_DISTRO-dwa-local-planner -y

1.  Optional: On Raspberry Pi, update .bashrc to include the setup for neato. Make the following changes and save the file. (cmd+o, cmd+x)

        nano ~/.bashrc

    At the end of the file paste the below command aliases. This will allow you to use the command before the equal sign to execute the command after the equal sign.

        alias neatobase='roslaunch neato base.launch'
        alias neatomap='roslaunch neato map.launch'
        alias neatonav='roslaunch neato move_base.launch'
        alias neatoui='roslaunch neato gui.launch'
        alias savemap='roscd neato/maps && rosrun map_server map_saver'

    Reopen terminal or run `source ~/.bashrc`

1.  Build the package

        cd ~/carkin_ws && catkin_make

#### Setup USB Devices

Create USB Assignment rules: Get the id of the device and create a rule for it to create a symbolic link to another file so that each device is always known. This package is setup to use the "neato" rather than ttyACM0 or similar name.

Reference: https://www.raspberrypi.org/forums/viewtopic.php?t=217511

    run `lsusb` to get all the devices's IDs.

Example Output:

    Bus 001 Device 004: ID 2108:780b
    Bus 001 Device 005: ID 0424:7800 Microchip Technology, Inc. (formerly SMSC)
    Bus 001 Device 003: ID 0424:2514 Microchip Technology, Inc. (formerly SMSC) USB 2.0 Hub
    Bus 001 Device 002: ID 0424:2514 Microchip Technology, Inc. (formerly SMSC) USB 2.0 Hub
    Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub

Create a rule for each device connected used by the robot. In this case you probably only have the Neato connected and in the example above It's the first one with Vendor ID: `2108`. Place this rule and any others in the `99_robot.rules` file you will create next.

    KERNEL=="ttyACM[0-9]", ATTRS{idVendor}=="2108", ATTRS{idProduct}=="780b", SYMLINK+="neato"

Note: If you use a Logitech controller it does not need to be assigned. Also, it's idProduct changes each time so it doesn't seem possible to create a simple rule for it.

Create a new rules file and add rules.

    sudo nano /etc/udev/rules.d/99_robot.rules

Reload udev

    sudo service udev reload
    sudo service udev restart

1.  If you have a logitech controller, plug the usb dongle into the raspberry pi. I use the Logitech F710. If you do not have a logitech controller, change the `base.launch` file in the `forky/forky/launch/include` to use a different controller to drive the robot.

### Secondary computer setup

This section is optional but is needed if you want to interact with the robot using any of the visual parts like rviz, visualize the map and drive the robot around using the map. If you use a mac you might find this project useful (https://github.com/nebocleaning/mac-ros)[https://github.com/nebocleaning/mac-ros]. It allows you to use Docker to run ROS on your mac as the secondary computer. I had some trouble resolving the Raspberry Pi using the hostname so I just used an Ubuntu Laptop I had already running ROS.You may have luck using a Windows machine. If you want you have to setup ROS on your own and loosely follow these instructions. I tried it and I was able to get it to work but prefer Ubuntu.

Note: These instructions were added in retrospect and may be missing some steps. Use the referenced site if you encounter problems.

1.  I started with a laptop running Ubuntu Desktop 20.04.02.
1.  Since this is also Ubuntu follow all of the above instructions in [ROS Install](#ros-install), [ROS Workspace Setup](#ros-workspace-setup) and [Setup ROS for Neato](#setup-ros-for-neato) sections. The only real difference is that this maching will not need to do any of the USB steps from above since you are not plugging in this machine to the Neato.

1.  Add the ros master environment variable. it should look like this after the changes. The hostname should be the one for the Raspberry Pi (Can also be the ip address). It's also a good idea to set the ROS_HOSTNAME of to the machine's hostname. In many cases this is not necessary but go ahead and do it while you are here.

    export ROS_MASTER_URI=http://neato1:11311
    export ROS_HOSTNAME=THIS_MACHINE_HOSTNAME_HERE

1.  Reopen terminal or run `source ~/.bashrc`

1.  Check date on both computers. They should be close and if not then you may have some trouble. Best thing to do is use chrony to sync them according to other sources. I didn't need to do this but wanted to add this in case you see a difference.

        date

1.  On this computer you can run `roslaunch neato gui.launch` to pull up rviz and interact with the robot.

## Running the robot

### Make a Map

You can run this package in two modes, Map Making and navigation. You must first make a map if you want to use the navigation. You are going to open several terminals on the Raspberry Pi.

On the Raspberry Pi run:

    roscore

In another terminal run (neatobase):

    roslaunch neato base.launch

In another terminal run (neatomap):

    roslaunch neato map.launch

On the secondary computer run (neatoui):

    roslaunch neato gui.launch

The lidar should start spinning on the Neato and Rviz should load on the secondary computer. I found that if the lidar doesn't show on the secondary computer when rviz loads, just close it down and restart the above command from the terminal. It may take a few seconds.

If you are using a logitech controller as described above you can start driving by holding down the 'A' button and driving with the left stick or left D-pad depending on how your controller is configured (switch on top of F710). The 'A' button on the controller is called the enable_button and is configured in the teleop_logitech.launch file. The 'A' button maps to button 1 but this can be changed if you would like.

If you are not using the logitech controller and didn't change the config, the easiest thing to do is open a new terminal on either computer and run `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` and the keys indicated to drive around (i,m,j,l).

### Save Your Map

When you follow the above steps you are creating a map using gmapping so you will need to save your map after you have driven around. The map will be used to run the navigation. If you wish to also run the move_base.launch at the same time you can do SLAM.

On a new terminal on the Raspberry Pi run (savemap):

    roscd neato_nav/maps
    rosrun map_server map_saver

### Navigate

This will load the map created previously and allow you to click on the map in RVIZ and have your robot navigate to that location. Close your previous roslaunch tasks mentioned above then:

On the Raspberry Pi run (startnav):

    roslaunch neato base_nav.launch

On the secondary computer run (startui):

    roslaunch neato map_gui.launch

You can still drive around manually if you wish using the logitech controller or using the keyboard as described above.

## Edit your map

If you would like, you may edit your map using a image editing program like Gimp. Open the `map.pgm` file saved previously. Use the grey, black, and white colors from your map to edit it. Black is a solid object, white is open space, and grey is unknown space. To save using Gimp, use the "Export as" function and save in raw form.
