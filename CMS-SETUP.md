# Steps for CMS-V1-C-EVR1M-GigE Camera Setup

## Table of Contents

[Specifications](#specifications)

[Ubuntu OS](#ubuntu-os)

[Windows OS](#windows-os)

## Specifications

### General

* Brand Name: CMS-V1-C-EVR1M-GigE
* Manifactureer: SILIOS Technologies
* S/N: CMS19110114
* Connection Interface: Gigabyte Ethernet

### Sensor Specifications

* Array type: CMOS (Si)
* Spectral band: 400 to 1000nm
* Resolution 1280(H) x 1024(V)

### Filter Specifications

* Macro-pixel size: 3x3 bands
* Wavelength range: 550 to 830 nm typical
* Type of pixel: 8 colors (narrow bands) + 1 B&W

## Ubuntu OS

1. Download drivers and necessary software for CMS-V - GigE (Ethernet Connection) from [https://en.ids-imaging.com/download-ueye-lin64.html](https://en.ids-imaging.com/download-ueye-lin64.html) for Ubuntu 64 or 32 respectively with name "ids-software-suite-linux-version.tgz" (To access IDS website it is necessary to register first).
2. Make software executable "chmod +x name.run" and run with "./name.run" with name.run the name of the file.
3. Download or clone ueye_cam package files from github repository from [https://github.com/anqixu/ueye_cam](https://github.com/anqixu/ueye_cam) in catkin_ws/src and run catkin_make to build them.
4. Copy launch file [/launch/ueye_camera_gige.launch](/launch/ueye_camera_gige.launch) in launch folder of the ROS project.
5. Connect the camera directly to the router or to the computer.
6. Create a new network with:
    * IPv4 Method: Manual.
    * IP: 192.168.0.2 (put your preference).
    * Netmask: 255.255.255.0 (put your preference).
    * Gateway: 192.168.0.1 (put your preference).
7. Run "sudo idscameramanager" to the terminal to run the camera suite.
8. Choose the option "Daemon control" from right panel. From Network adapters, choose the one that the name start with “en” and then press the button "Start ETH daemon". When the button become green press the "Close" button.
9. Press the button "Manual ETH Configuration".
10. Check "IP" and "Subnetmask". For this case:
    * IP: 192.168.0.1 (belongs to the network as previously installed).
    * Subnetmask: 255.255.255.0 (the same as previously installed).
11. Check the message bellow and it is possible to ask you to "Upload starter firmware". Perform it and wait until it finishes.
12. The camera is ready to use when the software displays on the main screen:
    * free: yes (top).
    * avail.: yes (top).
    * Status of device 1001: The camera is configured correctly and can be opened (bottom).
13. Close the "idscameramanager" and run again with "idscameramanager" command to the terminal or find it from available applications. Check that no error is displayed and close it again.
14. The application will display a message that "The program will keep running in system tray.".
15. The camera is ready to use.

## Windows OS

1. Download drivers and necessary software for CMS-V - GigE (Ethernet Connection) from [https://en.ids-imaging.com/download-ueye-win64.html](https://en.ids-imaging.com/download-ueye-win64.html) for Windows 64 or 32 respectively (To access IDS website it is necessary to register first).
2. Install the necessary drivers running the file uEye64_49300_WHQL.exe.
3. Connect the camera directly to the router or to the computer.
4. Go to "Control Panel/Network and Internet/Network Connections" and go to the "Properties" of the connected with the camera network.
5. Select the option "Internet Network Adapter Version 4 (TCP/IPv4)" and select the "Properties".
6. Select "Use the following IP Address:" and put the information bellow:
    * IP address: 192.168.0.2 (put your preference).
    * Subnet Mask: 255.255.255.0 (put your preference).
    * Default gateway: 192.168.0.1 (put your preference).
7. Run IDS Camera Manager.
8. Choose the option "ETH network service" and check the adapter options (IP address, subnet mask).
9. Press the button "Automatic ETH Configuration" for automatic network configuration.
10. Press the button "Manual ETH Configuration" and complete the statements as bellow:
    * Persistent IP: 192.168.0.1 (belongs to the network as previously installed).
    * Subnetmask: 255.255.255.0 (the same as previously installed).
11. Check the message bellow and it is possible to ask you to "Upload starter firmware". Perform it and wait until it finishes.
12. The camera is ready to use when the software displays on the main screen:
    * free: yes (top).
    * avail.: yes (top).
    * Status of device 1001: The camera is configured correctly and can be opened (bottom).
13.	The camera is ready to use.
