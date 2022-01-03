# Hookup 4G/LTE Dongle

## Guide

An example using D-Link DWM-222.

Fist, plugging into usb port on the dongle. And then reboot the host.
If the driver is set, you can see Mobile Setting on Network Panel.

<div> 
    <img width="300" src="./assets/LTE_dongle.png">
    <img width="300" src="./assets/network_panel-0.png">
    <img width="300" src="./assets/network_panel-1.png">
    <img width="300" src="./assets/network_panel-2.png">
    <img width="300" src="./assets/network_panel-3.png">
    <img width="300" src="./assets/network_panel-4.png">
    <img width="300" src="./assets/network_panel-5.png">
    <img width="300" src="./assets/network_panel-6.png">
    <img width="300" src="./assets/network_panel-7.png">
    <img width="300" src="./assets/network_panel-8.png">
</div>

If the driver is not set. It need to install "usb-modeswitch" then reboot.

    $ sudo apt-get update -y
    $ sudo apt-get install -y usb-modeswitch
    $ sudo modprobe qmi_wwan
    $ sudo modprobe option
    $ reboot

