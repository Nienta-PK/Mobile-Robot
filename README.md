# Mobile-Robot
* Here is a project in subject mobile robot. Below here is the setup for connect with the robocup-server.
* Firstly, you have to download Virtual Environment from: <https://image.raikmitl.com/index.php/s/O01qFLuAYUCySMX>
* P'Jamie video tutorial: <>
* **Password is ilc** 
# Refbox-Bridge & Refbox-server VM Setup
Set the Network like this in the Setting:
  * Check the box to 'Enable Network Adapter'
    * Attached to: 'Bridged Adapter'
    * Name: 'your wifi chip' such as MediaTek Wi-Fi 6 MT7921
  * Open advaced mode
    * Promiscuous Mode: Allow VMs
  <img width="701" height="556" alt="image" src="https://github.com/user-attachments/assets/c3a55743-fbe4-4f73-85d5-84213c125229" />
  
# IP check in Virtual Machines and your Raspberry Pi
Open the terminal (Ctrl + Alt + T) then type:
```bash
ifconfig enp0s3
```
It will show your machine network interface (such as 192.xx.xx.xxx). 

# VS-code modification of each Virtual Machine.
1. Just Follow the Video of P'Jamie explanation.
2. Try choosing the navigation first ( just to test if you can receive all of points which are send from website)

# Terminal Launch
