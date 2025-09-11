# Mobile-Robot
* Here is a project in subject mobile robot. Below here is the setup for connect with the robocup-server.
* Firstly, you have to download Virtual Environment from: <https://image.raikmitl.com/index.php/s/O01qFLuAYUCySMX>
* P'Jamie video tutorial: <https://youtu.be/iICBLDQte9I>
* **Password is 'ilc'** 
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

# Terminal Launch in ILC-Server
1. rcll-get-started
 Open the folder in terminal 
 <img width="888" height="583" alt="image" src="https://github.com/user-attachments/assets/455ea0e3-eec6-460a-b0ae-f362ecc10d25" />

 1.1 Source the file
 ```bash
 . setup.sh
 ```
 1.2 Start the server
 ```bash
 rc_start
 ```
 * Stop the server (use only when you want to stop the server)
 ```bash
 rc_stop
 ```
 * If you want to change the robot task in terminal, you will run this:
 ```bash
 nano local_setup.sh
 ```
 Then change comment or uncomment the task line.
2. rcll-relay-server
 2.1 Open the folder in terminal (using 'cd' or can use the same method as rcll-get-started)
 2.2 Run the Docker:
 ```bash
 docker compose up
 ```
# Terminal Launch in ILC-Bridge
1. Open the terminal of folder 'thopen-refbox-bridge' then run:
```bash
 docker compose up
```
2. Open Firefox and Open the website 'your-server-ip:8080' such as 192.xx.xx.xxx:8080
3. Reload Website: Ctrl + O
   <img width="1477" height="1108" alt="image" src="https://github.com/user-attachments/assets/33b35158-c504-488d-bd47-a431e63eb935" />


# MQTT Explorer Installation (Download in your window is ok since it is the network connection so it can communicate)
* Using for receive the point for navigation task
* Download-link: https://mqtt-explorer.com/
* Add the connection:
  <img width="1257" height="891" alt="image" src="https://github.com/user-attachments/assets/747e8ed0-7489-4442-bf82-28a3a31beecb" />

* How it should be
  <img width="1477" height="1108" alt="image" src="https://github.com/user-attachments/assets/3eb84c9c-d3c5-417e-bc4a-0c6c64711d6d" />
