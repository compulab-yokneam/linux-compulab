Sierra Wireless PCIe Driver
===========================
Supported Devices: EM9190 DV1/DV2/DV3

How to Build the driver
=======================
run "make" to generate the driver binaries: /controllers/mhictrl.ko 
/devices/mhiuci.ko, /devices/mhinet.ko and /devices/mhitty.ko

mhiuci.ko - (QMI-MBIM) Char Interface Driver
mhitty.ko - (DM/AT) TTY Interface driver
mhinet.ko - Network Interface Driver 

How to Install the driver
========================= 
1. run "make install"
2. reboot

How to verify driver installation
=================================
"lspci -v" should generate an output as below: 

02:00.0 Unassigned class [ff00]: Qualcomm Device 0306
	Subsystem: Qualcomm Device a01f
	Flags: bus master, fast devsel, latency 0, IRQ 123
	Memory at f7201000 (64-bit, non-prefetchable) [size=4K]
	Memory at f7200000 (64-bit, non-prefetchable) [size=4K]
	Capabilities: <access denied>
	Kernel driver in use: mhictrl
	Kernel modules: mhictrl

The Char driver exposes two device interfaces:
1. /dev/mhi_0306_00.02.00_pipe_12, MBIM interface
2. /dev/mhi_0306_00.02.00_pipe_14, QMI interface

00.02.00 is platform specfic

The TTY driver exposes two device interfaces:
1. /dev/mhitty0, DM port
2. /dev/mhitty1, AT Port

How to communicate with AT port 
==================================
run "sudo minicom -D /dev/mhitty1". 
To close it, press CTRL+A and then X

How to establish a data connection 
==================================
1. Turn on the radio
sudo mbimcli -d /dev/mhi_0306_00.02.00_pipe_12 -v -p --set-radio-state=on

2. Connect to the MB/cellular network
sudo mbim-network /dev/mhi_0306_00.02.00_pipe_12 start

3. Get IP addresses
sudo mbimcli -d /dev/mhi_0306_00.02.00_pipe_12 -v -p --query-ip-configuration

[/dev/mhichar0] IPv4 configuration available: 'address, gateway, dns, mtu'
     IP [0]: '25.168.250.74/30'
    Gateway: '25.168.250.73'
    DNS [0]: '64.71.255.254'
    DNS [1]: '64.71.255.253'
        MTU: '1460'

[/dev/mhichar0] IPv6 configuration available: 'address, gateway, dns, mtu'
     IP [0]: '2605:8d80:480:494d:687a:727d:2b6e:2bc3/64'
    Gateway: '2605:8d80:480:494d:9d20:80cb:80b8:b20e'
    DNS [0]: '2607:f798:18:10:0:640:7125:5254'
    DNS [1]: '2607:f798:18:10:0:640:7125:5253'
        MTU: '1460'

4. Set the IP addresses for the network interface
sudo ip addr add 25.168.250.74/30 dev mhi_netdev0
sudo ip addr add 2605:8d80:480:494d:687a:727d:2b6e:2bc3/64 dev mhi_netdev0

5. Set the MTU Size
sudo ifconfig mhi_netdev0 mtu 1460

6. Bring up the network interface and set the route 
sudo ip link set mhi_netdev0 up
sudo ip rout add default dev mhi_netdev0

7. Run "ifconfig" to check the status for mhi_netdev0
mhi_netdev0: flags=4291<UP,RUNNING>  mtu 1460
        inet 25.168.250.74  netmask 255.255.255.252  broadcast 0.0.0.0
        inet6 2605:8d80:480:494d:687a:727d:2b6e:2bc3  prefixlen 64  scopeid 0x0<global>
        RX packets 2  bytes 224 (224.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 44  bytes 9692 (9.6 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

8. DNS setting may have to be updated (one time only)
sudo vi /etc/systemd/resolved.conf

[Resolve]
#DNS=
FallbackDNS=8.8.8.8
#Domains=
#LLMNR=no
#MulticastDNS=no
#DNSSEC=no
#Cache=yes
#DNSStubListener=yes

Change FallbackDNS to have a value 8.8.8.8 as above. 

Restart systemd-resolved service
sudo systemctl restart systemd-resolved.service

The data connection should now be established.

How to disconnect a data connection 
==================================

1. Disonnect to the MB/cellular network
sudo mbim-network /dev/mhi_0306_00.02.00_pipe_12 stop

2. Bring down the network interface 
sudo ip link set mhi_netdev0 down

How to enable driver logging
==================================
Driver Logging is disabled by default. 

It may be useful to enable driver logging for troubleshooting.
To enable logging at compile time, the following changes are required and drivers to be recompiled.

1. /core/mhi_init.c
Change "bool debug = false;" to "bool debug = true;" to enable the log for mhictrl driver.

2. /devices/mhi_netdev.c
Update "bool debug = false;" to "bool debug = true;" to enable the log for mhinet driver.

3. /devices/mhitty.c
Update "bool debug = false;" to "bool debug = true;" to enable the log for mhitty driver.

4. /devices/mhiuci.c
Update "bool debug = false;" to "bool debug = true;" to enable the log for mhiuci driver.

How to enable driver logging at runtime
=======================================
Navigate to the directory /sys/module/mhictrl/parameters, run "sudo chmod 666 debug" and then "echo 1 > debug". 

Change mhictrl to mhinet, mhiuci or mhitty in the above directory to enable logging for mhinet, mhiuci or mhitty.

How to update FW 
==================================
sudo ./fwdwl-litearm64 -p /dev/mhi_0306_00.02.00_pipe_12 -c MBIM -d /dev/mhiqdl0 -f ./fw -m 4 -l ./fdt.log -e 1

fwdwl-litearm64 is the Firmware Download Tool and FW image and PRI are saved under directory "fw".

How to make multiple PDN connections
====================================

1. Add VLAN for individual PDP context.

sudo ip link add link mhi_netdev0 name vlan.0 type vlan id 4094

sudo ip link add link mhi_netdev0 name vlan.1 type vlan id 1

Two VLAN interfaces (vlan.0 and vlan.1) were created. To verify, enter "ifconfig -a".

2. Turn on the radio. 

sudo mbimcli -d /dev/mhi_0306_00.02.00_pipe_12 -v -p --set-radio-state=on

3. Establish the data connection. 

sudo mbimcli -d /dev/mhi_0306_00.02.00_pipe_12 -v -p --connect=apn=lteinternet.apn,ip-type=ipv4,session-id=0

After the connection is established, you can find the IP address, and MTU size. 
sudo mbimcli -d /dev/mhi_0306_00.02.00_pipe_12 -v -p --query-ip-configuration

4. Bring up the interface

sudo ip link set mhi_netdev0 up

sudo ip link set vlan.0 up

sudo ifconfig vlan.0 25.120.167.134/30 mtu 1460 up   (note: IP address and MTU size from 3)

5. Add the route for this IP address.

sudo route add -net 8.8.8.8 netmask 255.255.255.255 gw 25.120.167.134

6.  Establish another data connection.

sudo mbimcli -d /dev/mhi_0306_00.02.00_pipe_12 -v -p --connect=apn=ltestaticip.apn,ip-type=ipv4,session-id=1

7. Bring up the second interface

sudo ip link set vlan.1 up
sudo ifconfig vlan.1 72.139.242.197 mtu 1460 up (note: IP address and MTU size from 6)

8. Add the route for the second IP address.

sudo route add -net 4.2.2.2 netmask 255.255.255.255 gw 72.139.242.197

9. Run the ping test. 

ping 4.2.2.2 -c 1

ping 8.8.8.8 -c 1

You should see the responses for those ping requests.

10. Disconnect data call

sudo mbimcli -d /dev/mhi_0306_00.02.00_pipe_12 -v -p --disconnect=0
sudo mbimcli -d /dev/mhi_0306_00.02.00_pipe_12 -v -p --disconnect=1

11. Bring down the interface

sudo ip link set vlan.0 down
sudo ip link set vlan.1 down
sudo ip link set mhi_netdev0 down

12. Remove VLAN

sudo ip link del vlan.0
sudo ip link del vlan.1

How to use ADB port
==================================
run "make adb" to build the drivers and "make install" to install them, 
then reboot. A new ADB interface like /dev/mhi_0306_00.02.00_pipe_36 should be
available after reboot. 

To use ADB, run “sudo -i” and 
“nc -l 5555 > /dev/mhi_0306_00.02.00_pipe_36  < /dev/mhi_0306_00.02.00_pipe_36".

Open a new terminal, and run “adb devices”, now the ADB device should be listed
as below. 

List of devices attached
* daemon not running; starting now at tcp:5037
* daemon started successfully
emulator-5554	device
 








