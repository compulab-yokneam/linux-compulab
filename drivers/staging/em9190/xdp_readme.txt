XDP = eXpress Data Path
https://en.wikipedia.org/wiki/Express_Data_Path

The XDP technology avoids the overhead of using Socket Buffers (SKBs) in kernel network driver by using Extended Berkley Packet Filters (eBPF). The driver exposes a data receiving callback for an ebpf program which can process packets.
Steps on how to test XDP using the mhinet network driver. 
1.	Build network driver for XDP support: ‘make xdp”. Install the new driver.
2.	Establish a data connection, bring up the network interface and assign IP address, gateway. Please refer the readme.txt for details. 
3.	Compile an ebpf program, for example, xdp_dummy.o.
4.	Run “sudo ip link set dev mhi_netdev0 xdpdrv object xdp_dummy.o” to load the ebpf program
5.	Generate some data traffic and check the results
6.	After completing the test, run “sudo ip link set dev mhi_netdev0 xdpdrv off” to remove the ebpf program.

Here is very simple program to drop all the incoming packets.

// SPDX-License-Identifier: GPL-2.0

#define KBUILD_MODNAME "xdp_dummy"
#include <uapi/linux/bpf.h>
#include "bpf_helpers.h"

SEC("prog")
int xdp_dummy(struct xdp_md *ctx)
{
    return XDP_DROP;
}

char _license[] SEC("license") = "GPL";

More useful samples can be found here. 

https://github.com/torvalds/linux/tree/master/samples/bpf

https://github.com/netoptimizer/prototype-kernel/tree/master/kernel/samples/bpf
