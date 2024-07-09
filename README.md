# Kernel Build Manual

## Prerequisites
It is up to developers to prepare the host machine; it requires:

* [Setup Cross Compiler](https://github.com/compulab-yokneam/meta-bsp-imx8mp/blob/kirkstone/Documentation/toolchain.md#linaro-toolchain-how-to)

## CompuLab Linux Kernel setup

* WorkDir:
```
mkdir -p compulab-kernel/build && cd compulab-kernel
```

* Set a CompuLab machine:

| Machine | Command Line |
|---|---|
|ucm-imx8m-plus|```export MACHINE=compulab_v8```|
|som-imx8m-plus|```export MACHINE=compulab_v8```|
|iot-gate-imx8plus|```export MACHINE=compulab_v8```|
|iotdin-imx8p|```export MACHINE=compulab_v8```|
|ucm-imx93|```export MACHINE=ucm-imx93```|
|mcm-imx93|```export MACHINE=mcm-imx93```|

* Clone the source code:
```
git clone -b linux-compulab_v6.1.55 https://github.com/compulab-yokneam/linux-compulab.git
cd linux-compulab
```

## Compile the Kernel

* Apply the default CompuLab config:
```
make ${MACHINE}_defconfig compulab.config
```

* Ussue menuconfig on order to change the default CompuLab configuration:
```
make menuconfig
```

* Build the kernel
```
nice make -j`nproc`
```

* Create a bzip2 compressed tarball
 ```
nice make -j`nproc` cpl-tarbz2-pkg 
```

* Pre Install procedure

|Build Type|Procedure/Command|
|---|---|
|On target|``ln -sf $(linux-compulab-*.tar.bz2 \| tail -1) /tmp/linux-compulab.tar.bz2``|
|On another device|Copy linux-compulab-\<version\>-arm64.tar.bz2 to the target devive /tmp/linux-compulab.tar.bz2|

## Deploy the created image
* Issue CompuLab linux install script:
```
bash <(curl -L https://raw.githubusercontent.com/compulab-yokneam/Documentation/master/etc/cl-kernel-install.sh) /tmp/linux-compulab.tar.bz2
```
* Reboot the device.
* When device is up, issue:
<pre>
uname -r
</pre>
Make sure that the reported version matches the created kernel version.
