# Disclaimer

| !IMPORTANT! | This is a development branch, that is not relelased by CompuLab yet|
|---|---|


# Configuring the build

## Prerequisites
It is up to developers to prepare the host machine; it requires:

* [Setup Cross Compiler](https://github.com/compulab-yokneam/meta-bsp-imx8mp/blob/kirkstone/Documentation/toolchain.md#linaro-toolchain-how-to)

## Setup U-Boot environment

* WorkDir:
```
mkdir -p compulab-kernel/build && cd compulab-kernel
export BUILD=$(pwd)/build
```

* Set a CompuLab machine:

| Machine | Command Line |
|---|---|
|ucm-imx8m-plus|```export MACHINE=ucm-imx8m-plus```|
|som-imx8m-plus|```export MACHINE=som-imx8m-plus```|
|iot-gate-imx8plus|```export MACHINE=iot-gate-imx8plus```|

* Clone the source code:
```
git clone -b linux-compulab_v5.15.71 https://github.com/compulab-yokneam/linux-compulab.git
cd u-boot-compulab
```

## Create U-boot binary

* Apply the default CompuLab config:
```
make O=${BUILD} ${MACHINE}_defconfig compulab.config
```

* Ussue menuconfig on order to change the default CompuLab configuration:
```
make O=${BUILD} menuconfig
```

* Build the kernel
```
nice make -j`nproc` O=${BUILD}
```
