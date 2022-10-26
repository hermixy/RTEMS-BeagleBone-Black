# RTEMS for BeagleBone Black

This project provides a starting point or example of how to configure an RTEMS application for a project using BeagleBone Black. This project is based on the [Rtems Kernel Image Project v2 (rki2)](https://github.com/alanc98/rki2).

This project configuration provides:

- RAM Disk setup
- Network setup ( if the target supports it )
- Shell setup and shell startup script
- Example shell commands including whetstone and dhrystone benchmarks

This project formats a 64 Megabyte RAM disk with the RTEMS file system, then mounts the disk on /ram.

## RTEMS Installation
For the installation of RTEMS you will need to use a Linux machine preferable. The first time I did this was with **Ubuntu 22.04.1 LTS**.

### Before the installation
First you'll have to add some deb-src lines under /etc/apt/sources.list. For that:
```
$ software-properties-gtk
```
then under the **"Ubuntu Software"** tab click **"Source code"**. Before building and installing RTEMS you'll have to access as root user with:
```
sudo -i
```
After this, you'll have to install some packages:
```
$ apt-get update && apt-get install -y \
		apt-utils build-essential \
		vim u-boot-tools git cmake \
		bison flex texinfo bzip2 \
		xz-utils unzip python \
		libexpat1-dev \
		python-dev zlib1g-dev libtinfo-dev
$ apt-get install python2.7-dev
$ apt-get install python3
$ apt install python-is-python3
$ apt-get install pax
$ apt-get build-dep build-essential gcc-defaults g++ gdb \
		python3-dev libpython2-dev libncurses5-dev
$ apt-get update
$ apt-get upgrade
```

### Installation 🔧
Clone this repo in:
```
$ cd /home/edison369/Desktop
```
Unzip the RTEMS Source Builder (RSB), the RTEMS sources and the RTEMS libbsd sources:
```
$ mkdir -p /home/edison369/quick-start/src
$ cd /home/edison369/quick-start/src
$ cp /home/edison369/Desktop/RTEMS-BeagleBone-Black/src/rtems.zip /home/edison369/quick-start/src/ && unzip rtems.zip
$ cp /home/edison369/Desktop/RTEMS-BeagleBone-Black/src/rtems-libbsd.zip /home/edison369/quick-start/src/ && unzip rtems-libbsd.zip
$ cp /home/edison369/Desktop/RTEMS-BeagleBone-Black/src/rsb.zip /home/edison369/quick-start/src/ && unzip rsb.zip
$ cd rsb && ./source-builder/sb-check
```
Build and install the tool suite:
```
$ cd rtems
$ ../source-builder/sb-set-builder --prefix=/home/edison369/quick-start/rtems/6 6/rtems-arm
$ cd ..
$ rm -rf rsb
```
Build the RTEMS BeagleBone Black BSP with the testsuites:
```
$ cd rtems/
$ cp /home/edison369/Desktop/Demo_Alan/docker/rtems6-arm-bbb/config.ini /home/edison369/quick-start/src/rtems/
$ ./waf configure     --prefix=/home/edison369/quick-start/rtems/6
$ ./waf
$ ./waf install
$ cd..
```
Build RTEMS BeagleBone Black rtems-libbsd build:
```
$ cd rtems-libbsd
$ ./waf configure --prefix="/home/edison369/quick-start/rtems/6"     --rtems-tools="/home/edison369/quick-start/rtems/6"     --rtems-bsps=arm/beagleboneblack     --buildset=buildset/default.ini
$ ./waf && \
$ ./waf install && \
$ cd .. && \
$ rm -rf rtems-libbsd
```
### Kernel Image sample 📦
In order to build a sample kernel image of the testsuite, provided by RTEMS:
```
$ export PATH=$HOME/quick-start/rtems/6/bin:$PATH
$ cd $HOME/quick-start/rtems/
$ arm-rtems6-objcopy -Obinary $HOME/quick-start/src/rtems/build/arm/beagleboneblack/testsuites/samples/ticker.exe -O binary app.bin
$ gzip -9 app.bin
$ mkimage -A arm -O linux -T kernel -a 0x80000000 -e 0x80000000 -n RTEMS -d app.bin.gz rtems-app.img
```
### Extras 🖥️
If the BeagleBone Black is going to be connected to the computer via USB, suposing **edison369** is the computer's username:
```
$ sudo adduser edison369 dialout
$ sudo apt install screen
```
	
In order to monitor the serial port, we first need to determine it's Linux device name:
```
$ ls -l /dev/ttyUSB* /dev/ttyACM*
```
	
To use screen and connect to the BeagleBone Black, suposing the device name is **ttyUSB0**:
```
$ sudo screen /dev/ttyUSB0 115200
```

## How to build the Kernel Image using RTEMS 📦
1. Change to the build directory.
```
$ cd build
$ cd bbb-libbsd
```

2. Double check the options in the Makefile [rtems-paths.mak](https://github.com/edison369/RTEMS-Raspberry-Pi/blob/main/build/rtems-paths.mak). You usually just need to set the path to the cross compiler and RTEMS BSP. In case you followed the installation in [RTEMS Installation](<#rtems-installation>):
```
RTEMS_TOOL_BASE ?= /home/edison369/quick-start/rtems/6
RTEMS_BSP_BASE ?= /home/edison369/quick-start/rtems/6
```

3. Run make to build the executable:
```
$ make 
```

4. Run clean to get rid of all the files created, except the kernel image:
```
$ make clean
```

### Note
You'll notice that in order to build a kernel image, you just need an executable (.exe or .elf) and with the command:
```
$ arm-rtems6-objcopy -Obinary <executable file> kernel.img 
```
You'll get a kernel image. This is what is done in [Kernel Image sample 📦](<#kernel-image-sample->) and the [MakeFile](https://github.com/edison369/RTEMS-Raspberry-Pi/blob/main/build/rpi2/Makefile)

### Commands to try 📋
When you run the application on a target, you should get the shell prompt. Try some of these commands:

List all tasks
```
# task
```

List all semaphores
```
# sema
```

Run the Dhrystone benchmark
```
# dhrystone
```

Run the Whetstone benchmark
```
# whetstone
```

Create 5 tasks that run for 10 seconds
```
# taskdemo
```

There are quite a few more commands to run, just type
```
# help
```

You can also add your own commands, and include a startup script 

Check out what the startup script is doing by typing:
```
# cat shell-init
```
