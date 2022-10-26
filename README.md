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
then under the "Ubuntu Software" tab click "Source code".
After this, you'll have to install some packages:
```
$ sudo apt-get update
$ sudo apt install wget
$ sudo apt-get install python2.7-dev
$ sudo apt-get install git
$ sudo apt-get install python3
$ sudo apt-get build-dep build-essential gcc-defaults g++ gdb unzip \
            pax bison flex texinfo python3-dev libpython2-dev libncurses5-dev \
            zlib1g-dev
$ sudo apt install python-is-python3
$ sudo apt-get update
$ sudo apt-get upgrade
```

### Installation 🔧
#### Metodo 1
Obtain the RTEMS Source Builder (RSB), and the the RTEMS sources:
```
$ mkdir -p $HOME/quick-start/src
$ cd $HOME/quick-start/src
$ git clone git://git.rtems.org/rtems-source-builder.git rsb
$ git clone git://git.rtems.org/rtems.git
```
Offline download of all the sources to build the BeagleBone Black BSP:
```
$ cd $HOME/quick-start/src/rsb/rtems
$ ../source-builder/sb-set-builder --source-only-download 6/rtems-arm
```
BSP stack build (Tool suite, BSP and Packages):
```
$ cd $HOME/quick-start/src/rsb/rtems  
$ ../source-builder/sb-set-builder --prefix=$HOME/quick-start/rtems/5 \
    --with-rtems-tests=yes bsps/erc32
```


##### Metodo 2
```
$ cd
$ mkdir -p development/rtems/releases
$ cd development/rtems/releases
$ git clone git://git.rtems.org/rtems-source-builder.git rtems-source-builder-6
$ cd rtems-source-builder-6/rtems 
$ sudo ../source-builder/sb-set-builder \
      --prefix=$HOME/development/rtems/6 6/rtems-arm
$ cd ..
$ ./source-builder/sb-check  
$ ./source-builder/sb-set-builder --list-bsets 
$ cd $HOME/development/rtems
$ mkdir kernel
$ cd kernel
$ git clone git://git.rtems.org/rtems.git
$ export PATH=$HOME/development/rtems/6/bin:$PATH
$ cd $HOME/development/rtems/kernel/rtems
$ command -v arm-rtems6-gcc && echo "found" || echo "not found"
$ echo "[arm/beagleboneblack]" > config.ini
$ echo "BUILD_TESTS = True" >> config.ini
$ ./waf configure --prefix=$HOME/development/rtems/6
$ .waf
$ sudo ./waf install

```
### Kernel Image sample 📦 (review, maybe not with Metodo 1)
In order to build a sample kernel image, provided by RTEMS:
```
$ cd $HOME/development/rtems/
$ arm-rtems6-objcopy -Obinary $HOME/development/rtems/kernel/rtems/build/arm/beagleboneblack/testsuites/samples/ticker.exe -O binary app.bin
$ gzip -9 app.bin
$ mkimage -A arm -O linux -T kernel -a 0x80000000 -e 0x80000000 -n RTEMS -d app.bin.gz rtems-app.img
```
### Extras 🖥️
If the BeagleBone Black is going to be connected to the computer via USB, suposing **edimar369** is the computer's username:
```
$ sudo adduser edimar369 dialout
$ sudo apt install screen
```
	
In order to monitor the serial port, we first need to determine it's Linux device name:
```
$ ls -l /dev/ttyUSB* /dev/ttyACM*
```
	
To use screen and connect to the Raspberry Pi, suposing the device name is **ttyUSB0**:
```
$ sudo screen /dev/ttyUSB0 115200
```

## How to build a Kernel Image using RTEMS 📦
1. Change to the build directory.
```
$ cd build
$ cd rpi2
```

2. Double check the options in the Makefile [rtems-paths.mak](https://github.com/edison369/RTEMS-Raspberry-Pi/blob/main/build/rtems-paths.mak). You usually just need to set the path to the cross compiler and RTEMS BSP. In case you followed the installation in [RTEMS Installation](<#rtems-installation>):
```
RTEMS_TOOL_BASE ?= /home/edimar369/development/rtems/5
RTEMS_BSP_BASE ?= /home/edimar369/development/rtems/5
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
$ arm-rtems5-objcopy -Obinary <executable file> kernel.img 
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
