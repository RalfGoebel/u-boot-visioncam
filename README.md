# U-Boot - VisionCam XM / VisionSensor PV

This is the bootloader for the [IMAGO](https://www.imago-technologies.com) VisionCam XM and VisionSensor PV.

The VisionCam and the VisionSensor combine both camera and a processor to a freely programmable vision system.

## Key features VisionCam XM
* Processor: TI AM572x, 2x ARM Cortex-A15 1.5 GHz
* 1 GB DDR3 RAM
* CMOS sensors up to 5 MPixels, global shutter
* Line scan sensors up to 8k pixels
* Integrated LED ring light and strobe controller
* Opto-isolated I/Os
* "Real-Time Communication Controller": Controlls I/Os in real-time, operates independently from processor and OS
* Easy-to-use high-level API for C++
* Ethernet 1000BASE-T
* µSD card
* RS-232 interface
* IP65 version available

## Key features VisionSensor PV
* Processor: TI AM437x, 2x ARM Cortex-A9 1.0 GHz
* 512 MB RAM
* CMOS sensor 752 x 480, global shutter
* Ethernet 100BASE-TX
* µSD card
* RS-232 interface
* Digital I/Os
* Integrated LEDs and strobe controller
* Easy-to-use high-level API for C++

## Build instructions

### Setup cross-toolchain
    wget https://releases.linaro.org/components/toolchain/binaries/6.2-2016.11/arm-linux-gnueabihf/gcc-linaro-6.2.1-2016.11-x86_64_arm-linux-gnueabihf.tar.xz
    tar -Jxvf gcc-linaro-6.2.1-2016.11-x86_64_arm-linux-gnueabihf.tar.xz -C $HOME
    export PATH=$HOME/gcc-linaro-6.2.1-2016.11-x86_64_arm-linux-gnueabihf/bin:$PATH
    export CROSS_COMPILE=arm-linux-gnueabihf-

### Clone U-Boot source tree
    git clone -b imago-2017.01 https://github.com/RalfGoebel/u-boot-visioncam
    cd visioncam-u-boot

### Build U-Boot
    make am57xx_visioncam_xm_defconfig # or am43xx_visionsensor_pv_defconfig
    make -j4

Set LOCALVERSION to append a version string, e.g.:
    LOCALVERSION=-1.0.0.0 make -j4

After compilation, simply copy MLO and u-boot.img to the FAT partition on the SD card.
