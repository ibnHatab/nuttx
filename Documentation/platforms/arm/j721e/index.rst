==================
J721E
==================

J721E TDA4VM is Texas Instruments Cortex-A72 (MPU) and Cortex-R5F (MCU) based SoC.

====================== ======================================== ==================
CPU name in PSDK RTOS  CPU name in docs                         Notes                    
====================== ======================================== ==================
mcu1_0                 MCU_Cortex_R5_0                          System Firmware        
mcu1_1                 MCU_Cortex_R5_1                                                 
mcu2_0 / mcu2_1        MAIN_Cortex_R5_0_0 / MAIN_Cortex_R5_0_1                         
mcu3_0 / mcu3_1        MAIN_Cortex_R5_1_0 / MAIN_Cortex_R5_1_1                         
mpu1                   CortexA72_0_0                            Linux                  
c6x_1                  C66xx_0                                                         
c6x_2                  C66xx_1                                                         
c7x_1                  C71x_0                                                          
PRU                    PRU                                                             
====================== ======================================== ==================


Peripheral Support
==================

The following list of cores currently supported in NuttX:

 mcu1_0 / mcu1_1, mcu2_0 / mcu2_1, mcu3_0 / mcu3_1

with System Firmware (SYSFW) MCU1_0 core.

The following list indicates peripherals currently supported in NuttX:

============== =====
Peripheral     Notes
============== =====
GPIO           See Supported Boards documentation for available pins.
UART
I2C
SPI
PWM
IPC
UDMA
============== =====

Installation
============

1. Download Processor SDK RTOS J721E:

    https://www.ti.com/tool/PROCESSOR-SDK-J721E

    PROCESSOR-SDK-RTOS-J721E — RTOS SDK for DRA829 & TDA4VM Jacinto™ Processors


2. Set PDK_INSTALL_DIR environment variable::

    # Go to Processor SDK RTOS J721E install dir/pdk_jacinto_m_i_p

    export PDK_INSTALL_DIR=`pwd`
    

3. Download NuttX and the companion applications.  These must both be
   contained in the same directory::

    cd $PDK_INSTALL_DIR/packages/ti/kernel/
    git clone https://github.com/apache/nuttx.git nuttx
    git clone https://github.com/apache/nuttx-apps.git apps


    
Build PDK
=============

Only FreeRTOS and SafeRTOS has native build. PDK internal deps is as::

    CSL <- FreeRTOS <- OSAL <- Drivers

1. Patch PDK to get deps as::

    CSL <- NuttX <- OSAL <- NuttX Drivers

    patch -p0 < remoteproc_firmware_nuttx-bba64.patch

2. Build libs::

    BOARD = j721e_evm
    CORE = < mpu1_0, mcu1_0, mcu1_1, mcu2_0, mcu2_1, mcu3_0, mcu3_1, >
    make -s pdk_libs_allcores BOARD=<board>


Building NuttX
==============

1. Change to NuttX directory::

    cd nuttx

2. Select a configuration. The available configurations
   can be listed with the command::

    ./tools/configure.sh -L

3. Load the selected configuration.::

    make distclean
    ./tools/configure.sh <selected_configuration>

4. Modify the configuration as needed (optional)::

    make menuconfig

5. Build NuttX::

    make


Running NuttX
=============
 (examples are forBeagleBone® AI-64 )

    Run command to find which remoteproc serves which MCU::

    $ sudo beagle-version


| [   12.984094] platform 5e00000.r5f: configured R5F for remoteproc mode
| [   12.993031] remoteproc remoteproc18: 5e00000.r5f is available
| [   12.993226] remoteproc remoteproc18: Direct firmware load for j7-main-r5f1_0-fw failed with error -2
| [   12.993235] remoteproc remoteproc18: powering up 5e00000.r5f
| [   12.993254] remoteproc remoteproc18: Direct firmware load for j7-main-r5f1_0-fw failed with error -2
| [   12.993257] remoteproc remoteproc18: request_firmware failed: -2


    Maybe remoteproc18 is MCU2_0

EX 1::

    sudo cp test.elf /lib/firmware/
    #sudo echo stop > /sys/class/remoteproc/remoteproc18/state
    sudo echo test.elf > /sys/class/remoteproc/remoteproc18/firmware
    sudo echo start > /sys/class/remoteproc/remoteproc18/state
    sudo cat /sys/kernel/debug/remoteproc/remoteproc18/trace0

EX 2::

    # With chances to brick board
    sudo cp test.elf /lib/firmware/41000000.r5f
    reboot
    sudo cat  /sys/kernel/debug/remoteproc/remoteproc15/trace0

| Sciclient_boardCfgRm init Passed
| DM Built On: Feb  1 2023 16:31:40
| Sciserver Version: v2022.01.1.0-REL.CORESDK.08.06.00.09-14-ge559a+
| RM_PM_HAL Version: REL.CORESDK.08.06.00.09-7-g4da19
| Starting Sciserver..... PASSED
| IPC_echo_test (core : mcu1_0) .....


Debugging NuttX
================

From target Linux using latest OpenOCD with dmem driver::
    cd .../openocd/tcl
    sudo ../src/openocd -f listen-all.cfg -f board/ti_j721e_swd_native.cfg

| Open On-Chip Debugger 0.12.0+dev-02999-g1b0b07baa (2024-04-23-06:37)
| adapter speed: 2500 kHz
| Info : Listening on port 6666 for tcl connections
| Info : Listening on port 4444 for telnet connections
| Info : clock speed 2500 kHz
| Info : starting gdb server for j721e.cpu.sysctrl on 3333
| Info : Listening on port 3333 for gdb connections
| Info : starting gdb server for j721e.cpu.a72.0 on 3334
| Info : Listening on port 3334 for gdb connections
| Info : starting gdb server for j721e.cpu.a72.1 on 3335
| Info : Listening on port 3335 for gdb connections
| Info : starting gdb server for j721e.cpu.mcu_r5.0 on 3336
| Info : Listening on port 3336 for gdb connections
| Info : starting gdb server for j721e.cpu.mcu_r5.1 on 3337
| Info : Listening on port 3337 for gdb connections
| Info : starting gdb server for j721e.cpu.main0_r5.0 on 3338
| Info : Listening on port 3338 for gdb connections
| Info : starting gdb server for j721e.cpu.main0_r5.1 on 3339
| Info : Listening on port 3339 for gdb connections
| Info : starting gdb server for j721e.cpu.main1_r5.0 on 3340
| Info : Listening on port 3340 for gdb connections
| Info : starting gdb server for j721e.cpu.main1_r5.1 on 3341
| Info : Listening on port 3341 for gdb connections
| Info : accepting 'gdb' connection on tcp/3336

    cat listen-all.cfg

| bindto 0.0.0.0

From host::
    gdb-multiarch --eval-command="set architecture arm" --eval-command="target remote TA.RG.ET.IP:3340" test.elf

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*


