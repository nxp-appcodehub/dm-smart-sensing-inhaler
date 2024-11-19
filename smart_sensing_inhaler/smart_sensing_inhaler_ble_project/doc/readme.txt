Overview
========
This example use FRDM-MCXW71 and on-board FXLS8974CF accelerometer as Tilt sensor with FXPQ3115/MPL3115 absolute pressure sensor expansion board FRDM-STBC-P3115 to demonstrate connected smart sensing inhaler application via BLE wireless UART on NXP IoT Toolbox mobile App.

Toolchain supported
===================
- MCUXpresso IDE (ide version details are in the Release Notes)

Hardware requirements
=====================
- USB Cable Type C
- FRDM-MCXW71 MCU Board
- FRDMSTBI-B3115 or FRDMSTBC-P3115 sensor shield board.
- Personal Computer

Software requirements
=====================
- NXP IoT ToolBox App - Android Store or NXP IoT ToolBox App - Apple Store

Board settings
==============
- Flash your FRDM-MCXW71 with its corresponding NBU for wireless examples.
- Information regarding flashing NBU can be found on:
  https://community.nxp.com/t5/MCX-W71-Training-Secure-MCUs-for/FRDM-MCXW71-Hands-On-1-NBU-and-User-Firmware-Update-Using-ISP/ta-p/1970154

Prepare the Demo
================
- Connect a USB-C cable between the host PC and the MCU-Link USB port on the target FRDM-MCXW71 board.
- Right click on the project and select "Debug As". Select MCUXpresso IDE LinkServer (inc. CMSIS DAP) probes.
- Debugger will start downloading the program to device. MCU-LINK probes will be identified. Click “OK” to continue.
- Click on “Resume” button or press “F8” key on your keyboard to continue running the downloaded program on device.
- Click on “Terminate” button or press “CNTR + F2” to terminate the debug session.
- Disconnect the USB-C cable connected to FRDM-MCXW71 and reconnect.
- To enable your board to start its BLE advertising, press and release button SW3.

Running the demo
================
- Open the installed NXP IoT Toolbox (available from Play Store and App Store) on your mobile phone. Make sure bluetooth connection is ON.
- Inside the application, click on the Wireless UART widget. Scan should start automatically.
- When a device called NXP_WU appears, click on it. Your phone should now be connecting to the board.
- After connecting with the FRDM-MCXW71 device, the serial UART terminal will show this message:

 Wireless UART starting as GAP Peripheral.
 Advertising...
 Connected to device 0 as peripheral.

- Pull the syringe to create change in pressure (similar to when inhalation will be done). The pressure sensor will detect dip in pressure.
- The inhaler will detect the tidal breathing inhalation based on change in pressure, and dosage#1 will be dispensed.
- Similarly, try pulling syringe 2 more times and inhaler will show all 3 meds inhaled. The status can be seen on the "Inhaler View".
- Try changing the board up and down, the accelerometer will detect tilt and inform to correct the posture. This status can be seen on the "Inhaler View".
- The Wireless UART app on IoT Toolbox will show following messages:

  Start Inhaling Doses
  1st Dose Inhaled
  2nd Dose Inhaled
  3rd Dose Inhaled
  All 3 Meds Inhaled
  
  Reseting Inhaler



