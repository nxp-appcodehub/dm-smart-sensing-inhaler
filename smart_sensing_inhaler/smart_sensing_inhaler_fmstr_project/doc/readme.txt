Hardware requirements
=====================
- USB Cable Type C
- FRDM-MCXW71 MCU board
- FRDMSTBI-B3115 or FRDMSTBC-P3115 sensor shield board.
- Personal Computer
- FreeMASTER 3.2 or later installed on PC

Board settings
============
Follow the dm-smart-sensing-inhaler/README.md Hardware setup section for more details.

Prepare the demo
===============
Follow the dm-smart-sensing-inhaler/README.md for more details.
1.  Connect a USB cable between the host PC and the MCU-Link port on the target board.
2.  Compile and download the program to the target microcontroller.
3.  Run and resume application execution when debugger stops in the main() function.

Connect with FreeMASTER
=======================
Follow the dm-smart-sensing-inhaler/README.md "Run Demos" section for more details.
4.  Run FreeMASTER, use the Connection Wizard or open Project Options.
5.  Select serial communication, choose correct COMx port and connect at speed 115200 bps.
6.  Start communication, FreeMASTER loads the initial TSA Active Content links in the Welcome page.
7.  Click the "FreeMASTER Demonstration Project (embedded in device)" in the Welcome page.
8.  The demo is now running, you should be able to watch variable values and graphs.

More information
================
Read more information about FreeMASTER tool at http://www.nxp.com/freemaster.
Feel free to ask questions and report issues at FreeMASTER's 
community page at https://community.nxp.com/community/freemaster
