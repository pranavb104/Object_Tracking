# Object_Tracking
Using the Intel RealSense SR300/D435 to cluster objects and have real-time tracking (Using PCL Library)

Check the tracking video on my website https://www.pranavbe.com/white_table/2018/5/8/final-prototype 

This example uses the latest librealsense and PCL library(8.1).

To compile use CMake to build the code in Visual Studio.

The code is compatible with the latest Intel Depth Cameras (D435, D415, SR300).

D435, D415 have different input resolutions which need to pre-configured to have satisfying results. 
Look at this link for configuring depth camera feed https://github.com/IntelRealSense/librealsense/wiki/API-How-To 

# Instructions for Setting up

# Install drivers and software 
1.	Instal sdk for intel camera (Intel.RealSense.SDK.exe) https://github.com/IntelRealSense/librealsense/releases 
2.	Install OpenCV (opencv-3.4.0-vc14_vc15.exe) to any directory (preferably ProgramFilesx86).
3.	Install PCL (PCL-1.8.1-AllInOne-msvc2015-win64) for visual studio 2015 (or the 2017 version if you have visual studio 2017). While installing you will be prompted to check or uncheck “add pcl to system paths”. Check the box and continue with installation and include the 3rd party libraries in the installations page. 
4.	Install (cmake-3.11.3-win64-x64)  https://cmake.org/download/ 

# Add Dependencies 
Inside the installation directory of OpenCV go to opencv/build/x64/vc15/bin and copy the address into path of environment variables. (To find environment variables, go to My PC and click properties and then go to Advanced System Settings, you will find environment variables there.)
Inside environment variables press p to find path and press edit. Select on new and copy paste the address of opencv(to opencv/build/x64/vc15/bin) as well as of OpenNI2 (C:\Program Files\OpenNI2\Tools)  and librealsense (C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64).
This will help the progrem refer to the DLL files, so that you will not have to manually add them later for every project you create. 

# Generate your Project
Create a new folder and name it rs_visual. Copy the CMakeLists and cpp file (included in the commits above) into this folder. Create a new folder inside this folder called build. Run Cmake.exe and make a visual studio app for this project. To see how to use CMake look at https://www.youtube.com/watch?v=LxHV-KNEG3k . Or if you want a PCL example of CMake see this https://www.youtube.com/watch?v=hgboDlKHl4c&t=399s 

# Installing OpenCV and Librealsense in your project
Since these two libraires are not mentioned in your Cmake lists, they will not be included in your project automatically. You will have to add them manually.
When you open Visual Studio, make sure you choose Release and x64 for debugging. Look at your solution tree, right-click on your project (rs_visual) and go to properties. In the column VC++ directories you will see include directory and library directory. Add the address of opencv include (opencv\build\include) and librealsense include (C:\Program Files (x86)\Intel RealSense SDK 2.0\include) to include directory. Also add the lib directories for both, opencv (C:\Program Files (x86)\opencv\build\x64\vc15\lib) and librealsense (C:\Program Files (x86)\Intel RealSense SDK 2.0\lib\x64).
Next step, go to Linker on the left column, and click input. Here you have to add the additional dependencies of opencv and librealsense. Click edit and copy paste opencv (opencv_world340.lib) and librealsense (realsense2.lib).
You are now done setting up your project.

Execute and experiment!


If any questions, email me at pranavb104@gmail.com
