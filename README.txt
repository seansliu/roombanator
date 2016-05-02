This is a brief guide on how to get our code running.
More details can be found in the project report.

Preconditions:
	1. Be running Windows 7 or newer
	2. Have MATLAB 2016a installed (older versions probably OK, but not tested)
	3. Install Intel Perceptual Compute SDK (2013) 
		https://software.intel.com/en-us/articles/intel-perceptual-computing-sdk-installer

Setting up iRobot Create 2:
	1. Download and extract MTIC2
		https://www.mathworks.com/matlabcentral/fileexchange/52644-matlab-toolbox-for-the-irobot-create-2
	2. Download and install FTDI Drivers (this is for the cable)
		http://www.ftdichip.com/Drivers/VCP.htm
		Recommendation: download the setup executable to install this
	3. Add MTIC2 to Matlab path
	4. Connect USB-Serial cable to Robot and Computer
	5. Determine USB-Serial cable port number via device manager
	6. Run r=RoombaInit(x) where X is port number

Setting up Creative Senz3D Camera
	1. Install MinGW 4.9.2 (Matlab C/C++ compiler addon)
	2. Download and extract Senz3D Acquisition Interface
		https://www.mathworks.com/matlabcentral/fileexchange/42581-senz3d-acquisition-interface
	3. Add Senz3D Acquisiton Interface to matlab path
	4. In Matlab Senz3D Acquisition Interface source files:
		Modify compile_cpp_files.m
			First line: addpath: set to absolute path to MEX subfolder
			Add
			 [f2 ‘util_pipeline_emotion.cpp],...
			Under filename list of dependencies
		In toolbox file pxcColorImage.cpp, modify MapColorCoordinatesToDepth call:
			From 
				MapColorCoordinatesToDepth(npoints, posc, posd);
			To 
				PXCSizeU32 sz = {0, 0};
				projection->MapColorCoordinatesToDepth(rgbImage, posc, npoints, sz);
	5. In C:\Program Files (x86)\Intel\PCSDK\sample\common\src and C:\Program Files (x86)\Intel\PCSDK\sample\common\include:
		Find all instances of wcscpy_s and replace with wcscpy, and remove the cast operation
		Find all memcpy_s and replace with memcpy and remove 2nd argument (size_t numberOfElements)
	6. As an alternative to step 5, you MAY be able to copy the contents of folder 
		roombanator/MATLAB/IntelCreative3D 
	7. As an alternative to step 6, you MAY be able to copy the contents of folder 
		roombanator/Intel/PCSDK/sample/common 

Running the program:
	1. Initialize roomba with "r = RoombaInit(x)"
	2. Run "roombanator(r, 1)" (this assumes roombanator source files are in MATLAB path)
		This initializes our code in full-running mode
	3. A color image should pop up. Select the color of the target destination you want
		the roomba to follow.
	4. A depth image should pop up. This is the image the roomba will use to generate a 
		background image (for filtering purposes during navigation perception).
		This image should be an obstacle-free, uncluttered floor.
		Hit enter in the terminal to start the program.
	5. Occassionally when you run the code multiple times in a row, in step 3, instead of a 
		single large color image showing up, four images will show up.
		In this case, close the window and restart the program (result of dangling variables/state)
	6. Run "roombanator(0, 0)" for Camera only mode, where the Camera runs in a loop continuously
		and prints debug messages about what is happening. This is useful for debugging without
		running the robot.