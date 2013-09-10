CoSLAM
======

CoSLAM is a visual SLAM software that aims to use multiple freely moving cameras to simultaneously compute their egomotion and the 3D map of the surrounding scenes in a highly dynamic environment.
For more details, please refer to [CoSLAM project page](http://www.ece.nus.edu.sg/stfpage/eletp/Projects/SLAM/).


How to cite
-----------
To use the codes, please cite the paper,

Bibtex entry

        @article{
          zou2013coslam,
          title="Coslam: Collaborative visual slam in dynamic environments", 
          author="Zou, Danping and Tan, Ping", 
          journal="IEEE Trans. on Pattern Analysis and Machine Intelligence" 
          year="2013", 
          publisher="IEEE"
          }
          

          
System requirements
-----------

* A nvidia graphics card (To support Nvidia Cg language)
* Linux Unbuntu or Linux Mint (64 bit). It recommended to use linux Mint 14(nadia) 64-bit system where CoSLAM has been well tested. 


Dependent packages
-----------
Before compile the source codes, the following packages should be installed.

* LibVisualSLAM (a computer vision library for visual SLAM) 
* nvidia Cg toolkit (for GPU feature tracking) 
* GLEW (for shader support) 
* OpenGL,GLU,glut (for Visualization) 
* BLAS, LAPACK (for linear algebra)
* OpenCV (for Video I/O) 
* wxWidgets (for GUI) 

Here are the instructions for installing those packages.

#### LibVisualSLAM
This is a computer vision library developed for CoSLAM. Please click here.

#### Nvidia Cg toolkit
Please click [here](https://developer.nvidia.com/cg-toolkit-download) to download the nvidia Cg toolkit. Click the .deb file to install the package.

#### GLEW
Can be installed from the repository of Ubuntu or Linux Mint by typing 

    sudo apt-get install libglew-dev
    
#### OpenGL, GLU, and GLUT
OpenGL is supported by default after the nvidia graphics card driver being installed. To install GLU,GLUT, run 

    sudo apt-get install libglut1-mesa-dev
    sudo apt-get install freeglut3-dev
    
#### BLAS and LAPACK
Just run

    sudo apt-get install libblas-dev
    sudo apt-get install liblapack-dev
      
#### OpenCV
Before installing OpenCV, you should install [ffmpeg](https://trac.ffmpeg.org/wiki/UbuntuCompilationGuide) to enable the advance Video I/O. Then download [OpenCV](http://opencv.org/downloads.html) and install it following the [installation instructions](http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation). 

#### wxWidgets
It important that only new versions > 2.81 are supported! The source codes can be download from [here](http://www.wxwidgets.org/downloads/). Go to the source code directory of wxWidgets and run

    ./configure --with-opengl
    make
    sudo make install
      
Download & installation
-----------
After the dependent packages are installed, go to the source code directory of CoSLAM, and run

    mkdir CoSLAM-build
    cd CoSLAM-build
    cmake ..
    make
    sudo make install
    
    
Input
-----------
The input is a txt file that lists the input video files and corresponding camera parameter files. Here is a example.

    3 #number of sequences
    0 0 #number of frame to skip and a reserve number of single camera visual SLAM
    0 0
    0 0
    /xx/xxx/video1.avi #absolute path of video files
    /xx/xxx/video2.avi
    /xx/xxx/video3.avi
    /xx/xxx/cal1.txt #absolute path of calibration files
    /xx/xxx/cal2.txt
    /xx/xxx/cal3.txt

#### Video sequences
The video sequences should be temporally synchronized. Those sequences are suggested to be in 'MPEG-4' format, though other formats may also be supported. The number of video sequences are not restricted in our system. It is however suggested that the number of input sequences be less than five, as the speed of CoSLAM system decreases significantly with the number of sequences. 

#### Camera parameters (Calibration file)
Each video sequence is associated with a camera parameter file, which is in the following form. These parameters can be obtained by using the [calibration toolkit](http://www.vision.caltech.edu/bouguetj/calib_doc/)

    #intrinsic matrix 
    fx t cx
    0 fy cy
    0 0 1 
    #five parameters for distortion
    k0 k1 k2 k3 k4
    
Run
-----------
Go to the directory of the input file, then type

    CoSLAM ./input.txt

to run the system.

Outputs
-----------
The outputs are saved in a directory named by a time string under '~/slam_results'. The outputs include three files:

Contribute to CoSLAM
-----------
CoSLAM is a open source project that welcomes everyone to contribute to this project. 

Liciense
-----------
CoSLAM is released under [GPL v2.0](http://www.gnu.org/licenses/gpl-2.0.html).

Disclaimers
-----------
No contributer is responsible for any problems caused by using CoSLAM.

