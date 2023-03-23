# README #

### What is this repository for? ###

* SofaCV is a plugin for the simulation framework SOFA [link](www.sofa-framework.org). Its purpose is to provide computer vision tools to SOFA. More specifically, its goal is to enable the use of SOFA simulations in augmented reality, and virtual reality applications.
The SofaCV plugin is not a standalone plugin by itself. Instead it provides an API for submodules (mainly Sofa's ImageProcessing and DataAcquisition plugin) such as base datatypes, Python bindings, SofaQtQuick widgets etc, along with some utility SOFA Components to read, write or display an image in the scene view.

* Version 2.0

### To setup SofaCV: ###

1 Install Third party dependencies
* If SOFA is already installed on your system, chances are that you already have CMake installed. Otherwise, please refer to [sofa's installation documentation](https://www.sofa-framework.org/community/doc/)
* SofaCV depends on OpenCV 3.2. Thus, you will need to download it, either from your distrib's package manager (aptitude, pacman, yum, macports, brew etc),
    or build it directly from github (git clone [https://github.com/opencv/opencv.git](https://github.com/opencv/opencv)). I personally advise you to build it from it's git repository, especially if you plan on using Opencv's non-free features - opencv_contrib)
* SofaCV provides useful python bindings, and enables GUI functionalities in SofaQtQuick, provided that these tools are precompiled and installed on your system.
    * To get access to python bindings, enable the SofaPython plugin by compiling sofa with __-DPLUGIN_SOFAPYTHON=ON__    
    * To use the new GUI's functionalities, the SofaQtQuick plugin needs to be activated and compiled with SOFA. Checkout SofaQtQuick's documentation for more information (https://github.com/sofa-framework/SofaQtQuick)

2 Install SOFA with SofaCV
* Download Sofa-framework from Github [github.com/sofa-framework/sofa.git](github.com/sofa-framework/sofa)
* cd ${SOFA_SRC_DIR}; mkdir build ; cd build ; cmake-gui ${SOFA_SRC_DIR}
* in CMAKE's gui, search for the SOFA_EXTERNAL_DIRECTORIES, and set this variable to the path to you SofaCV repository
* Configure, generate, and then compile!

### Contribution guidelines ###

* __Contributing to SofaCV:__
    The __master__ branch for SofaCV is protected, and you need to create pull requests to contribute to the codebase. 
    
### Who do I talk to? ###

* Repo owner or admin: bruno.josue.marques@inria.fr
* Other community or team contact: mimesis@inria.fr
