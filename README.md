## Purpose 
The purpose of this course is to familiarize the student with contemporary challenges and technologies for developing self-driving vehicles as an example for embedded and cyber-physical systems. But instead of working with real vehicles, this course is working with scaled variants and data therefrom to allow the students to develop software features.

## Set up development environment 
you will set up your computer with the recommended tools to develop software in this course using git, C++, CMake, and Docker. The guidelines in the following refer to Ubuntu 18.04. 

## Steps 
If you are working on Windows: 
 - It is recommended to either install Ubuntu 18.04 LTS within a virtual machine using VirtualBox for example: https://www.virtualbox.org (Links to an external site.) and https://tecadmin.net/install-ubuntu-on-virtualbox/ (Links to an external site.) 
 - or as Ubuntu 18.04 LTS for dual-boot: https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/ (Links to an external site.) 

#### If you have already a Linux installation (or a Virtual Machine as described above):
Run the following in a terminal: 
```
sudo apt-get update
sudo apt-get upgrade
```

Install software development tools; run the following in a terminal: 
```
sudo apt-get update
sudo apt-get install build-essential cmake git
```

Next, you need to install Docker on your Ubuntu 18.04 LTS; you also add your user to the group docker so that you can run Docker without superuser privileges:
```
sudo usermod -aG docker $USER 
```
Make sure that all the following output from the programs listed above can be clearly seen in the terminal: 

```
g++ --version
make --version
cmake --version
git --version
docker --version
docker-compose --version
docker run --rm -ti hello-world
echo "last name, first name"
```

Build the project using the following commands:
```
mkdir build
cd build
cmake ..
make
```

Open a new terminal window in a folder with downloaded videos and type this command to run OpenDLV:
```
docker build -f Dockerfile -t group-16 .
```

Open a second terminal window in a folder with downloaded videos and type this command to run OpenDLV:

```
docker run --rm --init --net=host --name=opendlv-vehicle-view -v $PWD:/opt/vehicle-view/recordings -v /var/run/docker.sock:/var/run/docker.sock -p 8081:8081 chalmersrevere/opendlv-vehicle-view-multi:v0.0.60
```

Open a third terminal window and type this command to run H264 decoder:
```
xhost +
docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp h264decoder:v0.0.4 --cid=253 --name=img
```
Open a forth terminal window and type this command to run the microservice:
```
docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp group-16:latest --cid=253 --name=img --width=640 --height=480 --verbose
```

## Tools 
 - Linux
 - G++ 
 - Git 
 - CMake
 - Make
 - Docker 
 - OpenCV
 - C++

## Developers 
 - Fayona Cowperthwaite
 - Dominique Deramat
 - Victoria Vu
 - Zhijie Wei
