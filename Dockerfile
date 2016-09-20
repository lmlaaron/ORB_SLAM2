############################################################
# Dockerfile to build ORB_SLAM container images
# Based on Ubuntu
############################################################

# Set the base image to Ubuntu
FROM ubuntu

# File Author / Maintainer
MAINTAINER Example McAuthor

# Update the repository sources list
RUN apt-get update

################## BEGIN INSTALLATION ######################
# Install MongoDB Following the Instructions at MongoDB Docs
# Ref: http://docs.mongodb.org/manual/tutorial/install-mongodb-on-ubuntu/

# install all dependencies
RUN apt-get upgrade
RUN apt-get install g++ -y
RUN apt-get install git -y
RUN apt-get install make -y
RUN apt-get install cmake -y
RUN apt-get install libblas-dev -y
RUN apt-get install liblapack-dev -y

# install eigen
RUN apt-get install mercurial -y
RUN hg clone https://bitbucket.org/eigen/eigen
cd eigen
mkdir build
cd build
cmake ../
make install

#install pangolin
cd /home/root/
git clone https://github.com/stevenlovegrove/Pangolin.git
apt-get install libglew-dev -y
apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev -y
cd Pangolin
mkdir build
cd build
cmake -DCPP11_NO_BOOST=1 ..
make -j

#install opencv
cd /home/root
git clone https://github.com/opencv/opencv.git -b 2.4
cd opencv
mkdir build
cd build
cmake ../
make 
make install

#install protobuf
cd /home/root
apt-get install unzip -y
apt-get install libtool -y
git clone https://github.com/google/protobuf.git
cd protobuf
./autogen.sh
make
make check
make install
ldconfig # refresh shared library cache.

#install grpc
cd /home/root
apt-get install curl -y
apt-get install autoconf -y
apt-get install autotools -y
apt-get install libtool -y
git clone -b $(curl -L http://grpc.io/release) https://github.com/grpc/grpc
cd grpc
git submodule update --init
make
make install

COPY ORB_SLAM2_NEW /home/root/ORB_SLAM2
cd /home/root/ORB_SLAM2
RUN ./build.sh
##################### INSTALLATION END #####################

# Expose the default port
#EXPOSE 27017

# Default port to execute the entrypoint (MongoDB)
#CMD ["--port 27017"]

# Set default container command
#ENTRYPOINT /bin/bash
ENTRYPOINT /home/root/ORB_SLAM2/

