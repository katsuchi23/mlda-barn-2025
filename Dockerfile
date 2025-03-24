FROM ros:melodic

# Install dependencies for Python 3.9
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update \
    && apt-get install -y python3.8 python3.8-dev python3.8-distutils

# Set Python 3.8 as the default python3
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# Install pip for Python 3.8
RUN apt-get install -y curl
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3 get-pip.py && \
    rm get-pip.py

# Create workspace
RUN mkdir -p /jackal_ws/src
WORKDIR /jackal_ws/src

# Clone repositories
RUN apt install git
RUN git clone https://github.com/jackal/jackal.git --branch melodic-devel
RUN git clone https://github.com/jackal/jackal_simulator.git --branch melodic-devel
RUN git clone https://github.com/jackal/jackal_desktop.git --branch melodic-devel
RUN git clone https://github.com/MLDA-Robotics/mlda-barn-2025.git --branch main

# Build free_space_motion_tube
WORKDIR /jackal_ws/src/mlda-barn-2025/free_space_motion_tube
RUN mkdir -p build
WORKDIR /jackal_ws/src/mlda-barn-2025/free_space_motion_tube/build
RUN cmake ..
RUN make -j8 && make install

# Install ROS components
RUN apt-get update
RUN apt-get install -y ros-melodic-desktop-full
RUN apt-get install -y ros-melodic-gmapping
RUN apt-get install -y ros-melodic-robot-localization
RUN apt-get install -y ros-melodic-joint-state-publisher-gui
RUN apt-get install -y ros-melodic-navigation
RUN apt-get install -y ros-melodic-hector-gazebo-plugins
RUN apt-get install -y ros-melodic-velodyne-description
RUN apt-get install -y ros-melodic-rosdoc-lite
RUN apt-get install -y ros-melodic-twist-mux
RUN apt-get install -y ros-melodic-sick-tim
RUN apt-get install -y ros-melodic-teleop-twist-joy
RUN apt-get install -y ros-melodic-teleop-twist-keyboard
RUN apt-get install -y ros-melodic-pointgrey-camera-description
RUN apt-get install -y ros-melodic-interactive-marker-twist-server
RUN apt-get install -y ros-melodic-lms1xx
RUN apt-get install -y ros-melodic-laser-pipeline
RUN apt-get install -y ros-melodic-controller-manager
RUN apt-get install -y ros-melodic-tf-conversions
RUN apt-get install -y python3-scipy

# Install Python packages
RUN pip3 install --upgrade pip
RUN pip3 install defusedxml rospkg netifaces numpy jupyter scipy matplotlib casadi pandas easydict scikit-learn

# Install PyTorch with CUDA support
RUN pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu113

# Set up workspace
WORKDIR /jackal_ws

# Add start script
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "if [ ! -d /jackal_ws/devel ]; then ">> ~/.bashrc
RUN echo "  cd /jackal_ws" >> ~/.bashrc
RUN echo "  catkin_make" >> ~/.bashrc
RUN echo "else" >> ~/.bashrc
RUN echo "  source /jackal_ws/devel/setup.bash;" >> ~/.bashrc
RUN echo "fi" >> ~/.bashrc
RUN echo "alias python=python3" >> ~/.bashrc
RUN echo 'cd /jackal_ws/src/mlda-barn-2025' >> ~/.bashrc
WORKDIR /jackal_ws/src/mlda-barn-2025