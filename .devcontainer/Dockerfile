# Server RMF with simulation

# add AS builder to have a build enviroment see internet about
ARG rosimage=osrf/ros:jazzy-desktop-full

FROM $rosimage

#install Core rmf
RUN mkdir -p ~/ff_ws/src \
    && apt update && apt install -y ros-jazzy-rmf-dev ros-dev-tools vim tmux \
    && apt upgrade -y  && apt install -y wget python3-pip python3-vcstool ros-jazzy-rmw-cyclonedds-cpp \
    && pip3 install nudged eclipse-zenoh==1.1.0 pycdr2 rosbags --break-system-packages \
    && echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && /bin/bash -c "source /opt/ros/jazzy/setup.bash" \
    && wget https://raw.githubusercontent.com/open-rmf/rmf/jazzy/rmf.repos \
    && vcs import ~/ff_ws/src < rmf.repos

# # #install simulation code
RUN apt update && apt install -y ros-jazzy-nav2-bringup git \
    && git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations ~/turtlebot3_simulations \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nexport GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc \
    && rm -rf /var/lib/apt/lists/*

# # #install zenoh
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null \
    && apt update \
    && apt install -y systemctl zenoh \
    && rm -rf /var/lib/apt/lists/*

#install free fleet
RUN cd ~/ff_ws/src \
    && git clone -b main https://github.com/open-rmf/free_fleet.git \
    && apt update \
    && cd ~/ff_ws \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr \
    && /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release" \
    && echo "source ~/ff_ws/install/setup.bash" >> ~/.bashrc

#install D-start-trajectory-planner
RUN cd ~/ff_ws/src \
    && git clone -b jazzy https://github.com/ElettraSciComp/DStar-Trajectory-Planner.git \
    && apt update \
    && cd ~/ff_ws \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr \
    && /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select dstar_global_planner --cmake-args -DCMAKE_BUILD_TYPE=Release" 

#install tesi code
RUN cd ~/ff_ws/src \
    && git clone -b master https://github.com/michbelle/myTesiCode.git \
    && apt update \
    && cd ~/ff_ws \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr \
    && echo "export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ff_ws/src/myTesiCode/rmf_code/ign_world/elettra/modelOUT/" >> ~/.bashrc \
    && /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select myCode" \
    && echo "source ~/ff_ws/install/setup.bash" >> ~/.bashrc

# #set simulation robot for nav2
# ENV TURTLEBOT3_MODEL=burger