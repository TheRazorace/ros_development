# Use the official ROS image as the base image
FROM ros:noetic-ros-core

# Set the working directory to the catkin workspace
WORKDIR /catkin_ws

# Create the source directory
RUN mkdir src

# Copy the package into the source directory
COPY . /catkin_ws/src/wheel_control

# Copy the dependency installation script
COPY install_dependencies.sh /catkin_ws/src/install_dependencies.sh

# Make the script executable
RUN chmod +x /catkin_ws/src/install_dependencies.sh

# Install dependencies
RUN . /catkin_ws/src/install_dependencies.sh

# Source ROS setup files
RUN . /opt/ros/noetic/setup.sh && \
    catkin_init_workspace /catkin_ws/src && \
    catkin_make

# Copy the entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh 

# Make the entrypoint script executable
RUN chmod +x /ros_entrypoint.sh

# Set up the ROS environment
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set up the ROS environment
ENTRYPOINT ["/ros_entrypoint.sh"] 

# Set up the ROS environment
CMD ["bash"]