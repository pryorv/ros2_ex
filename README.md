
## Launching code
Clone git repo ROS2 packages into the source directory of ROS2 workspace

Open a new terminal, source your main ros2 installation. Go to root directory of workspace and 
build the packages using colcon build (colcon build --merge-install for Windows).

When actually deploying the solution, the launch file would be preferred to use, but
the launch file currently does not work.

The needed nodes to run can just be called with the following instructions:

Run the sensor.py code in a separate terminal (not part of the ROS2 network)

Open a new terminal, navigate to ros2_ws, and source the setup file: install/setup.bat

Run the service server: 
  ros2 run sensor_srvcli server

Open a new terminal, navigate to ros2_ws, and source the setup file: install/setup.bat

Run the service client: 
  ros2 run sensor_srvcli client



## Implementation Notes
The number of samples within each call was determined using the following notes and considerations.
- The sensor samples at a rate of 2000Hz = 1 sample every 0.5 ms
- Each call has an approximate ~1ms delay which means that if only a single sample was obtained for each call,
  the cost would be the sampling time for 3 samples.
- We must select a number of samples minimizes this cost and also keeps up with the set rates from the service servers and client without messages dropping
- The client publishes at 500Hz = 1 message every 2ms
- Assuming the data is filtered with an unspecified method, there must be enough samples to provide reasonable filtered data
- There are two of these sensors so, in theory, if the servers can keep up with 250 Hz = 1 call every 4ms each, the client can publish at 500 Hz
- In 4ms, the sensor can capture roughly ~6 samples. To account for the delay not being exact, the number of samples we will use is 5.
- In reality, there appears to be more significant delays from the network that need to also be taken into account if this solution
was continued.

