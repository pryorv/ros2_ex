# Robotic_HW
## Context
The design of our cells in Machina Labs has evolved over the past years. Currently, each of our cells has two articulated industrial robots on rails (a total of 7 axes) and a frame with hydraulic clamps. For the parts to form correctly, we must exert and maintain a dynamic force during the forming in a very accurate location in space. Currently, each robot is equipped with a load cell. See a quick video about our process [here](https://www.youtube.com/watch?v=iqYMprTEXRI). We are using ROS2 to collect the data from the network and control the robots in real-time. As a robotic engineer, we keep developing different modules for our network to add features to the system.  
 
## Objective
The goal of This project is to build a ROS2 network that collects data from 3-DOF sensors and makes the filtered data available as a ROS service and topic. Since we cannot send a real sensor to all of our applicants, we made a simple simulator (sensor.py) that mimics the behavior of a real sensor but with random data. 
- The first task is to make a costume service for 3-DOF sensor 
- The second task is to make a ROS2 service server that continuously reads data from the sensor and has the latest filter data available for the client service that you make. 
- Finally, please make a simple client that calls two of these services and publishes them a topic with 500Hz. Please keep in mind that your service servers can run slower than 500Hz. 
- You can define a second server in the simulator to modify the code and run two at the same time.
- You can check the example.py to see how to make calls to the sensor

## Grading Criteria
- We’re looking for code that is clean, readable, performant, and maintainable.
- The developer must think through the process of deploying and using the solution and provide the necessary documentation.
- The sensor samples with 2000Hz, and you can request a specific number of samples in each call. Each call also has a ~1ms delay on top of the sampling time. We would like to hear your thought on picking the number of samples that you read in each call. 

## Submission
To submit the assignment, do the following:

1. Navigate to GitHub's project import page: [https://github.com/new/import](https://github.com/new/import)

2. In the box titled "Your old repository's clone URL", paste the homework repository's link: [https://github.com/Machina-Labs/robotic_hw](https://github.com/Machina-Labs/robotic_hw)

3. In the box titled "Repository Name", add a name for your local homework (ex. `Robotic_soln`)

4. Set the privacy level to "Public", then click "Begin Import" button at bottom of the page.

5. Develop your homework solution in the cloned repository and push it to GitHub when you're done. Extra points for good Git hygiene.

6. Send us the link to your repository.

## Launching code
Clone git repo ROS2 packages into the source directory of ROS2 workspace

Run the sensor.py code in a separate terminal.

Open a new terminal, source your main ros2 installation. Go to root directory of workspace and 
build the packages using colcon build (colcon build --merge-install for Windows).

Launch file currently does not work but the needed nodes to run can just be called with the following instructions:
When deploying, the launch file would be preferred to use.

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
- There are two of these sensors so if the servers can keep up with 250 Hz = 1 call every 4ms each, the client can publish at 500 Hz
- In 4ms, the sensor can capture roughly ~6 samples. To account for the delay not being exact, the number of samples we will use is 5.

