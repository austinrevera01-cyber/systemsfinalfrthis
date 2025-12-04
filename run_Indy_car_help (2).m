%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	function [GPS,yaw_gyro,motor_counts,WP,lat_err]=run_Indy_car(volts,Vel,X0_values,WP_FILE);
%
%	Function to simulate the Indy Autonomous Challenge Dallar IndyLights Vehicle
%   at a constant velocity (Vel).  
%
%   Assumes time step of Ts=0.001 (1000 Hz).
%
%   Motor:  60 Watt, 24 Volt RE30 (part number: 268214 )
%   Gearhead:  GP 32, 21:1 (part number: 166160)
%   Encoder: HEDS 5540, 500 CPT, 3 Channels (part number: 110511)
%   Note that the encoder count is increased by 4x through quadrature
%   counting (i.e. counting rising and falling edge of the A and B
%   channels using something like the US Digital LS7183N chip 
%   or your favorite microcontroller).
%
%   Generates sensor measurement data for the IAC car given inital conditions 
%   (X0) at a specified velocity (Vel) and a commanded steering motor
%   voltage.  Velocity can be zero to test the steering system without the
%   car moving.  The function also returns a desired waypoint in (East,North) 
%   from a specified file (WP_FILE)
%
%   Note: If the function is called with only voltage, then the motor is run
%   with the motor gearbox disconnected from the steering rack.  This
%   allows users to run the motor-gearhead by itself.
%
%   Note:  Initial angular velocity and current are assumed to be zero.
%
%	GPS=[East (m)  North(m)  Heading(rad)];
%   yaw_gryo=yaw_rate (rad/sec)
%   motor_counts=steering motor encode counts (at the motor) 
%   WP = [East_desired (m)  North_desired (m)]
%   lat_err = lateral error from vehicle to waypoints (positive is right)
%
%   Vel = vehicle speed in m/s (can not be negative, but can be zero).
%   In this code, velocity can not be changed mid-simulation 
%   (i.e. it is a constant assigned at initialization). 
%
%   X0=[East0(m)  North0(m)  Heading0(rad) yaw_rate0(rad/s) motor_angle0(rad)];
%   Default X0=[0 0 0 0 0];
%   Default WP_FILE=0;
%
%   WP_FILE (and corresponding X0)
%       0:  None (any, although a desired heading of 0, +/-90 deg, or +/-180 deg work well)
%       1:  Double Lane Change (0,0,pi,0)
%       2:  Track #1 (IMS): (0,0,pi,0)  - Track Width is 15 meters
%       3:  Track #2 (Barnber): (0,0,pi/4,0)
%       4:  Track #3, not available yet
%
%   WP and Lateral Error return "NaN" for WP_FILE 0 
%   WP will ALSO return "NaN" if the end of the waypoint
%   file is reached for any of the other Waypoint Files.  You can check
%   with the matlab function "isnan(WP)"
%   
%   Note: You must run "clear all" to reset the initial conditions before
%   any new run (DO NOT RUN CLEAR ALL EACH TIME YOU CALL THE FUNCTION -
%   JUST EACH NEW SIMULATION/RUN).
%   You should also run "fclose('all'), at the end of your code but this is 
%   not as critical (unless you want to rename your txt file).
%
%   Note: the max vehicle steer angle the vehicle can turn (at the the tire) 
%   is 20 degrees.  Max Lateral Error before hitting the wall is 7 meters
%
%   Note this code will also save the vehicle's GPS postion to a file
%   titled "waypoint_file_for_GPSVisualizer.txt"  If you want to save each
%   run, you need to rename the file before executing the code again (as
%   each time the same file name will be over-written.  This file can then
%   be imported into GPS visualizer to create a KML file for importing into
%   Google Earth
%
%   Examples:
%   
%   [GPS(k+1,:),yaw_gyro(k+1),motor_enc(k+1)]=run_Indy_car(10);
%
%   The above will provide the output of the sensors on the car 0.001 seconds 
%   after sending 10 volts to the motor with the gearbox disconnected from the
%   steering rack (while the vehicle is stationary).
%
%   [GPS(k+1,:),yaw_gyro(k+1),motor_enc(k+1)]=run_Indy_car(10,0);
%
%   The above will provide the output of the sensors on the car 0.001 seconds 
%   after sending 10 volts to the motor with the gearbox connected to the
%   steering rack (while the vehicle is stationary).
%
%   [GPS(k+1,:),yaw_gyro(k+1),motor_enc(k+1)]=run_Indy_car(10, 30);
%   
%   The above will calculate the next GPS positions (E,N,Heading), Yaw
%   Gryo, and motor angle, 0.001 seconds after sending 10 volts to the 
%   motor attached to the steering rack at 30 m/s with all initial conditions 
%   set to zero.
%
%   [GPS(k+1,:),yaw_gyro(k+1),motor_enc(k+1)]=run_Indy_car(0,10,[0 0 0 0 theta_0]);
%   
%   The above will calculate the next GPS positions (E,N,Heading), Yaw
%   Gryo, and motor angle, 0.001 seconds after sending 0 volts 
%   (i.e. constant motor angle) to the motor attached to the steering rack 
%   at 10 m/s with the steer motor angle set to some initial angle theta_0.
%
%   X0=[0 0 -pi 0 0]
%   [GPS(k+1,:),yaw_gyro(k+1),motor_enc(k+1),WP(k+1,:),lateral_err(k+1)]=run_Indy_car(volts(k+1),Vel,X0,1);
%
%   The above will also return a desired waypoint and lateral error for file #1
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%