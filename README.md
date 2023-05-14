# Application of an inertial navigation system from IMU data

**The Inertial Navigation System (INS) exploits the information from the Inertial Measurement Unit (IMU) sensors to generate comprehensive and continuous navigational data, providing position awareness without any reliance on external devices**.  
Numerous instances of terminology misnomers can be found in the literature when comparing IMU and INS as if they were the same.  
  
<p align="center">
  <img width="556"alt="INS" src="https://github.com/RodolpheFmd/Inertial-Navigation-System/assets/92471439/4ad85d0b-c537-48a6-8c7c-c0dc7685c753">
<p align="center">
  
  The IMU provides raw acceleration and rotational data obtained from a set of accelerometers and gyroscopes mounted along orthogonal axes.  
*In order to provide even more accurate information, redundant sensors are often employed in IMUs. This typically involves the use of multiple accelerometers and gyroscopes (e.g., 12 accelerometers and 12 gyroscopes) to enhance accuracy and reliability.*  
    
**This GitHub project focuses on the algorithmic part of the INS from pre-existing IMU data.** [IMU_data.mat](https://github.com/RodolpheFmd/Inertial-Navigation-System/blob/main/IMU_data.mat)
  
## INS 
The build of a such system follows the next protocol:  
 - List of the constants required for the task.
 - The initial condition of the system (e.g., aircraft):  
       - Initial position (lat0, lon0, alt0)  
       - Initial orientation (roll, pitch, yaw)  
       - Initial velocity (with regards to the NED frame)
 - Define the quaternions that provide a compact and computationally efficient means of handling rotational information.
 - Coordinate system conversion using quternions:
       - Body / North, East, Down (NED) transition and reverse.
       - Cartesian / Earth-Centered, Earth-Fixed (ECEF) transition and reverse.  
 - From IMU rotational data:
       - Solve quaternion differential equation. (*The equation changes with regards to speed attitude)*  
       - Calculate the Direction Cosine Matrix (DCM)  
       - Calculate Euler angles. (*At this stage, we know the vector containing angles of the motion of the object accordingly to NED frame)*
  
  ![Euler_Angles](https://github.com/RodolpheFmd/Inertial-Navigation-System/assets/92471439/597d1190-acde-426e-b0ba-87d20fda7dfc)
  
 - From IMU acceleration data:  
       - Conversion of the data from body frame into NED coordinates.  
       - Integration for extracting the velocity.  
       - Second Integration for extracting the position.  
       - Express this position into cartesian frame. (*At this stage, we know the object's position in the space)*
       - Re-express the downstream position into ECEF coordinate (Actual latitude, longitude, altitude known)*  

  By knowing the time interval of the information provided by IMU we can draw the trajectory of the object.
  
  ![2D_Aircraft_Position_NED_Frame](https://github.com/RodolpheFmd/Inertial-Navigation-System/assets/92471439/8a972122-3aa5-4c33-a707-70e0ba795458)  
    
  
 ![3D_Aircraft_Position_NED_Frame](https://github.com/RodolpheFmd/Inertial-Navigation-System/assets/92471439/772ba9a7-85a8-4848-82bb-cb91143d8323)

  
## Extra
### Coase alignment method

