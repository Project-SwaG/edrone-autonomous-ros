
# Team: Swayamchalit Gaadi Project SwaG
Members
- Pradumn Mishra ([pradumn203](https://github.com/pradumn203))
- Abhisek Omkar Prasad ([AbhisekOmkar](https://github.com/AbhisekOmkar))
- Urja Jain ([Urja Jain](https://github.com/Urja-Jain))

## INTRODUCTION

Drones or Unmanned Aerial Vehicles (UAVs) come in two variants – fixed-wing and rotary drones. Rotary drones or multirotor drones consist of various configurations, and some common ones are the helicopter, four-rotor quadcopter, and six-rotor hex copter. Each rotor type has a specific usage and is useful for specific applications. The commonly used type of multirotor is the quadcopter (often shortened to quad). This is because it is a very mechanically, simple system. In quads, each motor spins in the opposite direction of the adjacent motor.

In order to maintain its 'pose' in-flight and stay stable, a quadcopter relies heavily on sensors that constantly monitor the quad's 'attitude.' These sensors provide feedback to the quad using which the flight controller makes corrections in the motor spin speed and thus adjusting and shifting the quad in flight so that it remains stable.

## Sensor Models Used

<strong>Inertial Measurement Unit (IMU):</strong> This sensor is a combination of an accelerometer,which measures linear acceleration (i.e., in X, Y & Z axis relative to the sensor) and agyroscope which measures angular velocity in three axes, roll, pitch, and yaw.Typically a magnetometer is also present, which by measuring the earth's magneticfield, serves to give a reference direction.

<strong>Barometer:</strong> Barometers are used to measure the air pressure and hence help flightcontrollers to predict the height of the quad from the ground. Barometers generallycome in handy when the quad is at large enough heights.

<strong>Ultrasonic Sensor:</strong>Ultrasonic sensors help give a precise height of the quad fromthe ground. Ultrasonic sensors are extremely useful when flying a quad within 50cmfrom the ground. Beyond that height, it is recommended to rely on the barometer forestimating the quad height with respect to the ground.

<strong>Time of Flight Sensor:</strong>It is a distance sensor that uses a laser to estimate the distance.

<strong>GPS Reciever:</strong>Incorporating this on the UAV enables it to locate itself using theGlobal Positioning System and other satellite positioning systems.

## QR Detection using Python
There are various libraries by which you can scan a QR with a single line of code (just passing an image to these libraries). Example pyzbar is such a library that is used to detect QR codes.

## CONTROL SYSTEM
<p align="center">
  <img src="/screenshots/MapControl.jpg">  
</p>

## Detection of Launchpad using OpenCV
 Here is a short script to detect the landing marker and display it
 ```
 import cv2 
 from matplotlib import pyplot as plt 
 logo_cascade = cv2.CascadeClassifier('data/cascade.xml') 
 img = cv2.imread('test_1.png')  # Source image 
 gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
 logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05) 
 for (x, y, w, h) in logo: 
    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2) 
 plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)) 
 plt.show() 
```

## ROS Nodes for Drone Simulation
We have successfully complied ROS nodes for Drone Simulation. These ROS Nodes can successfully perform the below tasks:- 


### Task 1
The aim of this task is to design controllers which will control the eDrone's orientation as well as position in Gazebo environment namely attitude and position controller scripts.

<img src="/screenshots/cascade_control_system.png" width="900" height="300">  

<img align="center" src="/screenshots/task.gif" width="600" height="500">

<img src="/screenshots/task1.png" width="900" height="500">  


### Task 2
The aim of this task is to pick a parcel and deliver it to its destination by:
- Scanning the QR code and finding out the destination GPS co-ordinates.
- Pick/Drop the parcel box.
- Avoiding dynamic obstacles and planing the path.

<img src="/screenshots/task2qr.png" width="900" height="500">  

<img src="/screenshots/task2obs.png" width="900" height="500">  

#### [Task 2 Video](https://www.youtube.com/watch?v=ow7JdfzKjZU)

### Task 3
The aim of this task is to refine our navigation and path planning algorithm so as to increase the functionality of the drone for doing more tasks such as delivery of multiple sets of packages to multiple destinations.

<img src="/screenshots/task3.png" width="900" height="500">  


### Task 4
The main aim of this task is to complete a set of deliveries form warehouse location to their destinations using our optimized algorithms in the previous steps.

<img src="/screenshots/task4boxes.png" width="900" height="500">  

<img src="/screenshots/task4delivery.png" width="900" height="500">  



### Task 5
The main aim of this task is to complete a set of deliveries and returns warehouse location to their destinations or vice versa in the most efficient order.

<img src="/screenshots/task5boxes.png" width="900" height="500">  

<img src="/screenshots/task5return.png" width="900" height="500">  



### Task 6
The aim of this task is to complete a set of deliveries and returns from warehouse location to their destinations or vice versa in the most efficient order to maximize the earnings produced from each delivery or return.


<img src="/screenshots/task6marker.png" width="900" height="500">  

<img src="/screenshots/task6flight.png" width="900" height="500">  


#### [Task 6 Video](https://www.youtube.com/watch?v=KohW11D1_Yc)


