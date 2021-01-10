# AircraftSimulationControl

This repository contains the simulation model of RCAM (research civil aircraft model).
It is done for personal interest (hobby) to self-teach how to develop a simulation model of an aircraft.
The reference is contained in the MATLAB file.

I do not intend to benefit from others' work. 
All methodologies of simulation modeling are taken from YouTube video (Dr. Christopher Lum's video).

Further adjustments of the aircraft simulation will be made in the near future.

Plan: Add controllers (PID, LQR, Adaptive Controller, etc.)
      Add estimator (Full-state, Reduced-order, Extended Kalman Filter)
      Add rate limit for aileron deflection, tailplane deflection, and rudder deflection for more realistic model.
      
Maybe: It is said the RCAM aircraft model is similar to Boeing Twin Engine 757 so I wish to build simulation model for other aircrafts.


Reference:
  
  "Building a Matlab/Simulink Model of an Aircraft: the Research Civil Aircraft Model (RCAM)"
  https://www.youtube.com/watch?v=m5sEln5bWuM&list=PLxdnSsBqCrrEx3A6W94sQGClk6Q4YCg-h&index=13&ab_channel=ChristopherLum
  
  "The Navigation Equations: Computing Position North, East, and Down"
  https://www.youtube.com/watch?v=XQZV-YZ7asE&list=PLxdnSsBqCrrEx3A6W94sQGClk6Q4YCg-h&index=16&t=2666s&ab_channel=ChristopherLum
  
  "Manipulating Aerodynamic Coefficients"
  https://www.youtube.com/watch?v=Mv6aUQkK59s&list=PLxdnSsBqCrrEx3A6W94sQGClk6Q4YCg-h&index=11&ab_channel=ChristopherLum
  
  "Trimming a Model of a Dynamic System Using Numerical Optimization"
  https://www.youtube.com/watch?v=YzZI1V2mJw8&list=PLxdnSsBqCrrEx3A6W94sQGClk6Q4YCg-h&index=14&ab_channel=ChristopherLum
  
  "Aircraft Dynamics From Modeling to Simulation" by Dr. Marcello R. Napolitano.
  
  Powerpoint files from Dr. Ken Bordignon's AE 413 class
