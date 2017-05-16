define kinect ranger
(
  sensor
  (
    range_max 6.5
    fov 58.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]

)

define camera_sensor camera
(
  # laser properties
  resolution [ 32 32 ]
  range [ 0.2 8.0 ]
  fov [ 70.0 40.0 ]
  pantilt [ 0.0 0.0 ]

  # model properties
  size [ 0.1 0.07 0.05 ]
  color "black"
  watts 100.0 # TODO find watts for sony pan-tilt camera
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  
  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
  camera_sensor(pose [ -0.1 0.0 -0.11 1.0 ])
)

