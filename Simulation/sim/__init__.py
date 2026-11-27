"""A pure-Python simulator for the roaming collector: a flat arena with permanent and temporary obstacles, a car with the same kind of
errors as the real one (gyro drift, turn and hop errors, noisy ultrasonic, a camera that misses balls), and the 'brain' that searches,
collects and returns to the bin using only what the real car has: gyro, ultrasonic, camera and the gripper sensor. No encoders."""
