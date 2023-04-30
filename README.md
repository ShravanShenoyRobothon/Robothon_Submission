# Robothon_Submission

This is the submission from Team IISc, Bangalore to Robothon Challenge 2023. 

Hardware:
UR5 Manipulator Arm,
AG-95 Gripper,
Intel RealSense d435i Depth Camera

Software:
ROS Noetic,
Pyrealsense,
MoveIt,
OpenCV,
OpenCV Aruco,
Pytorch and Torchvision,
CUDA Toolkit,
YOLOv7 Utility files - https://github.com/WongKinYiu/yolov7,
Rospy,
Roslib,
Pyttsx3x


Quickstart Section:
1. Run the launch files of the arm and gripper to calibrate and set-up communication between robot and host.  
2. In the current repository, run the final_submission.py script. 
3. The best weights for the trained YOLOv7 model are in the file named 'yolov7-custom.pt' in the main branch. 
4. CUDA Toolkit and CuDNN versions are dependent on the graphics cards installed in machine being used, but we hae installed NVIDIA Toolkit version 11.4 and CuDNN version 8.2.4 for a RTX 3080 Laptop GPU.
5. Home location is pre-programmed but can be changed. 
6. It searches for the CV2 arUco marker present on the board that is identified by a matrix and number of unique markers. ArUco generator link - https://chev.me/arucogen/. We have used a 4*4 arUco marker, ID can be anything. 
7. CV2 Windows will pop up on every loop iteration as we scan the board before each task to get depth and bounding box coordinated accurately. 
8. Speech prompts are given on the start and completion of each task in order to make it easier to understand. 

Thank you!
