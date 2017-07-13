# hypestogram
Simple ROS node to view live image intensity histograms.

# Installation
Clone the repository with:

SSH: `git clone git@github.com:navganti/hypestogram.git`
HTTPS: `git clone https://github.com/navganti/hypestogram.git`

Clone either directly to your catkin workspace, or place there via a symlink.

# Build
Run either `catkin_make` or `catkin build` to build the package.

# Run
To run, you must specify the image topic for the histogram. This must be 
specified in the roslaunch command.

Example topic: `/camera1/image_raw`

Then the launch file can be run with:

`roslaunch hypestogram hypestogram.launch topic:=/camera1/image_raw`

``
