This folder contains extra assets for the odometry, 3D field, and joystick views. For more details, see the "Custom Fields/Robots/Joysticks" page in the AdvantageScope documentation (available through the documentation tab in the app or the URL below).

https://docs.advantagescope.org/more-features/custom-assets

## Setup Instructions for Custom Robot Assets

To use the custom robot CAD in AdvantageScope:

### Option 1: Using the AdvantageScope Web App
The robot code logs the metadata `RobotConfig: "2713 2025"` which tells AdvantageScope which robot configuration to use. When viewing logs in the web app, AdvantageScope will look for the robot assets in this repository folder.

### Option 2: Using the Desktop App
1. Copy the `Robot_2713` folder from this directory to your AdvantageScope user assets folder:
   - Windows: `C:\Users\<username>\Documents\AdvantageScope\userAssets\Robots\`
   - macOS: `~/Documents/AdvantageScope/userAssets/Robots/`
   - Linux: `~/AdvantageScope/userAssets/Robots/`

2. The robot code already logs the required metadata: `Logger.recordMetadata("RobotConfig", "2713 2025");`

3. When you open a log file, go to the 3D Field view, and the "2713 2025" robot should appear in the robot selector dropdown.

### Components
The robot model includes 3 movable components:
- model_0.glb: Elevator
- model_1.glb: Arm  
- model_2.glb: Intake

These components are animated based on the `componentPoses` logged in Robot.java.