This folder contains extra assets for the odometry, 3D field, and joystick views. For more details, see the "Custom Fields/Robots/Joysticks" page in the AdvantageScope documentation (available through the documentation tab in the app or the URL below).

https://docs.advantagescope.org/more-features/custom-assets

## Setup Instructions for Custom Robot Assets

To use the custom "2713 2025" robot CAD in AdvantageScope, you need to make the assets accessible to AdvantageScope:

### Option 1: Use Custom Assets Folder (Recommended)
This allows AdvantageScope to load assets directly from the repository without copying files:

1. Open AdvantageScope desktop app
2. Go to `App`/`AdvantageScope` > `Use Custom Assets Folder`
3. Select this directory: `/path/to/Robot2025/advantagescope/assets/`
   - Select the parent folder that contains the `Robot_2713` subfolder
4. Restart AdvantageScope if needed
5. Open a log file and go to the 3D Field view
6. In the robot selector dropdown, manually select "2713 2025"

### Option 2: Copy Assets to User Folder
Alternatively, copy the `Robot_2713` folder to your AdvantageScope user assets folder:
   - Windows: `C:\Users\<username>\Documents\AdvantageScope\userAssets\`
   - macOS: `~/Documents/AdvantageScope/userAssets/`
   - Linux: `~/AdvantageScope/userAssets/`

Then in AdvantageScope, go to the 3D Field view and manually select "2713 2025" from the robot dropdown.

### Components
The robot model includes 3 movable components that are animated based on the `componentPoses` logged in Robot.java:
- model_0.glb: Elevator (RobotContainer.elevator.pose)
- model_1.glb: Arm (RobotContainer.arm.pose)
- model_2.glb: Intake (RobotContainer.intake.pose)

### Important Notes
- AdvantageScope does NOT automatically select robots based on metadata
- You must manually select "2713 2025" from the robot dropdown in the 3D Field view
- The robot will only appear in the dropdown after configuring AdvantageScope to use this assets folder