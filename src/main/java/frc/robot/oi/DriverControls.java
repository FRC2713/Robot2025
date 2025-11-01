package frc.robot.oi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import frc.robot.commands.ArmCmds;
import frc.robot.commands.DriveCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.IntakeCmds;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.scoreassist.ReefAlign;
import frc.robot.scoreassist.SourceAlign;
import frc.robot.util.RHRUtil;
import frc.robot.util.ReefTracker;
import frc.robot.util.ScoreAssistMessage.GoalType;
import frc.robot.util.ScoreLevel;
import java.util.HashMap;

public class DriverControls {
  private final CommandXboxController driver = new CommandXboxController(0);

  private Trigger reefAlignTrigger = new Trigger(ReefAlign.getInstance()::shouldDoReefAlign);
  private Trigger sourceAlignTrigger = new Trigger(SourceAlign.getInstance()::shouldDoSourceAlign);

  private Trigger prepProcessorTrigger =
      new Trigger(() -> ReefTracker.getInstance().getGoalTypeOrCoral() == GoalType.PROCESSOR);
  private Trigger prepBargeTrigger =
      new Trigger(() -> ReefTracker.getInstance().getGoalTypeOrCoral() == GoalType.BARGE);

  public void configureTriggers() {
    // reefAlignTrigger.onTrue(this.setToReefAlignCmd()).onFalse(this.setToNormalDriveCmd());
    // sourceAlignTrigger.onTrue(this.setToSourceAlignCmd()).onFalse(this.setToNormalDriveCmd());

    // prepBargeTrigger.onTrue(
    //     Commands.parallel(
    //         EndEffector.ALGAE_HOLD.get(),
    //         Commands.either(
    //             SuperStructure.BARGE_PREP_BACKWARDS.get(),
    //             SuperStructure.BARGE_PREP_FORWARDS.get(),
    //             () -> DriveAtLongitude.doBackwards(ScoreAssistConstants.bargeAlignmentX))));
    // prepProcessorTrigger.onTrue(SuperStructure.PROCESSOR_PREP.get());
  }

  public void configureButtonBindings() {

    // Reset gyro to 0 deg when start button is pressed
    driver
        .start()
        .onTrue(
            Commands.parallel(
                this.setToNormalDriveCmd(),
                Commands.runOnce(
                        () ->
                            RobotContainer.driveSubsystem.setPose(
                                new Pose2d(
                                    RobotContainer.driveSubsystem.getPose().getTranslation(),
                                    Rotation2d.fromDegrees(0))),
                        RobotContainer.driveSubsystem)
                    .ignoringDisable(true)));

    // Reset gyro to 180 deg when start button is pressed
    driver
        .back()
        .onTrue(
            Commands.parallel(
                    this.setToNormalDriveCmd(),
                    Commands.runOnce(
                        () ->
                            RobotContainer.driveSubsystem.setPose(
                                new Pose2d(
                                    RobotContainer.driveSubsystem.getPose().getTranslation(),
                                    Rotation2d.fromDegrees(180))),
                        RobotContainer.driveSubsystem))
                .ignoringDisable(true));

    // POV Precision Driving
    driver
        .povLeft()
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCmds.inch(RobotContainer.driveSubsystem, SetpointConstants.Drive.INCH_SPEED),
                "Inch Left"))
        .onFalse(this.setToNormalDriveCmd());
    driver
        .povRight()
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCmds.inch(
                    RobotContainer.driveSubsystem,
                    () -> -1 * SetpointConstants.Drive.INCH_SPEED.getAsDouble()),
                "Inch Right"))
        .onFalse(this.setToNormalDriveCmd());

    driver.a().onTrue(SuperStructure.L1.get());
    driver
        .b()
        .onTrue(
            Commands.either(
                SuperStructure.L3.get(),
                SuperStructure.L3_FLIPPED.get(),
                RHRUtil::shouldFlipSuperStructure));

    driver
        .y()
        .onTrue(
            Commands.either(
                SuperStructure.L4.get(),
                SuperStructure.L4_FLIPPED.get(),
                RHRUtil::shouldFlipSuperStructure));
    driver
        .leftBumper()
        .whileTrue(SuperStructure.CORAL_GRAB_GROUND.get())
        .onFalse(SuperStructure.STARTING_CONF.get());
    driver.povDown().onTrue(SuperStructure.PRE_DISABLE_CHECK.get());
    driver.rightTrigger(.25).onTrue(ArmCmds.handSetVoltage(12));
    // driver
    //     .leftTrigger(.25)
    //     .onTrue(SuperStructure.ALGAE_INTAKE.get())
    //     .onFalse(SuperStructure.ALGAE_CONF.get());

    var commands = new HashMap<ScoreLevel, Command>();
    commands.put(ScoreLevel.ONE, Commands.sequence(IntakeCmds.setVolts(-5)));
    commands.put(
        ScoreLevel.THREE,
        Commands.sequence(
            Commands.either(
                ArmCmds.armSetAngle(13),
                ArmCmds.armSetAngle(() -> ArmCmds.reflectArm(13)),
                () -> RobotContainer.isFLIPPED),
            ArmCmds.handSetVoltage(2)));
    commands.put(
        ScoreLevel.FOUR,
        Commands.sequence(
            ElevatorCmds.setHeight(24),
            Commands.parallel(
                Commands.either(
                    ArmCmds.armSetAngleAndWait(SetpointConstants.Arm.L4_ANGLE_DEG_SCORE),
                    ArmCmds.armSetAngleAndWait(
                        () -> ArmCmds.reflectArm(SetpointConstants.Arm.L4_ANGLE_DEG_SCORE.get())),
                    () -> RobotContainer.isFLIPPED),
                ArmCmds.handSetVoltage(2)),
            Commands.parallel(
                ArmCmds.armSetAngleAndWait(90),
                ElevatorCmds.setHeightAndWait(
                    SetpointConstants.Elevator.ELEVATOR_HANDOFF_HEIGHT))));

    driver.rightBumper().onTrue(Commands.select(commands, () -> RobotContainer.scoreLevel));
  }

  public double getLeftY() {
    return driver.getLeftY();
  }

  public double getLeftX() {
    return driver.getLeftX();
  }

  public double getRightX() {
    return driver.getRightX();
  }

  public double getRightY() {
    return driver.getRightY();
  }

  public void setToNormalDrive() {
    DriveCmds.setDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.normalDriveCmd(), "Default Joystick Drive");
  }

  public Command setToNormalDriveCmd() {
    return DriveCmds.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.normalDriveCmd(), "Default Joystick Drive");
  }

  public Command stopWithX() {
    return DriveCmds.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem,
        new InstantCommand(() -> RobotContainer.driveSubsystem.stopWithX()),
        "Stop With X");
  }

  private Command normalDriveCmd() {
    return DriveCmds.joystickDrive(
        RobotContainer.driveSubsystem,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX());
  }

  public void setToReefAlign() {
    DriveCmds.setDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.reefAlignCmd(), "Align To Reef");
  }

  public Command setToReefAlignCmd() {
    return DriveCmds.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.reefAlignCmd(), "Align To Reef");
  }

  private Command reefAlignCmd() {
    return DriveCmds.joystickDriveAtAngle(
        RobotContainer.driveSubsystem,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> ReefAlign.getInstance().inZone().orElse(RobotContainer.driveSubsystem.getRotation()));
  }

  public void setToSourceAlign() {
    DriveCmds.setDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.sourceAlignCmd(), "Align To Source");
  }

  public Command setToSourceAlignCmd() {
    return DriveCmds.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.sourceAlignCmd(), "Align To Source");
  }

  private Command sourceAlignCmd() {
    return DriveCmds.joystickDriveAtAngle(
        RobotContainer.driveSubsystem,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () ->
            SourceAlign.getInstance().inZone().orElse(RobotContainer.driveSubsystem.getRotation()));
  }
}
