// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoreassist;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PathfindToPose extends Command {
  private Supplier<Pose2d> node;
  private Drivetrain drive;
  private Command pathfindingCommand;
  private LoggedTunableNumber pathConstraintVelocityMPS =
      new LoggedTunableNumber("ScoreAssist/Pathfind/maxVelocityMPS", 2.5);
  private LoggedTunableNumber pathConstraintaccel =
      new LoggedTunableNumber("ScoreAssist/Pathfind/constraintAccel", 4.0);

  /** Creates a new PathScore. */
  public PathfindToPose(Drivetrain drive, Supplier<Pose2d> node) {
    addRequirements(drive);
    this.drive = drive;
    this.node = node;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d targetPose = node.get();
    Logger.recordOutput("ScoreAssist/Pathfind/targetPose", targetPose);

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            pathConstraintVelocityMPS.getAsDouble(),
            pathConstraintaccel.getAsDouble(),
            drive.getMaxAngularSpeedRadPerSec(),
            Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose, constraints, 0.2 // Goal end velocity in meters/sec
            );
    pathfindingCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathfindingCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pathfindingCommand != null) pathfindingCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}
