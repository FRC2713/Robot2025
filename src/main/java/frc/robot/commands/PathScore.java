// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.ScoreNode;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathScore extends Command {
  private ScoreNode node;
  private Drivetrain drive;
  private Command pathfindingCommand;

  /** Creates a new AutoScore. */
  public PathScore(Drivetrain drive, ScoreNode node) {
    addRequirements(drive);
    this.drive = drive;
    this.node = node;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d targetPose = node.getAutoScorePose();
    Logger.recordOutput("ScoreAssit/PathScore/targetPose", targetPose);

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            2.0, 1.0, drive.getMaxAngularSpeedRadPerSec(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose, constraints, 0.4 // Goal end velocity in meters/sec
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
