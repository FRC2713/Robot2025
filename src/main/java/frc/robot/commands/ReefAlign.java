package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ScoreNode;
import frc.robot.util.TrapezoidZone;
import frc.robot.util.TrapezoidZone.Point;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ReefAlign {
  private static ReefAlign INSTANCE = null;
  private TrapezoidZone[] trapezoids = {
    new TrapezoidZone(
        new Point(3.618733263015747, 3.5519957542419434),
        new Point(3.618733263015747, 4.458786964416504),
        2,
        Rotation2d.fromDegrees(30),
        false,
        ScoreNode.A.getPose().getRotation()),
    new TrapezoidZone(
        new Point(3.647531509399414, 3.5130202770233154),
        new Point(4.498320579528809, 3.0688583850860596),
        2,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.C.getPose().getRotation()),
    new TrapezoidZone(
        new Point(4.498320579528809, 3.0688583850860596),
        new Point(5.305319309234619, 3.494252920150757),
        1.5,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.E.getPose().getRotation()),
    new TrapezoidZone(
        new Point(5.305319309234619, 3.494252920150757),
        new Point(5.305319309234619, 4.495181083679199),
        1,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.G.getPose().getRotation()),
    new TrapezoidZone(
        new Point(5.305319309234619, 4.495181083679199),
        new Point(4.498320579528809, 4.9768781661987305),
        1.5,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.I.getPose().getRotation()),
    new TrapezoidZone(
        new Point(4.498320579528809, 4.9768781661987305),
        new Point(3.595618486404419, 4.509187698364258),
        2,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.K.getPose().getRotation())
  };

  private ReefAlign() {}

  public void periodic() {
    Logger.recordOutput("ReefAlign/disabled", RobotContainer.disableReefAlign);
    int index = 0;
    for (TrapezoidZone trapezoid : trapezoids) {
      Logger.recordOutput("ReefAlign/trap" + (index++), trapezoid.toPose2dArray());
    }
  }

  public boolean shouldDoReefAlign() {
    return DriverStation.isTeleopEnabled()
        && RobotContainer.rollers.hasCoral()
        && !RobotContainer.disableReefAlign
        && inZone().isPresent();
  }

  public Optional<Rotation2d> inZone() {
    var pose = RobotContainer.driveSubsystem.getPose();
    for (TrapezoidZone trapezoid : trapezoids) {
      if (trapezoid.isPointInside(pose)) {
        var commandedAngle = AllianceFlipUtil.apply(trapezoid.getOmega());
        Logger.recordOutput("ReefAlign/commandedAngle", commandedAngle);
        return Optional.of(commandedAngle);
      }
    }
    return Optional.empty();
  }

  public static ReefAlign getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ReefAlign();
    }
    return INSTANCE;
  }
}
