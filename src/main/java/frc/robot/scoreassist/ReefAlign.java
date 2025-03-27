package frc.robot.scoreassist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.scoreassist.ScoreAssist.ScoreDrivingMode;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.RHRUtil;
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
        ScoreNode.A.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180))),
    new TrapezoidZone(
        new Point(3.647531509399414, 3.5130202770233154),
        new Point(4.498320579528809, 3.0688583850860596),
        2,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.C.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180))),
    new TrapezoidZone(
        new Point(4.498320579528809, 3.0688583850860596),
        new Point(5.305319309234619, 3.494252920150757),
        1.75,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.E.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180))),
    new TrapezoidZone(
        new Point(5.305319309234619, 3.494252920150757),
        new Point(5.305319309234619, 4.495181083679199),
        1.75,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.G.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180))),
    new TrapezoidZone(
        new Point(5.305319309234619, 4.495181083679199),
        new Point(4.498320579528809, 4.9768781661987305),
        1.75,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.I.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180))),
    new TrapezoidZone(
        new Point(4.498320579528809, 4.9768781661987305),
        new Point(3.595618486404419, 4.509187698364258),
        2,
        Rotation2d.fromDegrees(30),
        true,
        ScoreNode.K.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180)))
  };

  private boolean lastTriggerState = false;

  private ReefAlign() {}

  public void periodic() {
    Logger.recordOutput("ReefAlign/disabled", RobotContainer.disableReefAlign);
    // int index = 0;
    // for (TrapezoidZone trapezoid : trapezoids) {
    //   Logger.recordOutput("ReefAlign/trap" + (index++), trapezoid.toPose2dArray());
    // }

    boolean nextTriggerState =
        DriverStation.isTeleopEnabled()
            && RobotContainer.endEffector.hasCoral()
            && !RobotContainer.disableReefAlign
            && (RobotContainer.scoreAssist.mode == ScoreDrivingMode.INACTIVE)
            && inZone().isPresent();

    // Don't update the default command during ScoreAssist
    if (RobotContainer.scoreAssist.mode == ScoreDrivingMode.INACTIVE)
      this.lastTriggerState = nextTriggerState;
  }

  public boolean shouldDoReefAlign() {
    return this.lastTriggerState;
  }

  public Optional<Rotation2d> inZone() {
    var projection =
        RHRUtil.integrate(
            RobotContainer.driveSubsystem.getChassisSpeeds(),
            RobotContainer.driveSubsystem.getPose(),
            0.2);
    Logger.recordOutput("ReefAlign/mindReading", projection);
    for (TrapezoidZone trapezoid : trapezoids) {
      if (trapezoid.isPointInside(projection)) {
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
