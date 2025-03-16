package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TrapezoidZone;
import frc.robot.util.TrapezoidZone.Point;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class SourceAlign {
  private static SourceAlign INSTANCE = null;
  private TrapezoidZone[] trapezoids = {
    new TrapezoidZone(
        new Point(0, 6.810469627380371),
        new Point(1.7274636030197144, 7.995840072631836),
        1,
        Rotation2d.fromDegrees(30),
        true,
        Rotation2d.fromDegrees(-55)),
    new TrapezoidZone(
        new Point(0, 1.1331689357757568),
        new Point(1.633881688117981, 0),
        1,
        Rotation2d.fromDegrees(30),
        false,
        Rotation2d.fromDegrees(55))
  };

  private SourceAlign() {}

  public void periodic() {
    // Logger.recordOutput("ReefAlign/disabled", RobotContainer.disableReefAlign);
    int index = 0;
    for (TrapezoidZone trapezoid : trapezoids) {
      Logger.recordOutput("SourceAlign/trap" + (index++), trapezoid.toPose2dArray());
    }
  }

  public boolean shouldDoSourceAlign() {
    return DriverStation.isTeleopEnabled()
        && !RobotContainer.disableSourceAlign
        && inZone().isPresent();
  }

  public Optional<Rotation2d> inZone() {
    var pose = RobotContainer.driveSubsystem.getPose();
    for (TrapezoidZone trapezoid : trapezoids) {
      if (trapezoid.isPointInside(pose)) {
        var commandedAngle = AllianceFlipUtil.apply(trapezoid.getOmega());
        Logger.recordOutput("SourceAlign/commandedAngle", commandedAngle);
        return Optional.of(commandedAngle);
      }
    }
    return Optional.empty();
  }

  public static SourceAlign getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SourceAlign();
    }
    return INSTANCE;
  }
}
