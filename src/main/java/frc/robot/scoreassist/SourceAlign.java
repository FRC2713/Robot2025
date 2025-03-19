package frc.robot.scoreassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.scoreassist.ScoreAssist.ScoreDrivingMode;
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

  public static Pose2d centeredLeft = new Pose2d(
                        new Translation2d(0.4480087459087372, 1.305626749992370),
                        Rotation2d.fromRadians(0.996491486039043)); 

  private boolean lastTriggerState = false;

  private SourceAlign() {}

  public void periodic() {
    // Logger.recordOutput("ReefAlign/disabled", RobotContainer.disableReefAlign);
    int index = 0;
    for (TrapezoidZone trapezoid : trapezoids) {
      Logger.recordOutput("SourceAlign/trap" + (index++), trapezoid.toPose2dArray());
    }

    boolean nextTriggerState =
        DriverStation.isTeleopEnabled()
            && !RobotContainer.disableSourceAlign
            && inZone().isPresent();

    // Don't update the default command during ScoreAssist
    if (RobotContainer.scoreAssist.mode == ScoreDrivingMode.INACTIVE)
      this.lastTriggerState = nextTriggerState;
  }

  public boolean shouldDoSourceAlign() {
    return lastTriggerState;
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
