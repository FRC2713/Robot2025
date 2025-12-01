package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.DriveConstants;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public enum ScoreNode {
  // indexes start at right branch facing driver station and move clockwise, which means that
  // B is zero
  A(FieldConstants.Reef.branchPositions2d.get(1).get(ReefLevel.L2), ScoreLocType.CORAL),
  AB_ALGAE(FieldConstants.Reef.centerFaces[0], ScoreLocType.ALGAE_L3),
  B(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L2), ScoreLocType.CORAL),

  C(FieldConstants.Reef.branchPositions2d.get(11).get(ReefLevel.L2), ScoreLocType.CORAL),
  CD_ALGAE(FieldConstants.Reef.centerFaces[5], ScoreLocType.ALGAE_L2),
  D(FieldConstants.Reef.branchPositions2d.get(10).get(ReefLevel.L2), ScoreLocType.CORAL),

  E(FieldConstants.Reef.branchPositions2d.get(9).get(ReefLevel.L2), ScoreLocType.CORAL),
  EF_ALGAE(FieldConstants.Reef.centerFaces[4], ScoreLocType.ALGAE_L3),
  F(FieldConstants.Reef.branchPositions2d.get(8).get(ReefLevel.L2), ScoreLocType.CORAL),

  G(FieldConstants.Reef.branchPositions2d.get(7).get(ReefLevel.L2), ScoreLocType.CORAL),
  GH_ALGAE(FieldConstants.Reef.centerFaces[3], ScoreLocType.ALGAE_L2),
  H(FieldConstants.Reef.branchPositions2d.get(6).get(ReefLevel.L2), ScoreLocType.CORAL),

  I(FieldConstants.Reef.branchPositions2d.get(5).get(ReefLevel.L2), ScoreLocType.CORAL),
  IJ_ALGAE(FieldConstants.Reef.centerFaces[2], ScoreLocType.ALGAE_L3),
  J(FieldConstants.Reef.branchPositions2d.get(4).get(ReefLevel.L2), ScoreLocType.CORAL),

  K(FieldConstants.Reef.branchPositions2d.get(3).get(ReefLevel.L2), ScoreLocType.CORAL),
  KL_ALGAE(FieldConstants.Reef.centerFaces[1], ScoreLocType.ALGAE_L2),
  L(FieldConstants.Reef.branchPositions2d.get(2).get(ReefLevel.L2), ScoreLocType.CORAL);

  private Pose2d pose;
  @Getter private ScoreLocType type;

  ScoreNode(Pose2d pose, ScoreLocType type) {
    this.pose = pose;
    this.type = type;
  }

  public Pose2d getPose() {
    return pose;
  }

  public double yOffset() {
    if (type == ScoreLocType.CORAL) {
      return (DriveConstants.driveBaseWidthWithBumpersMeters / 2.0);
    } else if (type == ScoreLocType.ALGAE_L2) {
      return (DriveConstants.driveBaseWidthWithBumpersMeters / 2.0)
          + DriveConstants.OFFSET_ALGAE_L2.getAsDouble();
    } else {
      return (DriveConstants.driveBaseWidthWithBumpersMeters / 2.0)
          + DriveConstants.OFFSET_ALGAE_L3.getAsDouble();
    }
  }

  public Pose2d getRobotAlignmentPose() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    double AUTON_MODE_OFFSET = DriverStation.isAutonomous() ? Units.inchesToMeters(.5) : 0;
    Logger.recordOutput("ScoreAssist/AUTON_MODE_OFFSET", AUTON_MODE_OFFSET);

    Transform2d robotOffset =
        new Transform2d(
            yOffset() + AUTON_MODE_OFFSET,
            DriveConstants.coralOffsetFromCenter
                .getAsDouble(), // offset of scoring mechanism from center of robot
            new Rotation2d(Math.PI));
    Pose2d alignmentPose = pose.transformBy(robotOffset);

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      alignmentPose = AllianceFlipUtil.flip(alignmentPose);
    }

    // Calculate the flipped angle (adds/subtracts 180 degrees)
    // The Rotation2d.plus() handles normalization automatically.
    Rotation2d flippedTargetAngle = alignmentPose.getRotation().plus(Rotation2d.fromDegrees(180));
    var currentAngle = RobotContainer.driveSubsystem.getPose().getRotation();
    // --- Calculate the Differences ---
    // difference1 will be between -180 and 180 degrees (shortest path)
    Rotation2d difference1 = alignmentPose.getRotation().minus(currentAngle);
    double angleToTarget = difference1.getDegrees();

    // difference2 will also be between -180 and 180 degrees (shortest path)
    Rotation2d difference2 = flippedTargetAngle.minus(currentAngle);
    double angleToFlippedTarget = difference2.getDegrees();

    // --- Compare the absolute differences to find the closest ---
    if (Math.abs(angleToTarget) < Math.abs(angleToFlippedTarget)) {
      // System.out.println("The original target angle is closer.");
      // The angle you should turn is 'angleToTarget'
      // You would use 'targetAngle' as your final setpoint
    } else {
      // System.out.println("The flipped target angle is closer.");
      alignmentPose =
          new Pose2d(
              alignmentPose.getTranslation(), alignmentPose.getRotation().minus(Rotation2d.kPi));
      // The angle you should turn is 'angleToFlippedTarget'
      // You would use 'flippedTargetAngle' as your final setpoint
    }

    return alignmentPose;
  }

  public boolean isFrontFacingReef() {
    var transform =
        new Transform2d(
            FieldConstants.Reef.center.getX(), FieldConstants.Reef.center.getY(), new Rotation2d());

    return false;
  }

  public Pose2d getPathScorePose() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    double AUTO_SCORE_OFFSET = 0.5;

    Transform2d robotOffset =
        new Transform2d(
            yOffset() + AUTO_SCORE_OFFSET,
            DriveConstants.coralOffsetFromCenter
                .getAsDouble(), // offset of scoring mechanism from center of robot
            new Rotation2d(Math.PI));
    Pose2d autoScorePose = pose.transformBy(robotOffset);
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return AllianceFlipUtil.flip(autoScorePose);
    }
    return autoScorePose;
  }
}
