package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.constants.DriveConstants;
import java.util.Optional;

public enum ScoreNode {
  // indexes start at right branch facing driver station and move clockwise, which means that
  // B is zero
  A(FieldConstants.Reef.branchPositions2d.get(1).get(ReefLevel.L2)),
  B(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L2)),

  C(FieldConstants.Reef.branchPositions2d.get(11).get(ReefLevel.L2)),
  D(FieldConstants.Reef.branchPositions2d.get(10).get(ReefLevel.L2)),

  E(FieldConstants.Reef.branchPositions2d.get(9).get(ReefLevel.L2)),
  F(FieldConstants.Reef.branchPositions2d.get(8).get(ReefLevel.L2)),

  G(FieldConstants.Reef.branchPositions2d.get(7).get(ReefLevel.L2)),
  H(FieldConstants.Reef.branchPositions2d.get(6).get(ReefLevel.L2)),

  I(FieldConstants.Reef.branchPositions2d.get(5).get(ReefLevel.L2)),
  J(FieldConstants.Reef.branchPositions2d.get(4).get(ReefLevel.L2)),

  K(FieldConstants.Reef.branchPositions2d.get(3).get(ReefLevel.L2)),
  L(FieldConstants.Reef.branchPositions2d.get(2).get(ReefLevel.L2));

  private Pose2d pose;

  ScoreNode(Pose2d pose) {
    this.pose = pose;
  }

  public Pose2d getPose() {
    return pose;
  }

  public Pose2d getRobotAlignmentPose() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    Transform2d robotOffset =
        new Transform2d(
            (DriveConstants.driveBaseWidthWithBumpersMeters / 2.0),
            DriveConstants.coralOffsetFromCenter
                .getAsDouble(), // offset of scoring mechanism from center of robot
            new Rotation2d(Math.PI));
    Pose2d alignmentPose = pose.transformBy(robotOffset);
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return AllianceFlipUtil.flip(alignmentPose);
    }
    return alignmentPose;
  }

  public Pose2d getPathScorePose() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    double AUTO_SCORE_OFFSET = 0.5;

    Transform2d robotOffset =
        new Transform2d(
            (DriveConstants.driveBaseWidthWithBumpersMeters / 2.0) + AUTO_SCORE_OFFSET,
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
