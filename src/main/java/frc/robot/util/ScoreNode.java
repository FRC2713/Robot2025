package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.constants.DriveConstants;

public enum ScoreNode {
  // TODO: Ensure these represent the correct poses
  // indexes start at right branch facing driver station and move counterclockwise, which means that
  // B is
  // zero
  A(
      FieldConstants.Reef.branchPositions2d
          .get(1)
          .get(ReefLevel.L2)
          .transformBy(
              new Transform2d(FieldConstants.Reef.centerToFaceCenters[0], new Rotation2d())
                  .div(FieldConstants.Reef.centerToFaceCenters[0].getNorm())
                  .times(DriveConstants.driveBaseWidthWithBumpersMeters / 2.0))),
  B(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L2)),
  // TODO: Do what we did for A for the other points
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
}
