package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ScoreNode {
  // TODO: Ensure these represent the correct poses
  A(new Pose2d(2.884676694869995, 4.201192378997803, new Rotation2d(0))),
  B(new Pose2d(2.884676694869995, 3.8600285053253174, new Rotation2d(0))),

  C(new Pose2d(3.497980833053589, 2.7730469703674316, new Rotation2d(1.0335119235044878))),
  D(new Pose2d(3.845728874206543, 2.5586025714874268, new Rotation2d(1.0335119235044878))),

  E(new Pose2d(5.097620964050293, 2.547010898590088, new Rotation2d(2.1090184202465467))),
  F(new Pose2d(5.375819206237793, 2.709293365478515, new Rotation2d(2.1090184202465467))),

  G(new Pose2d(6.089637756347656, 3.8427934646606445, new Rotation2d(-Math.PI))),
  H(new Pose2d(6.08963775634765, 4.1608147621154785, new Rotation2d(-Math.PI))),

  I(new Pose2d(5.444761276245117, 5.318058490753174, new Rotation2d(-2.0899421642315534))),
  J(new Pose2d(5.1355743408203125, 5.47706937789917, new Rotation2d(-2.0899421642315534))),

  K(new Pose2d(3.881157159805298, 5.494737148284912, new Rotation2d(-1.037952202199093))),
  L(new Pose2d(3.571969747543335, 5.326892375946045, new Rotation2d(-1.037952202199093)));

  private Pose2d pose;

  ScoreNode(Pose2d pose) {
    this.pose = pose;
  }

  public Pose2d getPose() {
    return pose;
  }
}
