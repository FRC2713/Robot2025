package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import lombok.Getter;

public class PoseAndTwist3d implements Sendable {

  @Getter public Pose3d pose;
  @Getter public Twist3d twist;

  public PoseAndTwist3d() {
    this(new Pose3d(), new Twist3d());
  }

  public PoseAndTwist3d(Pose3d pose, Twist3d twist) {
    this.pose = pose;
    this.twist = twist;
  }

  public static PoseAndTwist3d from(Pose2d pose, ChassisSpeeds speeds) {
    var inst = new PoseAndTwist3d();
    inst.update(pose, speeds);
    return inst;
  }

  public void update(Pose2d pose, ChassisSpeeds speeds) {
    this.pose = new Pose3d(pose);
    this.twist =
        new Twist3d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            0,
            0,
            0,
            speeds.omegaRadiansPerSecond);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PoseAndTwist3d");
    builder.addDoubleProperty("pose/translation/x", pose::getX, null);
    builder.addDoubleProperty("pose/translation/y", pose::getY, null);
    builder.addDoubleProperty("pose/translation/z", pose::getZ, null);
    var quaternion = pose.getRotation().getQuaternion();
    builder.addDoubleProperty("pose/rotation/x", quaternion::getX, null);
    builder.addDoubleProperty("pose/rotation/y", quaternion::getY, null);
    builder.addDoubleProperty("pose/rotation/z", quaternion::getZ, null);
    builder.addDoubleProperty("pose/rotation/w", quaternion::getW, null);

    builder.addDoubleProperty("twist/linear/x", () -> twist.dx, null);
    builder.addDoubleProperty("twist/linear/y", () -> twist.dy, null);
    builder.addDoubleProperty("twist/linear/z", () -> twist.dz, null);
    builder.addDoubleProperty("twist/angular/x", () -> twist.rx, null);
    builder.addDoubleProperty("twist/angular/y", () -> twist.ry, null);
    builder.addDoubleProperty("twist/angular/z", () -> twist.rz, null);
  }

  public static final PoseAndTwist3dStruct struct = new PoseAndTwist3dStruct();
}
