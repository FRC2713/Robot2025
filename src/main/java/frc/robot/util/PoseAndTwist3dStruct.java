// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class PoseAndTwist3dStruct implements Struct<PoseAndTwist3d> {
  @Override
  public Class<PoseAndTwist3d> getTypeClass() {
    return PoseAndTwist3d.class;
  }

  @Override
  public String getTypeName() {
    return "PoseAndTwist3d";
  }

  @Override
  public int getSize() {
    return Pose3d.struct.getSize() + Twist3d.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "Pose3d pose;Twist3d twist";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Pose3d.struct, Twist3d.struct};
  }

  @Override
  public PoseAndTwist3d unpack(ByteBuffer bb) {
    Pose3d pose = Pose3d.struct.unpack(bb);
    Twist3d twist = Twist3d.struct.unpack(bb);
    return new PoseAndTwist3d(pose, twist);
  }

  @Override
  public void pack(ByteBuffer bb, PoseAndTwist3d value) {
    Pose3d.struct.pack(bb, value.getPose());
    Twist3d.struct.pack(bb, value.getTwist());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
