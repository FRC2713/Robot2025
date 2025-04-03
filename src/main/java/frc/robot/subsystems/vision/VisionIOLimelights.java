package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.VisionConstants;
import frc.robot.subsystems.constants.VisionConstants.VisionOptions;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightInfo;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelights implements VisionIO {
  private final LimelightInfo primaryInfo;
  private final LimelightInfo secondaryInfo;

  public VisionInputs inputs = new VisionInputsAutoLogged();

  private CombinedMegaTagState state;
  private StddevCalculationState stddevCalcState;
  private Alert alert = new Alert("Null Limelight pose", AlertType.kWarning);
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("slamdunk");
  private final double[] defaultPose = {0, 0, 0, 0, 0, 0, 0, -1};

  public VisionIOLimelights(LimelightInfo primary, LimelightInfo secondary, Drivetrain drivetrain) {
    primary.setCameraPose_RobotSpace();
    secondary.setCameraPose_RobotSpace();

    LimelightHelpers.SetRobotOrientation(
        secondary.getNtTableName(),
        RobotContainer.driveSubsystem.getPose().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        primary.getNtTableName(),
        RobotContainer.driveSubsystem.getPose().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);

    this.primaryInfo = primary;
    this.secondaryInfo = secondary;
    this.state = CombinedMegaTagState.INIT;
    this.stddevCalcState = StddevCalculationState.INVALID_VISION_STATE;
  }

  @Override
  public void update() {

    var slamdunkposeArray = table.getEntry("pose").getDoubleArray(defaultPose);
    var slamdunkpose =
        new Pose3d(
            new Translation3d(slamdunkposeArray[0], slamdunkposeArray[1], slamdunkposeArray[2]),
            new Rotation3d(
                new Quaternion(
                    slamdunkposeArray[6],
                    slamdunkposeArray[3],
                    slamdunkposeArray[4],
                    slamdunkposeArray[5])));
    Logger.recordOutput("Odometry/Vision3d", slamdunkpose);

    var slamdunktime = slamdunkposeArray[7] / 1e6;
    var timeDiff = (Logger.getTimestamp() / 1e6) - slamdunktime;
    Logger.recordOutput("Vision/timeDiffSec", timeDiff);


    if (RobotContainer.driveSubsystem.getAngularVelocityRadPerSec() > Units.degreesToRadians(720)) {
      this.state = CombinedMegaTagState.REJECTED_DUE_TO_SPIN_BLUR;
    }

    LimelightHelpers.PoseEstimate primaryMT = null;
    LimelightHelpers.PoseEstimate secondaryMT = null;

    // 1) Choose between MegaTag1 and MegaTag2
    if (VisionConstants.ACTIVE_VISION_OPTION == VisionConstants.VisionOptions.MEGATAG2
        || VisionConstants.ACTIVE_VISION_OPTION
            == VisionConstants.VisionOptions.SLAMDUNK_MEGATAG2_MERGED) {

      // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
      // docs say this needs to be called every frame
      LimelightHelpers.SetRobotOrientation(
          primaryInfo.getNtTableName(),
          RobotContainer.driveSubsystem.getPose().getRotation().getDegrees(),
          0,
          0,
          0,
          0,
          0);
      LimelightHelpers.SetRobotOrientation(
          secondaryInfo.getNtTableName(),
          RobotContainer.driveSubsystem.getPose().getRotation().getDegrees(),
          0,
          0,
          0,
          0,
          0);

      primaryMT =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(primaryInfo.getNtTableName());
      secondaryMT =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(secondaryInfo.getNtTableName());
      Logger.recordOutput("Vision/version", "MegaTag2");
    } else {
      primaryMT = LimelightHelpers.getBotPoseEstimate_wpiBlue(primaryInfo.getNtTableName());
      secondaryMT = LimelightHelpers.getBotPoseEstimate_wpiBlue(secondaryInfo.getNtTableName());
      Logger.recordOutput("Vision/version", "MegaTag1");
    }

    if (primaryMT == null) {
      primaryMT = new LimelightHelpers.PoseEstimate();
      alert.set(true);
    }

    if (secondaryMT == null) {
      secondaryMT = new LimelightHelpers.PoseEstimate();
      alert.set(true);
    }

    if (primaryMT.tagCount > 0) {
      this.state = CombinedMegaTagState.UPDATED_WITH_PRIMARY;
    } else if (secondaryMT.tagCount > 0) {
      // uncomment to actually use secondary
      // this.state = CombinedMegaTagState.UPDATED_WITH_SECONDARY;
      this.state = CombinedMegaTagState.REJECTED_DUE_TO_NO_TAGS;
    } else {
      this.state = CombinedMegaTagState.REJECTED_DUE_TO_NO_TAGS;
    }

    if (this.state == CombinedMegaTagState.UPDATED_WITH_PRIMARY) {
      this.inputs.pose = primaryMT.pose;
      this.inputs.timestamp = primaryMT.timestampSeconds;
      this.inputs.tagCount = primaryMT.tagCount;
      this.inputs.averagTagDistance = primaryMT.avgTagDist;
      this.inputs.averageTagSize = primaryMT.avgTagArea;
    } else if (this.state == CombinedMegaTagState.UPDATED_WITH_SECONDARY) {
      this.inputs.pose = secondaryMT.pose;
      this.inputs.timestamp = secondaryMT.timestampSeconds;
      this.inputs.tagCount = secondaryMT.tagCount;
      this.inputs.averagTagDistance = secondaryMT.avgTagDist;
      this.inputs.averageTagSize = secondaryMT.avgTagArea;
    } else {
      // this.inputs.pose = new Pose2d();
      this.inputs.tagCount = 0;
    }

    if (primaryMT != null) {
      Logger.recordOutput("Vision/" + primaryInfo.getNtTableName() + "/pose", primaryMT.pose);
      Logger.recordOutput(
          "Vision/" + primaryInfo.getNtTableName() + "/tagCount", primaryMT.tagCount);
      Logger.recordOutput(
          "Vision/" + primaryInfo.getNtTableName() + "/timestamp", primaryMT.timestampSeconds);
    }

    if (secondaryMT != null) {
      Logger.recordOutput("Vision/" + secondaryInfo.getNtTableName() + "/pose", secondaryMT.pose);
      Logger.recordOutput(
          "Vision/" + secondaryInfo.getNtTableName() + "/tagCount", secondaryMT.tagCount);
      Logger.recordOutput(
          "Vision/" + secondaryInfo.getNtTableName() + "/timestamp", secondaryMT.timestampSeconds);
    }
  }

  @Override
  public void updatePoseEstimate(SwerveDrivePoseEstimator poseEstimator) {
    updatePoseEstimate(
        poseEstimator,
        VecBuilder.fill(
            this.inputs.translationStddev,
            this.inputs.translationStddev,
            this.inputs.rotationStddev));
  }

  public void updatePoseEstimate(SwerveDrivePoseEstimator poseEstimator, Vector<N3> stdDevs) {
    this.updateStddevs();
    if (this.stddevCalcState.applyToPoseEstimate()) {
      poseEstimator.setVisionMeasurementStdDevs(stdDevs);
      poseEstimator.addVisionMeasurement(this.inputs.pose, this.inputs.timestamp);
    }
  }

  /**
   * With MegaTag1 and 2, there are certain conditions where you should not use the pose for pose
   * estimation, or use it with a higher standard deviation
   *
   * <p>1) If using MegaTag 2, always use the pose and apply a std deviation based on the difference
   * on the pose's rotation and the gyros 2) If using MegaTag1 and you see no tags, do not use the
   * pose 3) If using MegaTag1 and you see 2 or more poses, use the pose with low stddevs 4) If
   * using MegaTag1 and you see 1 pose that is large/close, use the pose with medium stddevs 5) If
   * using MegaTag1 and you see 1 pose that is far, use the pose with high stddevs 6) If using
   * MegaTag1 and you are disbabled but see a tag, use the the pose with high stddevs
   */
  private void updateStddevs() {
    this.stddevCalcState = StddevCalculationState.INVALID_VISION_STATE;
    this.inputs.translationStddev = 0.01;
    this.inputs.rotationStddev = 9999999;

    if (this.state == CombinedMegaTagState.UPDATED_WITH_PRIMARY
        || this.state == CombinedMegaTagState.UPDATED_WITH_SECONDARY) {
      if ((VisionConstants.ACTIVE_VISION_OPTION == VisionOptions.MEGATAG2
              || VisionConstants.ACTIVE_VISION_OPTION == VisionOptions.SLAMDUNK_MEGATAG2_MERGED)
          && this.inputs.tagCount > 0) {
        this.inputs.translationStddev =
            MathUtil.interpolate(
                0.1,
                5.6,
                Math.abs(
                    RobotContainer.visionsubsystem.getPose().getRotation().getDegrees()
                        - this.getPose().getRotation().getDegrees() / 5.0));
        this.stddevCalcState = StddevCalculationState.MT2_ROTATION_BASED;
      } else if (VisionConstants.ACTIVE_VISION_OPTION == VisionOptions.MEGATAG
          && this.inputs.tagCount > 0) {
        double poseDifference =
            RobotContainer.driveSubsystem
                .getWheelBasedPose()
                .getTranslation()
                .getDistance(this.getPose().getTranslation());

        if (this.inputs.averagTagDistance >= 4.3) {
          this.stddevCalcState = StddevCalculationState.TAGS_TOO_FAR;

        } else if (this.inputs.tagCount >= 2) {
          this.stddevCalcState = StddevCalculationState.MULTIPLE_TAGS_FOUND;
          this.inputs.translationStddev = 0.5;
          this.inputs.rotationStddev = 6;

        } else if (this.inputs.averageTagSize > 0.9 && poseDifference < 0.3) {
          this.stddevCalcState = StddevCalculationState.CLOSE_TAG_CHECKED_AGAINST_ODOMETRY;
          this.inputs.translationStddev = 1.0;
          this.inputs.rotationStddev = 12;

        } else if (this.inputs.averagTagDistance > 0.1 && poseDifference < 0.3) {
          this.stddevCalcState = StddevCalculationState.FAR_TAG_CHECKED_AGAINST_ODOMETRY;
          this.inputs.translationStddev = 2.0;
          this.inputs.rotationStddev = 30;

        } else if (DriverStation.isDisabled()) {
          this.stddevCalcState = StddevCalculationState.ROBOT_DISABLED;
          this.inputs.translationStddev = 2.0;
          this.inputs.rotationStddev = 30;

        } else {
          this.stddevCalcState = StddevCalculationState.NO_GOOD_TAGS;
        }
      } else {
        this.stddevCalcState = StddevCalculationState.INVALID_VISION_STATE;
      }
    } else {
      this.stddevCalcState = StddevCalculationState.INVALID_VISION_STATE;
    }

    Logger.recordOutput("Vision/Deviation Calculation Mode", this.stddevCalcState);
  }

  @Override
  public Pose2d getPose() {
    return this.inputs.pose;
  }

  @Override
  public double getTimestamp() {
    return this.inputs.timestamp;
  }

  private enum CombinedMegaTagState {
    INIT,
    REJECTED_DUE_TO_SPIN_BLUR,
    REJECTED_DUE_TO_NO_TAGS,
    UPDATED_WITH_PRIMARY,
    UPDATED_WITH_SECONDARY
  }

  private enum StddevCalculationState {
    INVALID_VISION_STATE(false),
    MT2_ROTATION_BASED(true),
    TAGS_TOO_FAR(false),
    MULTIPLE_TAGS_FOUND(true),
    CLOSE_TAG_CHECKED_AGAINST_ODOMETRY(true),
    FAR_TAG_CHECKED_AGAINST_ODOMETRY(true),
    ROBOT_DISABLED(true),
    NO_GOOD_TAGS(false);

    boolean applyToPoseEstimate;

    StddevCalculationState(boolean applyToPE) {
      this.applyToPoseEstimate = applyToPE;
    }

    public boolean applyToPoseEstimate() {
      return this.applyToPoseEstimate;
    }
  }
}
