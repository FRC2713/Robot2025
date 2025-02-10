package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.VisionConstants;
import frc.robot.util.PoseAndTwist3d;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.net.http.HttpResponse.BodyHandlers;
import java.time.Duration;
import org.littletonrobotics.junction.Logger;

public class VisionIOOdometry implements VisionIO {
  public PoseAndTwist3d pose;
  public Pose2d pose2d;

  public VisionIOOdometry() {
    pose =
        PoseAndTwist3d.from(
            RobotContainer.driveSubsystem.getPose(),
            RobotContainer.driveSubsystem.getChassisSpeeds());
  }

  @Override
  public void update() {
    pose2d = RobotContainer.driveSubsystem.getPose();
    pose.update(pose2d, RobotContainer.driveSubsystem.getChassisSpeeds());
    Logger.recordOutput("slamdunk/wheelodometry", pose.toJSON());
  }

  @Override
  public Pose2d getPose() {
    return pose.pose.toPose2d();
  }

  @Override
  public void resetPose(Pose2d pose) {
    HttpClient client = HttpClient.newHttpClient();
    HttpRequest request =
        HttpRequest.newBuilder()
            .uri(URI.create(VisionConstants.VISION_SERVER_URL + "actions/restart-tagslam"))
            .timeout(Duration.ofSeconds(2))
            .build();
    client
        .sendAsync(request, BodyHandlers.ofString())
        .thenApply(HttpResponse::body)
        .thenAccept(System.out::println);
  }
}
