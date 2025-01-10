package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;

public class ScoreAssist {
  private ScoreAssist() {}

  public static Command scoreClosestLocation(Supplier<Pose2d> currentLoc, Command scoreCommand) {
    return scoreClosestLocation(currentLoc, scoreCommand, null);
  }

  public static Command scoreClosestLocation(
      Supplier<Pose2d> currentLoc, Command scoreCommand, Command prepCommand) {

    ScoreLoc closestLoc = ScoreLoc.A_ONE;
    double closestDist =
        currentLoc.get().getTranslation().getDistance(closestLoc.getPose().getTranslation());
    for (ScoreLoc loc : ScoreLoc.values()) {
      double dist = currentLoc.get().getTranslation().getDistance(loc.getPose().getTranslation());
      if (dist < closestDist) {
        closestLoc = loc;
        closestDist = dist;
      }
    }
    return scoreAtLocation(closestLoc, scoreCommand, prepCommand);
  }

  public static Command scoreAtLocation(ScoreLoc loc, Command scoreCommand) {
    return scoreAtLocation(loc, scoreCommand, null);
  }

  // TODO: Flipping for alliance
  public static Command scoreAtLocation(ScoreLoc loc, Command scoreCommand, Command prepCommand) {
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return Commands.sequence(
        Commands.parallel(
            AutoBuilder.pathfindToPose(
                loc.getPose(), constraints, 0.0 // Goal end velocity in meters/sec
                ),
            prepCommand != null ? prepCommand : Commands.none()),
        scoreCommand);
  }

  public static enum ScoreLoc {
    A_ONE(new Pose2d(2.884676694869995, 4.201192378997803, new Rotation2d(0))),
    A_TWO(new Pose2d(2.884676694869995, 3.8600285053253174, new Rotation2d(0))),

    B_ONE(new Pose2d(3.497980833053589, 2.7730469703674316, new Rotation2d(1.0335119235044878))),
    B_TWO(new Pose2d(3.845728874206543, 2.5586025714874268, new Rotation2d(1.0335119235044878))),

    C_ONE(new Pose2d(5.097620964050293, 2.547010898590088, new Rotation2d(2.1090184202465467))),
    C_TWO(new Pose2d(5.375819206237793, 2.709293365478515, new Rotation2d(2.1090184202465467))),

    D_ONE(new Pose2d(6.089637756347656, 3.8427934646606445, new Rotation2d(-Math.PI))),
    D_TWO(new Pose2d(6.08963775634765, 4.1608147621154785, new Rotation2d(-Math.PI))),

    E_ONE(new Pose2d(5.444761276245117, 5.318058490753174, new Rotation2d(-2.0899421642315534))),
    E_TWO(new Pose2d(5.1355743408203125, 5.47706937789917, new Rotation2d(-2.0899421642315534))),

    F_ONE(new Pose2d(3.881157159805298, 5.494737148284912, new Rotation2d(-1.037952202199093))),
    F_TWO(new Pose2d(3.571969747543335, 5.326892375946045, new Rotation2d(-1.037952202199093)));

    private Pose2d pose;

    ScoreLoc(Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose;
    }
  }
}
