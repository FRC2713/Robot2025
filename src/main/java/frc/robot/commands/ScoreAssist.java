package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.drive.Drivetrain;
import java.util.HashMap;
import java.util.Map;

public class ScoreAssist {

  public static Command scoreClosestL1(Drivetrain drivetrain) {
    return Commands.sequence(
        Commands.parallel(
            SuperStructure.L1_CORAL_PREP_ELEVATOR(),
            new SelectCommand<>(ScoreAssist.cmdsMap, drivetrain::getClosestScoringLocation)),
        SuperStructure.L1_CORAL_SCORE());
  }

  private static final PathConstraints scoreAssistConstraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private static Map<ScoreLoc, Command> cmdsMap = new HashMap<>();

  /**
   * AutoBuilder can only be called after it's configured, so this has to be a seperate method call
   * that is called in the RobotContainer constructor. IMPORTANT NOTE: all {@link #ScoreLoc}
   * enum should have an entry to the {@link #cmdsMap} object via this method
   */
  public static void initCommands() {
    ScoreAssist.cmdsMap.put(
        ScoreLoc.A_ONE,
        AutoBuilder.pathfindToPose(ScoreLoc.A_ONE.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreLoc.A_TWO,
        AutoBuilder.pathfindToPose(ScoreLoc.A_TWO.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreLoc.B_ONE,
        AutoBuilder.pathfindToPose(ScoreLoc.B_ONE.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreLoc.B_TWO,
        AutoBuilder.pathfindToPose(ScoreLoc.B_TWO.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreLoc.C_ONE,
        AutoBuilder.pathfindToPose(ScoreLoc.C_ONE.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreLoc.C_TWO,
        AutoBuilder.pathfindToPose(ScoreLoc.C_TWO.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreLoc.D_ONE,
        AutoBuilder.pathfindToPose(ScoreLoc.D_ONE.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreLoc.D_TWO,
        AutoBuilder.pathfindToPose(ScoreLoc.D_TWO.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreLoc.E_ONE,
        AutoBuilder.pathfindToPose(ScoreLoc.E_ONE.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreLoc.E_TWO,
        AutoBuilder.pathfindToPose(ScoreLoc.E_TWO.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreLoc.F_ONE,
        AutoBuilder.pathfindToPose(ScoreLoc.F_ONE.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreLoc.F_TWO,
        AutoBuilder.pathfindToPose(ScoreLoc.F_TWO.getPose(), scoreAssistConstraints, 0.0));
  }

  /**
   * IMPORTANT NOTE: if you add a new ScoreLoc, you should add an entry to the {@link #cmdsMap}
   * object via the {@link #initCommands()} method
   */
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

    ScoreLoc(Pose2d pose, Command cmd) {
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose;
    }
  }
}
