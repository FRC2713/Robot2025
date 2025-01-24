package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.ScoreLoc;
import frc.robot.util.ScoreNode;
import java.util.HashMap;
import java.util.Map;

public class ScoreAssist {

  public Command scoreClosestL1(Drivetrain drivetrain) {
    return Commands.sequence(
        Commands.parallel(
            SuperStructure.L1_CORAL_PREP_ELEVATOR(),
            new SelectCommand<>(ScoreAssist.cmdsMap, drivetrain::getClosestScoringLocation)),
        SuperStructure.L1_CORAL_SCORE());
  }

  private final PathConstraints scoreAssistConstraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private static Map<ScoreNode, Command> cmdsMap = new HashMap<>();

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private StringTopic topic = inst.getStringTopic("/scoreassist/goto");
  private StringSubscriber sub;

  private static ScoreAssist INSTANCE = null;

  public static ScoreAssist getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ScoreAssist();
    }
    return INSTANCE;
  }

  /**
   * AutoBuilder can only be called after it's configured, so this has to be a seperate method call
   * that is called in the RobotContainer constructor. IMPORTANT NOTE: all {@link #ScoreNode} enum
   * should have an entry to the {@link #cmdsMap} object via this method
   */
  private ScoreAssist() {
    sub = topic.subscribe("none");

    ScoreAssist.cmdsMap.put(
        ScoreNode.A,
        AutoBuilder.pathfindToPose(ScoreNode.A.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreNode.B,
        AutoBuilder.pathfindToPose(ScoreNode.B.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreNode.C,
        AutoBuilder.pathfindToPose(ScoreNode.C.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreNode.D,
        AutoBuilder.pathfindToPose(ScoreNode.D.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreNode.E,
        AutoBuilder.pathfindToPose(ScoreNode.E.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreNode.F,
        AutoBuilder.pathfindToPose(ScoreNode.F.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreNode.G,
        AutoBuilder.pathfindToPose(ScoreNode.G.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreNode.H,
        AutoBuilder.pathfindToPose(ScoreNode.H.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreNode.I,
        AutoBuilder.pathfindToPose(ScoreNode.I.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreNode.J,
        AutoBuilder.pathfindToPose(ScoreNode.J.getPose(), scoreAssistConstraints, 0.0));

    ScoreAssist.cmdsMap.put(
        ScoreNode.K,
        AutoBuilder.pathfindToPose(ScoreNode.K.getPose(), scoreAssistConstraints, 0.0));
    ScoreAssist.cmdsMap.put(
        ScoreNode.L,
        AutoBuilder.pathfindToPose(ScoreNode.L.getPose(), scoreAssistConstraints, 0.0));
  }

  public Trigger getTrigger() {
    return new Trigger(() -> ScoreLoc.checkNTValid(sub.get("none")));
  }

  public Command networkTablesDrive() {
    return Commands.sequence(
        Commands.parallel(
            //   loc.getLevel().getPrepCommand(),
            new SelectCommand<>(
                ScoreAssist.cmdsMap, () -> ScoreLoc.parseFromNT(sub.get("none")).getNode()))
        //   loc.getLevel().getScoreCommand()
        );
  }
}
