package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ScoreLoc;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class ScoreAssist {
  private final PathConstraints scoreAssistConstraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

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
   * should have an entry to the {@link #createCmdsMap} object via this method
   */
  private ScoreAssist() {
    sub = topic.subscribe("none");
  }

  private Command doScoreAtLoc(ScoreLoc loc) {
    return Commands.parallel(
        loc.getLevel().getPrepCommand().get(),
        Commands.sequence(
            new SelectCommand<>(
                Map.of(
                    DriverStation.Alliance.Red,
                        AutoBuilder.pathfindToPose(
                            AllianceFlipUtil.flip(loc.getNode().getPose()),
                            scoreAssistConstraints,
                            0.0),
                    DriverStation.Alliance.Blue,
                        AutoBuilder.pathfindToPose(
                            loc.getNode().getPose(), scoreAssistConstraints, 0.0)),
                () -> DriverStation.getAlliance().get()),
            loc.getLevel().getScoreCommand().get()));
  }

  private Map<ScoreLoc, Command> createCmdsMap() {
    var cmdsMap = new HashMap<ScoreLoc, Command>();
    cmdsMap.put(ScoreLoc.A_ONE, doScoreAtLoc(ScoreLoc.A_ONE));
    cmdsMap.put(ScoreLoc.A_TWO, doScoreAtLoc(ScoreLoc.A_TWO));
    cmdsMap.put(ScoreLoc.A_THREE, doScoreAtLoc(ScoreLoc.A_THREE));
    cmdsMap.put(ScoreLoc.A_FOUR, doScoreAtLoc(ScoreLoc.A_FOUR));

    cmdsMap.put(ScoreLoc.B_ONE, doScoreAtLoc(ScoreLoc.B_ONE));
    cmdsMap.put(ScoreLoc.B_TWO, doScoreAtLoc(ScoreLoc.B_TWO));
    cmdsMap.put(ScoreLoc.B_THREE, doScoreAtLoc(ScoreLoc.B_THREE));
    cmdsMap.put(ScoreLoc.B_FOUR, doScoreAtLoc(ScoreLoc.B_FOUR));

    cmdsMap.put(ScoreLoc.C_ONE, doScoreAtLoc(ScoreLoc.C_ONE));
    cmdsMap.put(ScoreLoc.C_TWO, doScoreAtLoc(ScoreLoc.C_TWO));
    cmdsMap.put(ScoreLoc.C_THREE, doScoreAtLoc(ScoreLoc.C_THREE));
    cmdsMap.put(ScoreLoc.C_FOUR, doScoreAtLoc(ScoreLoc.C_FOUR));

    cmdsMap.put(ScoreLoc.D_ONE, doScoreAtLoc(ScoreLoc.D_ONE));
    cmdsMap.put(ScoreLoc.D_TWO, doScoreAtLoc(ScoreLoc.D_TWO));
    cmdsMap.put(ScoreLoc.D_THREE, doScoreAtLoc(ScoreLoc.D_THREE));
    cmdsMap.put(ScoreLoc.D_FOUR, doScoreAtLoc(ScoreLoc.D_FOUR));

    cmdsMap.put(ScoreLoc.E_ONE, doScoreAtLoc(ScoreLoc.E_ONE));
    cmdsMap.put(ScoreLoc.E_TWO, doScoreAtLoc(ScoreLoc.E_TWO));
    cmdsMap.put(ScoreLoc.E_THREE, doScoreAtLoc(ScoreLoc.E_THREE));
    cmdsMap.put(ScoreLoc.E_FOUR, doScoreAtLoc(ScoreLoc.E_FOUR));

    cmdsMap.put(ScoreLoc.F_ONE, doScoreAtLoc(ScoreLoc.F_ONE));
    cmdsMap.put(ScoreLoc.F_TWO, doScoreAtLoc(ScoreLoc.F_TWO));
    cmdsMap.put(ScoreLoc.F_THREE, doScoreAtLoc(ScoreLoc.F_THREE));
    cmdsMap.put(ScoreLoc.F_FOUR, doScoreAtLoc(ScoreLoc.F_FOUR));

    cmdsMap.put(ScoreLoc.G_ONE, doScoreAtLoc(ScoreLoc.G_ONE));
    cmdsMap.put(ScoreLoc.G_TWO, doScoreAtLoc(ScoreLoc.G_TWO));
    cmdsMap.put(ScoreLoc.G_THREE, doScoreAtLoc(ScoreLoc.G_THREE));
    cmdsMap.put(ScoreLoc.G_FOUR, doScoreAtLoc(ScoreLoc.G_FOUR));

    cmdsMap.put(ScoreLoc.H_ONE, doScoreAtLoc(ScoreLoc.H_ONE));
    cmdsMap.put(ScoreLoc.H_TWO, doScoreAtLoc(ScoreLoc.H_TWO));
    cmdsMap.put(ScoreLoc.H_THREE, doScoreAtLoc(ScoreLoc.H_THREE));
    cmdsMap.put(ScoreLoc.H_FOUR, doScoreAtLoc(ScoreLoc.H_FOUR));

    cmdsMap.put(ScoreLoc.I_ONE, doScoreAtLoc(ScoreLoc.I_ONE));
    cmdsMap.put(ScoreLoc.I_TWO, doScoreAtLoc(ScoreLoc.I_TWO));
    cmdsMap.put(ScoreLoc.I_THREE, doScoreAtLoc(ScoreLoc.I_THREE));
    cmdsMap.put(ScoreLoc.I_FOUR, doScoreAtLoc(ScoreLoc.I_FOUR));

    cmdsMap.put(ScoreLoc.J_ONE, doScoreAtLoc(ScoreLoc.J_ONE));
    cmdsMap.put(ScoreLoc.J_TWO, doScoreAtLoc(ScoreLoc.J_TWO));
    cmdsMap.put(ScoreLoc.J_THREE, doScoreAtLoc(ScoreLoc.J_THREE));
    cmdsMap.put(ScoreLoc.J_FOUR, doScoreAtLoc(ScoreLoc.J_FOUR));

    cmdsMap.put(ScoreLoc.K_ONE, doScoreAtLoc(ScoreLoc.K_ONE));
    cmdsMap.put(ScoreLoc.K_TWO, doScoreAtLoc(ScoreLoc.K_TWO));
    cmdsMap.put(ScoreLoc.K_THREE, doScoreAtLoc(ScoreLoc.K_THREE));
    cmdsMap.put(ScoreLoc.K_FOUR, doScoreAtLoc(ScoreLoc.K_FOUR));

    cmdsMap.put(ScoreLoc.L_ONE, doScoreAtLoc(ScoreLoc.L_ONE));
    cmdsMap.put(ScoreLoc.L_TWO, doScoreAtLoc(ScoreLoc.L_TWO));
    cmdsMap.put(ScoreLoc.L_THREE, doScoreAtLoc(ScoreLoc.L_THREE));
    cmdsMap.put(ScoreLoc.L_FOUR, doScoreAtLoc(ScoreLoc.L_FOUR));

    return cmdsMap;
  }

  public Trigger getTrigger() {
    return new Trigger(() -> ScoreLoc.checkNTValid(sub.get("none")));
  }

  public Command scoreAtLoc(Supplier<ScoreLoc> loc) {
    return new SelectCommand<>(createCmdsMap(), loc);
  }

  public Command networkTablesDrive() {
    return scoreAtLoc(() -> ScoreLoc.parseFromNT(sub.get("none")));
  }
}
