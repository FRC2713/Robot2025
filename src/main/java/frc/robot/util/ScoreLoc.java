package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Constants;
import frc.robot.commands.ScoreAssist;
import java.util.Map;

public enum ScoreLoc {
  A_ONE(ScoreNode.A, ScoreLevel.ONE),
  A_TWO(ScoreNode.A, ScoreLevel.TWO),
  A_THREE(ScoreNode.A, ScoreLevel.THREE),
  A_FOUR(ScoreNode.A, ScoreLevel.FOUR),
  B_ONE(ScoreNode.B, ScoreLevel.ONE),
  B_TWO(ScoreNode.B, ScoreLevel.TWO),
  B_THREE(ScoreNode.B, ScoreLevel.THREE),
  B_FOUR(ScoreNode.B, ScoreLevel.FOUR),
  C_ONE(ScoreNode.C, ScoreLevel.ONE),
  C_TWO(ScoreNode.C, ScoreLevel.TWO),
  C_THREE(ScoreNode.C, ScoreLevel.THREE),
  C_FOUR(ScoreNode.C, ScoreLevel.FOUR),
  D_ONE(ScoreNode.D, ScoreLevel.ONE),
  D_TWO(ScoreNode.D, ScoreLevel.TWO),
  D_THREE(ScoreNode.D, ScoreLevel.THREE),
  D_FOUR(ScoreNode.D, ScoreLevel.FOUR),
  E_ONE(ScoreNode.E, ScoreLevel.ONE),
  E_TWO(ScoreNode.E, ScoreLevel.TWO),
  E_THREE(ScoreNode.E, ScoreLevel.THREE),
  E_FOUR(ScoreNode.E, ScoreLevel.FOUR),
  F_ONE(ScoreNode.F, ScoreLevel.ONE),
  F_TWO(ScoreNode.F, ScoreLevel.TWO),
  F_THREE(ScoreNode.F, ScoreLevel.THREE),
  F_FOUR(ScoreNode.F, ScoreLevel.FOUR),
  G_ONE(ScoreNode.G, ScoreLevel.ONE),
  G_TWO(ScoreNode.G, ScoreLevel.TWO),
  G_THREE(ScoreNode.G, ScoreLevel.THREE),
  G_FOUR(ScoreNode.G, ScoreLevel.FOUR),
  H_ONE(ScoreNode.H, ScoreLevel.ONE),
  H_TWO(ScoreNode.H, ScoreLevel.TWO),
  H_THREE(ScoreNode.H, ScoreLevel.THREE),
  H_FOUR(ScoreNode.H, ScoreLevel.FOUR),
  I_ONE(ScoreNode.I, ScoreLevel.ONE),
  I_TWO(ScoreNode.I, ScoreLevel.TWO),
  I_THREE(ScoreNode.I, ScoreLevel.THREE),
  I_FOUR(ScoreNode.I, ScoreLevel.FOUR),
  J_ONE(ScoreNode.J, ScoreLevel.ONE),
  J_TWO(ScoreNode.J, ScoreLevel.TWO),
  J_THREE(ScoreNode.J, ScoreLevel.THREE),
  J_FOUR(ScoreNode.J, ScoreLevel.FOUR),
  K_ONE(ScoreNode.K, ScoreLevel.ONE),
  K_TWO(ScoreNode.K, ScoreLevel.TWO),
  K_THREE(ScoreNode.K, ScoreLevel.THREE),
  K_FOUR(ScoreNode.K, ScoreLevel.FOUR),
  L_ONE(ScoreNode.L, ScoreLevel.ONE),
  L_TWO(ScoreNode.L, ScoreLevel.TWO),
  L_THREE(ScoreNode.L, ScoreLevel.THREE),
  L_FOUR(ScoreNode.L, ScoreLevel.FOUR),
  ;

  private ScoreNode node;
  private ScoreLevel level;

  public ScoreNode getNode() {
    return node;
  }

  public ScoreLevel getLevel() {
    return level;
  }

  ScoreLoc(ScoreNode node, ScoreLevel level) {
    this.node = node;
    this.level = level;
  }

  public Command getScoreCommand() {
    return Commands.parallel(
        level.getPrepCommand().get(),
        Commands.sequence(
            new SelectCommand<>(
                Map.of(
                    DriverStation.Alliance.Red,
                    ScoreAssist.buildOTFPath(
                        AllianceFlipUtil.flip(node.getRobotAlignmentPose()),
                        Constants.scoreAssistConstraints,
                        0.0),
                    DriverStation.Alliance.Blue,
                    ScoreAssist.buildOTFPath(
                        node.getRobotAlignmentPose(), Constants.scoreAssistConstraints, 0.0)),
                () -> DriverStation.getAlliance().get()),
            level.getScoreCommand().get()));
  }

  public static boolean checkNTValid(String ntloc) {
    if (ntloc == "none") {
      return false;
    }
    var split = ntloc.indexOf(',');
    try {
      Integer.parseInt(ntloc.substring(0, split));
      Integer.parseInt(ntloc.substring(split + 1));
    } catch (Exception e) {
      return false;
    }
    return true;
  }

  public static ScoreLoc parseFromNT(String ntloc) {
    if (ntloc == "none") {
      return null;
    }
    var split = ntloc.indexOf(',');
    var index = -1;
    var level = -1;
    try {
      index = Integer.parseInt(ntloc.substring(0, split));
      level = Integer.parseInt(ntloc.substring(split + 1)) + 1;
    } catch (Exception e) {
      return null;
    }

    switch (index) {
      case 0:
        switch (level) {
          case 1:
            return G_ONE;
          case 2:
            return G_TWO;
          case 3:
            return G_THREE;
          case 4:
            return G_FOUR;
          default:
            return null;
        }
      case 1:
        switch (level) {
          case 1:
            return H_ONE;
          case 2:
            return H_TWO;
          case 3:
            return H_THREE;
          case 4:
            return H_FOUR;
          default:
            return null;
        }
      case 2:
        switch (level) {
          case 1:
            return I_ONE;
          case 2:
            return I_TWO;
          case 3:
            return I_THREE;
          case 4:
            return I_FOUR;
          default:
            return null;
        }
      case 3:
        switch (level) {
          case 1:
            return J_ONE;
          case 2:
            return J_TWO;
          case 3:
            return J_THREE;
          case 4:
            return J_FOUR;
          default:
            return null;
        }
      case 4:
        switch (level) {
          case 1:
            return K_ONE;
          case 2:
            return K_TWO;
          case 3:
            return K_THREE;
          case 4:
            return K_FOUR;
          default:
            return null;
        }
      case 5:
        switch (level) {
          case 1:
            return L_ONE;
          case 2:
            return L_TWO;
          case 3:
            return L_THREE;
          case 4:
            return L_FOUR;
          default:
            return null;
        }
      case 6:
        switch (level) {
          case 1:
            return A_ONE;
          case 2:
            return A_TWO;
          case 3:
            return A_THREE;
          case 4:
            return A_FOUR;
          default:
            return null;
        }
      case 7:
        switch (level) {
          case 1:
            return B_ONE;
          case 2:
            return B_TWO;
          case 3:
            return B_THREE;
          case 4:
            return B_FOUR;
          default:
            return null;
        }
      case 8:
        switch (level) {
          case 1:
            return C_ONE;
          case 2:
            return C_TWO;
          case 3:
            return C_THREE;
          case 4:
            return C_FOUR;
          default:
            return null;
        }
      case 9:
        switch (level) {
          case 1:
            return D_ONE;
          case 2:
            return D_TWO;
          case 3:
            return D_THREE;
          case 4:
            return D_FOUR;
          default:
            return null;
        }
      case 10:
        switch (level) {
          case 1:
            return E_ONE;
          case 2:
            return E_TWO;
          case 3:
            return E_THREE;
          case 4:
            return E_FOUR;
          default:
            return null;
        }
      case 11:
        switch (level) {
          case 1:
            return F_ONE;
          case 2:
            return F_TWO;
          case 3:
            return F_THREE;
          case 4:
            return F_FOUR;
          default:
            return null;
        }
      default:
        return null;
    }
  }

  public static ScoreLoc fromNodeAndLevel(ScoreNode node2, ScoreLevel level2) {
    for (ScoreLoc loc : ScoreLoc.values()) {
      if (loc.node == node2 && loc.level == level2) {
        return loc;
      }
    }
    return null;
  }
}
