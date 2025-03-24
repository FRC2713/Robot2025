package frc.robot.util;

import java.util.Map;
import lombok.NonNull;

/** Consists of a score node (field position) and score level (reef branch level) */
public class ScoreLoc {

  private ScoreNode node;
  private ScoreLevel level;
  private String name = "";

  public ScoreNode getNode() {
    return node;
  }

  public ScoreLevel getLevel() {
    return level;
  }

  public ScoreLoc(@NonNull ScoreNode node, @NonNull ScoreLevel level) {
    this.node = node;
    this.level = level;
    this.name = this.node.name().toUpperCase() + "_" + this.level.name().toUpperCase();
  }

  @Override
  public boolean equals(Object o) {
    if (o == this) return true;
    if (o == null || !(o instanceof ScoreLoc)) {
      return false;
    }

    ScoreLoc other = (ScoreLoc) o;
    return this.getNode() == other.getNode() && this.getLevel() == other.getLevel();
  }

  @Override
  public int hashCode() {
    return (this.getNode().hashCode() * 271) + (this.getLevel().hashCode() * 13);
  }

  @Override
  public String toString() {
    return this.name;
  }

  /** Predefined scoring locations */
  public class ScoreLocations {
    public static ScoreLoc A_ONE = new ScoreLoc(ScoreNode.A, ScoreLevel.ONE);
    public static ScoreLoc A_TWO = new ScoreLoc(ScoreNode.A, ScoreLevel.TWO);
    public static ScoreLoc A_THREE = new ScoreLoc(ScoreNode.A, ScoreLevel.THREE);
    public static ScoreLoc A_FOUR = new ScoreLoc(ScoreNode.A, ScoreLevel.FOUR);
    public static ScoreLoc B_ONE = new ScoreLoc(ScoreNode.B, ScoreLevel.ONE);
    public static ScoreLoc B_TWO = new ScoreLoc(ScoreNode.B, ScoreLevel.TWO);
    public static ScoreLoc B_THREE = new ScoreLoc(ScoreNode.B, ScoreLevel.THREE);
    public static ScoreLoc B_FOUR = new ScoreLoc(ScoreNode.B, ScoreLevel.FOUR);
    public static ScoreLoc C_ONE = new ScoreLoc(ScoreNode.C, ScoreLevel.ONE);
    public static ScoreLoc C_TWO = new ScoreLoc(ScoreNode.C, ScoreLevel.TWO);
    public static ScoreLoc C_THREE = new ScoreLoc(ScoreNode.C, ScoreLevel.THREE);
    public static ScoreLoc C_FOUR = new ScoreLoc(ScoreNode.C, ScoreLevel.FOUR);
    public static ScoreLoc D_ONE = new ScoreLoc(ScoreNode.D, ScoreLevel.ONE);
    public static ScoreLoc D_TWO = new ScoreLoc(ScoreNode.D, ScoreLevel.TWO);
    public static ScoreLoc D_THREE = new ScoreLoc(ScoreNode.D, ScoreLevel.THREE);
    public static ScoreLoc D_FOUR = new ScoreLoc(ScoreNode.D, ScoreLevel.FOUR);
    public static ScoreLoc E_ONE = new ScoreLoc(ScoreNode.E, ScoreLevel.ONE);
    public static ScoreLoc E_TWO = new ScoreLoc(ScoreNode.E, ScoreLevel.TWO);
    public static ScoreLoc E_THREE = new ScoreLoc(ScoreNode.E, ScoreLevel.THREE);
    public static ScoreLoc E_FOUR = new ScoreLoc(ScoreNode.E, ScoreLevel.FOUR);
    public static ScoreLoc F_ONE = new ScoreLoc(ScoreNode.F, ScoreLevel.ONE);
    public static ScoreLoc F_TWO = new ScoreLoc(ScoreNode.F, ScoreLevel.TWO);
    public static ScoreLoc F_THREE = new ScoreLoc(ScoreNode.F, ScoreLevel.THREE);
    public static ScoreLoc F_FOUR = new ScoreLoc(ScoreNode.F, ScoreLevel.FOUR);
    public static ScoreLoc G_ONE = new ScoreLoc(ScoreNode.G, ScoreLevel.ONE);
    public static ScoreLoc G_TWO = new ScoreLoc(ScoreNode.G, ScoreLevel.TWO);
    public static ScoreLoc G_THREE = new ScoreLoc(ScoreNode.G, ScoreLevel.THREE);
    public static ScoreLoc G_FOUR = new ScoreLoc(ScoreNode.G, ScoreLevel.FOUR);
    public static ScoreLoc H_ONE = new ScoreLoc(ScoreNode.H, ScoreLevel.ONE);
    public static ScoreLoc H_TWO = new ScoreLoc(ScoreNode.H, ScoreLevel.TWO);
    public static ScoreLoc H_THREE = new ScoreLoc(ScoreNode.H, ScoreLevel.THREE);
    public static ScoreLoc H_FOUR = new ScoreLoc(ScoreNode.H, ScoreLevel.FOUR);
    public static ScoreLoc I_ONE = new ScoreLoc(ScoreNode.I, ScoreLevel.ONE);
    public static ScoreLoc I_TWO = new ScoreLoc(ScoreNode.I, ScoreLevel.TWO);
    public static ScoreLoc I_THREE = new ScoreLoc(ScoreNode.I, ScoreLevel.THREE);
    public static ScoreLoc I_FOUR = new ScoreLoc(ScoreNode.I, ScoreLevel.FOUR);
    public static ScoreLoc J_ONE = new ScoreLoc(ScoreNode.J, ScoreLevel.ONE);
    public static ScoreLoc J_TWO = new ScoreLoc(ScoreNode.J, ScoreLevel.TWO);
    public static ScoreLoc J_THREE = new ScoreLoc(ScoreNode.J, ScoreLevel.THREE);
    public static ScoreLoc J_FOUR = new ScoreLoc(ScoreNode.J, ScoreLevel.FOUR);
    public static ScoreLoc K_ONE = new ScoreLoc(ScoreNode.K, ScoreLevel.ONE);
    public static ScoreLoc K_TWO = new ScoreLoc(ScoreNode.K, ScoreLevel.TWO);
    public static ScoreLoc K_THREE = new ScoreLoc(ScoreNode.K, ScoreLevel.THREE);
    public static ScoreLoc K_FOUR = new ScoreLoc(ScoreNode.K, ScoreLevel.FOUR);
    public static ScoreLoc L_ONE = new ScoreLoc(ScoreNode.L, ScoreLevel.ONE);
    public static ScoreLoc L_TWO = new ScoreLoc(ScoreNode.L, ScoreLevel.TWO);
    public static ScoreLoc L_THREE = new ScoreLoc(ScoreNode.L, ScoreLevel.THREE);
    public static ScoreLoc L_FOUR = new ScoreLoc(ScoreNode.L, ScoreLevel.FOUR);

    // maps the ReefTracker app's node indexes to what the robot defines them as
    private static Map<Integer, ScoreNode> ntNodeMappings =
        Map.ofEntries(
            Map.entry(Integer.valueOf(0), ScoreNode.B),
            Map.entry(Integer.valueOf(1), ScoreNode.A),
            Map.entry(Integer.valueOf(2), ScoreNode.L),
            Map.entry(Integer.valueOf(3), ScoreNode.K),
            Map.entry(Integer.valueOf(4), ScoreNode.J),
            Map.entry(Integer.valueOf(5), ScoreNode.I),
            Map.entry(Integer.valueOf(6), ScoreNode.H),
            Map.entry(Integer.valueOf(7), ScoreNode.G),
            Map.entry(Integer.valueOf(8), ScoreNode.F),
            Map.entry(Integer.valueOf(9), ScoreNode.E),
            Map.entry(Integer.valueOf(10), ScoreNode.D),
            Map.entry(Integer.valueOf(11), ScoreNode.C));

    // maps the ReefTracker app's level numbers to what the robot defines them as
    private static Map<Integer, ScoreLevel> ntLevelMappings =
        Map.ofEntries(
            Map.entry(Integer.valueOf(1), ScoreLevel.ONE),
            Map.entry(Integer.valueOf(2), ScoreLevel.TWO),
            Map.entry(Integer.valueOf(3), ScoreLevel.THREE),
            Map.entry(Integer.valueOf(4), ScoreLevel.FOUR));

    /**
     * Validates a string from the reef tracker app as parsable to a ScoreLoc
     *
     * @param ntloc a string representation of a scoring location given from the reef tracker app
     * @return
     */
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

    /**
     * Parses a string from the reef tracker app to a ScoreLoc
     *
     * @param ntloc a string representation of a scoring location given from the reef tracker app
     * @return the ScoreLoc that is parsed from the string, or null if its not parsable
     */
    public static ScoreLoc parseFromNT(String ntloc) {
      if (ntloc == "none") {
        return null;
      }
      var split = ntloc.indexOf(',');
      var index = -1;
      var level = -1;
      try {
        index = Integer.parseInt(ntloc.substring(0, split));
        level = Integer.parseInt(ntloc.substring(split + 1));
      } catch (Exception e) {
        return null;
      }

      ScoreNode parsedNode = ntNodeMappings.getOrDefault(index, null);
      ScoreLevel parsedLevel = ntLevelMappings.getOrDefault(level, null);
      if (parsedNode == null || parsedLevel == null) {
        return null;
      }

      return new ScoreLoc(parsedNode, parsedLevel);
    }
  }
}
