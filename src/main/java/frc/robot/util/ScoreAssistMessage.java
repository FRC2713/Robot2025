package frc.robot.util;

public class ScoreAssistMessage {
  public GoalType goal;
  public int level;
  public int index;

  public ScoreAssistMessage(String ntString) {
    // check if string is in the correct format
    // example string format: "1,4,11"
    if (!ntString.matches("^\\d+,\\d+,\\d+$")) {
      throw new IllegalArgumentException("Invalid format: " + ntString);
    }
    var parts = ntString.split(",");
    var goalVal = Integer.parseInt(parts[0]);
    this.level = Integer.parseInt(parts[1]);
    this.index = Integer.parseInt(parts[2]);
    this.goal = GoalType.values()[goalVal];
  }

  public static enum GoalType {
    CORAL(1),
    ALGAE(2),
    CAGE(3),
    PROCESSOR(4),
    BARGE(5);

    private final int value;

    GoalType(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }
}
