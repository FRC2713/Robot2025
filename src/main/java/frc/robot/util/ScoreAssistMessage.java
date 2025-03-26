package frc.robot.util;

public class ScoreAssistMessage {
    public GoalType goal;
    public int level;
    public int index;

    public ScoreAssistMessage(String ntString) {
        if (!ntString.matches("^\\d+,\\d+,\\d+$")) {
            throw new IllegalArgumentException("Invalid format: " + ntString);
        }
        // example string format: "1,4,11"
        var split = ntString.indexOf(',');
        var goal = Integer.parseInt(ntString.substring(0, split));
        this.level = Integer.parseInt(ntString.substring(split + 1));
        this.index = Integer.parseInt(ntString.substring(split + 1));
        this.goal = GoalType.values()[goal];
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
