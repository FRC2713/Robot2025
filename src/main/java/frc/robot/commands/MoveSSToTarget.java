package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.ScoreLevel;
import java.util.function.Supplier;

/**
 * Essentially this is a specialized version of SequentialCommandGroup. This lets us only creates
 * the sub-commands when the command actually starts
 */
public class MoveSSToTarget extends Command {

  private Supplier<ScoreLevel> moveTarget;
  private boolean moveIsAPrep;
  private boolean scoreAfter;

  private Command movingCmd;
  private Command scoreCmd;

  private State state = State.INIT;

  private enum State {
    INIT,
    MOVING,
    POST_MOVE,
    FINISHED
  }

  /**
   * Move to this level
   *
   * @param level what level to go to
   * @param prep prep or full SS move?
   * @param score score after moving?
   */
  public MoveSSToTarget(Supplier<ScoreLevel> level, boolean prep, boolean score) {

    this.moveTarget = level;
    this.moveIsAPrep = prep;
    this.scoreAfter = score;
    this.state = State.INIT;
  }

  /**
   * Move to this level (without scoring)
   *
   * @param level what level to go to
   * @param prep prep or full SS move?
   */
  public MoveSSToTarget(Supplier<ScoreLevel> level, boolean prep) {
    this(level, prep, false);
  }

  @Override
  public void initialize() {
    this.movingCmd =
        Commands.either(
            moveTarget.get().getPrepCommand().get(), // move for scoring
            moveTarget.get().getSsCommand().get(), // move for prepping
            () -> this.moveIsAPrep);

    this.movingCmd.initialize();
    this.state = State.MOVING;
    this.scoreCmd = moveTarget.get().getScoreCommand().get();
  }

  public void execute() {
    if (this.state == State.MOVING) {
      movingCmd.execute();
      if (movingCmd.isFinished()) {
        movingCmd.end(false);
        this.state = State.POST_MOVE;
        if (scoreAfter) scoreCmd.initialize();
        else this.state = State.FINISHED;
      }
    } else if (this.state == State.POST_MOVE && scoreAfter) {
      scoreCmd.execute();
      if (scoreCmd.isFinished()) {
        scoreCmd.end(false);
        this.state = State.FINISHED;
      }
    }
  }

  public boolean isFinished() {
    return this.state == State.FINISHED;
  }
}
