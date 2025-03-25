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

  private Command movingCmd;
  private Command postMoveCmd;

  private State state = State.INIT;

  private enum State {
    INIT,
    MOVING,
    POST_MOVE,
    FINISHED
  }

  private MoveSSToTarget(Supplier<ScoreLevel> level, Command scoreCommand, boolean moveIsAPrep) {

    this.moveTarget = level;
    this.postMoveCmd = scoreCommand;
    this.moveIsAPrep = moveIsAPrep;
    this.state = State.INIT;
  }

  /**
   * Move to this level with the intention of scoring after
   *
   * @param level what level to go to
   * @param scoreCommand what to do after level is reached
   */
  public MoveSSToTarget(Supplier<ScoreLevel> level, Command scoreCommand) {
    this(level, scoreCommand, false);
  }

  /**
   * Move to this level, nothing else will happen
   *
   * @param level what level to go to
   */
  public MoveSSToTarget(Supplier<ScoreLevel> level) {
    this(level, Commands.none(), true);
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
  }

  public void execute() {
    if (this.state == State.MOVING) {
      movingCmd.execute();
      if (movingCmd.isFinished()) {
        movingCmd.end(false);
        postMoveCmd.initialize();
        this.state = State.POST_MOVE;
      }
    } else if (this.state == State.POST_MOVE) {
      postMoveCmd.execute();
      if (postMoveCmd.isFinished()) {
        postMoveCmd.end(false);
        this.state = State.FINISHED;
      }
    }
  }

  public boolean isFinished() {
    return this.state == State.FINISHED;
  }
}
