package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LocalADStarAK implements Pathfinder {
  private final ADStarIO io = new ADStarIO();

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    if (!Logger.hasReplaySource()) {
      io.updateIsNewPathAvailable();
    }

    Logger.processInputs("LocalADStarAK", io);

    return io.isNewPathAvailable;
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    if (!Logger.hasReplaySource()) {
      io.updateCurrentPathPoints(constraints, goalEndState);
    }

    Logger.processInputs("LocalADStarAK", io);

    if (io.currentPathPoints.isEmpty()) {
      return null;
    }

    return PathPlannerPath.fromPathPoints(io.currentPathPoints, constraints, goalEndState);
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setStartPosition(startPosition);
    }
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    if (!Logger.hasReplaySource()) {
      io.lastGoal = new Pose2d(goalPosition, new Rotation2d());
      io.adStar.setGoalPosition(goalPosition);
    }
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path to properly avoid obstacles
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setDynamicObstacles(obs, currentRobotPos);
    }
  }

  private static class ADStarIO implements LoggableInputs {
    public LocalADStar adStar = new LocalADStar();
    public boolean isNewPathAvailable = false;
    public List<PathPoint> currentPathPoints = Collections.emptyList();

    public Pose2d lastGoal = new Pose2d();

    @Override
    public void toLog(LogTable table) {
      table.put("IsNewPathAvailable", isNewPathAvailable);

      Pose2d[] pointsLogged = new Pose2d[currentPathPoints.size()];
      int idx = 0;
      for (PathPoint point : currentPathPoints) {
        pointsLogged[idx] = new Pose2d(point.position, new Rotation2d());
        idx++;
      }

      table.put("CurrentPathPoints", pointsLogged);
      table.put("LastTarget", lastGoal);
    }

    @Override
    public void fromLog(LogTable table) {
      isNewPathAvailable = table.get("IsNewPathAvailable", false);

      Pose2d[] pointsLogged = table.get("CurrentPathPoints", new Pose2d[0]);

      List<PathPoint> pathPoints = new ArrayList<>();
      for (int i = 0; i < pointsLogged.length; i += 1) {
        pathPoints.add(new PathPoint(pointsLogged[i].getTranslation(), null));
      }

      currentPathPoints = pathPoints;
    }

    public void updateIsNewPathAvailable() {
      isNewPathAvailable = adStar.isNewPathAvailable();
    }

    public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goalEndState) {
      this.lastGoal = new Pose2d(this.lastGoal.getTranslation(), goalEndState.rotation());
      PathPlannerPath currentPath = adStar.getCurrentPath(constraints, goalEndState);

      if (currentPath != null) {
        currentPathPoints = currentPath.getAllPathPoints();
      } else {
        currentPathPoints = Collections.emptyList();
      }
    }
  }
}
