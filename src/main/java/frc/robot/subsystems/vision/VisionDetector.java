package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Detection;
import java.util.HashMap;

public class VisionDetector extends SubsystemBase {
  public static final String CAMERA_ID = "2d380202_e23e_4a1b_8b93_d2dfd499df45";
  public static final String DETECTION_BASE = "detections" + CAMERA_ID + "/";
  public static final String DETECTION_CX = DETECTION_BASE + "cx";
  public static final String DETECTION_CY = DETECTION_BASE + "cy";
  public static final String DETECTION_AREA = DETECTION_BASE + "area";
  public static final String DETECTION_ID = DETECTION_BASE + "id";
  public static final String DETECTION_TIME = DETECTION_BASE + "timeLeave";

  public HashMap<Long, Detection> detections = new HashMap<>();
  private NetworkTableInstance inst;
  private NetworkTable table;
  private double lastTimestamp = -1;

  public VisionDetector() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
  }

  @Override
  public void periodic() {
    detections.clear();
    detections.put((long) 12, new Detection(12, 540., 485., 41_000, 56));
    //     long[] defint = {};
    //     double[] def = {};
    //     var ids = table.getEntry(DETECTION_ID).getIntegerArray(defint);
    //     var cxs = table.getEntry(DETECTION_CX).getDoubleArray(def);
    //     var cys = table.getEntry(DETECTION_CY).getDoubleArray(def);
    //     var areas = table.getEntry(DETECTION_AREA).getDoubleArray(def);
    //     var timeLeave = table.getEntry(DETECTION_TIME).getDouble(0);
    //     try {
    //       for (int i = 0; i < ids.length; i++) {
    //         detections.put(ids[i], new Detection(ids[i], cxs[i], cys[i], areas[i], timeLeave));
    //       }
    //     } catch (Exception e) {
    //       System.out.println("malformed detector data");
    //     }
  }
}
