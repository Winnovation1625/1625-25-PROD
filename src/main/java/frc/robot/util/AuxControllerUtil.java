package frc.robot.util;

import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.FieldConstants.ReefPosition;
import java.util.List;

public class AuxControllerUtil {

  public static CoralObjective getCoralObjective(
      List<Boolean> reefPositionButtons, List<Boolean> reefLevelButtons) {
    int reefPositionIndex = getTrueIndex(reefPositionButtons);
    int reefLevelIndex = getTrueIndex(reefLevelButtons);
    return CoralObjective.builder()
        .position(ReefPosition.fromIndex(reefPositionIndex))
        .reefLevel(ReefLevel.fromLevel(reefLevelIndex))
        .build();
  }

  private static int getTrueIndex(List<Boolean> list) {
    for (int i = 0; i < list.size(); i++) {
      if (list.get(i)) {
        return i;
      }
    }
    return -1; // Return -1 if no true value is found
  }
}
