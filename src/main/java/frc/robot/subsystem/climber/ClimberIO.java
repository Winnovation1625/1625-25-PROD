package frc.robot.subsystem.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double positionRads = 0.0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setRelease(boolean releaseState) {}
}
