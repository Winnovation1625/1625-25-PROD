package frc.robot.subsystem.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double positionRads = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setRelease(boolean releaseState) {}
}
