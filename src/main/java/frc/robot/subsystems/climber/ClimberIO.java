package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public class ClimberIOInputs {

    public double servoPositionRads = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setRelease(boolean releaseState) {}
}
