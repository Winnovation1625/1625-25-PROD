package frc.robot.subsystems.superstructure.wrist;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public static class WristIOInputs {

    public double positionRad = 0.0;
    public double appliedVoltageOut = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedCurrentOut = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setPosition(DoubleSupplier positionSetpoint) {}

  public default void stop() {}

  public default void setBrakeMode(boolean isBrake) {}

  public default void setPID(double p, double d, double i) {}
}
