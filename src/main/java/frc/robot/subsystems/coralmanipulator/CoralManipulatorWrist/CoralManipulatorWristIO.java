package frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface CoralManipulatorWristIO {

  @AutoLog
  public static class CoralManipulatorWristIOInputs {

    public double positionRad = 0.0;
    public double appliedVoltageOut = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedCurrentOut = 0.0;
  }

  public default void updateInputs(CoralManipulatorWristIOInputs inputs) {}

  public default void setPosition(DoubleSupplier positionSetpoint) {}

  public default void stop() {}

  public default void setBrakeMode(boolean isBrake) {}
}
