package frc.robot.subsystem.coralmanipulator;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface CoralManipulatoIO {

  @AutoLog
  public static class CoralManipulatorIOInputs {

    public double appliedVoltageOut = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedCurrentOut = 0.0;
  }

  public default void updateInputs(CoralManipulatorIOInputs inputs) {}

  public default void setVoltage(DoubleSupplier voltageSetpoint) {}

  public default void stop() {}
}
