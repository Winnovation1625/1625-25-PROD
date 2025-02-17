package frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface CoralManipulatorRollersIO {

  @AutoLog
  public static class CoralManipulatorRollersIOInputs {

    public double appliedVoltageOut = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedCurrentOut = 0.0;
  }

  public default void updateInputs(CoralManipulatorRollersIOInputs inputs) {}

  public default void setVoltage(DoubleSupplier voltageSetpoint) {}

  public default void stop() {}
}
