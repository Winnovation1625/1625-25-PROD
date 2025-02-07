package frc.robot.subsystems.algaemanipulator;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeManipulatorIO {

  @AutoLog
  public static class AlgaeManipulatorIOInputs {

    public double appliedVoltageOut = 0.0;
    public double appliedCurrentOut = 0.0;
    public double velocityRadsPerSecond = 0.0;
    public double positionRad = 0.0;
  }

  public default void updateInputs(AlgaeManipulatorIOInputs inputs) {}

  public default void setVoltageOutput(DoubleSupplier volts) {}

  public default void setCurrentOutput(DoubleSupplier current) {}

  public default void stop() {}
}
