package frc.robot.subsystems.manipulator;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {

  @AutoLog
  public class ManipulatorIOInputs {

    public double appliedVoltageOut = 0.0;
    public double appliedCurrentOut = 0.0;
    public double velocityRadsPerSecond = 0.0;
    public double positionRad = 0.0;
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setVoltage(DoubleSupplier volts) {}

  public default void setCurrentOutput(DoubleSupplier current) {}

  public default void stop() {}
}
