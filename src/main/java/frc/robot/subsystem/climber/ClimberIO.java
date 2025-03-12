package frc.robot.subsystem.climber;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public class ClimberIOInputs {

    public double servoPositionRads = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setRelease(DoubleSupplier angle) {}
}
