package frc.robot.subsystem.superstructure.arm;

import frc.robot.subsystem.manipulator.Manipulator.GamepieceState;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  class ArmIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmPosition(double desiredPosition, GamepieceState gamepieceState) {}

  public default void setBrakeMode(boolean enabled) {}

  public default void setPID(
      double p,
      double i,
      double d,
      double kS,
      double kV,
      double kA,
      double kG,
      double cruiseA,
      double cruiseV,
      int slot) {}

  public default void setInverted(boolean inverted) {}

  public default void runCurrent(double amps) {}

  public default void stop() {}

  public default void runVolts(double volts) {}

  public default double getKg() {
    return 0;
  }
}
