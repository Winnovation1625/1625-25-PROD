package frc.robot.subsystem.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {

    public double positionRad;
    public double velocityRadPerSec;
    public double appliedVolts;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double velocityRps;
    public double appliedVoltage;
    public double supplyCurrent;
    public double tempCelsius;
    public double setPointError;
    public double motorPositionDifferential;
    public double positionRotationsFollower;
    public double setPointErrorFollower;
    public double velocityRpsFollower;
    public double appliedVoltageFollower;
    public double supplyCurrentFollower;
    public double tempCelsiusFollower;
    public double torqueCurrentFollower;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setBrakeMode(boolean enabled) {}

  public default void setPositionPID(
      double p,
      double i,
      double d,
      double kG,
      double kS,
      double kV,
      double kA,
      double cruiseA,
      double cruiseV,
      double cruiseJ) {}

  public default void runCurrent(double currentSetpoint) {}

  public default void stop() {}

  public default void setPosition(double positionSetpointRads) {}
}
