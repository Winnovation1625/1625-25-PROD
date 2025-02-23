package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {

    public double positionRad;
    public double velocityRadPerSec;
    public double appliedVolts;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
  ;

  public default void setBrakeMode(boolean enabled) {}
  ;

  public default void setPID(double p, double i, double d) {}
  ;

  public default void runCurrent(double currentSetpoint) {}
  ;

  public default void stop() {}
  ;

  public default void setPosition(double positionSetpointRads) {}
  ;
}
