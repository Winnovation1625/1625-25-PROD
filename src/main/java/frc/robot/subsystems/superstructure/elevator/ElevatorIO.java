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

  public default void setBreakMode(boolean enabled) {}
  ;

  public default void setPID(double p, double i, double d) {}
  ;

  public default void runCurrent(double currentSetpoint) {}
  ;

  public default void stop() {}
  ;

  // public default void setVelocity(double velocitySetpoint) {};

  // public default void setPosition(double positionSetpoint) {};

  // public default void setVolts(double voltageSetpoint) {};

}
