package frc.robot.subsystems.superstructure.arm;

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
  ;

  public default void setArmPosition(double desiredPosition) {}
  ;

  public default void setBrakeMode(boolean enabled) {}
  ;

  public default void setPID(double p, double i, double d) {}
  ;

  public default void runCurrent(double amps) {}
  ;

  public default void stop() {}
  ;

  public default void runVolts(double volts) {}
  ;
}
