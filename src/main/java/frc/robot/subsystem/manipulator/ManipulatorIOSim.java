package frc.robot.subsystem.manipulator;

import static frc.robot.subsystem.manipulator.ManipulatorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.function.DoubleSupplier;

public class ManipulatorIOSim implements ManipulatorIO {

  private double appliedVolts;

  DCMotorSim manipulatorMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), ROLLERS_REDUCTION, 0.008),
          DCMotor.getKrakenX60Foc(1));

  public ManipulatorIOSim() {}

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    inputs.appliedVoltageOut = appliedVolts;
    inputs.appliedCurrentOut = manipulatorMotor.getCurrentDrawAmps();
    inputs.velocityRadsPerSecond = manipulatorMotor.getAngularVelocityRadPerSec();
    inputs.positionRad = manipulatorMotor.getAngularPositionRad();
  }

  @Override
  public void setVoltage(DoubleSupplier voltage) {
    appliedVolts = voltage.getAsDouble();
    manipulatorMotor.setInputVoltage(voltage.getAsDouble());
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    manipulatorMotor.setInputVoltage(0.0);
  }
}
