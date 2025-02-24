package frc.robot.subsystem.coralmanipulator;

import static frc.robot.subsystem.coralmanipulator.CoralManipulatorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.function.DoubleSupplier;

public class CoralManipulatorIOSim implements CoralManipulatoIO {

  DCMotorSim rollersMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), ROLLERS_REDUCTION, 0.008),
          DCMotor.getKrakenX60Foc(1));
  private double appliedVolts = 0.0;

  public CoralManipulatorIOSim() {}

  @Override
  public void updateInputs(CoralManipulatorIOInputs inputs) {
    inputs.appliedVoltageOut = appliedVolts;
    inputs.appliedCurrentOut = rollersMotor.getCurrentDrawAmps();
    inputs.velocityRadPerSec = rollersMotor.getAngularVelocityRadPerSec();
    inputs.positionRad = rollersMotor.getAngularPositionRad();
  }

  @Override
  public void setVoltage(DoubleSupplier volts) {
    appliedVolts = volts.getAsDouble();
    rollersMotor.setInputVoltage(volts.getAsDouble());
  }

  public void stop() {
    rollersMotor.setInputVoltage(0);
  }
}
