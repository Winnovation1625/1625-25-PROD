package frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers;

import static frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers.CoralManipulatorRollersConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.function.DoubleSupplier;

public class CoralManipulatorRollersIOSim implements CoralManipulatorRollersIO {

  DCMotorSim rollersMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), ROLLERS_REDUCTION, 0.008),
          DCMotor.getKrakenX60Foc(1));
  private double appliedVolts = 0.0;

  public CoralManipulatorRollersIOSim() {}

  @Override
  public void updateInputs(CoralManipulatorRollersIOInputs inputs) {
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
