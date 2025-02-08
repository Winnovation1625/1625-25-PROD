package frc.robot.subsystems.algaemanipulator;

import static frc.robot.subsystems.algaemanipulator.AlgaeManipulatorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.function.DoubleSupplier;

public class AlgaeManipulatorIOSim implements AlgaeManipulatorIO {

  DCMotorSim manipulatorMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), MANIPULATOR_REDUCTION, 0.008),
          DCMotor.getKrakenX60Foc(1));
  private double appliedVolts = 0.0;

  public AlgaeManipulatorIOSim() {}

  public void updateInputs(AlgaeManipulatorIOInputs inputs) {
    inputs.appliedVoltageOut = appliedVolts;
    inputs.appliedCurrentOut = manipulatorMotor.getCurrentDrawAmps();
    inputs.velocityRadsPerSecond = manipulatorMotor.getAngularVelocityRadPerSec();
    inputs.positionRad = manipulatorMotor.getAngularPositionRad();
  }

  public void setVoltageOutput(DoubleSupplier volts) {
    appliedVolts = volts.getAsDouble();
    manipulatorMotor.setInputVoltage(volts.getAsDouble());
  }

  public void stop() {
    manipulatorMotor.setInputVoltage(0);
  }
}
