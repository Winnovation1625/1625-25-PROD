package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private DCMotor elevatorMotors = DCMotor.getKrakenX60Foc(2);
  private LinearSystem<N2, N1, N2> elevatorId =
      LinearSystemId.createElevatorSystem(
          elevatorMotors, ELEVATOR_MASS_KG, ELEVATOR_TOLERANCE_METERS, ELEVATOR_GEARING);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(
          ELEVATOR_PID_P,
          ELEVATOR_PID_I,
          ELEVATOR_PID_D,
          new TrapezoidProfile.Constraints(ELEVATOR_MAX_VELOCITY, ELEVATOR_MAX_ACCELERATION));

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorId,
          elevatorMotors,
          ELEVATOR_MIN_HEIGHT_METERS,
          ELEVATOR_MAX_HEIGHT_METERS,
          true,
          ELEVATOR_MIN_HEIGHT_METERS);

  public ElevatorIOSim() {
    elevatorSim.setState(ELEVATOR_MIN_HEIGHT_METERS, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.positionRad = elevatorSim.getPositionMeters() / ELEVATOR_DRUM_RADIUS;
    inputs.velocityRadPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.supplyCurrentAmps = Math.abs(elevatorSim.getCurrentDrawAmps());

    elevatorSim.update(0.02);
    if (DriverStation.isDisabled()) {
      stop();
      inputs.appliedVolts = 0;
    } else {
      var output = MathUtil.clamp(pidController.calculate(inputs.positionRad), -12, 12);
      elevatorSim.setInputVoltage(output);
      inputs.appliedVolts = output;
    }
  }

  @Override
  public void stop() {
    elevatorSim.setInputVoltage(0);
  }

  @Override
  public void setPosition(double positionSetpoint) {
    pidController.setGoal(positionSetpoint / ELEVATOR_DRUM_RADIUS);
  }
}
