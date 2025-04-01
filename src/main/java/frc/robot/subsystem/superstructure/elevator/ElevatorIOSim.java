package frc.robot.subsystem.superstructure.elevator;

import static frc.robot.subsystem.superstructure.elevator.ElevatorConstants.*;

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
          elevatorMotors, ELEVATOR_MASS_KG, ELEVATOR_DRUM_RADIUS, ELEVATOR_GEARING);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(
          positionGains.kP(),
          positionGains.kI(),
          positionGains.kD(),
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

    elevatorSim.update(0.02);
    if (DriverStation
        .isDisabled()) { // TODO: This is probably not the only time you want to make sure it
      // doesn't move
      stop();
      inputs.appliedVolts = 0;
      pidController.reset(inputs.positionRad);
    } else {
      var output =
          MathUtil.clamp(
              pidController.calculate(elevatorSim.getPositionMeters() / ELEVATOR_DRUM_RADIUS),
              -12,
              12);
      inputs.appliedVolts = output;
      setInputVoltage(inputs.appliedVolts);
    }

    inputs.positionRad = elevatorSim.getPositionMeters() / ELEVATOR_DRUM_RADIUS;
    inputs.positionRotationsFollower = inputs.positionRad;
    inputs.velocityRadPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.supplyCurrentAmps = Math.abs(elevatorSim.getCurrentDrawAmps());
  }

  public void setInputVoltage(double volts) {
    elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    elevatorSim.setInputVoltage(0);
  }

  @Override
  public void setPosition(double positionSetpointRads) {
    pidController.setGoal(positionSetpointRads);
  }

  @Override
  public void setPositionPID(
      double p,
      double i,
      double d,
      double kG,
      double kS,
      double kV,
      double kA,
      double cruiseA,
      double cruiseV,
      double cruiseJ) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.reset(elevatorSim.getPositionMeters() / ELEVATOR_DRUM_RADIUS);
  }
}
