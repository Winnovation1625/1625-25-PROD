package frc.robot.subsystem.superstructure.arm;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystem.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystem.manipulator.Manipulator.GamepieceState;

public class ArmIOSim implements ArmIO {
  private DCMotor armMotorSim = DCMotor.getKrakenX60Foc(1);
  private LinearSystem<N2, N1, N2> armID =
      LinearSystemId.createSingleJointedArmSystem(armMotorSim, ARM_MOI, ARM_GEARING);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(
          gains.kP(),
          gains.kI(),
          gains.kD(),
          new TrapezoidProfile.Constraints(ARM_MAX_VELOCITY, ARM_MAX_ACCELERATION));
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armID,
          armMotorSim,
          ARM_GEARING,
          ARM_LENGTH,
          ARM_MIN_ANGLE_RADS.in(Radians),
          ARM_MAX_ANGLE_RADS.in(Radians),
          true,
          .1);

  public ArmIOSim() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armSim.update(0.02);
    if (DriverStation.isDisabled()) {
      stop();
      inputs.appliedVolts = 0;
    } else {
      var appliedVolts =
          MathUtil.clamp(pidController.calculate(armSim.getAngleRads()), -12.0, 12.0);
      inputs.appliedVolts = appliedVolts;
      setInputVoltage(inputs.appliedVolts);
    }
    inputs.positionRad = armSim.getAngleRads();
    inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.supplyCurrentAmps = armSim.getCurrentDrawAmps();
  }

  @Override
  public void stop() {
    armSim.setInputVoltage(0);
  }

  private void setInputVoltage(double volts) {
    armSim.setInputVoltage(volts);
  }

  @Override
  public void setArmPosition(double positionSetpoint, GamepieceState gamepieceState) {
    pidController.setGoal(positionSetpoint);
  }
}
