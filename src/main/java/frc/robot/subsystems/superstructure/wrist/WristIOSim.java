package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.superstructure.wrist.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.function.DoubleSupplier;

public class WristIOSim implements WristIO {

  private DCMotor wristMotors = DCMotor.getKrakenX60Foc(1);

  private LinearSystem<N2, N1, N2> wristId =
      LinearSystemId.createSingleJointedArmSystem(
          wristMotors, WRIST_MOI, WRIST_GEARING);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(
          WRIST_PID_P,
          WRIST_PID_I,
          WRIST_PID_D,
          new TrapezoidProfile.Constraints(WRIST_MAX_VELOCITY, WRIST_MAX_ACCELERATION));

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          wristId,
          wristMotors, // needs real number
          WRIST_GEARING,
          WRIST_LENGTH,
          Units.degreesToRadians(minAngle),
          Units.degreesToRadians(maxAngle),
          true,
          Double.MIN_VALUE
          // Add noise with a std-dev of 1 tick
          );

  public WristIOSim() {}

  @Override
  public void updateInputs(WristIOInputs inputs) {
    sim.update(loopPeriodSecs);
    // Reset voltages when disabled
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0);
      inputs.appliedVoltageOut = 0;
    } else {
      var appliedVolts = MathUtil.clamp(pidController.calculate(inputs.positionRad), -12.0, 12.0);
      inputs.appliedVoltageOut = appliedVolts;
      setInputVoltage(inputs.appliedVoltageOut);
    }
    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedCurrentOut = sim.getCurrentDrawAmps();
  }

  private void setInputVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPosition(DoubleSupplier desiredArmPosition) {
    pidController.setGoal(desiredArmPosition.getAsDouble());
  }
}
