package frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class CoralManipulatorRollersIOKraken implements CoralManipulatorRollersIO {

  TalonFX rollersMotor;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Angle> position;

  private final VoltageOut voltageControl =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public CoralManipulatorRollersIOKraken() {

    rollersMotor = new TalonFX(0);

    // config.Slot0.kP = gains.kP();
    // config.Slot0.kI = gains.kI();
    // config.Slot0.kD = gains.kD();
    // config.Slot0.kS = gains.ffkS();
    // config.Slot0.kV = gains.ffkV();
    // config.Slot0.kG = gains.ffkG();
    //  config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    // config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    // config.MotorOutput.Inverted =
    //     inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // config.Feedback.FeedbackRemoteSensorID = Constants.getMotorIds().INTAKE_CANCODER;
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // config.Feedback.RotorToSensorRatio = armReduction;
    // config.Feedback.SensorToMechanismRatio = 1.0;
    // config.MotionMagic.MotionMagicAcceleration = cruiseAcceleration;
    // config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    // config.MotionMagic.MotionMagicJerk = cruiseJerk;
    // armTalon.getConfigurator().apply(config, 1.0);

    // // Status signals
    position = rollersMotor.getPosition();
    velocity = rollersMotor.getVelocity();
    supplyCurrent = rollersMotor.getSupplyCurrent();
    appliedVoltage = rollersMotor.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, position, velocity, appliedVoltage, supplyCurrent);

    // Optimize bus utilization
    rollersMotor.optimizeBusUtilization(0, 1);
  }

  @Override
  public void updateInputs(CoralManipulatorRollersIOInputs inputs) {
    inputs.appliedCurrentOut = supplyCurrent.getValueAsDouble();
    inputs.appliedVoltageOut = appliedVoltage.getValueAsDouble();
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(velocity.getValueAsDouble() / 60);
  }

  @Override
  public void setVoltage(DoubleSupplier voltageSetpoint) {
    rollersMotor.setControl(voltageControl.withOutput(voltageSetpoint.getAsDouble()));
  }

  @Override
  public void stop() {
    rollersMotor.stopMotor();
  }
}
