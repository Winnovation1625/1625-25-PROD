package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.superstructure.arm.ArmIO.ArmIOInputs;
import java.util.function.DoubleSupplier;

public class ArmIOKraken implements ArmIO {
  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<AngularVelocity> velocityRadPerSec;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0);
  private final MotionMagicTorqueCurrentFOC positionControl = new MotionMagicTorqueCurrentFOC(0.0);
  private final TalonFX armTalon;
  private final TalonFXConfiguration armConfig;
  private final NeutralOut neutralOut = new NeutralOut();

  public ArmIOKraken() {
    armTalon = new TalonFX(0);
    armConfig = new TalonFXConfiguration();

    // armConfig.Slot0.kP = gains.kP();
    // armConfig.Slot0.kI = gains.kI();
    // armConfig.Slot0.kD = gains.kD();
    // armConfig.Slot0.kS = gains.ffkS();
    // armConfig.Slot0.kV = gains.ffkV();
    // armConfig.Slot0.kG = gains.ffkG();
    // armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // armConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    // armConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    // armConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    // armConfig.MotorOutput.Inverted =
    //     inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    // armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // armConfig.Feedback.FeedbackRemoteSensorID = Constants.getMotorIds().SHOOTER_CANCODER;
    // armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // armConfig.Feedback.RotorToSensorRatio = armReduction;
    // armConfig.Feedback.SensorToMechanismRatio = 1.0;
    // armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(maxAngle);
    // armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(minAngle);
    // armConfig.MotionMagic.MotionMagicAcceleration = cruiseAcceleration;
    // armConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    // armConfig.MotionMagic.MotionMagicJerk = cruiseJerk;
    // armTalon.getConfigurator().apply(armConfig, 1.0);

    positionRotations = armTalon.getPosition();
    velocityRadPerSec = armTalon.getVelocity();
    appliedVolts = armTalon.getMotorVoltage();
    supplyCurrentAmps = armTalon.getSupplyCurrent();
    tempCelsius = armTalon.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations, velocityRadPerSec, appliedVolts, supplyCurrentAmps, tempCelsius);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations, appliedVolts, velocityRadPerSec, supplyCurrentAmps, tempCelsius);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.positionRad = Units.rotationsToRadians(positionRotations.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.velocityRadPerSec = velocityRadPerSec.getValueAsDouble();
  }

  @Override
  public void setArmPosition(DoubleSupplier desiredPositionRad) {
    armTalon.setControl(
        positionControl.withPosition(Units.radiansToRotations(desiredPositionRad.getAsDouble())));
  }

  @Override
  public void stop() {
    armTalon.setControl(neutralOut);
  }

  @Override
  public void runCurrent(double currentSetpoint) {
    armTalon.setControl(currentControl.withOutput(currentSetpoint));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    armTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(double p, double i, double d) {
    armConfig.Slot0.kP = p;
    armConfig.Slot0.kI = i;
    armConfig.Slot0.kP = p;
    armTalon.getConfigurator().apply(armConfig);
  }
}
