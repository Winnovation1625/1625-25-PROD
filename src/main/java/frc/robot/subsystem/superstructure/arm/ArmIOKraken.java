package frc.robot.subsystem.superstructure.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystem.superstructure.arm.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystem.superstructure.arm.ArmIO.ArmIOInputs;

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
    armTalon = new TalonFX(Constants.SUPERSTRUCTURE_CAN_IDS.armMotor(), Constants.CANIVORE_NAME);
    armConfig = new TalonFXConfiguration();

    armConfig.Slot0.kP = gains.kP();
    armConfig.Slot0.kI = gains.kI();
    armConfig.Slot0.kD = gains.kD();
    armConfig.Slot0.kS = gains.ffkS();
    armConfig.Slot0.kV = gains.ffkV();
    armConfig.Slot0.kG = gains.ffkG();
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    armConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    armConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    armConfig.Feedback.FeedbackRemoteSensorID = Constants.SUPERSTRUCTURE_CAN_IDS.armEncoder();
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;
    armConfig.Feedback.FeedbackRotorOffset = ARM_ENCODER_OFFSET.in(Rotations);
    armConfig.Feedback.RotorToSensorRatio = ARM_GEARING; // when using a real encoder this is ARM_GEARING
    armConfig.Feedback.SensorToMechanismRatio = 1.0; // when using a real encoder this is 1
    armConfig.ClosedLoopGeneral.ContinuousWrap = false;
    armConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
    armConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ARM_MAX_ANGLE_RADS.in(Rotations);
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ARM_MIN_ANGLE_RADS.in(Rotations);
    armConfig.MotionMagic.MotionMagicAcceleration = gains.cruiseAcceleration();
    armConfig.MotionMagic.MotionMagicCruiseVelocity = gains.cruiseVelocity();
    armTalon.getConfigurator().apply(armConfig, 1.0);

    positionRotations = armTalon.getPosition();
    velocityRadPerSec = armTalon.getVelocity();
    appliedVolts = armTalon.getMotorVoltage();
    supplyCurrentAmps = armTalon.getSupplyCurrent();
    tempCelsius = armTalon.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(250, positionRotations);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, velocityRadPerSec, appliedVolts, supplyCurrentAmps, tempCelsius);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations, appliedVolts, velocityRadPerSec, supplyCurrentAmps, tempCelsius);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.positionRad =
        Units.rotationsToRadians(positionRotations.getValueAsDouble())
            - ARM_ENCODER_OFFSET.in(Radians);
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.velocityRadPerSec = velocityRadPerSec.getValueAsDouble();
  }

  @Override
  public void setArmPosition(double desiredPositionRad) {
    armTalon.setControl(
        positionControl
            .withPosition(Units.radiansToRotations(desiredPositionRad))
            .withUpdateFreqHz(100));
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
  public void setPID(
      double p,
      double i,
      double d,
      double kS,
      double kV,
      double kA,
      double kG,
      double cruiseA,
      double cruiseV) {
    armConfig.Slot0.kP = p;
    armConfig.Slot0.kI = i;
    armConfig.Slot0.kD = d;
    armConfig.Slot0.kS = kS;
    armConfig.Slot0.kV = kV;
    armConfig.Slot0.kA = kA;
    armConfig.Slot0.kG = kG;
    armConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseV;
    armConfig.MotionMagic.MotionMagicAcceleration = cruiseA;
    armTalon.getConfigurator().apply(armConfig, 0.5);
  }

  @Override
  public void setInverted(boolean inverted) {
    armConfig.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    armTalon.getConfigurator().apply(armConfig, 0.5);
  }
}
