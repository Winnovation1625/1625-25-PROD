package frc.robot.subsystem.superstructure.arm;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystem.superstructure.arm.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystem.manipulator.Manipulator.GamepieceState;
import frc.robot.subsystem.superstructure.arm.ArmIO.ArmIOInputs;

public class ArmIOKraken implements ArmIO {

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<AngularVelocity> velocityRadPerSec;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0);

  private final MotionMagicTorqueCurrentFOC positionControl =
      new MotionMagicTorqueCurrentFOC(0.0).withSlot(0);
  private final TalonFX armTalon;
  private final CANcoder armEncoder;
  private final TalonFXConfiguration armConfig;
  private final CANcoderConfiguration armEncoderConfig;
  private final NeutralOut neutralOut = new NeutralOut();

  public ArmIOKraken() {
    armTalon = new TalonFX(Constants.SUPERSTRUCTURE_CAN_IDS.armMotor(), Constants.CANIVORE_NAME);
    armEncoder =
        new CANcoder(Constants.SUPERSTRUCTURE_CAN_IDS.armEncoder(), Constants.CANIVORE_NAME);

    armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = ARM_ENCODER_OFFSET.in(Rotations);
    armEncoder.getConfigurator().apply(armEncoderConfig);

    armConfig = new TalonFXConfiguration();
    armConfig.Slot0.kP = gains.kP();
    armConfig.Slot0.kI = gains.kI();
    armConfig.Slot0.kD = gains.kD();
    armConfig.Slot0.kS = gains.ffkS();
    armConfig.Slot0.kV = gains.ffkV();
    armConfig.Slot0.kG = gains.ffkG();
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    armConfig.Slot1.kP = gains.kP();
    armConfig.Slot1.kI = gains.kI();
    armConfig.Slot1.kD = gains.kD();
    armConfig.Slot1.kS = gains.ffkS();
    armConfig.Slot1.kV = gains.ffkV();
    armConfig.Slot1.kG = gains.ffkG();
    armConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    armConfig.Slot2.kP = gains.kP();
    armConfig.Slot2.kI = gains.kI();
    armConfig.Slot2.kD = gains.kD();
    armConfig.Slot2.kS = gains.ffkS();
    armConfig.Slot2.kV = gains.ffkV();
    armConfig.Slot2.kG = gains.ffkG();
    armConfig.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    armConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    armConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    armConfig.Feedback.FeedbackRemoteSensorID = Constants.SUPERSTRUCTURE_CAN_IDS.armEncoder();
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    armConfig.Feedback.RotorToSensorRatio =
        ARM_GEARING; // when using a real encoder this is ARM_GEARING
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
    inputs.positionRad = Units.rotationsToRadians(positionRotations.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.velocityRadPerSec = velocityRadPerSec.getValueAsDouble();
  }

  @Override
  public void setArmPosition(double desiredPositionRad, GamepieceState gamepieceState) {
    if (gamepieceState == GamepieceState.CORAL_IN_MANIPULATOR
        || gamepieceState == GamepieceState.CORAL_STAGING) {
      armTalon.setControl(
          positionControl
              .withPosition(Units.radiansToRotations(desiredPositionRad))
              .withUpdateFreqHz(100)
              .withSlot(1));
    } else if (gamepieceState == GamepieceState.ALGAE_IN_CLAW) {
      armTalon.setControl(
          positionControl
              .withPosition(Units.radiansToRotations(desiredPositionRad))
              .withUpdateFreqHz(100)
              .withSlot(2));
    } else {
      armTalon.setControl(
          positionControl
              .withPosition(Units.radiansToRotations(desiredPositionRad))
              .withUpdateFreqHz(100)
              .withSlot(0));
    }
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
      double cruiseV,
      int slot) {
    switch (slot) {
      case 0:
        armConfig.Slot0.kP = p;
        armConfig.Slot0.kI = i;
        armConfig.Slot0.kD = d;
        armConfig.Slot0.kS = kS;
        armConfig.Slot0.kV = kV;
        armConfig.Slot0.kA = kA;
        armConfig.Slot0.kG = kG;
        break;
      case 1:
        armConfig.Slot1.kP = p;
        armConfig.Slot1.kI = i;
        armConfig.Slot1.kD = d;
        armConfig.Slot1.kS = kS;
        armConfig.Slot1.kV = kV;
        armConfig.Slot1.kA = kA;
        armConfig.Slot1.kG = kG;
        break;
      default:
        armConfig.Slot2.kP = p;
        armConfig.Slot2.kI = i;
        armConfig.Slot2.kD = d;
        armConfig.Slot2.kS = kS;
        armConfig.Slot2.kV = kV;
        armConfig.Slot2.kA = kA;
        armConfig.Slot2.kG = kG;
    }
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
