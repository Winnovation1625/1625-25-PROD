package frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class CoralManipulatorWristIOKraken implements CoralManipulatorWristIO {

  private final TalonFX wristMotor;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;

  private final MotionMagicTorqueCurrentFOC positionControl =
      new MotionMagicTorqueCurrentFOC(0).withUpdateFreqHz(null);

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final NeutralOut neutralout = new NeutralOut();
  private double prevDesiredArmPosition = 0;
  private boolean isBrake = false;

  public CoralManipulatorWristIOKraken() {

    wristMotor = new TalonFX(0);

    // numbers from Wrist Constants that will be implemented later.
    // config.Slot0.kP = gains.kP();
    // config.Slot0.kI = gains.kI();
    // config.Slot0.kD = gains.kD();
    // config.Slot0.kS = gains.ffkS();
    // config.Slot0.kV = gains.ffkV();
    // config.Slot0.kG = gains.ffkG();
    // config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    // config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    // config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    // config.MotorOutput.Inverted =
    //     inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // config.Feedback.FeedbackRemoteSensorID = Constants.getMotorIds().SHOOTER_CANCODER;
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // config.Feedback.RotorToSensorRatio = armReduction;
    // config.Feedback.SensorToMechanismRatio = 1.0;
    // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(maxAngle);
    // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(minAngle);
    // config.MotionMagic.MotionMagicAcceleration = cruiseAcceleration;
    // config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    // config.MotionMagic.MotionMagicJerk = cruiseJerk;
    // armTalon.getConfigurator().apply(config, 1.0);

    positionRotations = wristMotor.getPosition();
    velocityRps = wristMotor.getVelocity();
    appliedVoltage = wristMotor.getMotorVoltage();
    supplyCurrent = wristMotor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations, velocityRps, appliedVoltage, supplyCurrent);

    wristMotor.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(CoralManipulatorWristIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocityRps, appliedVoltage, supplyCurrent, positionRotations)
        .isOK();

    inputs.appliedCurrentOut = supplyCurrent.getValueAsDouble();
    inputs.appliedVoltageOut = appliedVoltage.getValueAsDouble();
    inputs.positionRad = Units.rotationsToRadians(positionRotations.getValueAsDouble());
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(velocityRps.getValueAsDouble() * 60);
  }

  @Override
  public void setPosition(DoubleSupplier positionSetpoint) {
    wristMotor.setControl(positionControl.withPosition(positionSetpoint.getAsDouble()));
    this.prevDesiredArmPosition = positionSetpoint.getAsDouble();
  }

  @Override
  public void stop() {
    wristMotor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean isBrake) {

    if (isBrake != this.isBrake) {
      this.isBrake = isBrake;
      config.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      wristMotor.getConfigurator().apply(config);
    }
  }
}
