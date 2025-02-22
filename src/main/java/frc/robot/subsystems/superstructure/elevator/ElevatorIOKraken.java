package frc.robot.subsystems.superstructure.elevator;

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

public class ElevatorIOKraken implements ElevatorIO {

  private final TalonFX elevatorTalon;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  // private final StatusSignal<Double> setPointError;

  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final MotionMagicTorqueCurrentFOC positionControl =
      new MotionMagicTorqueCurrentFOC(0).withUpdateFreqHz(null);

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final NeutralOut neutralout = new NeutralOut();

  public ElevatorIOKraken() {
    elevatorTalon = new TalonFX(0); // will use device Id's when created.

    // numbers from Elevator Constant that will be implemented later.
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

    positionRotations = elevatorTalon.getPosition();
    // setPointError = elevatorTalon.getClosedLoopError();
    velocityRps = elevatorTalon.getVelocity();
    appliedVoltage = elevatorTalon.getMotorVoltage();
    supplyCurrent = elevatorTalon.getSupplyCurrent();
    tempCelsius = elevatorTalon.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations, velocityRps, appliedVoltage, supplyCurrent, tempCelsius);

    // elevatorTalon.optimizeBusUtilization(1.0);

  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        positionRotations, velocityRps, appliedVoltage, supplyCurrent, tempCelsius);

    inputs.positionRad =
        Units.rotationsToRadians(positionRotations.getValueAsDouble()); // / reduction;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocityRps.getValueAsDouble()); // / reduction;
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runCurrent(double currentSetpoint) {

    elevatorTalon.setControl(currentControl.withOutput(currentSetpoint));
  }

  @Override
  public void stop() {
    elevatorTalon.setControl(neutralout);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    elevatorTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(double p, double i, double d) {

    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    elevatorTalon.getConfigurator().apply(config);
  }

  @Override
  public void setPosition(double positionSetpoint) {
    elevatorTalon.setControl(
        positionControl
            .withPosition(Units.radiansToRotations(positionSetpoint))
            .withUpdateFreqHz(50));
  }
}
