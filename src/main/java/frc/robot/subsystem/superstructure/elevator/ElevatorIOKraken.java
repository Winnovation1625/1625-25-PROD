package frc.robot.subsystem.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystem.superstructure.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialFollower;
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

public class ElevatorIOKraken implements ElevatorIO {

  private final TalonFX elevatorTalon;
  private final TalonFX elevatorFollower;
  private final DifferentialFollower elevator;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final MotionMagicTorqueCurrentFOC positionControl =
      new MotionMagicTorqueCurrentFOC(0).withUpdateFreqHz(250);
  // private final PositionTorqueCurrentFOC followerDifferentialCurrentFOC =
  //     new PositionTorqueCurrentFOC(0).withUpdateFreqHz(null);

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final NeutralOut neutralout = new NeutralOut();
  private final StatusSignal<Double> setPointError;
  private final StatusSignal<Double> motorPositionDifferential;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Angle> positionRotationsFollower;
  private final StatusSignal<Double> setPointErrorFollower;
  private final StatusSignal<AngularVelocity> velocityRpsFollower;
  private final StatusSignal<Voltage> appliedVoltageFollower;
  private final StatusSignal<Current> supplyCurrentFollower;
  private final StatusSignal<Temperature> tempCelsiusFollower;
  private final StatusSignal<Current> torqueCurrentFollower;

  public ElevatorIOKraken() {
    elevatorTalon =
        new TalonFX(Constants.SUPERSTRUCTURE_CAN_IDS.elevatorLeft(), Constants.CANIVORE_NAME);
    elevatorFollower =
        new TalonFX(Constants.SUPERSTRUCTURE_CAN_IDS.elevatorRight(), Constants.CANIVORE_NAME);

    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kS = gains.ffkS();
    config.Slot0.kV = gains.ffkV();
    config.Slot0.kA = gains.ffkA();
    config.Slot0.kG = gains.ffkG();
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = 5.0;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Radians.of(73).in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    config.MotionMagic.MotionMagicAcceleration = gains.cruiseA();
    config.MotionMagic.MotionMagicCruiseVelocity = gains.cruiseV();
    config.MotionMagic.MotionMagicJerk = gains.cruiseJ();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorTalon.getConfigurator().apply(config, 1.0);
    elevatorFollower.getConfigurator().apply(config, 1.0);

    elevator =
        new DifferentialFollower(Constants.SUPERSTRUCTURE_CAN_IDS.elevatorRight(), true)
            .withUpdateFreqHz(250);
    elevatorFollower.setControl(elevator);
    positionRotations = elevatorTalon.getPosition();
    setPointError = elevatorTalon.getClosedLoopError();
    velocityRps = elevatorTalon.getVelocity();
    appliedVoltage = elevatorTalon.getMotorVoltage();
    supplyCurrent = elevatorTalon.getSupplyCurrent();
    tempCelsius = elevatorTalon.getDeviceTemp();
    torqueCurrent = elevatorTalon.getTorqueCurrent();
    motorPositionDifferential = elevatorFollower.getDifferentialClosedLoopError();
    positionRotationsFollower = elevatorFollower.getPosition();
    setPointErrorFollower = elevatorFollower.getClosedLoopError();
    velocityRpsFollower = elevatorFollower.getVelocity();
    appliedVoltageFollower = elevatorFollower.getMotorVoltage();
    supplyCurrentFollower = elevatorFollower.getSupplyCurrent();
    tempCelsiusFollower = elevatorFollower.getDeviceTemp();
    torqueCurrentFollower = elevatorFollower.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(250, positionRotations);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        tempCelsius,
        setPointError,
        torqueCurrent,
        motorPositionDifferential,
        positionRotationsFollower,
        setPointErrorFollower,
        velocityRpsFollower,
        appliedVoltageFollower,
        supplyCurrentFollower,
        tempCelsiusFollower,
        torqueCurrentFollower);

    elevatorTalon.optimizeBusUtilization();
    elevatorFollower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        positionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        tempCelsius,
        motorPositionDifferential,
        positionRotationsFollower,
        setPointErrorFollower,
        velocityRpsFollower,
        appliedVoltageFollower,
        supplyCurrentFollower,
        tempCelsiusFollower,
        torqueCurrentFollower);

    inputs.positionRad = positionRotations.getValue().in(Radians);
    inputs.velocityRadPerSec = velocityRps.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVoltage.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.tempCelsius = tempCelsius.getValue().in(Celsius);
    inputs.motorPositionDifferential = motorPositionDifferential.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.positionRotationsFollower = positionRotationsFollower.getValue().in(Radians);
    inputs.setPointErrorFollower = setPointError.getValue();
    inputs.velocityRpsFollower = velocityRpsFollower.getValue().in(RadiansPerSecond);
    inputs.appliedVoltageFollower = appliedVoltageFollower.getValue().in(Volts);
    inputs.supplyCurrentFollower = supplyCurrentFollower.getValue().in(Amps);
    inputs.tempCelsiusFollower = tempCelsiusFollower.getValue().in(Celsius);
    inputs.torqueCurrentFollower = torqueCurrentFollower.getValue().in(Amps);

    // elevatorFollower.setControl(elevator);
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
  public void setPID(
      double p,
      double i,
      double d,
      double kG,
      double kS,
      double kV,
      double kA,
      double cruiseV,
      double cruiseA,
      double cruiseJ) {

    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    config.Slot0.kG = kG;
    config.Slot0.kA = kA;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    config.MotionMagic.MotionMagicAcceleration = cruiseA;
    config.MotionMagic.MotionMagicCruiseVelocity = cruiseV;
    config.MotionMagic.MotionMagicJerk = cruiseJ;
    tryUntilOk(5, () -> elevatorTalon.getConfigurator().apply(config));
  }

  @Override
  public void setPosition(double positionSetpointRads) {
    elevatorTalon.setControl(
        positionControl
            .withPosition(Units.radiansToRotations(positionSetpointRads))
            .withUpdateFreqHz(250));
  }
}
