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

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final MotionMagicTorqueCurrentFOC positionControl =
      new MotionMagicTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final MotionMagicTorqueCurrentFOC positionControlFollower =
      new MotionMagicTorqueCurrentFOC(0).withUpdateFreqHz(0);
  // private final PositionTorqueCurrentFOC followerDifferentialCurrentFOC =
  //     new PositionTorqueCurrentFOC(0).withUpdateFreqHz(250).withSlot(1);

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final TalonFXConfiguration followerConfig = new TalonFXConfiguration();
  private final NeutralOut neutralout = new NeutralOut();
  private final StatusSignal<Double> setPointError;
  private final StatusSignal<Double> setPointErrorFollower;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Angle> positionRotationsFollower;
  private final StatusSignal<AngularVelocity> velocityRpsFollower;
  private final StatusSignal<Voltage> appliedVoltageFollower;
  private final StatusSignal<Current> supplyCurrentFollower;
  private final StatusSignal<Temperature> tempCelsiusFollower;
  private final StatusSignal<Current> torqueCurrentFollower;

  public ElevatorIOKraken() {
    elevatorTalon =
        new TalonFX(Constants.SUPERSTRUCTURE_CAN_IDS.elevatorRight(), Constants.CANIVORE_NAME);
    elevatorFollower =
        new TalonFX(Constants.SUPERSTRUCTURE_CAN_IDS.elevatorLeft(), Constants.CANIVORE_NAME);

    config.Slot0.kP = positionGains.kP();
    config.Slot0.kI = positionGains.kI();
    config.Slot0.kD = positionGains.kD();
    config.Slot0.kS = positionGains.ffkS();
    config.Slot0.kV = positionGains.ffkV();
    config.Slot0.kA = positionGains.ffkA();
    config.Slot0.kG = positionGains.ffkG();
    // config.Slot1.kP = positionGains.kP();
    // config.Slot1.kI = positionGains.kI();
    // config.Slot1.kD = positionGains.kD();
    // config.Slot1.kS = positionGains.ffkS();
    // config.Slot1.kV = positionGains.ffkV();
    // config.Slot1.kA = positionGains.ffkA();
    // config.Slot1.kG = positionGains.ffkG();

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = 5.0;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Radians.of(69.2913385827).in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
    config.MotionMagic.MotionMagicAcceleration = positionGains.cruiseA();
    config.MotionMagic.MotionMagicCruiseVelocity = positionGains.cruiseV();
    config.MotionMagic.MotionMagicJerk = positionGains.cruiseJ();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // config.Voltage.SupplyVoltageTimeConstant = 0.1;
    // config.Feedback.VelocityFilterTimeConstant = 0.05;
    config.TorqueCurrent.TorqueNeutralDeadband = 5.0;
    followerConfig.Slot0.kP = positionGains.kP();
    followerConfig.Slot0.kI = positionGains.kI();
    followerConfig.Slot0.kD = positionGains.kD();
    followerConfig.Slot0.kS = positionGains.ffkS();
    followerConfig.Slot0.kV = positionGains.ffkV();
    followerConfig.Slot0.kA = positionGains.ffkA();
    followerConfig.Slot0.kG = positionGains.ffkG();
    // config.Slot1.kP = positionGains.kP();
    // config.Slot1.kI = positionGains.kI();
    // config.Slot1.kD = positionGains.kD();
    // config.Slot1.kS = positionGains.ffkS();
    // config.Slot1.kV = positionGains.ffkV();
    // config.Slot1.kA = positionGains.ffkA();
    // config.Slot1.kG = positionGains.ffkG();

    followerConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    followerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    followerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    followerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    followerConfig.Feedback.SensorToMechanismRatio = 5.0;
    followerConfig.TorqueCurrent.TorqueNeutralDeadband = 5.0;
    followerConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
    followerConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
    followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    followerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Radians.of(69.2913385827).in(Rotations);
    followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    followerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    followerConfig.MotionMagic.MotionMagicAcceleration = positionGains.cruiseA();
    followerConfig.MotionMagic.MotionMagicCruiseVelocity = positionGains.cruiseV();
    followerConfig.MotionMagic.MotionMagicJerk = positionGains.cruiseJ();
    // config.DifferentialSensors.DifferentialSensorSource =
    //     DifferentialSensorSourceValue.RemoteTalonFX_Diff;
    // config.DifferentialSensors.DifferentialTalonFXSensorID =
    //     Constants.SUPERSTRUCTURE_CAN_IDS.elevatorLeft();
    elevatorTalon.getConfigurator().apply(config, 1.0);
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorFollower.getConfigurator().apply(followerConfig, 1.0);

    // elevator =
    //     new DifferentialFollower(Constants.SUPERSTRUCTURE_CAN_IDS.elevatorRight(), true)
    //         .withUpdateFreqHz(250);
    // elevatorMechanism = new DifferentialMechanism(elevatorTalon, elevatorFollower, false);
    // elevatorMechanism.applyConfigs();
    positionRotations = elevatorTalon.getPosition();
    setPointError = elevatorTalon.getClosedLoopError();
    velocityRps = elevatorTalon.getVelocity();
    appliedVoltage = elevatorTalon.getMotorVoltage();
    supplyCurrent = elevatorTalon.getSupplyCurrent();
    tempCelsius = elevatorTalon.getDeviceTemp();
    torqueCurrent = elevatorTalon.getTorqueCurrent();
    // motorPositionDifferential = elevatorFollower.getDifferentialClosedLoopError();
    positionRotationsFollower = elevatorFollower.getPosition();
    setPointErrorFollower = elevatorFollower.getClosedLoopError();
    velocityRpsFollower = elevatorFollower.getVelocity();
    appliedVoltageFollower = elevatorFollower.getMotorVoltage();
    supplyCurrentFollower = elevatorFollower.getSupplyCurrent();
    tempCelsiusFollower = elevatorFollower.getDeviceTemp();
    torqueCurrentFollower = elevatorFollower.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(250, positionRotations, positionRotationsFollower);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        tempCelsius,
        setPointError,
        torqueCurrent,
        velocityRpsFollower,
        appliedVoltageFollower,
        supplyCurrentFollower,
        tempCelsiusFollower,
        setPointErrorFollower,
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
        torqueCurrent,
        torqueCurrentFollower,
        setPointError,
        velocityRpsFollower,
        positionRotationsFollower,
        appliedVoltageFollower,
        supplyCurrentFollower,
        tempCelsiusFollower);

    inputs.positionRad = positionRotations.getValue().in(Radians);
    inputs.velocityRadPerSec = velocityRps.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVoltage.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.tempCelsius = tempCelsius.getValue().in(Celsius);
    // inputs.motorPositionDifferential = motorPositionDifferential.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.positionRotationsFollower = positionRotationsFollower.getValue().in(Radians);
    inputs.setPointErrorFollower = setPointError.getValue();
    inputs.velocityRpsFollower = velocityRpsFollower.getValue().in(RadiansPerSecond);
    inputs.appliedVoltageFollower = appliedVoltageFollower.getValue().in(Volts);
    inputs.supplyCurrentFollower = supplyCurrentFollower.getValue().in(Amps);
    inputs.tempCelsiusFollower = tempCelsiusFollower.getValue().in(Celsius);
    inputs.torqueCurrentFollower = torqueCurrentFollower.getValue().in(Amps);
    // elevatorMechanism.periodic();
    // elevatorFollower.setControl(elevator);
  }

  @Override
  public void runCurrent(double currentSetpoint) {
    elevatorFollower.setControl(currentControl.withOutput(currentSetpoint));
    elevatorTalon.setControl(currentControl.withOutput(currentSetpoint));
  }

  @Override
  public void stop() {
    elevatorTalon.setControl(neutralout);
    elevatorFollower.setControl(neutralout);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    elevatorTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    elevatorFollower.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
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
    followerConfig.Slot0.kP = p;
    followerConfig.Slot0.kI = i;
    followerConfig.Slot0.kD = d;
    followerConfig.Slot0.kG = kG;
    followerConfig.Slot0.kA = kA;
    followerConfig.Slot0.kV = kV;
    followerConfig.Slot0.kS = kS;
    followerConfig.MotionMagic.MotionMagicAcceleration = cruiseA;
    followerConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseV;
    followerConfig.MotionMagic.MotionMagicJerk = cruiseJ;
    tryUntilOk(5, () -> elevatorTalon.getConfigurator().apply(config));
    tryUntilOk(5, () -> elevatorFollower.getConfigurator().apply(followerConfig));
  }

  @Override
  public void setPosition(double positionSetpointRads) {
    elevatorTalon.setControl(
        positionControl
            .withPosition(Units.radiansToRotations(positionSetpointRads))
            .withUpdateFreqHz(250));
    elevatorFollower.setControl(
        positionControlFollower
            .withPosition(Units.radiansToRotations(positionSetpointRads))
            .withUpdateFreqHz(250));
  }
}
