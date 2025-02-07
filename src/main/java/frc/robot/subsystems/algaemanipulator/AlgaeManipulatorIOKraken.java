package frc.robot.subsystems.algaemanipulator;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class AlgaeManipulatorIOKraken implements AlgaeManipulatorIO {

  private final TalonFX motor;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent; // supply current or torqe current?
  private final StatusSignal<Angle> position;

  private final VoltageOut voltageControl =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public AlgaeManipulatorIOKraken() {
    motor = new TalonFX(0); // motor contstants ID is needed
    

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
    // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(maxAngle);
    // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(minAngle);
    // config.MotionMagic.MotionMagicAcceleration = cruiseAcceleration;
    // config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    // config.MotionMagic.MotionMagicJerk = cruiseJerk;
    // armTalon.getConfigurator().apply(config, 1.0);

    // // Status signals
    position = motor.getPosition();
    velocity = motor.getVelocity();
    supplyCurrent = motor.getSupplyCurrent();
    appliedVoltage = motor.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, position, velocity, appliedVoltage, supplyCurrent);

    // Optimize bus utilization
    motor.optimizeBusUtilization(1.0);
  }

  @Override
  public void updateInputs(AlgaeManipulatorIOInputs inputs) {
    inputs.appliedCurrentOut = supplyCurrent.getValueAsDouble();
    inputs.appliedVoltageOut = appliedVoltage.getValueAsDouble();
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(velocity.getValueAsDouble() / 60);
  }

  @Override
  public void setCurrentOutput(DoubleSupplier current) {
    motor.setControl(currentControl.withOutput(current.getAsDouble()));
  }

  @Override
  public void setVoltageOutput(DoubleSupplier voltage) {
    motor.setControl(voltageControl.withOutput(voltage.getAsDouble()));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
