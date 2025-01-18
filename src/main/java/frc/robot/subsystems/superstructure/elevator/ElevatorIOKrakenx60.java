package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;

// private final TalonFXConfiguration config = new TalonFXConfiguration();

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOKrakenx60 implements ElevatorIO {

    private final TalonFX elevatorTalon; 

    private final StatusSignal<Double> positionRotations;
    private final StatusSignal<AngularVelocity> velocityRps;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> tempCelsius;
    private final StatusSignal<Double> setPointError;
    
    private final VoltageOut voltageControl =
     new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final MotionMagicTorqueCurrentFOC positionControl =
     new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public ElevatorIOKrakenx60(){
       elevatorTalon = new TalonFX(0); //will use device Id's when created. 

    // numbers from Arm Constant that will be implemented later. 
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

    //internalPositionRotations = armTalon.getPosition();
    setPointError = elevatorTalon.getClosedLoopError();
    //encoderAbsolutePositionRotations = armCancoder.getAbsolutePosition();
    //encoderRelativePositionRotations = armCancoder.getPosition();
    velocityRps = elevatorTalon.getVelocity();
    appliedVoltage = elevatorTalon.getMotorVoltage();
    supplyCurrent = elevatorTalon.getSupplyCurrent();
    //torqueCurrent = armTalon.getTorqueCurrent();
    tempCelsius = elevatorTalon.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        //internalPositionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        //torqueCurrent,
        tempCelsius);

   // BaseStatusSignal.setUpdateFrequencyForAll(
       // 250, encoderAbsolutePositionRotations, encoderRelativePositionRotations);
       
    elevatorTalon.optimizeBusUtilization(1.0);
    //armCancoder.optimizeBusUtilization(1.0);







    }





}
