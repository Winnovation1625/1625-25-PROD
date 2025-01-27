package frc.robot.subsystems.algaemanipulator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import au.grapplerobotics.ConfigurationFailedException;

public class AlgaeManipulatorIOKrakenx60 implements AlgaeManipulatorIO{

    private final TalonFX motor;
    private final LaserCan sensor;
    private final StatusSignal<Velocity> velocityRadPerSec;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent; //supply current or torqe current?
    private final StatusSignal<AngularVelocity> positionRad;

    private LaserCan.Measurement[] measurements = new LaserCan.Measurement[5];
    
    private final VoltageOut voltageControl = 
        new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentControl = 
        new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

        private final TalonFXConfiguration config = new TalonFXConfiguration();

    public AlgaeManipulatorIOKrakenx60() {
        motor = new TalonFX(0); //motor contstants ID is needed
        sensor = new LaserCan(0); // constants ID is needed

        try{
            configureLaserCans(sensor);
        } catch(ConfigurationFailedException e){
            System.out.println("initial intake cannot config " + e.getMessage());
        }

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
        // internalPositionRotations = armTalon.getPosition();
        // encoderAbsolutePositionRotations = armCancoder.getAbsolutePosition();
        // encoderRelativePositionRotations = armCancoder.getPosition();
        // velocityRps = armTalon.getVelocity();
        // appliedVoltage = armTalon.getMotorVoltage();
        // supplyCurrent = armTalon.getSupplyCurrent();
        // torqueCurrent = armTalon.getTorqueCurrent();
        // tempCelsius = armTalon.getDeviceTemp();
        // BaseStatusSignal.setUpdateFrequencyForAll(
        //     100,
        //     internalPositionRotations,
        //     velocityRps,
        //     appliedVoltage,
        //     supplyCurrent,
        //     torqueCurrent,
        //     tempCelsius);

        // BaseStatusSignal.setUpdateFrequencyForAll(
        //     250, encoderAbsolutePositionRotations, encoderRelativePositionRotations);

        // // Optimize bus utilization
        // armTalon.optimizeBusUtilization(1.0);
        // armCancoder.optimizeBusUtilization(1.0);


    }

    @Override
    public void updateInputs(AlgaeManipulatorIOInputs inputs){
        inputs.appliedCurrentOut = motor.getSupplyCurrent().getValueAsDouble();
        inputs.appliedVoltageOut = motor.getMotorVoltage().getValueAsDouble();
        inputs.positionRad = motor.getPosition().getValueAsDouble()
    }

    private void configureLaserCans(LaserCan laserCan) throws ConfigurationFailedException {
    laserCan.setRangingMode(RangingMode.SHORT);
    laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
  }
    
}
