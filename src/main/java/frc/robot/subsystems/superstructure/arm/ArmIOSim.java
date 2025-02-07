package frc.robot.subsystems.superstructure.arm;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO{
    private DCMotor armMotorSim = DCMotor.getKrakenX60(1);
    private LinearSystem<N2,N1,N2> armID = LinearSystemId.createSingleJointedArmSystem(armMotorSim, ARM_MOI,ARM_GEARING);
    private final ProfiledPIDController pidController = new ProfiledPIDController(ARM_PID_P, ARM_PID_I, ARM_PID_D, new TrapezoidProfile.Constraints(ARM_MAX_VELOCITY,ARM_MAX_ACCELERATION));
    private SingleJointedArmSim armSim = new SingleJointedArmSim(armID, armMotorSim, ARM_GEARING, ARM_LENGTH, ARM_MIN_ANGLE_RADS, ARM_MAX_ANGLE_RADS, true, 0, null);

    public ArmIOSim(){
        armSim.setState(ARM_MIN_ANGLE_RADS,0.0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        inputs.positionRad = armSim.getAngleRads();
        inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
        inputs.supplyCurrentAmps = armSim.getCurrentDrawAmps();

        armSim.update(0.02);
        if(DriverStation.isDisabled()){
            stop();
            inputs.appliedVolts=0;
        }
        else{
            var output = MathUtil.clamp(pidController.calculate(inputs.positionRad),-12,12);
            armSim.setInputVoltage(output);
            inputs.appliedVolts = output;
        }
    }
    
    @Override
    public void stop(){
        armSim.setInputVoltage(0);
    }

    @Override
    public void setArmPosition(double positionSetpoint){
        pidController.setGoal(positionSetpoint);
    }
}