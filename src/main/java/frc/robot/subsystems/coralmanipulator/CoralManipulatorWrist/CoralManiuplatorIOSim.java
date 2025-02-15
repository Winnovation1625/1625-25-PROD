package frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import static frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist.CoralManipulatorWristConstants.*;

public class CoralManiuplatorIOSim {
    
  private DCMotor wristMotors = DCMotor.getKrakenX60Foc(1);

  private LinearSystem<N2, N1, N2> wristId =
      LinearSystemId.createElevatorSystem(
          wristMotors, WRIST_MASS_KG, WRIST_TOLERANCE_METERS, WRIST_GEARING);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(
          WRIST_PID_P,
          WRIST_PID_I,
          WRIST_PID_D,
          new TrapezoidProfile.Constraints(WRIST_MAX_VELOCITY, WRIST_MAX_ACCELERATION));


          private final SingleJointedArmSim m_armSim =
          new SingleJointedArmSim(
              wristId,
              m_gearing,
              SingleJointedArmSim.estimateMOI(Constants.kArmLength, WRIST_MASS_KG),
              Constants.kArmLength,
              Constants.kMinAngleRads,
              Constants.kMaxAngleRads,
              true,
              0,
              Constants.kArmEncoderDistPerPulse,
              0.0 // Add noise with a std-dev of 1 tick
              );
    
}
