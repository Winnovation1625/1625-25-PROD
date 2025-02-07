package frc.robot.subsystems.superstructure.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import lombok.Setter;

public class Arm {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIoInputsAutoLogged();
    private final ArmVisualizer visualizer = new ArmVisualizer();
    // private static final LoggedTunableNumber kP =
    //     new LoggedTunableNumber("Arm/kP", gains.kP());
    // private static final LoggedTunableNumber kI =
    //     new LoggedTunableNumber("Arm/kI", gains.kI());
    // private static final LoggedTunableNumber kD =
    //     new LoggedTunableNumber("Arm/kD", gains.kD());
    // private static final LoggedTunableNumber kS =
    //     new LoggedTunableNumber("Arm/kS", gains.ffkS());
    // private static final LoggedTunableNumber kV =
    //     new LoggedTunableNumber("Arm/kV", gains.ffkV());
    // private static final LoggedTunableNumber kA =
    //     new LoggedTunableNumber("Arm/kA", gains.ffkA());
    // private static final LoggedTunableNumber kG =
    //     new LoggedTunableNumber("Arm/kG", gains.ffkG());
    // private static final LoggedTunableNumber cruiseV =
    //     new LoggedTunableNumber("Arm/cruiseV", cruiseVelocity);
    // private static final LoggedTunableNumber cruiseA =
    //     new LoggedTunableNumber("Arm/cruiseA", cruiseAcceleration);
    // private static final LoggedTunableNumber cruiseJ =
    //     new LoggedTunableNumber("Arm/cruiseJ", cruiseJerk);
    @RequiredArgsConstructor
    public enum ArmState{
        STOP(),
        STOW(),
        ALGL(),
        ALGH(),
        CLVL1(),
        CLVL2(),
        ClVL3(),
        CLVL4(),
        INTAKE(),
        PROCESS(),
        BARGE();
        private  DoubleSupplier armSetpointSupplier;

        private double getRads(){
            return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
        }
    }
    @AutoLogOutput(key = "Superstructure/Arm/ArmState")
    //@Getter
    //@Setter
    private ArmState armState = ArmState.STOW;

    public Arm(ArmIO io) {
      this.io = io;
    }

    private boolean characterizing;
    private boolean brakeModeEnabled;
    private BooleanSupplier disableSupplier = DriverStation::isDisabled;
    private BooleanSupplier coastSupplier = () -> false;

    public void periodic(){
        io.updateInputs(inputs);

        Logger.processInputs("Superstructre/Arm", inputs);
        if(disableSupplier.getAsBoolean()||armState==ArmState.STOP){
            io.stop();
        }
        io.setBrakeMode(!coastSupplier.getAsBoolean()||armState==ArmState.STOW);

        if(!characterizing&&!disableSupplier.getAsBoolean()&&armState!=ArmState.STOP){
            Logger.recordOutput("Superstructure/Arm/ArmSetpoint",Units.radiansToDegrees(armState.getRads()));
        }

        @AutoLogOutput(key = "Superstructre/Arm/AtGoal");
        public boolean atGoal(){
            return Equals.util.epsilonEquals(inputs.positionRad,armState.getRads,ARM_TOLERANCE);
        }

        public void setBreakMode(boolean enabled){
            if(brakeModeEnabled==enabled)return;
            brakeModeEnabled=enabled;
            io.setBrakeMode(enabled);
        }

        public void runCharacterizaiton(double amps){
            characterizing=true;
            io.runCurrent(amps);
        }

        public double getCharacterizationVelocity(){
            return inputs.velocityRadPerSec;
        }

        public void endCharacterization(){
            characterizing=false;
        }

        public void moveArm(){
            armState=ArmState.CLVL1;
            io.setArmPosition(ArmState.CLVL1.getRads());
        }

        public void tempStop(){
            io.stop();
        }

        public Command tempCommand(){
            return startEnd(()->moveArm(),()->tempStop());
        }
    }
}