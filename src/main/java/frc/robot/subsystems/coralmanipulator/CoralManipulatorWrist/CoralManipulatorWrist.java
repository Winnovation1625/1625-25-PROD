package frc.robot.subsystems.coralmanipulator.CoralManipulatorWrist;

import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;
import lombok.Getter;
import lombok.Setter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralManipulatorWrist {

    private final CoralManipulatorWristIO io;
    //private final CoralManipulatorWristIOInputsAutoLogged inputs = new CoralManipulatorWristIOInputsAutoLogged();
    // private final IntakeArmVisualizer visualizer =
    //   new IntakeArmVisualizer(IntakeArmConstants.intakePose);
    // private static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakeArm/kP", gains.kP());
    // private static final LoggedTunableNumber kI = new LoggedTunableNumber("IntakeArm/kI", gains.kI());
    // private static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakeArm/kD", gains.kD());
    // private static final LoggedTunableNumber kS =
    //     new LoggedTunableNumber("IntakeArm/kS", gains.ffkS());
    // private static final LoggedTunableNumber kV =
    //     new LoggedTunableNumber("IntakeArm/kV", gains.ffkV());
    // private static final LoggedTunableNumber kA =
    //     new LoggedTunableNumber("IntakeArm/kA", gains.ffkA());
    // private static final LoggedTunableNumber kG =
    //     new LoggedTunableNumber("IntakeArm/kG", gains.ffkG());
    // private static final LoggedTunableNumber cruiseV =
    //     new LoggedTunableNumber("IntakeArm/cruiseV", cruiseVelocity);
    // private static final LoggedTunableNumber cruiseA =
    //     new LoggedTunableNumber("IntakeArm/cruiseA", cruiseAcceleration);
    // private static final LoggedTunableNumber cruiseJ =
    //     new LoggedTunableNumber("IntakeArm/cruiseJ", cruiseJerk);

    public CoralManipulatorWrist(CoralManipulatorWristIO io) {
        this.io = io;
      }

    @AutoLogOutput @Getter @Setter private WristState wristState = 
    WristState.STOW;
    private boolean characterizing;
    private boolean brakeModeEnabled;
    private BooleanSupplier disableSupplier = DriverStation::isDisabled;
    private BooleanSupplier coastSupplier = () -> false;


  @RequiredArgsConstructor
  public enum WristState {
    STOW(new LoggedTunableNumber("Wrist/Stow", 0)),
    TROUGH(new LoggedTunableNumber("Wrist/Trough", 10)),
    LEVELONE(1.0),
    LEVELTWO(2.0),
    LEVELTHREE(3.0);

    private final DoubleSupplier wristSetpointSupplier;

    private double getRads() {
      return Units.degreesToRadians(wristSetpointSupplier.getAsDouble());
    }
  }
        
}
