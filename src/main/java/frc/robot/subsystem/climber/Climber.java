package frc.robot.subsystem.climber;

import static frc.robot.subsystem.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climber {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  @AutoLogOutput @Setter private boolean servoRelease = false;
  ClimberState climberState = ClimberState.IDLE;

  @RequiredArgsConstructor
  public enum ClimberState {
    IDLE(new LoggedTunableNumber("Climb_Idle", ClimberConstants.hookReleaseHomeAngleRads)),
    RELEASE(new LoggedTunableNumber("Climb_Release", ClimberConstants.hookReleaseReleaseAngleRads));

    @Getter private final DoubleSupplier angularSupplier;
  }

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    if (DriverStation.isDisabled() || climberState == ClimberState.IDLE) {}

    if (climberState == ClimberState.RELEASE) {}
  }

  public Command runClimber(ClimberState state) {
    climberState = state;
    return Commands.runOnce(() -> io.setRelease(state.angularSupplier));
  }
}
