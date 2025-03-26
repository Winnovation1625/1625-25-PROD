package frc.robot.subsystem.climber;

import static frc.robot.subsystem.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

@RequiredArgsConstructor
public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  @AutoLogOutput @Setter private boolean servoRelease = false;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (DriverStation.isDisabled()) {
      io.setRelease(false);
    }
    io.setRelease(servoRelease);
  }
}
