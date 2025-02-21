package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DriverStation;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climber {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  @AutoLogOutput @Setter private boolean servoRelease = false;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    if (DriverStation.isDisabled()) {
      io.setRelease(false);
    }
    io.setRelease(servoRelease);
  }
}
