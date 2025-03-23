package frc.robot.subsystem.climber;

import static frc.robot.subsystem.climber.ClimberConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;

public class ClimberIOServo implements ClimberIO {

  private final Servo climberReleaseServo;

  public ClimberIOServo() {
    climberReleaseServo = new Servo(9);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionRads = climberReleaseServo.getAngle();
  }

  @Override
  public void setRelease(boolean releaseState) {
    if (releaseState) {
      climberReleaseServo.setAngle(hookReleaseAngle);
    } else {
      climberReleaseServo.setAngle(hookHomeAngle);
    }
  }

  @Override
  public void goToHome(boolean goToHome) {
    if (goToHome) {
      climberReleaseServo.setAngle(hookHomeAngle);
    }
  }
}
