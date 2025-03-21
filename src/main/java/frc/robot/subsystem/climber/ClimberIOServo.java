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
    inputs.positionRads = Units.degreesToRadians(climberReleaseServo.getAngle());
  }

  @Override
  public void setRelease(boolean releaseState) {
    if (releaseState) {
      climberReleaseServo.setAngle(Units.radiansToDegrees(hookReleaseAngleRads));
    } else {
      climberReleaseServo.setAngle(Units.radiansToDegrees(hookHomeAngleRads));
    }
  }
}
