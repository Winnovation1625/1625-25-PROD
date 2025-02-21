package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;

public class ClimberIOServo implements ClimberIO {

  private Servo servo;

  public ClimberIOServo() {
    servo = new Servo(0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.servoPositionRads = Units.degreesToRadians(servo.getAngle());
  }

  @Override
  public void setRelease(boolean releaseState) {
    if (releaseState) {
      servo.setAngle(Units.radiansToDegrees(hookReleaseHomeAngleRads));
    } else {
      servo.setAngle(Units.radiansToDegrees(hookReleaseHomeAngleRads));
    }
  }
}
