package frc.robot.subsystem.climber;

import static frc.robot.subsystem.climber.ClimberConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import java.util.function.DoubleSupplier;

public class ClimberIOServo implements ClimberIO {

  private Servo servo;

  public ClimberIOServo() {
    servo = new Servo(9);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.servoPositionRads = Units.degreesToRadians(servo.getAngle());
  }

  @Override
  public void setRelease(DoubleSupplier angle) {

    servo.setAngle(angle.getAsDouble());
  }
}
