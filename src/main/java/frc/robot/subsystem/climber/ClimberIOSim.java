package frc.robot.subsystem.climber;

import java.util.function.DoubleSupplier;

public class ClimberIOSim implements ClimberIO {

  private double servoPosition = 0.0;
  ;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.servoPositionRads = servoPosition;
  }

  @Override
  public void setRelease(DoubleSupplier angle) {
    servoPosition = angle.getAsDouble();
  }
}
