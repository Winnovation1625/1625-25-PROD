package frc.robot.subsystem.climber;

import static frc.robot.subsystem.climber.ClimberConstants.*;

public class ClimberIOSim implements ClimberIO {

  private double servoPosition = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionRads = servoPosition;
  }

  @Override
  public void setRelease(boolean releaseState) {
    if (releaseState) {
      servoPosition = hookReleaseAngleRads;
    } else {
      servoPosition = hookHomeAngleRads;
    }
  }
}
