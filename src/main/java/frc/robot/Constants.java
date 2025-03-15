// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import lombok.Builder;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final boolean TUNING_MODE = true;
  public static final Mode SIM_MODE = Mode.REAL;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;
  public static final String CANIVORE_NAME = "canivore";
  public static final DriveCANIds DRIVE_CAN_IDS =
      DriveCANIds.builder()
          .driveFrontLeft(0)
          .steerFrontLeft(1)
          .steerEncoderFrontLeft(2)
          .driveFrontRight(3)
          .steerFrontRight(4)
          .steerEncoderFrontRight(5)
          .driveBackLeft(6)
          .steerBackLeft(7)
          .steerEncoderBackLeft(8)
          .driveBackRight(9)
          .steerBackRight(10)
          .steerEncoderBackRight(11)
          .build();
  public static final SuperstructureCANIds SUPERSTRUCTURE_CAN_IDS =
      SuperstructureCANIds.builder()
          .elevatorLeft(7)
          .elevatorRight(6)
          .armMotor(14)
          .armEncoder(15)
          .build();
  public static final ManipulatorCANIds MANIPULATOR_CAN_IDS =
      ManipulatorCANIds.builder()
          .manipulatorMotor(15)
          .algaeCANRange(42)
          .coralBackCANRange(41)
          .coralFrontCANRange(43)
          .build();

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  @Builder
  public static final record DriveCANIds(
      Integer driveFrontLeft,
      Integer steerFrontLeft,
      Integer steerEncoderFrontLeft,
      Integer driveFrontRight,
      Integer steerFrontRight,
      Integer steerEncoderFrontRight,
      Integer driveBackLeft,
      Integer steerBackLeft,
      Integer steerEncoderBackLeft,
      Integer driveBackRight,
      Integer steerBackRight,
      Integer steerEncoderBackRight) {}

  @Builder
  public static final record SuperstructureCANIds(
      Integer elevatorLeft, Integer elevatorRight, Integer armMotor, Integer armEncoder) {}

  @Builder
  public static final record ManipulatorCANIds(
      Integer manipulatorMotor,
      Integer algaeCANRange,
      Integer coralBackCANRange,
      Integer coralFrontCANRange) {}
}
