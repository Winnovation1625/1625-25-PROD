package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystem.drive.Drive;
import frc.robot.subsystem.manipulator.Manipulator;
import frc.robot.subsystem.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystem.superstructure.Superstructure;
import frc.robot.subsystem.superstructure.Superstructure.SuperstructureStates;
import frc.robot.util.LoggedTunableNumber;
import java.util.Map;

public class AutoScoreCommands {
  private static final LoggedTunableNumber superStructureOffset =
      new LoggedTunableNumber("AutoScore/SuperstructureOffsetInches", 0.0);
  private static final LoggedTunableNumber rotationDelay =
      new LoggedTunableNumber("AutoScore/DriveRotationDelayMeters", 0.5);
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("AutoScore/DriveGoalToleranceInches", 4.0);
  private static final LoggedTunableNumber driveReefOffsetX =
      new LoggedTunableNumber("AutoScore/DriveReefOffsetXInches", -9.117165);
  private static final LoggedTunableNumber driveReefOffsetY =
      new LoggedTunableNumber("AutoScore/DriveReefOffsetYInches", 31.937500 / 2);
  private static final PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(
          MetersPerSecond.of(16.16),
          MetersPerSecondPerSecond.of(32),
          RadiansPerSecond.of(2 * Math.PI),
          RadiansPerSecondPerSecond.of(10),
          Volts.of(12),
          false);

  public static Command getAutoScoreCommand(Drive drive, CoralObjective coralObjective) {
    Map<ReefLevel, Pose2d> reefBranch =
        FieldConstants.Reef.branchPositions2d.get(
            coralObjective.position().getFieldConstantsIndex());
    Pose2d reefPose = reefBranch.get(coralObjective.reefLevel());
    Pose2d desiredRobotPose =
        reefPose.transformBy(
            new Transform2d(
                Units.inchesToMeters(driveReefOffsetX.get()),
                Units.inchesToMeters(driveReefOffsetY.get()),
                Rotation2d.kZero));
    Command driveCommand = AutoBuilder.pathfindToPose(desiredRobotPose, PATH_CONSTRAINTS);
    return null;
  }

  public static Command autoScoreL1(Superstructure superstructure, Manipulator manipulator) {
    return superstructure.setSuperstructureCommand(() -> SuperstructureStates.TROUGH)
    .andThen(Commands.waitUntil(superstructure::atSuperStructureGoal))
    .andThen(manipulator.setManipulatorState(ManipulatorState.SHOOTING_CORAL))
    .andThen(Commands.waitTime(Seconds.of(1.0)))
    .andThen(manipulator.setManipulatorState(ManipulatorState.IDLE))
    .alongWith(superstructure.setSuperstructureCommand(() -> SuperstructureStates.STOW));
  }
}
