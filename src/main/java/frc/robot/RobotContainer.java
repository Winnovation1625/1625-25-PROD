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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.apriltagvision.AprilTagVision;
import frc.robot.subsystem.apriltagvision.AprilTagVisionConstants;
import frc.robot.subsystem.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystem.apriltagvision.AprilTagVisionIOPhotonSim;
import frc.robot.subsystem.drive.Drive;
import frc.robot.subsystem.drive.DriveConstants;
import frc.robot.subsystem.drive.GyroIO;
import frc.robot.subsystem.drive.GyroIOPigeon2;
import frc.robot.subsystem.drive.GyroIOSim;
import frc.robot.subsystem.drive.ModuleIO;
import frc.robot.subsystem.drive.ModuleIOTalonFXReal;
import frc.robot.subsystem.drive.ModuleIOTalonFXSim;
import frc.robot.subsystem.leds.Leds;
import frc.robot.subsystem.manipulator.Manipulator;
import frc.robot.subsystem.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystem.manipulator.ManipulatorIO;
import frc.robot.subsystem.manipulator.ManipulatorIOSim;
import frc.robot.subsystem.manipulator.manipulatorSensors.ManipulatorSensorIO;
import frc.robot.subsystem.manipulator.manipulatorSensors.ManipulatorSensorIOSim;
import frc.robot.subsystem.superstructure.Superstructure;
import frc.robot.subsystem.superstructure.Superstructure.SuperstructureStates;
import frc.robot.subsystem.superstructure.arm.Arm;
import frc.robot.subsystem.superstructure.arm.ArmIO;
import frc.robot.subsystem.superstructure.arm.ArmIOSim;
import frc.robot.subsystem.superstructure.elevator.Elevator;
import frc.robot.subsystem.superstructure.elevator.ElevatorIO;
import frc.robot.subsystem.superstructure.elevator.ElevatorIOSim;
import frc.robot.util.AllianceFlipUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Arm arm;
  private final Elevator elevator;
  private final Superstructure superstructure;
  private final Manipulator manipulator;
  private Leds leds = Leds.getInstance();

  @SuppressWarnings("unused")
  private final AprilTagVision aprilTagVision;

  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        aprilTagVision =
            new AprilTagVision(
                drive::accept,
                drive::getPose,
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {});
        manipulator = new Manipulator(new ManipulatorIO() {}, new ManipulatorSensorIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        arm = new Arm(new ArmIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        aprilTagVision =
            new AprilTagVision(
                drive::accept,
                drive::getPose,
                new AprilTagVisionIOPhotonSim(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(0),
                    drive::getRotation,
                    driveSimulation::getSimulatedDriveTrainPose),
                new AprilTagVisionIOPhotonSim(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(1),
                    drive::getRotation,
                    driveSimulation::getSimulatedDriveTrainPose),
                new AprilTagVisionIOPhotonSim(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(2),
                    drive::getRotation,
                    driveSimulation::getSimulatedDriveTrainPose),
                new AprilTagVisionIOPhotonSim(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(3),
                    drive::getRotation,
                    driveSimulation::getSimulatedDriveTrainPose));
        manipulator = new Manipulator(new ManipulatorIOSim(), new ManipulatorSensorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        aprilTagVision =
            new AprilTagVision(
                drive::accept,
                drive::getPose,
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {});
        manipulator = new Manipulator(new ManipulatorIO() {}, new ManipulatorSensorIO() {});
        break;
    }

    drive.setNearBargeSupplier(
        () ->
            AllianceFlipUtil.apply(drive.getPose()).getY()
                    - AllianceFlipUtil.apply(FieldConstants.Barge.middleCage).getY()
                < 0.4);

    superstructure = new Superstructure(arm, elevator, manipulator::hasAlgae);
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  Trigger readyToShoot =
        new Trigger(
            () -> drive.);

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.CURRENT_MODE == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // controller
    //     .x()
    //     .onTrue(superstructure.setSuperstructureCommand(() -> SuperstructureStates.INTAKING));
    // controller
    //     .leftTrigger()
    //     .onTrue(superstructure.setSuperstructureCommand(() -> SuperstructureStates.CLVL2));
    // controller
    //     .leftBumper()
    //     .onTrue(superstructure.setSuperstructureCommand(() -> SuperstructureStates.STOW));
    // controller
    //     .rightBumper()
    //     .onTrue(superstructure.setSuperstructureCommand(() -> SuperstructureStates.CLVL4));

    controller
        .rightTrigger()
        .and(() -> manipulator.getGamepieceState() == Manipulator.GamepieceState.ALGAE_IN_CLAW)
        .onTrue(
            superstructure
                .setSuperstructureCommand(() -> SuperstructureStates.BARGE)
                .andThen(
                    Commands.waitUntil(() -> superstructure.atSuperStructureGoal())
                        .andThen(manipulator.setManipulatorState(ManipulatorState.SHOOTING_ALGAE)))
                .andThen(
                    Commands.waitUntil(
                        () ->
                            manipulator.getGamepieceState()
                                != Manipulator.GamepieceState.ALGAE_IN_CLAW))
                .andThen(superstructure.setSuperstructureCommand(() -> SuperstructureStates.STOW)));

    controller
        .leftBumper()
        .onTrue(
            superstructure
                .setSuperstructureCommand(() -> SuperstructureStates.STOW)
                .alongWith(manipulator.setManipulatorState(ManipulatorState.IDLE)));

    controller
        .leftTrigger()
        .onTrue(
            superstructure
                .setSuperstructureCommand(() -> SuperstructureStates.ALGAE_INTAKING)
                .andThen(
                    Commands.waitUntil(() -> superstructure.atSuperStructureGoal())
                        .andThen(manipulator.setManipulatorState(ManipulatorState.INTAKING_ALGAE)))
                .andThen(
                    Commands.waitUntil(
                        () ->
                            manipulator.getGamepieceState()
                                == Manipulator.GamepieceState.ALGAE_IN_CLAW))
                .andThen(controllerRumbleCommand().withTimeout(0.5))
                .andThen(
                    superstructure.setSuperstructureCommand(
                        () -> SuperstructureStates.ALGAE_STOW)));

    // controller
    //     .rightBumper()
    //     .onTrue(
    //         superstructure
    //             .setSuperstructureCommand(() -> SuperstructureStates.CORAL_INTAKING)
    //             .andThen(
    //                 Commands.waitUntil(() -> superstructure.atSuperStructureGoal())
    //
    // .andThen(manipulator.setManipulatorState(ManipulatorState.INTAKING_CORAL)))
    //             .andThen(
    //                 Commands.waitUntil(
    //                     () ->
    //                         manipulator.getGamepieceState()
    //                             == Manipulator.GamepieceState.CORAL_IN_MANIPULATOR))
    //             .andThen(superstructure.setSuperstructureCommand(() ->
    // SuperstructureStates.STOW)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }
}
