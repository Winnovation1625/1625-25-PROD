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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.apriltagvision.AprilTagVision;
import frc.robot.subsystem.apriltagvision.AprilTagVisionConstants;
import frc.robot.subsystem.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystem.apriltagvision.AprilTagVisionIOPhoton;
import frc.robot.subsystem.apriltagvision.AprilTagVisionIOPhotonSim;
import frc.robot.subsystem.coralmanipulator.*;
import frc.robot.subsystem.drive.Drive;
import frc.robot.subsystem.drive.GyroIO;
import frc.robot.subsystem.drive.GyroIOPigeon2;
import frc.robot.subsystem.drive.ModuleIO;
import frc.robot.subsystem.drive.ModuleIOSim;
import frc.robot.subsystem.drive.ModuleIOTalonFX;
import frc.robot.subsystem.superstructure.arm.Arm;
import frc.robot.subsystem.superstructure.arm.Arm.ArmState;
import frc.robot.subsystem.superstructure.arm.ArmIO;
import frc.robot.subsystem.superstructure.arm.ArmIOKrakenx60;
import frc.robot.subsystem.superstructure.arm.ArmIOSim;
import frc.robot.subsystem.superstructure.elevator.Elevator;
import frc.robot.subsystem.superstructure.elevator.ElevatorIO;
import frc.robot.subsystem.superstructure.elevator.ElevatorIOKraken;
import frc.robot.subsystem.superstructure.elevator.ElevatorIOSim;
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
  private final AprilTagVision aprilTagVision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<String> modeChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        arm = new Arm(new ArmIOKrakenx60());
        elevator = new Elevator(new ElevatorIOKraken());
        aprilTagVision =
            new AprilTagVision(
                drive::addVisionMeasurement,
                new AprilTagVisionIOPhoton(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(0),
                    drive::getRotation,
                    drive::getPose),
                new AprilTagVisionIOPhoton(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(1),
                    drive::getRotation,
                    drive::getPose),
                new AprilTagVisionIOPhoton(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(2),
                    drive::getRotation,
                    drive::getPose));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        arm = new Arm(new ArmIOSim());

        elevator = new Elevator(new ElevatorIOSim());
        aprilTagVision =
            new AprilTagVision(
                drive::addVisionMeasurement,
                new AprilTagVisionIOPhotonSim(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(0),
                    drive::getRotation,
                    drive::getPose),
                new AprilTagVisionIOPhotonSim(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(1),
                    drive::getRotation,
                    drive::getPose),
                new AprilTagVisionIOPhotonSim(
                    AprilTagVisionConstants.CAMERA_CONFIGS.get(2),
                    drive::getRotation,
                    drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        aprilTagVision =
            new AprilTagVision(
                drive::addVisionMeasurement,
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    modeChooser = new LoggedDashboardChooser<>("modeChooser");
    modeChooser.addOption("Show This", "Do This");
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
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    controller
        .leftTrigger()
        .and(() -> arm.getArmState() == ArmState.STOW)
        .onTrue(arm.setDesiredStateCommand(ArmState.CLVL1));
    controller
        .rightBumper()
        .and(() -> arm.getArmState() == ArmState.CLVL1)
        .onTrue(arm.setDesiredStateCommand(ArmState.CLVL2));
    controller
        .leftBumper()
        .and(() -> arm.getArmState() != ArmState.STOW)
        .onTrue(arm.setDesiredStateCommand(ArmState.STOW));

    controller.leftTrigger().whileTrue(elevator.tempCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
