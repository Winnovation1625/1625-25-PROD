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
import frc.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulatorIOKraken;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulatorIOSim;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulatorSensorIOLaserCan;
import frc.robot.subsystems.algaemanipulator.AlgaeManipulatorSensorIOSim;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers.CoralManipulatorRollers;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers.CoralManipulatorRollers.RollersState;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers.CoralManipulatorRollersIO;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers.CoralManipulatorRollersIOKraken;
import frc.robot.subsystems.coralmanipulator.CoralManipulatorRollers.CoralManipulatorRollersIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
  private final AlgaeManipulator algaeManipulator;
  private final CoralManipulatorRollers coralManipulatorRollers;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<String> modeChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        algaeManipulator =
            new AlgaeManipulator(
                new AlgaeManipulatorIOKraken(), new AlgaeManipulatorSensorIOLaserCan());

        coralManipulatorRollers =
            new CoralManipulatorRollers(new CoralManipulatorRollersIOKraken());

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
        algaeManipulator =
            new AlgaeManipulator(new AlgaeManipulatorIOSim(), new AlgaeManipulatorSensorIOSim());

        coralManipulatorRollers = new CoralManipulatorRollers(new CoralManipulatorRollersIOSim());

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
        algaeManipulator =
            new AlgaeManipulator(new AlgaeManipulatorIOSim(), new AlgaeManipulatorSensorIOSim());

        coralManipulatorRollers = new CoralManipulatorRollers(new CoralManipulatorRollersIO() {});

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

    // controller
    //     .rightTrigger()
    //     .and(() -> algaeManipulator.getGamepieceState() == GamepieceState.NONE)
    //     .whileTrue(algaeManipulator.setDesiredStateCommand(ManipulatorState.INTAKING));

    // controller
    //     .rightBumper()
    //     .whileTrue(algaeManipulator.setDesiredStateCommand(ManipulatorState.SHOOTING));

    controller
        .rightBumper()
        .and(() -> coralManipulatorRollers.getRollersState() != RollersState.INTAKING)
        .whileTrue(coralManipulatorRollers.setDesiredStateCommand(RollersState.EJECTING));

    controller
        .rightTrigger()
        .and(() -> coralManipulatorRollers.getRollersState() != RollersState.EJECTING)
        .whileTrue(coralManipulatorRollers.setDesiredStateCommand(RollersState.INTAKING));
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
