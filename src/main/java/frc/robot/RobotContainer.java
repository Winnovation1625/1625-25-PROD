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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.FieldConstants.ReefPosition;
import frc.robot.commands.AutoScoreCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToHumanStation;
import frc.robot.commands.DriveToReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.apriltagvision.AprilTagVision;
import frc.robot.subsystem.apriltagvision.AprilTagVisionConstants;
import frc.robot.subsystem.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystem.apriltagvision.AprilTagVisionIOPhoton;
import frc.robot.subsystem.apriltagvision.AprilTagVisionIOPhotonSim;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.ClimberIO;
import frc.robot.subsystem.climber.ClimberIOServo;
import frc.robot.subsystem.climber.ClimberIOSim;
import frc.robot.subsystem.drive.Drive;
import frc.robot.subsystem.drive.DriveConstants;
import frc.robot.subsystem.drive.GyroIO;
import frc.robot.subsystem.drive.GyroIOPigeon2;
import frc.robot.subsystem.drive.GyroIOSim;
import frc.robot.subsystem.drive.ModuleIO;
import frc.robot.subsystem.drive.ModuleIOTalonFX;
import frc.robot.subsystem.leds.Leds;
import frc.robot.subsystem.manipulator.Manipulator;
import frc.robot.subsystem.manipulator.Manipulator.GamepieceState;
import frc.robot.subsystem.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystem.manipulator.ManipulatorIO;
import frc.robot.subsystem.manipulator.ManipulatorIOKraken;
import frc.robot.subsystem.manipulator.ManipulatorIOSim;
import frc.robot.subsystem.manipulator.manipulatorSensors.ManipulatorSensorIO;
import frc.robot.subsystem.manipulator.manipulatorSensors.ManipulatorSensorIOSim;
import frc.robot.subsystem.manipulator.manipulatorSensors.ManipulatorSensorsIOCANrange;
import frc.robot.subsystem.superstructure.Superstructure;
import frc.robot.subsystem.superstructure.Superstructure.SuperstructureStates;
import frc.robot.subsystem.superstructure.arm.Arm;
import frc.robot.subsystem.superstructure.arm.Arm.ArmState;
import frc.robot.subsystem.superstructure.arm.ArmIO;
import frc.robot.subsystem.superstructure.arm.ArmIOKraken;
import frc.robot.subsystem.superstructure.arm.ArmIOSim;
import frc.robot.subsystem.superstructure.elevator.Elevator;
import frc.robot.subsystem.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystem.superstructure.elevator.ElevatorIO;
import frc.robot.subsystem.superstructure.elevator.ElevatorIOKraken;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AuxControllerUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Setter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
  private final Climber climber;
  private Leds leds = Leds.getInstance();

  private final LoggedTunableNumber endgameAlert1 = new LoggedTunableNumber("EndgameAlert2", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("Endgame Alert #2", 15.0);
  private final LoggedDashboardChooser<BooleanSupplier> pathOverride;
  @Setter BooleanSupplier pathingOverrideSupplier;

  @SuppressWarnings("unused")
  private final AprilTagVision aprilTagVision;

  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick auxController = new Joystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert auxDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);

  private final Supplier<Rotation2d> getHumanPlayerAngle;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    pathOverride = new LoggedDashboardChooser<>("Pathing Override");

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
        manipulator =
            new Manipulator(new ManipulatorIOKraken(), new ManipulatorSensorsIOCANrange());
        arm = new Arm(new ArmIOKraken(), manipulator::getGamepieceState);
        elevator = new Elevator(new ElevatorIOKraken());
        aprilTagVision =
            new AprilTagVision(
                drive::accept,
                drive::getPose,
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
        // new AprilTagVisionIOPhoton(
        //     AprilTagVisionConstants.CAMERA_CONFIGS.get(3),
        //     drive::getRotation,
        //     drive::getPose));
        climber = new Climber(new ClimberIOServo());
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
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        manipulator = new Manipulator(new ManipulatorIOSim(), new ManipulatorSensorIOSim());
        arm = new Arm(new ArmIOSim(), manipulator::getGamepieceState);
        elevator = new Elevator(new ElevatorIO() {});
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
        climber = new Climber(new ClimberIOSim());
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
        manipulator = new Manipulator(new ManipulatorIO() {}, new ManipulatorSensorIO() {});
        arm = new Arm(new ArmIO() {}, manipulator::getGamepieceState);
        elevator = new Elevator(new ElevatorIO() {});
        aprilTagVision =
            new AprilTagVision(
                drive::accept,
                drive::getPose,
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    // drive.setNearBargeSupplier(
    //     () ->
    //         AllianceFlipUtil.apply(drive.getPose()).getY()
    //                 - AllianceFlipUtil.apply(FieldConstants.Barge.middleCage).getY()
    //             < 0.4);

    superstructure = new Superstructure(arm, elevator, manipulator::hasAlgae);
    // Set up auto routines
    NamedCommands.registerCommand(
        "AutoScoreL1", AutoScoreCommands.autoScoreTrough(superstructure, manipulator));
    NamedCommands.registerCommand(
        "AutoScoreL2", AutoScoreCommands.autoScoreCoral(superstructure, manipulator, ReefLevel.L2));
    NamedCommands.registerCommand("DriveToReefFace4", new DriveToReef(drive, 4, true));
    NamedCommands.registerCommand("DriveToReefFace3", new DriveToReef(drive, 3, true));
    NamedCommands.registerCommand(
        "LeaveStartConfigAutoScoreL4",
        AutoScoreCommands.autoScoreCoral(superstructure, manipulator, ReefLevel.L4, true));
    NamedCommands.registerCommand(
        "LeaveStartConfigAutoScoreL2",
        AutoScoreCommands.autoScoreCoral(superstructure, manipulator, ReefLevel.L2, true));
    for (var position : FieldConstants.ReefPosition.values()) {
      DriveToReef reefCommand = new DriveToReef(drive, () -> position);
      NamedCommands.registerCommand(
          "DriveToBranch" + position.name(),
          reefCommand.until(
              () ->
                  reefCommand.isRunning()
                      && reefCommand.withinTolerance(
                          Units.inchesToMeters(2.5), new Rotation2d(Units.degreesToRadians(3.0)))));
    }

    NamedCommands.registerCommand(
        "DriveToLeftHumanPlayerStation",
        new DriveToHumanStation(drive, () -> FieldConstants.HumanStation.LEFT));
    NamedCommands.registerCommand(
        "DriveToRightHumanPlayerStation",
        new DriveToHumanStation(drive, () -> FieldConstants.HumanStation.RIGHT)
            .until(
                () ->
                    manipulator.getGamepieceState() == GamepieceState.CORAL_STAGING
                        || manipulator.getGamepieceState() == GamepieceState.CORAL_IN_MANIPULATOR));
    NamedCommands.registerCommand(
        "CoralIntake", AutoScoreCommands.coralIntake(superstructure, manipulator));
    NamedCommands.registerCommand(
        "AutoScoreL4", AutoScoreCommands.autoScoreCoral(superstructure, manipulator, ReefLevel.L4));
    // Set up SysId routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureSysId();

    // Configure the button bindings
    configureButtonBindings();

    getHumanPlayerAngle =
        () ->
            (drive.getPose().getY() > FieldConstants.fieldWidth / 2
                        && !AllianceFlipUtil.shouldFlip())
                    || (drive.getPose().getY() < FieldConstants.fieldWidth / 2
                        && AllianceFlipUtil.shouldFlip())
                ? AllianceFlipUtil.apply(new Rotation2d(Degrees.of(-54)))
                : AllianceFlipUtil.apply(new Rotation2d(Degrees.of(54)));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5)
                .beforeStarting(() -> leds.setEndGameWarning(true))
                .finallyDo(() -> leds.setEndGameWarning(false)));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9) // Rumble three times
                .beforeStarting(() -> leds.setEndGameWarning(true))
                .finallyDo(() -> leds.setEndGameWarning(false)));

    // new Trigger(() -> auxController.getRawButton(19)).onTrue(getAutonomousCommand());

    pathOverride.addDefaultOption("off", pathingOverrideSupplier = () -> false);
    pathOverride.addOption("on", pathingOverrideSupplier = () -> true);
  }

  private void configureSysId() {
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
  }

  private Supplier<ReefPosition> getCoralObjective =
      () ->
          AuxControllerUtil.getCoralObjective(
                  List.of(
                      auxController.getRawButton(2), // A
                      auxController.getRawButton(1), // B
                      auxController.getRawButton(12), // C
                      auxController.getRawButton(11), // D
                      auxController.getRawButton(10), // E
                      auxController.getRawButton(9), // F
                      auxController.getRawButton(8), // G
                      auxController.getRawButton(7), // H
                      auxController.getRawButton(6), // I
                      auxController.getRawButton(5), // J
                      auxController.getRawButton(4), // K
                      auxController.getRawButton(3) // L
                      ),
                  List.of(
                      auxController.getRawButton(19), // L1
                      auxController.getRawButton(20), // L2
                      auxController.getRawButton(21), // L3
                      auxController.getRawButton(22) // L4
                      ))
              .position();

  private Supplier<SuperstructureStates> getCoralLevel =
      () -> {
        if (auxController.getRawButton(19)) {
          return SuperstructureStates.TROUGH;
        } else if (auxController.getRawButton(20)) {
          return SuperstructureStates.CLVL2;
        } else if (auxController.getRawButton(21)) {
          return SuperstructureStates.CLVL3;
        } else if (auxController.getRawButton(22)) {
          return SuperstructureStates.CLVL4;
        } else {
          return SuperstructureStates.TROUGH;
        }
      };

  private Supplier<SuperstructureStates> getAlgaeScore =
      () ->
          auxController.getRawButton(23)
              ? SuperstructureStates.BARGE
              : SuperstructureStates.PROCESS;

  private Supplier<SuperstructureStates> getAlgaeReefLevel =
      () -> {
        if (auxController.getRawButton(17)
            || auxController.getRawButton(13)
            || auxController.getRawButton(15)) {
          return SuperstructureStates.ALVL3;
        } else {
          // if(auxController.getRawButton(12) || auxController.getRawButton(14) ||
          // auxController.getRawButton(16))
          return SuperstructureStates.ALVL2;
        }
      };

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

    // Lock to Human Player Station when Y button is held

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

    controller.back().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    // Algae and Coral Shared Commands
    // score command for coral and algae
    // controller
    //     .a()
    //     .and(() -> superstructure.getSuperstructureGoal() != SuperstructureStates.STOW)
    //     .whileTrue(
    //         Commands.either(
    //             manipulator.setManipulatorState(ManipulatorState.SHOOTING_ALGAE),
    //             manipulator.setManipulatorState(ManipulatorState.SHOOTING_CORAL),
    //             manipulator::hasAlgae));

    // cancel/stow command
    controller
        .b()
        .onTrue(
            superstructure
                .setSuperstructureCommand(() -> SuperstructureStates.STOW)
                .alongWith(manipulator.setManipulatorState(ManipulatorState.IDLE)));

    // Coral Controls
    // intake coral when no gamepiece in bot
    controller
        .rightTrigger()
        .and(() -> manipulator.getGamepieceState() == Manipulator.GamepieceState.NONE)
        .onTrue(
            superstructure
                .setSuperstructureCommand(() -> SuperstructureStates.CORAL_INTAKING)
                .alongWith(manipulator.setManipulatorState(ManipulatorState.INTAKING_CORAL))
                .andThen(
                    Commands.waitUntil(
                        () ->
                            superstructure.atSuperStructureGoal()
                                && (manipulator.getGamepieceState()
                                        == Manipulator.GamepieceState.CORAL_STAGING
                                    || manipulator.getGamepieceState()
                                        == Manipulator.GamepieceState.CORAL_IN_MANIPULATOR)))
                .andThen(
                    manipulator
                        .setManipulatorState(ManipulatorState.IDLE)
                        .alongWith(
                            superstructure.setSuperstructureCommand(
                                () -> SuperstructureStates.STOW)))
                .andThen(controllerRumbleCommand().withTimeout(Seconds.of(0.5))));

    // go to coral score position when having coral
    controller
        .rightTrigger()
        .and(
            () ->
                manipulator.getGamepieceState() == Manipulator.GamepieceState.CORAL_IN_MANIPULATOR)
        .onTrue(
            superstructure
                .setSuperstructureCommand(getCoralLevel)
                .andThen(
                    Commands.waitUntil(
                        () ->
                            superstructure.atSuperStructureGoal()
                                && manipulator.getGamepieceState()
                                    == Manipulator.GamepieceState.NONE))
                .andThen(superstructure.setSuperstructureCommand(() -> SuperstructureStates.STOW)));

    // auto path coral to reeef
    controller.leftBumper().whileTrue(new DriveToReef(drive, getCoralObjective));

    // controller.povDown()

    // Algae Controls
    // score algae in barge or processor depending on aux button state
    controller
        .leftTrigger()
        .and(() -> manipulator.getGamepieceState() == Manipulator.GamepieceState.ALGAE_IN_CLAW)
        .onTrue(
            superstructure
                .setSuperstructureCommand(getAlgaeScore)
                .andThen(
                    Commands.waitUntil(
                        () ->
                            superstructure.atSuperStructureGoal()
                                && manipulator.getGamepieceState()
                                    == Manipulator.GamepieceState.NONE))
                .andThen(superstructure.setSuperstructureCommand(() -> SuperstructureStates.STOW)));

    // ground pickup algae
    // controller
    //     .x()
    //     .and(() -> manipulator.getGamepieceState() == Manipulator.GamepieceState.NONE)
    //     .onTrue(
    //         superstructure
    //             .setSuperstructureCommand(() -> SuperstructureStates.ALGAE_INTAKING)
    //             .alongWith(manipulator.setManipulatorState(ManipulatorState.INTAKING_ALGAE))
    //             .andThen(
    //                 Commands.waitUntil(
    //                     () ->
    //                         manipulator.getGamepieceState()
    //                             == Manipulator.GamepieceState.ALGAE_IN_CLAW))
    //             .andThen(manipulator.setManipulatorState(ManipulatorState.IDLE))
    //             .andThen(Commands.waitUntil(() -> superstructure.atArmGoal()))
    //             .andThen(superstructure.setSuperstructureCommand(() ->
    // SuperstructureStates.STOW)));

    controller
        .x()
        .and(() -> manipulator.getGamepieceState() == GamepieceState.NONE)
        .onTrue(
            superstructure
                .setSuperstructureCommand(() -> SuperstructureStates.ALGAE_INTAKING)
                .alongWith(manipulator.setManipulatorState(ManipulatorState.INTAKING_ALGAE))
                .andThen(
                    Commands.waitUntil(
                        () -> manipulator.getGamepieceState() == GamepieceState.ALGAE_IN_CLAW))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(manipulator.setManipulatorState(ManipulatorState.IDLE))
                .andThen(superstructure.runArmToState(() -> ArmState.FLAT))
                .andThen(Commands.waitUntil(() -> superstructure.atArmGoal()))
                .andThen(superstructure.setSuperstructureCommand(() -> SuperstructureStates.STOW)));

    // controller.povDown().onTrue(superstructure.leaveStartConfigToGoal(SuperstructureStates.STOW));

    // When left trigger and no algae in bot, grab algae from reef height dependent on aux algae pos
    // button
    controller
        .leftTrigger()
        .and(() -> manipulator.getGamepieceState() == Manipulator.GamepieceState.NONE)
        .onTrue(
            superstructure
                .setSuperstructureCommand(getAlgaeReefLevel)
                .alongWith(manipulator.setManipulatorState(ManipulatorState.INTAKING_ALGAE))
                .andThen(
                    Commands.waitUntil(
                        () ->
                            superstructure.atSuperStructureGoal()
                                && manipulator.getGamepieceState()
                                    == Manipulator.GamepieceState.ALGAE_IN_CLAW))
                .andThen(
                    superstructure
                        .setSuperstructureCommand(() -> SuperstructureStates.STOW)
                        .alongWith(manipulator.setManipulatorState(ManipulatorState.IDLE))));
    // Algae and Coral Shared Commands
    // score command for coral and algae
    // with left side button
    controller
        .a()
        .and(
            () ->
                superstructure.getSuperstructureGoal() != SuperstructureStates.STOW
                    && superstructure.getSuperstructureGoal()
                        != SuperstructureStates.ALGAE_INTAKING)
        .whileTrue(
            Commands.either(
                manipulator.setManipulatorState(ManipulatorState.SHOOTING_ALGAE),
                manipulator.setManipulatorState(ManipulatorState.SHOOTING_CORAL),
                manipulator::hasAlgae));

    controller
        .a()
        .and(
            () ->
                superstructure.getSuperstructureGoal() != SuperstructureStates.ALGAE_INTAKING
                    || superstructure.getSuperstructureGoal()
                        != SuperstructureStates.CORAL_INTAKING)
        .onFalse(manipulator.setManipulatorState(ManipulatorState.IDLE));

    controller
        .y()
        .and(() -> superstructure.getSuperstructureGoal() != SuperstructureStates.STOW)
        .and(() -> manipulator.getGamepieceState() == GamepieceState.CORAL_IN_MANIPULATOR)
        .whileTrue(manipulator.setManipulatorState(ManipulatorState.RETURN_CORAL));

    controller
        .y()
        .and(() -> superstructure.getSuperstructureGoal() != SuperstructureStates.STOW)
        .and(() -> manipulator.getGamepieceState() == GamepieceState.ALGAE_IN_CLAW)
        .whileTrue(manipulator.setManipulatorState(ManipulatorState.RETURN_ALGAE));

    controller
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> getHumanPlayerAngle.get()));

    // controller
    //     .leftBumper()
    //     .and(() -> manipulator.getGamepieceState() == Manipulator.GamepieceState.NONE)
    //     .onTrue(
    //         superstructure
    //             .setSuperstructureCommand(getAlgaeReefLevel)
    //             .alongWith(manipulator.setManipulatorState(ManipulatorState.INTAKING_ALGAE))
    //             .andThen(
    //                 Commands.waitUntil(
    //                     () ->
    //                         superstructure.atSuperStructureGoal()
    //                             && manipulator.getGamepieceState()
    //                                     == Manipulator.GamepieceState.ALGAE_IN_CLAW))
    //                             .andThen(
    //                                 superstructure
    //                                     .setSuperstructureCommand(() ->
    // SuperstructureStates.STOW)
    //
    // .alongWith(manipulator.setManipulatorState(ManipulatorState.IDLE))));

    // Climb Commands
    controller
        .start()
        .and(() -> auxController.getRawButton(24))
        .onTrue(
            Commands.runOnce(() -> climber.setServoRelease(true))
                .andThen(Commands.waitSeconds(3))
                .andThen(Commands.runOnce(() -> climber.setServoRelease(false))));
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

  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(controller.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(controller.getHID().getPort()));
    auxDisconnected.set(!DriverStation.isJoystickConnected(auxController.getPort()));
  }

  public Pose2d getRobotPose() {
    return drive.getPose();
  }

  public void setElevatorEncoderPosition() {
    superstructure.setElevatorEncoderPosition(ElevatorConstants.ELEVATOR_OFFSET_ROTATIONS);
  }
}
