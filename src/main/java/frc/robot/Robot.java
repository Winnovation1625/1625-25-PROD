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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.util.CanivoreReader;
import frc.robot.util.Elastic;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.VirtualSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private static final double lowBatteryVoltage = 11.8;
  private static final double lowBatteryDisabledTime = 1.5;
  private static final double canivoreErrorTimeThreshold = 0.5;
  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private Field2d dashboardField = new Field2d();
  private double autoStart;
  private boolean autoMessagePrinted;
  private final CanivoreReader canivoreReader = new CanivoreReader("*");

  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);

  private final Alert canivoreErrorAlert =
      new Alert("CANivore errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    SmartDashboard.putData("Field", dashboardField);

    // Start AdvantageKit logger
    Logger.start();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));
    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }
    if (Constants.CURRENT_MODE == frc.robot.Constants.Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }
    RobotController.setBrownoutVoltage(6.0);
    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK());
    FollowPathCommand.warmupCommand().schedule();
    Elastic.selectTab("Auton");
    canInitialErrorTimer.start();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {

    VirtualSubsystem.periodicAll();

    // dashboardField.setRobotPose(RobotState.getInstance().getEstimatedPose());

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    robotContainer.checkControllers();
    dashboardField.setRobotPose(robotContainer.getRobotPose());

    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
      var canivoreStatus = canivoreReader.getStatus();
      if (canivoreStatus.isPresent()) {
        Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.get().Status.getName());
        Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.get().BusUtilization);
        Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.get().BusOffCount);
        Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.get().TxFullCount);
        Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.get().REC);
        Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.get().TEC);
        if (!canivoreStatus.get().Status.isOK()
            || canivoreStatus.get().TEC > 0
            || canivoreStatus.get().REC > 0) {
          canivoreErrorTimer.restart();
        }
      }
      canivoreErrorAlert.set(
          canivoreErrorTimer.hasElapsed(canivoreErrorTimeThreshold) && canInitialErrorTimer.hasElapsed(30));
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.resetSimulationField();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.setElevatorEncoderPosition();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    Elastic.selectTab("Teleop");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();
  }
}
