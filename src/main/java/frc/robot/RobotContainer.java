// Copyright 2021-2024 FRC 6328
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

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMixed;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.AutoController;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
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
  private final AprilTagVision aprilTagVision;

  // Non-subsystems
  private final PoseManager poseManager = new PoseManager();

  // Controllers + driving
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandPS5Controller operator =
      new CommandPS5Controller(1); // TODO will we still be using a PS5?
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);

  public boolean slowMode = false;
  private final LoggedTunableNumber slowDriveMultiplier =
      new LoggedTunableNumber("Slow Drive Multiplier", 0.6);
  private final LoggedTunableNumber slowTurnMultiplier =
      new LoggedTunableNumber("Slow Turn Multiplier", 0.5);

  private final DriveCommandsConfig driveCommandsConfig =
      new DriveCommandsConfig(driver, () -> slowMode, slowDriveMultiplier, slowTurnMultiplier);

  // Autos
  private final AutoFactory autoFactory;
  private final AutoController autoController;
  private final AutoChooser autoChooser;
  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOMixed(0),
                new ModuleIOMixed(1),
                new ModuleIOMixed(2),
                new ModuleIOMixed(3),
                poseManager,
                driveCommandsConfig);
        aprilTagVision =
            new AprilTagVision(new AprilTagVisionIOLimelight("limelight"), poseManager);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                poseManager,
                driveCommandsConfig);
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {}, poseManager);
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
                poseManager,
                driveCommandsConfig);
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {}, poseManager);
        break;
    }

    // Set up auto controller
    autoController = new AutoController(drive);

    // Set up auto factory
    autoFactory =
        Choreo.createAutoFactory(
            drive,
            poseManager::getPose,
            autoController,
            AllianceFlipUtil::shouldFlip,
            new AutoBindings(),
            (Trajectory<SwerveSample> traj, Boolean bool) -> {
              Logger.recordOutput(
                  "Drive/Choreo/Active Traj",
                  (AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj).getPoses());
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj End Pose",
                  traj.getFinalPose(AllianceFlipUtil.shouldFlip()));
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj Start Pose",
                  traj.getInitialPose(AllianceFlipUtil.shouldFlip()));
            });

    // Set up auto chooser for choreo
    autoChooser = new AutoChooser(autoFactory, "Auto Chooser Chor");

    // Add choreo auto options
    autoChooser.addAutoRoutine(
        "circle",
        (AutoFactory factory) -> {
          final AutoLoop loop = factory.newLoop("circle");
          final AutoTrajectory trajectory = factory.trajectory("circle", loop);

          loop.enabled()
              .onTrue(
                  Commands.runOnce(
                          () ->
                              poseManager.setPose(
                                  trajectory
                                      .getInitialPose()
                                      .orElseGet(
                                          () -> {
                                            loop.kill();
                                            return new Pose2d();
                                          })))
                      .andThen(trajectory.cmd())
                      .withName("circle entry point"));

          return loop.cmd();
        });

    autoChooser.addAutoRoutine(
        "better circle",
        (AutoFactory factory) -> {
          final AutoLoop loop = factory.newLoop("better circle");
          final AutoTrajectory trajectory = factory.trajectory("better circle", loop);

          loop.enabled()
              .onTrue(
                  Commands.runOnce(
                          () ->
                              poseManager.setPose(
                                  trajectory
                                      .getInitialPose()
                                      .orElseGet(
                                          () -> {
                                            loop.kill();
                                            return new Pose2d();
                                          })))
                      .andThen(trajectory.cmd())
                      .withName("better circle entry point"));

          return loop.cmd();
        });

    autoChooser.addAutoRoutine(
        "get a note and shoot it",
        (AutoFactory factory) -> {
          final AutoLoop loop = factory.newLoop("get a note and shoot it");
          final AutoTrajectory trajectory = factory.trajectory("get a note and shoot it", loop);

          loop.enabled()
              .onTrue(
                  Commands.runOnce(
                          () ->
                              poseManager.setPose(
                                  trajectory
                                      .getInitialPose()
                                      .orElseGet(
                                          () -> {
                                            loop.kill();
                                            return new Pose2d();
                                          })))
                      .andThen(trajectory.cmd())
                      .withName("get a note and shoot it entry point"));

          return loop.cmd();
        });

    if (!DriverStation.isFMSAttached()) {
      // Set up test choreo routines

      // SysID & non-choreo routines
      if (!isChoreoAuto) {
        nonChoreoChooser.addOption("Module Drive Tuning", drive.tuneModuleDrive());
        nonChoreoChooser.addOption("Module Turn Tuning", drive.tuneModuleTurn());

        // Set up SysId routines
        nonChoreoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        nonChoreoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        nonChoreoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        nonChoreoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      }
    }

    // Configure the button bindings
    configureButtonBindings();

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.INFO).set(true);
    }
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // Default cmds
    drive.setDefaultCommand(drive.joystickDrive());

    // Driver controls
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driver
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        poseManager.setPose(
                            new Pose2d(poseManager.getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driver.leftBumper().onTrue(Commands.runOnce(() -> slowMode = !slowMode, drive));

    // Operator controls for intake
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(
                driver.getHID().getPort())); // Should be an XBox controller
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || DriverStation.getJoystickIsXbox(
                operator.getHID().getPort())); // Should not be an XBox controller
  }

  public void updateAutoChooser() {
    autoChooser.update();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return isChoreoAuto ? autoChooser.getSelectedAutoRoutine() : nonChoreoChooser.get();
  }
}
