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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoController;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final Drive drive;
  private final PoseManager poseManager;

  private final AutoFactory autoFactory;
  private final AutoController autoController;
  private final AutoChooser autoChooser;

  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  public Autos(Drive drive, PoseManager poseManager) {
    this.drive = drive;
    this.poseManager = poseManager;

    autoController = new AutoController(drive);

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
