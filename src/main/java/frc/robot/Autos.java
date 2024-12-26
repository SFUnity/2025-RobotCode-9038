package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.AllianceFlipUtil;
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
//   private final Drive drive;
  private final PoseManager poseManager;

  private final AutoFactory factory;
  private final AutoController controller;
  private final AutoChooser chooser;

  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  public Autos(Drive drive, PoseManager poseManager) {
    // this.drive = drive;
    this.poseManager = poseManager;

    controller = new AutoController(drive);

    factory =
        Choreo.createAutoFactory(
            drive,
            poseManager::getPose,
            controller,
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

    chooser = new AutoChooser(factory, "Auto Chooser Chor");

    // Add choreo auto options
    chooser.addAutoRoutine("circle", this::circle);

    chooser.addAutoRoutine("better circle", this::betterCircle);

    chooser.addAutoRoutine("get a note and shoot it", this::getNoteAndShoot);

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

  private Command resetOdometry(AutoTrajectory traj, AutoRoutine routine) {
    var optPose = traj.getInitialPose();
    if (optPose.isEmpty()) {
      routine.kill();
      return Commands.print("Killed routine due to lack of starting pose");
    }
    return Commands.runOnce(() -> poseManager.setPose(optPose.get())).withName("ResetOdometry");
  }

  private AutoRoutine getNoteAndShoot(final AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("get a note and shoot it");

    final AutoTrajectory trajectory = factory.trajectory("get a note and shoot it", routine);

    // entry point for the auto
    routine
        .enabled()
        .onTrue(
            resetOdometry(trajectory, routine)
                .andThen(trajectory.cmd())
                .withName("get a note and shoot it entry point"));

    return routine;
  }

  private AutoRoutine betterCircle(final AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("better circle");

    final AutoTrajectory trajectory = factory.trajectory("better circle", routine);

    // entry point for the auto
    routine
        .enabled()
        .onTrue(
            resetOdometry(trajectory, routine)
                .andThen(trajectory.cmd())
                .withName("better circle entry point"));

    return routine;
  }

  private AutoRoutine circle(final AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("circle");

    final AutoTrajectory trajectory = factory.trajectory("circle", routine);

    // entry point for the auto
    routine
        .enabled()
        .onTrue(
            resetOdometry(trajectory, routine)
                .andThen(trajectory.cmd())
                .withName("circle entry point"));

    return routine;
  }

  public void updateAutoChooser() {
    chooser.update();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return isChoreoAuto ? chooser.getSelectedAutoRoutine().cmd() : nonChoreoChooser.get();
  }
}
