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
  private final Drive drive;
  private final PoseManager poseManager;

  private final AutoFactory factory;
  private final AutoController controller;
  private final AutoChooser chooser;

  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  public Autos(Drive drive, PoseManager poseManager) {
    this.drive = drive;
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
    chooser.addAutoRoutine("betterCircle", this::betterCircle);
    chooser.addAutoRoutine("twoNoteFromSource", this::twoNoteFromSource);

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

  private AutoRoutine betterCircle(final AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("betterCircle");

    final AutoTrajectory trajectory = factory.trajectory("betterCircle", routine);

    // entry point for the auto
    routine
        .enabled()
        .onTrue(
            resetOdometry(trajectory, routine)
                .andThen(trajectory.cmd())
                .withName("betterCircleEntryPoint"));

    return routine;
  }

  private AutoRoutine twoNoteFromSource(final AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("twoNoteFromSource");

    final AutoTrajectory SRCtoM5 = factory.trajectory("SRCtoM5", routine);
    final AutoTrajectory M5toS1 = factory.trajectory("M5toS1", routine);
    final AutoTrajectory S1toM3 = factory.trajectory("S1toM3", routine);
    final AutoTrajectory M3toS2 = factory.trajectory("M3toS2", routine);

    // entry point for the auto
    routine
        .enabled()
        .onTrue(
            resetOdometry(SRCtoM5, routine)
                .andThen(Commands.waitSeconds(0.3), SRCtoM5.cmd())
                .withName("twoNoteFromSourceEntryPoint"));

    SRCtoM5.done().onTrue(M5toS1.cmd());
    M5toS1.done().onTrue(drive.run(drive::stop).withTimeout(.3).andThen(S1toM3.cmd()));
    S1toM3.done().onTrue(M3toS2.cmd());

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
