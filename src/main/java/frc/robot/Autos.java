package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil6328;
import frc.robot.util.AutoController;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;

  private final PoseManager poseManager;

  private final AutoFactory factory;
  private final AutoController controller;
  private final AutoChooser chooser;

  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  public Autos(Drive drive, Intake intake, Shooter shooter, PoseManager poseManager) {
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;

    this.poseManager = poseManager;

    controller = new AutoController(drive);

    factory =
        Choreo.createAutoFactory(
            drive,
            poseManager::getPose,
            controller,
            AllianceFlipUtil6328::shouldFlip,
            new AutoBindings(),
            (Trajectory<SwerveSample> traj, Boolean bool) -> {
              Logger.recordOutput(
                  "Drive/Choreo/Active Traj",
                  (AllianceFlipUtil6328.shouldFlip() ? traj.flipped() : traj).getPoses());
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj End Pose",
                  traj.getFinalPose(AllianceFlipUtil6328.shouldFlip()));
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj Start Pose",
                  traj.getInitialPose(AllianceFlipUtil6328.shouldFlip()));
            });

    chooser = new AutoChooser(factory, "Auto Chooser Chor");

    // Add choreo auto options
    chooser.addAutoRoutine("betterCircle", this::betterCircle);
    chooser.addAutoRoutine("threeNoteFromSource", this::threeNoteFromSource);
    chooser.addAutoRoutine("threeNoteFromSourceDriveOnly", this::threeNoteFromSourceDriveOnly);

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

  // Routines

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

  private AutoRoutine threeNoteFromSourceDriveOnly(final AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("threeNoteFromSourceDriveOnly");

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
                .withName("threeNoteFromSourceDriveOnlyEntryPoint"));

    SRCtoM5.done().onTrue(M5toS1.cmd());
    M5toS1.done().onTrue(drive.run(drive::stop).withTimeout(.3).andThen(S1toM3.cmd()));
    S1toM3.done().onTrue(M3toS2.cmd());

    return routine;
  }

  private AutoRoutine threeNoteFromSource(final AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("threeNoteFromSource");

    final AutoTrajectory SRCtoM5 = factory.trajectory("SRCtoM5", routine);
    final AutoTrajectory M5toS1 = factory.trajectory("M5toS1", routine);
    final AutoTrajectory S1toM3 = factory.trajectory("S1toM3", routine);
    final AutoTrajectory M3toS2 = factory.trajectory("M3toS2", routine);

    // entry point for the auto
    routine
        .enabled()
        .onTrue(
            resetOdometry(SRCtoM5, routine)
                .andThen(shootAtStart(), Commands.parallel(intakeGP(), SRCtoM5.cmd()))
                .withName("threeNoteFromSourceEntryPoint"));

    // Pick up first note then shoot it
    SRCtoM5.done().onTrue(M5toS1.cmd());
    M5toS1.done().onTrue(autoShoot().andThen(new ScheduleCommand(S1toM3.cmd())));

    // Pick up second note then shoot it
    S1toM3.active().onTrue(intakeGP());
    S1toM3.done().onTrue(M3toS2.cmd());
    M3toS2.done().onTrue(autoShoot());

    return routine;
  }

  // Commands

  private Command resetOdometry(AutoTrajectory traj, AutoRoutine routine) {
    return Commands.runOnce(
            () -> {
              var optPose = traj.getInitialPose();
              if (optPose.isEmpty()) {
                routine.kill();
                System.out.println("Killed routine due to lack of starting pose");
              } else {
                poseManager.setPose(optPose.get());
              }
            })
        .withName("ResetOdometry");
  }

  private Command autoShoot() {
    return drive
        .headingDrive(
            () -> poseManager.getHorizontalAngleTo(FieldConstants.Speaker.centerSpeakerOpening))
        .until(() -> drive.thetaAtGoal())
        .alongWith(shooter.setAutoAimShot().until(shooter::atDesiredAngle))
        .andThen(shooter.feedNoteToFlywheels().onlyWhile(shooter::noteInShooter));
  }

  private Command shootAtStart() {
    return shooter
        .setManualSpeakerShot()
        .withTimeout(0.3)
        .andThen(shooter.feedNoteToFlywheels().onlyWhile(shooter::noteInShooter));
  }

  private Command intakeGP() {
    return shooter.setIntaking().deadlineWith(intake.intakeAndLowerCmd());
  }
}
