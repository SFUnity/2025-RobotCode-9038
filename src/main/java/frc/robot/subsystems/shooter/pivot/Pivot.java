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

package frc.robot.subsystems.shooter.pivot;

import static frc.robot.subsystems.shooter.pivot.PivotConstants.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import frc.robot.util.Util;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Pivot/Gains/kP", gains.kP());

  private final PivotIO io;
  private final PoseManager poseManager;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final PivotVisualizer measuredVisualizer;
  private final PivotVisualizer setpointVisualizer;
  private ShuffleboardTab driversTab = Shuffleboard.getTab("Drivers");
  private final Timer atGoalTimer = new Timer();

  private GenericEntry feedingAngleEntry =
      driversTab
          .addPersistent("Feeding Angle", PivotConstants.kFeedingAngleRevRotations)
          .withPosition(8, 0)
          .withSize(1, 1)
          .getEntry();

  private GenericEntry angleOffset =
      driversTab
          .addPersistent("Angle Offset", PivotConstants.kSpeakerAngleOffsetRevRotations)
          .withPosition(8, 1)
          .withSize(1, 1)
          .getEntry();

  private double desiredAngle = 0;

  /** Creates a new Pivot. */
  public Pivot(PivotIO io, PoseManager poseManager) {
    this.io = io;
    this.poseManager = poseManager;

    io.setP(gains.kP());

    atGoalTimer.reset();
    atGoalTimer.start();

    measuredVisualizer = new PivotVisualizer("Measured", Color.kRed);
    setpointVisualizer = new PivotVisualizer("Setpoint", Color.kBlue);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Pivot", inputs);

    // Update controllers
    LoggedTunableNumber.ifChanged(hashCode(), () -> io.setP(kP.get()), kP);

    measuredVisualizer.update(inputs.positionRots);
    setpointVisualizer.update(desiredAngle);
    Util.logSubsystem(this, "Shooter/Pivot");
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Stops the pivot. */
  public void stop() {
    io.stop();
  }

  public void readyShootSpeakerManual() {
    desiredAngle = PivotConstants.kSpeakerManualAngleRevRotations;
  }

  // TODO needs testing and maybe some bs numbers. not sure
  public void readyShootSpeakerAutomatic() {
    Translation3d speakerOpening = FieldConstants.Speaker.centerSpeakerOpening;
    double heightOfTarget = speakerOpening.getY();
    double angleRad = Math.atan(heightOfTarget / poseManager.getDistanceTo(speakerOpening));
    double angleDeg = Math.toDegrees(angleRad);
    desiredAngle = angleDeg + angleOffset.getDouble(PivotConstants.kSpeakerAngleOffsetRevRotations);
  }

  public void readyShootAmp() {
    desiredAngle = PivotConstants.kDesiredAmpAngleRevRotations;
  }

  public void readyShootFeed() {
    desiredAngle = feedingAngleEntry.getDouble(PivotConstants.kFeedingAngleRevRotations);
  }

  public void readyShooterSourceIntake() {
    desiredAngle = PivotConstants.kDesiredSourceIntakeRevRotations;
  }

  public void readyShooterIntake() {
    desiredAngle = PivotConstants.kDesiredIntakeAngleRevRotations;
  }

  public void readyShooterEject() {
    desiredAngle = PivotConstants.kDesiredEjectAngleRevRotations;
  }

  @AutoLogOutput(key = "Shooter/Pivot/atDesiredAngle")
  public boolean atDesiredAngle() {
    if (!Util.equalsWithTolerance(inputs.positionRots, desiredAngle, 0.15)) {
      atGoalTimer.reset();
    }
    return atGoalTimer.hasElapsed(Constants.loopPeriodSecs);
  }

  // TODO any setter methods used in these commands should be made private
  public Command setManualSpeakerAngle() {
    return run(() -> {
          readyShootSpeakerManual();
          io.setAngleMotor(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotor(desiredAngle);
            })
        .withName("setManualShootAngle");
  }

  public Command setAutoSpeakerAngle() {
    return run(() -> {
          readyShootSpeakerAutomatic();
          io.setAngleMotor(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotor(desiredAngle);
            })
        .withName("setAutoShootAngle");
  }

  public Command setAmpAngle() {
    return run(() -> {
          readyShootAmp();
          io.setAngleMotor(desiredAngle);
        })
        .withName("setAmpAngle");
  }

  public Command setFeedAngle() {
    return run(() -> {
          readyShootFeed();
          io.setAngleMotor(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotor(desiredAngle);
            })
        .withName("gotta be a team player");
  }

  public Command setIntakeAngle() {
    return run(() -> {
          readyShooterIntake();
          io.setAngleMotor(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotor(desiredAngle);
            })
        .withName("setIntakeAngle");
  }

  public Command setSourceIntakeAngle() {
    return run(() -> {
          readyShooterSourceIntake();
          io.setAngleMotor(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotor(desiredAngle);
            })
        .withName("sourceIntake");
  }
}
