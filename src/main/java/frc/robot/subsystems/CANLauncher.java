// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxWrapper;
import frc.robot.utilities.ThriftyNova;

public class CANLauncher extends SubsystemBase {
  ThriftyNova m_launchWheel;
  ThriftyNova m_feedWheel;
 // sets the ID for the KLauncher, and feeder.
  private static final int kLauncherID = 7; // 6
  private static final int kFeederID = 6; // 7
  // sets the feeder, and Launcher current limits
  private static final int kLauncherSupplyCurrentLimit = 8;
  private static final int kFeederSupplyCurrentLimit = 8;
  private static final int kLauncherStatorCurrentLimit = 80;
  private static final int kFeederStatorCurrentLimit = 80;
  // Sets the speed in reverse for intake.
  private static final double kIntakeFeederSpeed = -0.3;
  private static final double kIntakeLauncherSpeed = -0.3;

  /** Creates a new Launcher. */
  public CANLauncher() {
    m_launchWheel = new ThriftyNova(kLauncherID);
    m_feedWheel = new ThriftyNova(kFeederID);

    m_launchWheel.setMaxCurrent(kLauncherSupplyCurrentLimit);
    m_feedWheel.setMaxCurrent(kFeederSupplyCurrentLimit);

    m_launchWheel.setInverted(true);
    m_feedWheel.setInverted(true);

    // m_launchWheel.setRampDown(0.5);
    // m_launchWheel.setRampUp(0.5);

    // m_feedWheel.setRampDown(1.0);
    // m_feedWheel.setRampUp(1.0);

    //Didn't appear to affect motors
    // m_launchWheel.setBrakeMode(true);
    // m_feedWheel.setBrakeMode(true);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(kIntakeFeederSpeed);
          setLaunchWheel(kIntakeLauncherSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  @Override
  public void periodic() {
    Logger.recordOutput(this.getClass().getSimpleName() + "/" + "feedVelocity:", m_feedWheel.getVelocity());
    Logger.recordOutput(this.getClass().getSimpleName() + "/" + "launcherVelocity:", m_launchWheel.getVelocity());
    Logger.recordOutput(this.getClass().getSimpleName() + "/" + "feedCurrentDraw/supply:", m_feedWheel.getSupplyCurrent());
    Logger.recordOutput(this.getClass().getSimpleName() + "/" + "launcherCurrentDraw/supply:", m_launchWheel.getSupplyCurrent());
    Logger.recordOutput(this.getClass().getSimpleName() + "/" + "feedCurrentDraw/stator:", m_feedWheel.getStatorCurrent());
    Logger.recordOutput(this.getClass().getSimpleName() + "/" + "launcherCurrentDraw/stator:", m_launchWheel.getStatorCurrent());
    Logger.recordOutput(this.getClass().getSimpleName() + "/" + "feedPosition:", m_feedWheel.getPosition());
    Logger.recordOutput(this.getClass().getSimpleName() + "/" + "launcherPosition:", m_launchWheel.getPosition());
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheel.setPercentOutput(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheel.setPercentOutput(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheel.setPercentOutput(0);
    m_feedWheel.setPercentOutput(0);
  }
}
