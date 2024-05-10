// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxWrapper;

public class CANLauncher extends SubsystemBase {
  CANSparkMax m_launchWheel;
  CANSparkMax m_feedWheel;
 // sets the ID for the KLauncher, and feeder.
  private static final int kLauncherID = 6;
  private static final int kFeederID = 7;
  // sets the K Feed, and Launcher limits' to 80
  private static final int kLauncherCurrentLimit = 80;
  private static final int kFeedCurrentLimit = 80;
  // Sets the speed in reverse for intake.
  private static final double kIntakeFeederSpeed = -0.3;
  private static final double kIntakeLauncherSpeed = -0.3;

  /** Creates a new Launcher. */
  public CANLauncher() {
    m_launchWheel = new SparkMaxWrapper(kLauncherID, MotorType.kBrushless);
    m_feedWheel = new SparkMaxWrapper(kFeederID, MotorType.kBrushless);

    m_launchWheel.setSmartCurrentLimit(kLauncherCurrentLimit);
    m_feedWheel.setSmartCurrentLimit(kFeedCurrentLimit);
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

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheel.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheel.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheel.set(0);
    m_feedWheel.set(0);
  }
}
