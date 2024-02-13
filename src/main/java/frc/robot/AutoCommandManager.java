package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LimeLightIntakeCommand;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.utilities.GamePieceDetectionUtility;

public class AutoCommandManager {
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    public AutoCommandManager(SwerveDrivetrainSubsystem drivetrain) {
        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("SelectAuto", m_chooser);
    }
    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }
    public Command getAutoManagerSelected(){
        return m_chooser.getSelected();
    }
}
