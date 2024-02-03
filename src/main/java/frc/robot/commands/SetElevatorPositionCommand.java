package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorPositionCommand extends Command {

    private ElevatorSubsystem m_ElevatorSubsystem;
    private double m_TargetPos;

    public SetElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, double targetPos) {
        m_ElevatorSubsystem = elevatorSubsystem;
        m_TargetPos = targetPos;
        addRequirements(m_ElevatorSubsystem);
    }
    
    @Override
    public void initialize() {
        m_ElevatorSubsystem.setTargetHeight(m_TargetPos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}