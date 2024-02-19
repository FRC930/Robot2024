package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorPositionCommand extends Command {

    protected ElevatorSubsystem m_elevator;
    private double m_targetPos;

    public SetElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, double targetPos) {
        m_elevator = elevatorSubsystem;
        m_targetPos = targetPos;
        addRequirements(m_elevator);
    }
    
    @Override
    public void initialize() {
        m_elevator.setTarget(m_targetPos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}