package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class SetPositionsCommand extends Command {
    
    private double m_pivotPos;
    private PivotSubsystem m_pivot;
    private ElevatorSubsystem m_elevator;
    private double m_elevatorPos;

    public SetPositionsCommand(PivotSubsystem pivotSubsystem, double pivotPosition) {
        m_pivot = pivotSubsystem;
        m_pivotPos = pivotPosition;
        addRequirements(pivotSubsystem);
    }

    public SetPositionsCommand(ElevatorSubsystem elevatorSubsystem, double elevatorPosition) {
        m_elevator = elevatorSubsystem;
        m_elevatorPos = elevatorPosition;
        addRequirements(elevatorSubsystem);
    }
    
    public SetPositionsCommand(
        PivotSubsystem pivotSubsystem, double pivotPosition, 
        ElevatorSubsystem elevatorSubsystem, double elevatorPosition) {

        m_pivot = pivotSubsystem;
        m_pivotPos = pivotPosition;

        m_elevator = elevatorSubsystem;
        m_elevatorPos = elevatorPosition;

        addRequirements(pivotSubsystem);
        addRequirements(elevatorSubsystem);
    }
    
    @Override
    public void initialize() {
        if (m_pivot != null) {
            m_pivot.setPosition(m_pivotPos);
        }
        if (m_elevator != null) {
            m_elevator.setTargetHeight(m_elevatorPos);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
