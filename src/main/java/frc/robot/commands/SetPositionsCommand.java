package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SetPositionsCommand extends Command {

    private double m_pivotPos;
    private PivotSubsystem m_pivot;
    private ElevatorSubsystem m_elevator;
    private double m_elevatorPos;
    private TurretSubsystem m_turret;
    private double m_turretPos;

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

    public SetPositionsCommand(TurretSubsystem turretSubsystem, double turretPosition) {
        m_turret = turretSubsystem;
        m_turretPos = turretPosition;
        addRequirements(turretSubsystem);
    }

    public SetPositionsCommand(TurretSubsystem turretSubsystem, double turretPosition, ElevatorSubsystem elevatorSubsystem, double elevatorPosition) {
        m_turret = turretSubsystem;
        m_turretPos = turretPosition;
        m_elevator = elevatorSubsystem;
        m_elevatorPos = elevatorPosition;
        addRequirements(elevatorSubsystem);
        addRequirements(turretSubsystem);
    }

    public SetPositionsCommand(TurretSubsystem turretSubsystem, double turretPosition, PivotSubsystem pivotSubsystem, double pivotPosition) {
        m_turret = turretSubsystem;
        m_turretPos = turretPosition;
        m_pivot = pivotSubsystem;
        m_pivotPos = pivotPosition;
        addRequirements(pivotSubsystem);
        addRequirements(turretSubsystem);
    }

    public SetPositionsCommand(PivotSubsystem pivotSubsystem, double pivotPosition, ElevatorSubsystem elevatorSubsystem, double elevatorPosition) {
        m_pivot = pivotSubsystem;
        m_pivotPos = pivotPosition;
        m_elevator = elevatorSubsystem;
        m_elevatorPos = elevatorPosition;
        addRequirements(elevatorSubsystem);
        addRequirements(pivotSubsystem);
    }

    public SetPositionsCommand(TurretSubsystem turretSubsystem, double turretPosition, PivotSubsystem pivotSubsystem, double pivotPosition, ElevatorSubsystem elevatorSubsystem, double elevatorPosition) {
        m_turret = turretSubsystem;
        m_turretPos = turretPosition;
        m_pivot = pivotSubsystem;
        m_pivotPos = pivotPosition;
        m_elevator = elevatorSubsystem;
        m_elevatorPos = elevatorPosition;
        addRequirements(elevatorSubsystem);
        addRequirements(pivotSubsystem);
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        if (m_pivot != null) {
            m_pivot.setPosition(m_pivotPos);
        }
        if (m_elevator != null) {
            m_elevator.setTargetHeight(m_elevatorPos);
        }
        if (m_turret != null) {
            m_turret.setPosition(m_turretPos);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}