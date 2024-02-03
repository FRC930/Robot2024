package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SetPivotPositionCommand extends Command {

    private PivotSubsystem m_pivot;
    private double m_turretPos;

    public SetPivotPositionCommand(PivotSubsystem turretSubsystem, double turretPosition) {
        m_pivot = turretSubsystem;
        m_turretPos = turretPosition;
        addRequirements(turretSubsystem);
    }

    // @Override
    // public void initialize() {
    //     m_pivot.setPosition(m_turretPos);
    // }

    @Override
    public void initialize() {
        m_pivot.setPosition(SmartDashboard.getNumber("PivotSetPosition", 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_pivot.setPosition(0);
    }
        
    

}