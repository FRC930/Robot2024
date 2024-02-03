package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SetTurretPositionCommand extends Command {

    private TurretSubsystem m_turret;
    private double m_turretPos;

    public SetTurretPositionCommand(TurretSubsystem turretSubsystem, double turretPosition) {
        m_turret = turretSubsystem;
        m_turretPos = turretPosition;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        m_turret.setPosition(SmartDashboard.getNumber("TurretSetPosition", 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}