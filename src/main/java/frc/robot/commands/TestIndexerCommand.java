package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class TestIndexerCommand extends Command {

    private IndexerSubsystem m_indexer;

    private double m_speed;

    public TestIndexerCommand(IndexerSubsystem indexer) {
        m_indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        m_speed = MathUtil.clamp(SmartDashboard.getNumber("IndexerSetSpeed", 0.0)/100, -1.0, 1.0); //Converts the inputted percentage

        m_indexer.setSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stop();
    }
}
