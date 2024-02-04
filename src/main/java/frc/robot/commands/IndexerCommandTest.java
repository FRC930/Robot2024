package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * <h3>IndexerCommand</h3>
 * Sets the speed to run the indexer motor
 */
@Deprecated
public class IndexerCommandTest extends IndexerCommand {
    

    public IndexerCommandTest(IndexerSubsystem indexer, double speed) {
        super(indexer, speed);
        SmartDashboard.putNumber("IndexerMotor", 0.0);
    }

    @Override
    public void initialize() {
        m_indexer.setSpeed(MathUtil.clamp(SmartDashboard.getNumber("IndexerMotor", 0.0)/100, -1.0, 1.0));
    }
}
