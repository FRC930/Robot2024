package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * <h3>IndexerCommand</h3>
 * Sets the speed to run the indexer motor
 */
public class IndexerCommand extends Command {

    protected IndexerSubsystem m_indexer;

    private double m_speed;

    /**
     * <img src="https://images.squarespace-cdn.com/content/v1/52cf38d4e4b00e94d88c24af/2dcbb481-f80f-4759-a7af-496125ac077e/WHITE_ROBOTICS_2000px+%281%29.png?format=40w" style = "display: block;margin-left: auto;margin-right: auto;width: 50%;">
     * <h3>IndexerCommand</h3>
     * Creates a new indexer command
     * @param indexer the indexer to use
     * @param speed the speed of the indexer in percent output [-1.0,1.0]
     */
    public IndexerCommand(IndexerSubsystem indexer, double speed) {
        m_indexer = indexer;
        m_speed = speed;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        double speed = MathUtil.clamp(m_speed, -1.0, 1.0); //Converts the inputted percentage

        m_indexer.setSpeed(speed);
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
