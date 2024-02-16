package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;

public final class CommandFactoryUtility {

    // {LeftShooterSpeed, RightShooterSpeed, IndexerSpeed, PivotPosition, ElevatorPosition}
    private final double[][] SHOOTING_CONSTANTS = 
    {
        {0.7, 0.7, 0.9, 40.0, 0.0}, // "close" 3' 7" from speaker bumper to front of frame
        {0.7, 0.7, 0.9, 33.0, 0.0}, // "medium" 7' 8" from speaker bumper to front of frame
        {0.85, 0.85, 0.9, 31.0, 0.0} // "far" 12' 7" from speaker bumper to front of frame (ONLY WORKS WITH UNTRIMMED NOTES)
    };

    

}
