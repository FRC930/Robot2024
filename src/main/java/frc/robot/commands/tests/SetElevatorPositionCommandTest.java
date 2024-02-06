package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * <h3>SetPivotPositionCommand</h3>
 * Sets the pivot's position
 */
@Deprecated
public class SetElevatorPositionCommandTest extends SetElevatorPositionCommand {


    /**
    * <h3>SetPivotPositionCommand</h3>
    * Constructs a command to set the pivot's position
    */
    public SetElevatorPositionCommandTest(ElevatorSubsystem elevatorSubsystem, double turretPosition) {
        super(elevatorSubsystem, turretPosition);
        SmartDashboard.putNumber("ElevatorSetPosition", 0.0);
    }

    @Override
    public void initialize() {
        m_elevator.setTargetHeight(SmartDashboard.getNumber("ElevatorSetPosition", 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setTargetHeight(0);
    }
        
    

}