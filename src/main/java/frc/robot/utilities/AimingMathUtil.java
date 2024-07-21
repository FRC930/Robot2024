package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public abstract class AimingMathUtil {
    
    /*  SHOOTER DATA SHEET HERE:
    // These factors control how severely note trajectory is accounted for.
    // SEE TUNING INSTRUCTIONS: https://docs.google.com/document/d/1Bgt9aPj9Tyn7bf3AqmL5ytYnkF5hi1Utf2yvQWLiGkc/edit?usp=sharing
    // OPTIONS FOR TUNING: 
    // 1. You can add more data points for the regression. Make sure to update the link though! The link to the calculator will change after saving it.
    // 2. Use piecewise overrides or modifiers for areas that don't work
    */
    private static final double TRAJ_OFFSET_LINEAR_FACTOR = -0.0265424;

    private static final double TRAJ_OFFSET_ZERO = 15.6514; //13.6514 Offset for old note (changed to new note offset)

    //Pivot
    public static final double DIST_FUDGE = 16.0;
    public static final double BLUE_DIST_FUDGE = 0.0;
    public static final double RED_DIST_FUDGE = 0.0;
    public static final double BLUE_ANGLE_FUDGE = 2.0;
    public static final double RED_ANGLE_FUDGE = 0.0;
    //End Pivot

    /**
     * Given a certain distance, returns a certain offset to the turret to adjust for the notes flight characteristics
     * If this is somehow reversed based on alliance, just invert what this method returns. I don't think it is, but maybe.
     * Takes inches.
     */
    public static double getTurretOffsetForDistance(double distance) {
        //Quick test for if this offsets in the right direction, since small offsets being in the wrong direction could go unnoticed until late.
        //return 25; 
        if(distance > 230.0){ // Blue feed shot
            if(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue){
                return 45.0;
            }
        }

        // This models the note's path as a parabolic trajectory, and gets the angle from the turret to a point on that trajectory. It then returns the negative of that angle.
        return (TRAJ_OFFSET_LINEAR_FACTOR * (distance - DIST_FUDGE - SmartDashboard.getNumber("offsets/distanceOffset", 0.0))) + TRAJ_OFFSET_ZERO;
    }
}