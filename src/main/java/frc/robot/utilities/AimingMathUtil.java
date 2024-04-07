package frc.robot.utilities;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.Odometry;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public abstract class AimingMathUtil {
    
    /*  SHOOTER DATA SHEET HERE:
    // These factors control how severely note trajectory curvature is accounted for.
    // Small values for this reduce how much we compensate for initial velocity.
    // You can find the regression for these values here: https://www.desmos.com/calculator/cv5ulx956y
    // OPTIONS FOR TUNING: 
    // 1. You can add more data points for the regression. Make sure to update the link though! The link to the calculator will change after saving it.
    // 2. Use piecewise overrides or modifiers for areas that don't work
    */
    private static final double TRAJ_OFFSET_LINEAR_FACTOR = -0.0265424;

    private static final double TRAJ_OFFSET_ZERO = 13.6514;

    /**
     * Given a certain distance, returns a certain offset to the turret to adjust for the notes flight characteristics
     * If this is somehow reversed based on alliance, just invert what this method returns. I don't think it is, but maybe.
     * Takes inches.
     */
    public static double getTurretOffsetForDistance(double distance) {
        //Quick test for if this offsets in the right direction, since small offsets being in the wrong direction could go unnoticed until late.
        //return 25;

        // This models the note's path as a parabolic trajectory, and gets the angle from the turret to a point on that trajectory. It then returns the negative of that angle.
        return (TRAJ_OFFSET_LINEAR_FACTOR * distance) + TRAJ_OFFSET_ZERO;
    }
}