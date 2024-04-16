package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utilities.AimingMathUtil;
import frc.robot.utilities.RobotOdometryUtility;
import frc.robot.utilities.SpeakerScoreUtility;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

//Automatically aims the turret to one of the speakers based on the alliance.
public class TurretAimCommand extends Command{

    private static final double AIM_OFFSET = Units.inchesToMeters(23.0); // May be dynamic
    private static final double NON_AMP_AIM_OFFSET = Units.inchesToMeters(13.0); // May be dynamic

    //Has the shooter use custom offsets from SmartDashboard and logs extra info
    public static final boolean debugMode_TESTONLY = true;

    private TurretSubsystem m_TurretSubsystem;
    private Pose2d m_AmpSideBlueTargetPose;
    private Pose2d m_AmpSideRedTargetPose;
    private Pose2d m_NonAmpSideBlueTargetPose;
    private Pose2d m_NonAmpSideRedTargetPose;
    private Pose2d m_TargetPose;
    private Pose2d m_CurrentPose;
    private final boolean useProxyPose;
    private double m_CurrentRobotHeading;
    private double m_DesiredHeading;

    private final AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private double tx; //target x
    private double ty; //target y
    private double rx; //robot x
    private double ry; //robot y
    private boolean ampSide;
    private double idealHeading;
    private Pose2d m_ProxyPoseRed;
    private Pose2d m_ProxyPoseBlue;

    //We default to the new model when using odometry
    public TurretAimCommand(TurretSubsystem turretSubsystem) {
        this(turretSubsystem, null, null);
    }

    /**
     * <h3> TurretAimCommand </h3>
     * Aims the turret to the speaker apriltags based on the current alliance.
     * Constructor
     * @param turretSubsystem
     * @param proxyPoseRed
     * @param proxyPoseBlue
     * @param usesNewModel
     */
    public TurretAimCommand(TurretSubsystem turretSubsystem,Pose2d proxyPoseRed, Pose2d proxyPoseBlue) {
        if (debugMode_TESTONLY) {
            SmartDashboard.putNumber("PivotOffset", 0.0);
            SmartDashboard.putNumber("TurretOffset", 0.0);
        }
        m_AmpSideRedTargetPose = m_AprilTagFieldLayout.getTagPose(4).get().toPose2d();
        Logger.recordOutput("turretTargets/redIdealTarget", m_AmpSideRedTargetPose);
        m_AmpSideRedTargetPose = new Pose2d(
            m_AmpSideRedTargetPose.getX() + 0.5,
            m_AmpSideRedTargetPose.getY() - Units.inchesToMeters(23.0 + 12.0 - 6.0), 
            m_AmpSideRedTargetPose.getRotation());
        m_AmpSideBlueTargetPose = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
        Logger.recordOutput("turretTargets/blueIdealTarget", m_AmpSideBlueTargetPose);
        m_AmpSideBlueTargetPose = new Pose2d(
            m_AmpSideBlueTargetPose.getX() - 0.5, 
            m_AmpSideBlueTargetPose.getY() + Units.inchesToMeters(10.0), 
            m_AmpSideBlueTargetPose.getRotation());
        
        m_NonAmpSideRedTargetPose = new Pose2d(
            m_AmpSideRedTargetPose.getX() + 0.5, 
            m_AmpSideRedTargetPose.getY() + NON_AMP_AIM_OFFSET , 
            m_AmpSideRedTargetPose.getRotation());
        m_NonAmpSideBlueTargetPose = new Pose2d(
            m_AmpSideBlueTargetPose.getX() - 0.5, 
            m_AmpSideBlueTargetPose.getY() + Units.inchesToMeters(30 + 24), 
            m_AmpSideBlueTargetPose.getRotation());

        m_TargetPose = m_AmpSideBlueTargetPose;

        if(proxyPoseRed != null && proxyPoseBlue != null) {
            m_ProxyPoseRed = proxyPoseRed;
            m_ProxyPoseBlue = proxyPoseBlue;
            useProxyPose = true;
        } else {
            useProxyPose = false;
        }

        m_TurretSubsystem = turretSubsystem;
        addRequirements(m_TurretSubsystem);

        Logger.recordOutput("turretTargets/ampSideRedTarget", m_AmpSideRedTargetPose);
        Logger.recordOutput("turretTargets/nonAmpSideRedTarget", m_NonAmpSideRedTargetPose);
        Logger.recordOutput("turretTargets/ampSideBlueTarget", m_AmpSideBlueTargetPose);
        Logger.recordOutput("turretTargets/nonAmpSideBlueTarget", m_NonAmpSideBlueTargetPose);
        Logger.recordOutput("debug/AmpAndNonAmpThresholdLine", new Pose2d(0,4.5,new Rotation2d(0)));
    }
    
    @Override
    public void execute() {
        // If there is an alliance present it sets the target pose based on the alliance; otherwise defaults to blue.
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        Alliance alliance;
        if (optionalAlliance.isPresent()){
            alliance = optionalAlliance.get();
        } else {
            alliance = Alliance.Blue;
        }

        if(!useProxyPose) {
            // gets the robots position, and gets the robots heading.
            m_CurrentPose = RobotOdometryUtility.getInstance().getRobotOdometry();
        } else {
            m_CurrentPose = alliance == Alliance.Red ? m_ProxyPoseRed : m_ProxyPoseBlue;
            m_CurrentPose = new Pose2d(m_CurrentPose.getX(),m_CurrentPose.getY(), RobotOdometryUtility.getInstance().getRobotOdometry().getRotation());
        }

        m_CurrentRobotHeading = m_CurrentPose.getRotation().getDegrees();
        // logs the robot heding
        // SmartDashboard.putNumber("AutoAim/RobotHeading", m_CurrentRobotHeading);
        
        // sets the target x/y, and sets the robots x/y
        rx = m_CurrentPose.getX();
        ry = m_CurrentPose.getY();

        m_DesiredHeading = calcTurretAngleExpo(alliance);
        
        if(debugMode_TESTONLY) {
            m_DesiredHeading += SmartDashboard.getNumber("TurretOffset", 0.0);
            Logger.recordOutput("AutoAim/display/targetHeadingDisplay", new Pose2d(rx,ry,new Rotation2d(Units.degreesToRadians( m_CurrentRobotHeading - m_DesiredHeading))).transformBy(new Transform2d(new Translation2d(10,0),new Rotation2d())));
        }

        Logger.recordOutput("AutoAim/tx", tx);
        Logger.recordOutput("AutoAim/ty", ty);
        Logger.recordOutput("AutoAim/rx", rx);
        Logger.recordOutput("AutoAim/ry", ry);

        //Logs the desired heading
        // SmartDashboard.putNumber("AutoAim/Math", Math.toDegrees(Math.atan2(ty - ry, tx - rx)));
        Logger.recordOutput("AutoAim/DesiredHeading", m_DesiredHeading);
        Logger.recordOutput("AutoAim/HeadingOffset", m_DesiredHeading - idealHeading);
        // actually moves the robots turret to the desired position
        // TODO sussex back in
        m_TurretSubsystem.setPosition(m_DesiredHeading);
    }

    private double calcTurretAngleExpo(Alliance alliance) {
        double txi = alliance == Alliance.Red ? m_AprilTagFieldLayout.getTagPose(4).get().toPose2d().getX() : m_AprilTagFieldLayout.getTagPose(7).get().toPose2d().getX();
        double tyi = alliance == Alliance.Red ? m_AprilTagFieldLayout.getTagPose(4).get().toPose2d().getY() : m_AprilTagFieldLayout.getTagPose(7).get().toPose2d().getY();
        double firstPart = -Math.IEEEremainder(Math.toDegrees(Math.atan2(tyi - ry, txi - rx)) - m_CurrentRobotHeading, 360);
        idealHeading = firstPart;
        Logger.recordOutput("AutoAim/IdealHeading", firstPart);
        if(debugMode_TESTONLY) {
            Logger.recordOutput("AutoAim/display/targetDisplay", alliance == Alliance.Red ? m_AprilTagFieldLayout.getTagPose(4).get().toPose2d() : m_AprilTagFieldLayout.getTagPose(7).get().toPose2d());
            Logger.recordOutput("AutoAim/display/idealHeadingDisplay", new Pose2d(rx,ry,new Rotation2d(Units.degreesToRadians( m_CurrentRobotHeading - firstPart))).transformBy(new Transform2d(new Translation2d(10,0),new Rotation2d())));
        }
        return firstPart + AimingMathUtil.getTurretOffsetForDistance(SpeakerScoreUtility.inchesToSpeaker());
    }
    
    // Makes it so that this command never ends.
    @Override
    public boolean isFinished() {
        return false;
    }
}
