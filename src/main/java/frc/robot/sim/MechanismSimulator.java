package frc.robot.sim;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.IndexedShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class MechanismSimulator {
    private Mechanism2d m_mainAssembly;
    //private Mechanism2d m_turretBaseAssembly;

    private MechanismLigament2d m_shootingElevatorControl;
    private MechanismLigament2d m_climbingElevatorControl;
    private MechanismLigament2d m_pivotControl;
    //private MechanismLigament2d m_turretcontrol;

    private MechanismRoot2d pivotRoot;
    private MechanismRoot2d climbingElevatorRoot;
    //private MechanismRoot2d turretRoot;
    
    private PivotSubsystem m_pivot;
    private ElevatorSubsystem m_shootingElevator;
    private ElevatorSubsystem m_climbingElevator;

    public MechanismSimulator(PivotSubsystem pivot,ElevatorSubsystem shootingElevator, ElevatorSubsystem climbingElevator) {
        m_pivot = pivot;
        m_shootingElevator = shootingElevator;
        m_climbingElevator = climbingElevator;

        m_mainAssembly = new Mechanism2d(Units.inchesToMeters(100), Units.inchesToMeters(100));
        pivotRoot = m_mainAssembly.getRoot("shooting_elevator", Units.inchesToMeters(7.35), Units.inchesToMeters(10));
        m_mainAssembly.getRoot("Robot", 0, Units.inchesToMeters(7)).append(
            new MechanismLigament2d("frame", Units.inchesToMeters(26+7.5), 0, 5, new Color8Bit(Color.kBlanchedAlmond))
        );
        climbingElevatorRoot = m_mainAssembly.getRoot("climbing_elevator",Units.inchesToMeters(26),Units.inchesToMeters(10));

        // Adds elevator to the robot in simulation
        m_shootingElevatorControl =
            pivotRoot.append(
                new MechanismLigament2d(
                    "shootingElevator",
                    Units.inchesToMeters(10),
                    90,
                    15,
                    new Color8Bit(Color.kGray)
                )
            );
        
        m_pivotControl =
            m_shootingElevatorControl.append(
                new MechanismLigament2d(
                    "PivotAndShooter",
                    Units.inchesToMeters(10),
                    180,
                    10,
                    new Color8Bit(Color.kAzure)
                )
            );
        
        m_climbingElevatorControl = 
            climbingElevatorRoot.append(
                new MechanismLigament2d(
                    "climbingElevator",
                    Units.inchesToMeters(10),
                    90,
                    10,
                    new Color8Bit(Color.kGreen)
                )
            );

    }   

    public void periodic() {
        m_climbingElevatorControl.setLength(m_climbingElevator.getHeight()+0.2);
        m_pivotControl.setAngle(-90+m_pivot.getPosition());
        m_shootingElevatorControl.setLength(m_shootingElevator.getHeight()+0.2);

        SmartDashboard.putData(this.getClass().getSimpleName()+"/mechanism2d",m_mainAssembly);
    }
}
