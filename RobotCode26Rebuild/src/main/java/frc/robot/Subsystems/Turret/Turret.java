package frc.robot.Subsystems.Turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.Math.Funcs;

public class Turret extends SubsystemBase{
    private EverSparkMax m_turret;
    private EverPIDController m_turretPid;
    private EverEncoder m_turretEncoder;
    public Turret(){
        m_turret = new EverSparkMax(9);
        m_turret.setPosConversionFactor(1.0 / 15.0 * 360);
        m_turretPid = new EverSparkMaxPIDController(m_turret);
        m_turretPid.setPID(0.05, 0, 0);
        m_turretEncoder = new EverSparkInternalEncoder(m_turret);
    }

    private double calcWantedTurretAngle(Pose2d targetPos){
        Pose2d pos = SwerveLocalizer.getInstance().getCurrentPoint();
        double x = targetPos.getX() - pos.getX();
        double y = targetPos.getY() - pos.getY();

        double theta = Math.atan2(y, x);

    //     double currentAngle = m_turretEncoder.getPos();
    //     double targetAngle = -Swerve.getInstance().getGyroOrientedAngle();
        
    //     double optimizedFlippedDeltaTargetAngle = Funcs.getShortestAnglePath(currentAngle, targetAngle - 180);
    //     double optimizedNormalDeltaTargetAngle = Funcs.getShortestAnglePath(currentAngle, targetAngle);

    //     double optimizedDeltaTargetAngle = 0;
    //     if (Math.abs(optimizedNormalDeltaTargetAngle) > Math.abs(optimizedFlippedDeltaTargetAngle)) {
    //         optimizedDeltaTargetAngle = optimizedFlippedDeltaTargetAngle;
    //     } else {
    //         optimizedDeltaTargetAngle = optimizedNormalDeltaTargetAngle;
    //     }

    //     // turn module to target angle
    //     return (currentAngle + optimizedDeltaTargetAngle);
        return theta  ;
    }

    @Override
    public void periodic(){
        m_turretPid.activate(calcWantedTurretAngle(new Pose2d(1,1, new Rotation2d())), ControlType.kPos);
        SmartDashboard.putNumber("current angle", m_turretEncoder.getPos());
        SmartDashboard.putNumber("goal", calcWantedTurretAngle(new Pose2d(1,1, new Rotation2d())));
        SmartDashboard.putNumber("angle", Swerve.getInstance().getGyroOrientedAngle());
    }
}
