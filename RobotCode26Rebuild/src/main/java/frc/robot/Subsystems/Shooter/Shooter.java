package frc.robot.Subsystems.Shooter;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverMotorControllerGroup;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;

public class Shooter extends SubsystemBase{

    private static Shooter m_instance = new Shooter();

    private double m_targetAngle, m_targetSpeed;

    private boolean m_isShooting = false;

    private EverMotorControllerGroup m_shootingMotors;

    public EverCANCoder m_angleAbsEncoder = ShooterConsts.ANGLE_CAN_CODER;

    public static final EverPIDController 
            m_anglePID = new EverTalonFXPIDController((EverTalonFX)ShooterConsts.ANGLE_MOTOR),
            m_shootingPID = new EverSparkMaxPIDController((EverSparkMax)ShooterConsts.LEFT_MOTOR);

    private EverEncoder m_encoder;

    private Shooter(){
        m_shootingMotors = new EverMotorControllerGroup(ShooterConsts.LEFT_MOTOR, ShooterConsts.RIGHT_MOTOR);
        m_encoder = new EverTalonFXInternalEncoder((EverTalonFX)ShooterConsts.ANGLE_MOTOR);
        

    }

    public static Shooter getInstance(){
        return m_instance;
    }

    public double getCurrentVel(){
        return m_targetSpeed;
    }

    public void setShootingAngle(double targetAngle){
        m_targetAngle = targetAngle;
    }

    public void setShootingSpeed(double targetVel){
        m_targetSpeed = targetVel;
    }

    private void setShootingAngle(){
        m_targetAngle = getMinShootingAngle() + ShooterConsts.ANGLE_ERROR_MARGIN;
        if(m_targetAngle > ShooterConsts.MAX_ANGLE){
            m_targetAngle = ShooterConsts.MAX_ANGLE;
        }
        else if(m_targetAngle < ShooterConsts.MIN_ANGLE){
            m_targetAngle = ShooterConsts.MIN_ANGLE;
        }
    }

    /*
     * calculates the target speed and angle needed to shoot the ball to the hub based on the current distance and hight of the shooter
     * the formula is based on the physics of projectile motion
    */
    private void setShootingSpeed(){
        setShootingAngle();
        double mone = 9.81 * Math.pow(shootingDistance(),2);
        double mechane =
        2 * Math.pow(Math.cos(m_targetAngle), 2) * (shootingDistance() * Math.tan(m_targetAngle) - ShooterConsts.SHOOTING_HIGHT); 
        m_targetSpeed = Math.sqrt(mone / mechane);
    }

    private double shootingDistance(){
        double locX = SwerveLocalizer.getInstance().getCurrentPoint().getX();
        double locY = SwerveLocalizer.getInstance().getCurrentPoint().getY();
        return Math.sqrt(Math.pow(ShooterConsts.HUB_POINT.getX() - locX, 2) + Math.pow(ShooterConsts.HUB_POINT.getY() - locY, 2));
    }

    /*
     * calculates the minimum angle needed to shoot the ball to the hub based on the current distance and hight of the shooter
     * the formula is based on the physics of projectile motion
     */
    private double getMinShootingAngle(){
        return Math.atan(shootingDistance() / Math.sqrt(Math.pow(shootingDistance(), 2) + Math.pow(ShooterConsts.SHOOTING_HIGHT, 2)));
    }

    public void setIsShooting(boolean isShooting){
        m_isShooting = isShooting;
    }

    public double getShootingSpeed(){
        return m_targetSpeed;
    }

    
    public boolean getIsShooting() {
        return m_isShooting;
    }  

    @Override
    public void periodic() {
        if((m_angleAbsEncoder.getAbsPos() > ShooterConsts.MAX_ANGLE && m_shootingMotors.get() > 0) || 
            (m_angleAbsEncoder.getAbsPos() < ShooterConsts.MIN_ANGLE && m_shootingMotors.get() < 0)){
            m_targetAngle = m_angleAbsEncoder.getAbsPos();
        }

        if(m_isShooting){
            setShootingSpeed();
        }
        else{
            m_targetSpeed = 0;
        }
        m_shootingPID.activate(m_targetSpeed, ControlType.kVel);
        m_anglePID.activate(m_targetAngle, ControlType.kPos);
    }


}
