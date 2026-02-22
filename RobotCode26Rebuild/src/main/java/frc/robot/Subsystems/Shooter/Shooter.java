package frc.robot.Subsystems.Shooter;

import javax.xml.crypto.KeySelector.Purpose;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
import frc.robot.Utils.Math.Vector2d;

public class Shooter extends SubsystemBase{

    private static Shooter m_instance = new Shooter();

    private double m_targetAngle, m_targetSpeed,
            m_deltaSpeed, m_currentSpeed;

    private boolean m_isShooting = false;

    private EverMotorControllerGroup m_shootingMotors;

    private EverCANCoder m_angleAbsEncoder = ShooterConsts.ANGLE_CAN_CODER;

    private EverPIDController 
            m_anglePID = new EverTalonFXPIDController((EverTalonFX)ShooterConsts.ANGLE_MOTOR),
            m_shootingPID = new EverSparkMaxPIDController((EverSparkMax)ShooterConsts.LEFT_MOTOR);

    private EverEncoder m_encoder;

    private Pose2d m_targetHub;

    private Shooter(){
        m_shootingMotors = new EverMotorControllerGroup(ShooterConsts.LEFT_MOTOR, ShooterConsts.RIGHT_MOTOR);
        m_encoder = new EverTalonFXInternalEncoder((EverTalonFX)ShooterConsts.ANGLE_MOTOR);
        
        if(Robot.m_alliance == Alliance.Blue){
            m_targetHub = ShooterConsts.BLUE_HUB_POSE;
        }
        else{
            m_targetHub = ShooterConsts.RED_HUB_POSE;
        }
        

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
        double mone = 2 * ShooterConsts.GRAVITY * Math.pow(getShootingDistance(),2);
        double mechane =
        2 * Math.pow(Math.cos(m_targetAngle), 2) * (getShootingDistance() * Math.tan(m_targetAngle) - ShooterConsts.SHOOTING_HIGHT); 
        m_targetSpeed = Math.sqrt(mone / mechane);
    }

    private double getShootingDistance(){
        double locX = SwerveLocalizer.getInstance().getCurrentPoint().getX();
        double locY = SwerveLocalizer.getInstance().getCurrentPoint().getY();
        return Math.sqrt(Math.pow(m_targetHub.getX() - locX, 2) + Math.pow(m_targetHub.getY() - locY, 2));
    }

    /*
     * calculates the minimum angle needed to shoot the ball to the hub based on the current distance and hight of the shooter
     * the formula is based on the physics of projectile motion
     */
    /*private double getMinShootingAngle(){
        return Math.atan(getShootingDistance() / Math.sqrt(Math.pow(getShootingDistance(), 2) + Math.pow(ShooterConsts.SHOOTING_HIGHT, 2)));
    }*/
    private double getMinShootingAngle(){
        double mone = Math.pow(m_targetSpeed,2) - 
                    Math.sqrt(Math.pow(m_targetSpeed,4) - ShooterConsts.GRAVITY * (ShooterConsts.GRAVITY * Math.pow(getShootingDistance(), 2) + 2 * ShooterConsts.SHOOTING_HIGHT * Math.pow(m_targetSpeed, 2)));
        double mechane = ShooterConsts.GRAVITY * getShootingDistance();
        return Math.toDegrees(Math.atan(mone / mechane));
    }

    public void setIsShooting(boolean isShooting){
        m_isShooting = isShooting;
    }

    public double getShootingSpeed(){
        return m_targetSpeed;
    }

    public double getRobotShootingOffsetAngle(){
        Pose2d pos = SwerveLocalizer.getInstance().getCurrentPoint();
        double x = m_targetHub.getX() - pos.getX();
        double y = m_targetHub.getY() - pos.getY();

        double robotsOffSetAngle = Math.toDegrees(Math.atan2(y, x));

        Vector2d fuelVectorAngle = new Vector2d(0,m_targetSpeed);
        Vector2d robotVector = Swerve.getInstance().getRobotOrientedVelocity();

        Vector2d ballVector = new Vector2d(robotVector.x, robotVector.y + fuelVectorAngle.y);
        return robotsOffSetAngle + Math.toDegrees(Math.atan2(ballVector.y, ballVector.x));
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

        m_currentSpeed = m_encoder.getVel();
        m_deltaSpeed = m_targetSpeed - m_currentSpeed;
    }


}
