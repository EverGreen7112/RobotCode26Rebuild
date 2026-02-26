package frc.robot.Subsystems.Shooter;

import javax.xml.crypto.KeySelector.Purpose;

import org.opencv.core.Point;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.DeltaTime;
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
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;


public class Shooter extends SubsystemBase{

    public enum ShooterState{
        kStop, kScoring, kDelivery
    }

    private static Shooter m_instance = new Shooter();

    private double m_targetAngle, m_targetSpeed, m_predictedSpeed;

    private EverMotorControllerGroup m_shootingMotors;

    private EverMotorController m_angleMotor;

    private EverCANCoder m_angleAbsEncoder = ShooterConsts.ANGLE_CAN_CODER;

    private EverPIDController m_anglePID, m_shootingPID;

    private EverEncoder m_angleEncoder, m_shootingEncoder;

    private Pose2d m_targetHub;

    private ShooterState m_shooterState, m_previousShooterState;

    private DeltaTime m_deltaTime;


    private Shooter(){
        ShooterConsts.config();

        m_shootingMotors = new EverMotorControllerGroup(ShooterConsts.LEFT_MOTOR, ShooterConsts.RIGHT_MOTOR);
        m_angleMotor = ShooterConsts.ANGLE_MOTOR;
        m_angleEncoder = ShooterConsts.ANGLE_CAN_CODER;
        m_shootingEncoder = ShooterConsts.SHOOTING_ENCODER;

        m_targetSpeed = Funcs.getSpeedInMPS(ShooterConsts.WHEEL_RADIUS, ShooterConsts.TARGET_RPM);

        m_deltaTime = new DeltaTime();

        m_anglePID = ShooterConsts.ANGLE_PID_CONTROLLER;
        m_shootingPID = ShooterConsts.SHOOTING_PID_CONTROLLER;

        m_shooterState = ShooterState.kStop;
        m_previousShooterState = ShooterState.kStop;
    }

    public static Shooter getInstance(){
        return m_instance;
    }

    public void setAllianceHub(boolean isBlue){
        m_targetHub = isBlue ? ShooterConsts.BLUE_HUB_POSE : ShooterConsts.RED_HUB_POSE;
    }
    
    public void setShooterState(ShooterState shooterState){
        m_shooterState = shooterState;
    }

    private double getShootingDistance(){
        double locX = SwerveLocalizer.getInstance().getCurrentPoint().getX();
        double locY = SwerveLocalizer.getInstance().getCurrentPoint().getY();
        return Math.sqrt(Math.pow(m_targetHub.getX() - locX, 2) + Math.pow(m_targetHub.getY() - locY, 2));
    }
    
    /**
     * calculates the speed at which we predict the shooter will be at when the ball is fed into it based on the current speed and the target speed
     * @return the predicted shooter speed in m/s
     */
    private double calcPredictedShooterSpeed(){
        double currentSpeed = m_shootingEncoder.getVel();
        double deltaSpeed = m_targetSpeed - currentSpeed; //TODO: add a const to store the feeding time
        return currentSpeed + (deltaSpeed / m_deltaTime.get()) * 0.1; // 0.1 -> FEEDING_TIME 
    }

    /**
     * calculates the minimum shooting angle needed to shoot the ball to the hub based on the current distance and hight of the shooter
     * the formula is based on the physics of projectile motion and it is derived from the formula
     * @param shootingSpeed the speed at which the ball is shot in m/s
     * @return the minimum shooting angle in degrees
     */
    private double calcMinShootingAngle(double shootingSpeed){
        double verticalVelocity = Math.pow(shootingSpeed,2) - 
        Math.sqrt(Math.pow(shootingSpeed,4) - ShooterConsts.GRAVITY * (ShooterConsts.GRAVITY * Math.pow(getShootingDistance(), 2) + 2 * ShooterConsts.SHOOTING_HIGHT * Math.pow(shootingSpeed, 2)));
        
        double horizontalForce = ShooterConsts.GRAVITY * getShootingDistance();
        return Math.toDegrees(Math.atan(verticalVelocity / horizontalForce));
    }

    /**
     * calculates the offset angle of the robot relative to the target hub and the angle at which the ball will go out of the shooter based on the current speed and direction of the robot
     * @return the offset angle in degrees that needs to be added to the shooting angle to compensate for the robot's movement and field-relative orientation
     */
    // move this to a correct place
    public double calcRobotShootingOffsetAngle(){
        Pose2d pos = SwerveLocalizer.getInstance().getCurrentPoint();
        double x = m_targetHub.getX() - pos.getX();
        double y = m_targetHub.getY() - pos.getY();

        double robotsOffsetAngle = Math.toDegrees(Math.atan2(y, x));

        Vector2d robotVelocity = Swerve.getInstance().getRobotOrientedVelocity();

        return robotsOffsetAngle + Math.toDegrees(Math.atan2(robotVelocity.y + m_predictedSpeed, robotVelocity.x));
    }

    @Override
    public void periodic() {

        if((m_angleAbsEncoder.getAbsPos() >= ShooterConsts.MAX_ANGLE && m_angleMotor.get() > 0)){
            m_angleMotor.stop();
        }
        else if(m_angleAbsEncoder.getAbsPos() <= ShooterConsts.MIN_ANGLE && m_angleMotor.get() < 0){
            m_angleMotor.stop();
        }
        
        switch (m_shooterState) {
            case kScoring:
                    m_targetAngle = MathUtil.clamp(m_targetAngle, ShooterConsts.MIN_ANGLE, ShooterConsts.MAX_ANGLE);
                    m_shootingPID.activate(Funcs.convertMStoRPM(ShooterConsts.WHEEL_RADIUS ,m_targetSpeed), ControlType.kVel); //TODO: i would change the name of Funcs.getSpeedInRPM to something like Funcs.convertMpsToRpm or something that indicates that it is converting the speed from m/s to rpm 
                    m_anglePID.activate(m_targetAngle, ControlType.kPos);
                break;
        
            case kDelivery:
                    m_shootingPID.activate(Funcs.convertMStoRPM(ShooterConsts.WHEEL_RADIUS ,m_targetSpeed), ControlType.kVel);
                    m_anglePID.activate(ShooterConsts.DELIVERY_ANGLE, ControlType.kPos);
                break;
            
            case kStop: 
                if(m_previousShooterState != ShooterState.kStop){ 
                    m_shootingPID.stop();
                    m_anglePID.stop();
                    m_shootingMotors.stop();
                }
                break;
        }

        m_previousShooterState = m_shooterState;

        if(ShooterConsts.DEBUG_MODE){
            log();
        }
    }

    public void log(){
        SmartDashboard.putNumber("Shooter Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Shooter Current Angle", m_angleAbsEncoder.getAbsPos());
        SmartDashboard.putNumber("Shooter Speed", Funcs.convertMStoRPM(ShooterConsts.WHEEL_RADIUS, m_predictedSpeed));
        SmartDashboard.putNumber("Shooter Current Speed", Funcs.getSpeedInMPS(ShooterConsts.WHEEL_RADIUS, m_predictedSpeed));
    }

}
