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

    private ShooterState m_shooterState,m_previousShooterState;

    private DeltaTime m_deltaTime;


    private Shooter(){
        ShooterConsts.config();

        m_shootingMotors = new EverMotorControllerGroup(ShooterConsts.LEFT_MOTOR, ShooterConsts.RIGHT_MOTOR);
        m_angleMotor = ShooterConsts.ANGLE_MOTOR;
        m_angleEncoder = ShooterConsts.ANGLE_CAN_CODER;
        m_shootingEncoder = ShooterConsts.SHOOTING_ENCODER;

        //TODO: Consider re-initializing m_targetHub on alliance change since i would be locked to a certain alliance and would make debugging harder
        m_targetHub = Robot.m_alliance == Alliance.Blue ? ShooterConsts.BLUE_HUB_POSE : ShooterConsts.RED_HUB_POSE; 

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
    
    public void setShooterState(ShooterState shooterState){
        m_shooterState = shooterState;
    }

    //TODO: I would change the order of the functions to be in the order of which they are calling each other(1. getShootingDistance() 2. getMinShootingAngle ...)
    
    /*TODO:I would change the way these functions are written — instead of
      modifying class member variables, they should receive the necessary
      parameters as input and return the values they are responsible for. */

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

    //TODO: change the names of mone mechane to some kind of logic-related name 
    /**
     * calculates the minimum shooting angle needed to shoot the ball to the hub based on the current distance and hight of the shooter
     * the formula is based on the physics of projectile motion and it is derived from the formula
     * @param shootingSpeed the speed at which the ball is shot in m/s
     * @return the minimum shooting angle in degrees
     */
    private double calcMinShootingAngle(double shootingSpeed){
        double mone = Math.pow(shootingSpeed,2) - 
        Math.sqrt(Math.pow(shootingSpeed,4) - ShooterConsts.GRAVITY * (ShooterConsts.GRAVITY * Math.pow(getShootingDistance(), 2) + 2 * ShooterConsts.SHOOTING_HIGHT * Math.pow(shootingSpeed, 2)));
        double mechane = ShooterConsts.GRAVITY * getShootingDistance();
        return Math.toDegrees(Math.atan(mone / mechane));
    }
    
    private void setShootingAngleAndSpeed(){
        m_predictedSpeed = calcPredictedShooterSpeed();
        m_targetAngle = calcMinShootingAngle(m_predictedSpeed) + ShooterConsts.ANGLE_ERROR_MARGIN;
        m_targetAngle = MathUtil.clamp(m_targetAngle, ShooterConsts.MIN_ANGLE, ShooterConsts.MAX_ANGLE);
    }

    /**
     * calculates the offset angle of the robot relative to the target hub and the angle at which the ball will go out of the shooter based on the current speed and direction of the robot
     * @return the offset angle in degrees that needs to be added to the shooting angle to compensate for the robot's movement and field-relative orientation
     */
    public double calcRobotShootingOffsetAngle(){
        Pose2d pos = SwerveLocalizer.getInstance().getCurrentPoint();
        double x = m_targetHub.getX() - pos.getX();
        double y = m_targetHub.getY() - pos.getY();

        double robotsOffSetAngle = Math.toDegrees(Math.atan2(y, x));//TODO: Offset is a single word

        Vector2d robotVector = Swerve.getInstance().getRobotOrientedVelocity();//TODO: I would change the name of robotVector to something like robotVelocity to indicate that it is the velocity of the robot and not just a random vector

        return robotsOffSetAngle + Math.toDegrees(Math.atan2(robotVector.y + m_targetSpeed, robotVector.x)); //TODO: shouldnt you use predicted speed here? also i would consider moving this function out of this subsystem 
    }

    @Override
    public void periodic() {

        if((m_angleAbsEncoder.getAbsPos() >= ShooterConsts.MAX_ANGLE && m_angleMotor.get() > 0)){
            m_targetAngle = ShooterConsts.MAX_ANGLE; //TODO: i dont see a reason for this line to exist since you are clamping the target angle in setShootingAngleAndSpeed()
            m_angleMotor.stop();
        }
        else if(m_angleAbsEncoder.getAbsPos() <= ShooterConsts.MIN_ANGLE && m_angleMotor.get() < 0){
            m_targetAngle = ShooterConsts.MIN_ANGLE;//TODO: same here
            m_angleMotor.stop();
        }
        
        switch (m_shooterState) {
            case kScoring:
                    setShootingAngleAndSpeed();
                    m_shootingPID.activate(Funcs.getSpeedInRPM(ShooterConsts.WHEEL_RADIUS ,m_predictedSpeed), ControlType.kVel); //TODO: i would change the name of Funcs.getSpeedInRPM to something like Funcs.convertMpsToRpm or something that indicates that it is converting the speed from m/s to rpm 
                    m_anglePID.activate(m_targetAngle, ControlType.kPos);
                break;
        
            case kDelivery:
                    m_shootingPID.activate(Funcs.getSpeedInRPM(ShooterConsts.WHEEL_RADIUS ,m_predictedSpeed), ControlType.kVel);
                    //TODO: Extract hardcoded angle 45 to a constant (e.g., DELIVERY_ANGLE)
                    m_anglePID.activate(45, ControlType.kPos);
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
        SmartDashboard.putNumber("Shooter Speed", Funcs.getSpeedInRPM(ShooterConsts.WHEEL_RADIUS, m_predictedSpeed));
        SmartDashboard.putNumber("Shooter Current Speed", m_predictedSpeed * 60 / (ShooterConsts.WHEEL_RADIUS * 2 * Math.PI));
    }


}
