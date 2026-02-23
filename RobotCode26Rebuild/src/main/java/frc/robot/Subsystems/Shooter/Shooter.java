package frc.robot.Subsystems.Shooter;

import javax.xml.crypto.KeySelector.Purpose;

import org.opencv.core.Point;

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
import frc.robot.Utils.Math.Vector2d;


public class Shooter extends SubsystemBase{
    //TODO: move hardware to ShooterConsts only abstract pointers should be used in the subsystem itself, otherwise using everkit is pointless....

    private static Shooter m_instance = new Shooter();

    private double m_targetAngle, m_targetSpeed,
            m_deltaSpeed, m_currentSpeed;


    private EverMotorControllerGroup m_shootingMotors;

    private EverCANCoder m_angleAbsEncoder = ShooterConsts.ANGLE_CAN_CODER;

    private EverPIDController 
            m_anglePID = new EverTalonFXPIDController((EverTalonFX)ShooterConsts.ANGLE_MOTOR),
            m_shootingPID = new EverSparkMaxPIDController((EverSparkMax)ShooterConsts.LEFT_MOTOR);

    private EverEncoder m_encoder;

    private Pose2d m_targetHub;

    private ShooterConsts.ShooterState m_shooterState = ShooterConsts.ShooterState.kStop,
             m_previousShooterState = ShooterConsts.ShooterState.kStop;

    private DeltaTime m_deltaTime = new DeltaTime();

    //TODO: why are so many vars are initiated outside of the constructor

    private Shooter(){
        m_shootingMotors = new EverMotorControllerGroup(ShooterConsts.LEFT_MOTOR, ShooterConsts.RIGHT_MOTOR);
        m_encoder = new EverTalonFXInternalEncoder((EverTalonFX)ShooterConsts.ANGLE_MOTOR);
        
        //TODO: i would use () ? : ;
        if(Robot.m_alliance == Alliance.Blue){
            m_targetHub = ShooterConsts.BLUE_HUB_POSE;
        }
        else{
            m_targetHub = ShooterConsts.RED_HUB_POSE;
        }

        m_targetSpeed = ShooterConsts.TARGET_RPM * ShooterConsts.WHEEL_RADIUS * 2 * Math.PI / 60; // convert rpm to m/s // TODO: move the conversion between rpm and m/s to a function
    }

    public static Shooter getInstance(){
        return m_instance;
    }
    
    public void setShooterState(ShooterConsts.ShooterState shooterState){ //TODO: i would move ShooterState enum to this class or to a new file
        m_shooterState = shooterState;
    }

    //TODO: I would change the order of the functions to be in the order of which they are calling each other(1. getShootingDistance() 2. getMinShootingAngle ...)
    /*TODO:I would change the way these functions are written — instead of
      modifying class member variables, they should receive the necessary
      parameters as input and return the values they are responsible for. */

    private void setShootingAngle(){
        m_targetAngle = getMinShootingAngle() + ShooterConsts.ANGLE_ERROR_MARGIN;
        //TODO: just use MathUtil.clamp 
        if(m_targetAngle > ShooterConsts.MAX_ANGLE){
            m_targetAngle = ShooterConsts.MAX_ANGLE;
        }
        else if(m_targetAngle < ShooterConsts.MIN_ANGLE){
            m_targetAngle = ShooterConsts.MIN_ANGLE;
        }
    }

    //TODO: use normal javadocs
    /*
     * calculates the target speed and angle needed to shoot the ball to the hub based on the current distance and hight of the shooter
     * the formula is based on the physics of projectile motion
    */
    private void setShootingSpeed(){
        setShootingAngle();
        //TODO: from what i understood from the explanation we keep the shooting speed a const what is this calculation 
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

    //TODO: use normal javadocs
    //TODO: change the names of mone mechane to some kind of logic-related name 
    /*
     * calculates the minimum angle needed to shoot the ball to the hub based on the current distance and hight of the shooter
     * the formula is based on the physics of projectile motion
     */
    private double getMinShootingAngle(){
        double mone = Math.pow(m_currentSpeed,2) - 
                    Math.sqrt(Math.pow(m_currentSpeed,4) - ShooterConsts.GRAVITY * (ShooterConsts.GRAVITY * Math.pow(getShootingDistance(), 2) + 2 * ShooterConsts.SHOOTING_HIGHT * Math.pow(m_currentSpeed, 2)));
        double mechane = ShooterConsts.GRAVITY * getShootingDistance();
        return Math.toDegrees(Math.atan(mone / mechane));
    }

    public double getRobotShootingOffsetAngle(){
        Pose2d pos = SwerveLocalizer.getInstance().getCurrentPoint();
        double x = m_targetHub.getX() - pos.getX();
        double y = m_targetHub.getY() - pos.getY();

        double robotsOffSetAngle = Math.toDegrees(Math.atan2(y, x));

        Vector2d fuelVectorAngle = new Vector2d(0,m_targetSpeed);
        Vector2d robotVector = Swerve.getInstance().getRobotOrientedVelocity();

        Vector2d ballVector = new Vector2d(robotVector.x, robotVector.y + fuelVectorAngle.y); //TODO: why not just add m_targetSpeed directly?
        return robotsOffSetAngle + Math.toDegrees(Math.atan2(ballVector.y, ballVector.x)); //TODO: you could reduce the amount of lines here by using the values instead of creating vectors since you are not using any of the vector functions
    }

    public double getSpeedInRPM(){
        return m_targetSpeed * 60 / (ShooterConsts.WHEEL_RADIUS * 2 * Math.PI); // convert m/s to rpm
    }

    @Override
    public void periodic() {
        //TODO: if this is an attempt to stop the shooter from moving past the max/min angles you should stop the motor 
        if((m_angleAbsEncoder.getAbsPos() >= ShooterConsts.MAX_ANGLE && ShooterConsts.ANGLE_MOTOR.get() > 0))
            m_targetAngle = ShooterConsts.MAX_ANGLE;
        else if(m_angleAbsEncoder.getAbsPos() <= ShooterConsts.MIN_ANGLE && ShooterConsts.ANGLE_MOTOR.get() < 0)
                m_targetAngle = ShooterConsts.MIN_ANGLE;
        
        //Nadav said to do not sure how to use 
        m_currentSpeed = m_encoder.getVel(); //TOOD: why is m_deltaSpeed not a local variable and the predicted speed calculation seems off
        m_deltaSpeed = m_targetSpeed - m_currentSpeed; 
        double predictedSpeed = m_currentSpeed + (m_deltaSpeed / m_deltaTime.get()) * 0.1; // 0.1 -> FEEDING_TIME //TODO: predicted speed is not even used 

        //TODO: i would use switch case
        if(m_shooterState == ShooterConsts.ShooterState.kScoring ){ 
            setShootingSpeed(); 
            m_shootingPID.activate(getSpeedInRPM(), ControlType.kVel);
            m_anglePID.activate(m_targetAngle, ControlType.kPos);
        }
        else if(m_shooterState == ShooterConsts.ShooterState.kDelivery){
            m_shootingPID.activate(getSpeedInRPM(), ControlType.kVel);
            m_anglePID.activate(45, ControlType.kPos);
        }
        if(m_shooterState == ShooterConsts.ShooterState.kStop && m_previousShooterState != ShooterConsts.ShooterState.kStop){
            m_shootingPID.stop();
            m_anglePID.stop();
            m_shootingMotors.stop();
        }
        m_previousShooterState = m_shooterState;

        if(ShooterConsts.DEBUG_MODE){
            log();
        }
    }

    public void log(){
        SmartDashboard.putNumber("Shooter Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Shooter Current Angle", m_angleAbsEncoder.getAbsPos());
        SmartDashboard.putNumber("Shooter Target Speed", getSpeedInRPM());
        SmartDashboard.putNumber("Shooter Current Speed", m_currentSpeed * 60 / (ShooterConsts.WHEEL_RADIUS * 2 * Math.PI));
    }


}
