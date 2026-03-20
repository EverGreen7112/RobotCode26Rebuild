package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Commands.ResetRobotCommand;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Consts.ShooterConsts;
import frc.robot.Subsystems.Conveyor.Conveyer;
import frc.robot.Subsystems.Feeder.Feeder;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.DeltaTime;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverDutyCycleEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverMotorControllerGroup;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.Math.Funcs;

import java.lang.annotation.Target;

import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import frc.robot.Utils.Math.Vector2d;

public class Shooter extends SubsystemBase implements Consts.ShooterConsts {

    public enum ShooterState {
        kStop,
        kScoring,
        kDelivery,
        kClose,
        kStaticPoint,
        kTest
    }

    private static Shooter m_instance = new Shooter();

    private double m_targetAngle, m_targetSpeed, m_predictedBallV0, m_prevSpeed, m_prevAngle;

    private EverMotorController m_angleMotor;

    private EverPIDController m_anglePID, m_frontShootingController, m_backShootingController;
    
    private EverEncoder m_angleEncoder, m_frontShootingEncoder, m_backShootingEncoder;

    private Pose2d m_targetHub;

    private ShooterState m_shooterState, m_previousShooterState;

    private InterpolatingDoubleTreeMap m_shooterSpeedToPredictedBallV0;

    private DeltaTime m_deltaTime;

    private Pose2d m_staticShootingPose;
    
    private Pose2d m_deliveryPoints;

    private EverMotorController m_backMotor, m_frontMotor;

    private Shooter() {
        ShooterConsts.config();
        m_backMotor = ShooterConsts.FRONT_MOTOR;
        m_frontMotor = ShooterConsts.BACK_MOTOR;
        m_angleMotor = ShooterConsts.ANGLE_MOTOR;
        m_angleEncoder = ShooterConsts.ANGLE_ENCODER;
        m_frontShootingEncoder = ShooterConsts.FRONT_SHOOTING_ENCODER;
        m_backShootingEncoder = ShooterConsts.BACK_SHOOTING_ENCODER;

        m_targetSpeed = Funcs.convertRPStoMS(ShooterConsts.FRONT_WHEEL_RADIUS, ShooterConsts.TARGET_RPS);

        m_deltaTime = new DeltaTime();

        m_anglePID = ShooterConsts.ANGLE_PID_CONTROLLER;
        m_frontShootingController = ShooterConsts.FRONT_SHOOTING_PID_CONTROLLER_;
        m_backShootingController = ShooterConsts.BACK_SHOOTING_PID_CONTROLLER_;

        m_shooterState = ShooterState.kScoring;
        m_previousShooterState = ShooterState.kStop;

        m_shooterSpeedToPredictedBallV0 = ShooterConsts.BALL_SPEED_TO_SHOOTER_TABLE;

        m_targetHub = ShooterConsts.BLUE_HUB_POSE;

        m_staticShootingPose = ShooterConsts.STATIC_SHOOT_POSE_BLUE;
        m_deliveryPoints = DELIVERY_POSES_BLUE;

        m_targetAngle = 1;
        m_predictedBallV0 = 1;
        m_prevSpeed = m_targetSpeed;
    }

    public static Shooter getInstance() {
        return m_instance;
    }

    public void ConfigureAllianceShootingSetting(boolean isBlue) {
        m_targetHub = isBlue ? ShooterConsts.BLUE_HUB_POSE : ShooterConsts.RED_HUB_POSE;
        m_staticShootingPose = isBlue ? ShooterConsts.STATIC_SHOOT_POSE_BLUE : ShooterConsts.STATIC_SHOOT_POSE_RED;
        m_deliveryPoints = isBlue ? DELIVERY_POSES_BLUE : DELIVERY_POSES_RED;
        
    }

    public void setShooterState(ShooterState shooterState) {
        m_shooterState = shooterState;
    }

    private double getShootingDistance() {
        double locX = SwerveLocalizer.getInstance().getCurrentPoint().getX();
        double locY = SwerveLocalizer.getInstance().getCurrentPoint().getY();
        if(m_shooterState == ShooterState.kStaticPoint){
            locX = m_staticShootingPose.getX();
            locY = m_staticShootingPose.getY();
        }
        else if(m_shooterState == ShooterState.kDelivery){
            locX = m_deliveryPoints.getX();
            locY = m_deliveryPoints.getY();
        }
        //return Math.sqrt(Math.pow(m_targetHub.getX() - locX, 2) + Math.pow(m_targetHub.getY() - locY, 2));
        return 5;
    }

    /**
     * calculates the predicted speed of the shooter when the ball will be fed into the it
     * @return the predicted shooter speed in m/s
     */
    private double calcBallV0MS() {
        return 0;
    }


    // TODO: place holder for the real thing
    private double calcShooterSpeed(double ballV0MS){
        //return m_shooterSpeedToPredictedBallV0.get(MathUtil.clamp(shooterSpeedMS, ShooterConsts.SHOOTER_SPEED[0], m_targetSpeed));
        return ShooterConsts.BALL_SPEED_TO_SHOOTER_TABLE.get(ballV0MS);
    }



    public double calcRobotShootingOffsetAngle(Pose2d robotPose) {
        Pose2d pos = robotPose;
        double x = m_targetHub.getX() - pos.getX();
        double y = m_targetHub.getY() - pos.getY();

        double robotsOffsetAngleFromHub = Math.toDegrees(Math.atan2(y, x));

        Vector2d robotVelocity = Swerve.getInstance().getRobotOrientedVelocity();

        return robotsOffsetAngleFromHub + Math.toDegrees(Math.atan2(robotVelocity.y + m_predictedBallV0, robotVelocity.x));
    }

    public ShooterState getShooterState(){
        return m_shooterState;
    }

    private void shootingFunc(){
        m_predictedBallV0 = calcBallV0MS(calcPredictedShooterSpeed());
        m_targetAngle = MathUtil.clamp(
            calcShootingAngle(m_predictedBallV0), ShooterConsts.MIN_ANGLE, ShooterConsts.MAX_ANGLE);
        //SwerveAngleController.getInstance().start(calcRobotShootingOffsetAngle(SwerveLocalizer.getInstance().getCurrentPoint()));
    }

    @Override
    public void periodic() {

        if (m_angleEncoder.getPos() >= ShooterConsts.MAX_ANGLE && m_angleMotor.get() > 0) {
            m_angleMotor.stop();
        } else if (m_angleEncoder.getPos() <= ShooterConsts.MIN_ANGLE && m_angleMotor.get() < 0) {
            m_angleMotor.stop();
        }

        switch (m_shooterState) {// kScoring is default
            default:
                m_predictedBallV0 = calcBallV0MS(calcPredictedShooterSpeed());
                double targetAngle = MathUtil.clamp(
                    calcShootingAngle(m_predictedBallV0), ShooterConsts.MIN_ANGLE, ShooterConsts.MAX_ANGLE);
                if(Math.abs(targetAngle - m_prevAngle) > DEAD_ZONE ){
                    m_prevAngle = m_targetAngle;
                    m_targetAngle = targetAngle;
                }
                m_frontShootingController.activate(ShooterConsts.TARGET_RPS, ControlType.kVel);
                m_backShootingController.activate(ShooterConsts.TARGET_RPS * ShooterConsts.WHEELS_RATIO, ControlType.kVel);
                anglePos(m_targetAngle);
                    Feeder.getInstance().startFeed();
                    Conveyer.getInstance().startConveying(-0.5);
                break;
            case kDelivery:
                shootingFunc();
                m_frontShootingController.activate(ShooterConsts.DELIVERY_RPS, ControlType.kVel);
                m_backShootingController.activate(ShooterConsts.DELIVERY_RPS * ShooterConsts.WHEELS_RATIO, ControlType.kVel);
                break;

            case kStaticPoint:
                shootingFunc();
                m_frontShootingController.activate(ShooterConsts.TARGET_RPS, ControlType.kVel);
                m_backShootingController.activate(ShooterConsts.TARGET_RPS * ShooterConsts.WHEELS_RATIO, ControlType.kVel);
                break;
                
            case kClose:
                m_frontShootingController.stop();
                m_backShootingController.stop();
                //SwerveAngleController.getInstance().stop();
                break;
                
            case kStop:
                if (m_previousShooterState != ShooterState.kStop) {
                    m_frontShootingController.stop();
                    m_backShootingController.stop();
                    m_anglePID.stop();
                    m_frontMotor.stop();
                    m_backMotor.stop();
                    //SwerveAngleController.getInstance().stop(); 
                }
                break;
            case kTest:
                frontShootingRpm(37.59);
                backShootingRpm(37.59 * WHEELS_RATIO);
                anglePos(20);
        }

        m_previousShooterState = m_shooterState;

        if (ShooterConsts.DEBUG_MODE) {
            log();
        }
        m_deltaTime.setNow();
    }

    public void log() {
        SmartDashboard.putNumber("Shooter Current Angle", m_angleEncoder.getPos());
        SmartDashboard.putNumber("target angle", m_targetAngle);
        SmartDashboard.putNumber("Ball speed", m_predictedBallV0);
        SmartDashboard.putNumber("predicted speed", calcPredictedShooterSpeed());

        SmartDashboard.putNumber("calc angle",calcShootingAngle(m_predictedBallV0));

        SmartDashboard.putNumber("Front Wheel Shooting Speed", m_frontShootingEncoder.getVel());
        SmartDashboard.putNumber("Back Wheel Shooting Speed", m_backShootingEncoder.getVel());

        SmartDashboard.putString("shooter state", m_shooterState + "");
    }

    public void frontShootingRpm(double rps) {
        m_frontShootingController.activate(rps, ControlType.kVel);
    }

    public void backShootingRpm(double rps) {
        m_backShootingController.activate(rps, ControlType.kVel);
    }

    public void anglePos(double angle) {
        m_anglePID.activate(angle, ControlType.kPos);
    }
}
