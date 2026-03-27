package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Commands.ResetRobotCommand;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Consts.FeedAndConveyConsts;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

    private double m_targetAngle, m_targetSpeed, m_predictedBallV0, m_prevSpeed;

    private EverPIDController m_frontShootingController, m_backShootingController;

    private EverEncoder m_angleEncoder;
    
    private EverEncoder m_frontShootingEncoder, m_backShootingEncoder;

    private Pose2d m_targetHub;

    private ShooterState m_shooterState, m_previousShooterState;

    private DeltaTime m_deltaTime;

    private Pose2d m_staticShootingPose;
    
    private Pose2d m_deliveryPoints;

    private EverMotorController m_backMotor, m_frontMotor;

    private InterpolatingDoubleTreeMap m_ballV0ToRPS;

    private Shooter() {
        ShooterConsts.config();
        m_backMotor = ShooterConsts.FRONT_MOTOR;
        m_frontMotor = ShooterConsts.BACK_MOTOR;
        m_frontShootingEncoder = ShooterConsts.FRONT_SHOOTING_ENCODER;
        m_backShootingEncoder = ShooterConsts.BACK_SHOOTING_ENCODER;

        m_targetSpeed = 0;

        m_deltaTime = new DeltaTime();

        m_frontShootingController = ShooterConsts.FRONT_SHOOTING_PID_CONTROLLER_;
        m_backShootingController = ShooterConsts.BACK_SHOOTING_PID_CONTROLLER_;

        m_shooterState = ShooterState.kStop;
        m_previousShooterState = ShooterState.kStop;
        //14.9;
        m_targetHub = ShooterConsts.RED_HUB_POSE;

        m_staticShootingPose = ShooterConsts.STATIC_SHOOT_POSE_RED;
        m_deliveryPoints = DELIVERY_POSES_RED;

        m_angleEncoder = ANGLE_ENCODER;
        m_angleEncoder.setPos(0);

        m_ballV0ToRPS = BALL_SPEED_TO_SHOOTER_TABLE;

        m_prevSpeed = 0;

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
        return 1.2;
    }

    /**
     * calculates the fuels speed 
     * @return speed in m/s
     */
    private double calcBallV0MS() {
        double robotDistance = getShootingDistance();
        double mone = GRAVITY * Math.pow(robotDistance,2);
        double mechana = Math.max(2 * (Math.pow(Math.cos(Math.toRadians(SHOOTING_ANGLE)) ,2) * (robotDistance * Math.tan(Math.toRadians(SHOOTING_ANGLE)) - SHOOTING_HEIGHT) ), 0.0);
        SmartDashboard.putNumber("ballMs",Math.sqrt(mone / mechana) );
        return Math.sqrt(mone / mechana);
        //return 10;
    }


    private double calcShooterSpeed(double ballV0MS){

        double minInput = BALL_V0_DATA[0];
        double maxInput = BALL_V0_DATA[BALL_V0_DATA.length - 1];

        double clampedInput = MathUtil.clamp(ballV0MS, minInput, maxInput);
        return m_ballV0ToRPS.get(clampedInput);
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

    @Override
    public void periodic() {
        SmartDashboard.putString("state", m_shooterState + "");

        if (m_angleEncoder.getPos() >= ShooterConsts.MAX_ANGLE && ANGLE_MOTOR.get() > 0) {
            ANGLE_MOTOR.stop();
        } else if (m_angleEncoder.getPos() <= ShooterConsts.MIN_ANGLE && ANGLE_MOTOR.get() < 0) {
            ANGLE_MOTOR.stop();
        }

        switch (m_shooterState) {// kScoring is default
            case kScoring:
                m_targetSpeed = calcShooterSpeed(calcBallV0MS()) * 1.15;
                m_frontShootingController.activate(m_targetSpeed, ControlType.kVel);
                m_backShootingController.activate((m_targetSpeed * WHEELS_RATIO), ControlType.kVel);
                //anglePos(SCORING_ANGLE);
                break;
            case kDelivery:
                m_frontShootingController.activate(ShooterConsts.DELIVERY_RPS, ControlType.kVel);
                m_backShootingController.activate(ShooterConsts.DELIVERY_RPS * ShooterConsts.WHEELS_RATIO, ControlType.kVel);
                break;

            case kStaticPoint:
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
                    m_frontMotor.stop();
                    m_backMotor.stop();
                    //SwerveAngleController.getInstance().stop(); 
                }
                break;
            case kTest:
                frontShootingRpm(32);
                backShootingRpm(32 * WHEELS_RATIO);

        }

        m_previousShooterState = m_shooterState;

        if (ShooterConsts.DEBUG_MODE) {
            log();
        }
        m_deltaTime.setNow();
    }

    public void log() {
        SmartDashboard.putNumber("target angle", m_angleEncoder.getPos());
        SmartDashboard.putNumber("Ball speed", calcBallV0MS());

        SmartDashboard.putNumber("shooter speed", m_targetSpeed);

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

    public void anglePos(double angle){
        ANGLE_PID_CONTROLLER.activate(angle, ControlType.kPos);
    }

}
