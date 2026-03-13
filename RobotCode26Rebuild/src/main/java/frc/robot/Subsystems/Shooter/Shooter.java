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

    private double m_targetAngle, m_targetSpeed, m_predictedBallV0;

    private EverMotorControllerGroup m_shootingMotors;

    private EverMotorController m_angleMotor;

    private EverPIDController m_anglePID, m_frontShootingController, m_backShootingController;
    
    private EverEncoder m_angleEncoder, m_frontShootingEncoder, m_backShootingEncoder;

    private Pose2d m_targetHub;

    private ShooterState m_shooterState, m_previousShooterState;

    private InterpolatingDoubleTreeMap m_shooterSpeedToPredictedBallV0;

    private DeltaTime m_deltaTime;

    private Pose2d m_staticShootingPose;

    private EverMotorController m_backMotor, m_frontMotor;

    private double m_filteredFrontShootingSpeed, m_filteredBackShootingSpeed, m_prevFilteredFrontShootingSpeed, m_prevFilteredBackShootingSpeed;

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

        m_shooterState = ShooterState.kStop;
        m_previousShooterState = ShooterState.kStop;

        m_shooterSpeedToPredictedBallV0 = ShooterConsts.SHOOTER_TO_BALL_SPEED_TABLE;

        m_targetHub = ShooterConsts.BLUE_HUB_POSE;

        m_staticShootingPose = ShooterConsts.STATIC_SHOOT_POSE_BLUE;

        m_prevFilteredFrontShootingSpeed = 0;
        m_prevFilteredBackShootingSpeed = 0;
    }

    public static Shooter getInstance() {
        return m_instance;
    }

    public void ConfigureAllianceShootingSetting(boolean isBlue) {
        m_targetHub = isBlue ? ShooterConsts.BLUE_HUB_POSE : ShooterConsts.RED_HUB_POSE;
        m_staticShootingPose = isBlue ? ShooterConsts.STATIC_SHOOT_POSE_BLUE : ShooterConsts.STATIC_SHOOT_POSE_RED;
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
        return Math.sqrt(Math.pow(m_targetHub.getX() - locX, 2) + Math.pow(m_targetHub.getY() - locY, 2));
    }

    /**
     * calculates the predicted speed of the shooter when the ball will be fed into the it
     * @return the predicted shooter speed in m/s
     */
    private double calcPredictedShooterSpeed() {
        double currentSpeed = m_frontShootingEncoder.getVel();
        double deltaSpeed = m_targetSpeed - currentSpeed;
        return currentSpeed + (deltaSpeed / m_deltaTime.get()) * Consts.FeedAndConveyConsts.FEEDING_TIME; 
    }

    // TODO: need to consider which angle we take. the function return two angles + and -
    /**
     * calculates the shooting angle needed to shoot the ball to the hub
     * based on the current distance and hight of the shooter
     * the formula is based on the physics of projectile motion
     * 
     * @param shootingSpeed the speed at which the ball will be shot in m/s
     * @return the shooting angle in degrees
     */
    private double calcShootingAngle(double shootingSpeed) {
        double shootingDistance = getShootingDistance();
        double verticalVelocity = Math.pow(shootingSpeed, 2) -
                Math.sqrt(Math.pow(shootingSpeed, 4)
                        - GRAVITY * (GRAVITY * Math.pow(shootingDistance, 2)
                                + 2 * SHOOTING_HEIGHT * Math.pow(shootingSpeed, 2)));

        double horizontalForce = GRAVITY * shootingDistance;
        return Math.toDegrees(Math.atan(verticalVelocity / horizontalForce)) - SHOOTING_OFFSET_FROM_GROUND;
    }

    /**
     * calculates the offset angle of the robot relative to the target hub and the
     * angle at which the ball will go out of the shooter based on the current speed
     * and direction of the robot
     * 
     * @return the offset angle in degrees that needs to be added to the robot's
     *         shooting angle to compensate for the robot's movement and
     *         field-relative orientation
     */
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

    public boolean isClosed(){
        return !ANGLE_MOTOR.getControllerInstance().getReverseLimitSwitch().isPressed();
    }


    @Override
    public void periodic() {

        if (m_angleEncoder.getPos() >= ShooterConsts.MAX_ANGLE && m_angleMotor.get() > 0) {
            m_angleMotor.stop();
        } else if (m_angleEncoder.getPos() <= ShooterConsts.MIN_ANGLE && m_angleMotor.get() < 0 && isClosed()) {
            m_angleMotor.stop();
        }

        if(isClosed()){
            m_angleEncoder.setPos(0);
        }

        switch (m_shooterState) {
            case kScoring:
                m_predictedBallV0 = m_shooterSpeedToPredictedBallV0.get(MathUtil.clamp(
                    calcPredictedShooterSpeed(), ShooterConsts.SHOOTER_SPEED[0], m_targetSpeed));
                m_targetAngle = MathUtil.clamp(
                    calcShootingAngle(m_predictedBallV0), ShooterConsts.MIN_ANGLE, ShooterConsts.MAX_ANGLE);
                SwerveAngleController.getInstance().start(calcRobotShootingOffsetAngle(SwerveLocalizer.getInstance().getCurrentPoint()));
                m_anglePID.activate(m_targetAngle, ControlType.kPos);
                m_frontShootingController.activate(ShooterConsts.TARGET_RPS, ControlType.kVel);
                m_backShootingController.activate(ShooterConsts.TARGET_RPS * ShooterConsts.WHEELS_RATIO, ControlType.kVel);
                break;

            case kDelivery:
                m_anglePID.activate(ShooterConsts.DELIVERY_ANGLE, ControlType.kPos);
                m_frontShootingController.activate(ShooterConsts.DELIVERY_RPS, ControlType.kVel);
                m_backShootingController.activate(ShooterConsts.DELIVERY_RPS * ShooterConsts.WHEELS_RATIO, ControlType.kVel);
                SwerveAngleController.getInstance().stop();
                break;

                
            case kClose:
                m_frontShootingController.stop();
                if(m_previousShooterState != ShooterState.kClose && !isClosed()){
                    m_angleMotor.set(-0.2);
                }
                else if(isClosed()){
                    m_angleMotor.stop();
                }
                SwerveAngleController.getInstance().stop();
                break;
                
            case kStaticPoint:
                m_predictedBallV0 = m_shooterSpeedToPredictedBallV0.get(MathUtil.clamp(
                    calcPredictedShooterSpeed(), ShooterConsts.SHOOTER_SPEED[0], m_targetSpeed));
                m_targetAngle = MathUtil.clamp(
                    calcShootingAngle(m_predictedBallV0), ShooterConsts.MIN_ANGLE, ShooterConsts.MAX_ANGLE);
                SwerveAngleController.getInstance().start(calcRobotShootingOffsetAngle(m_staticShootingPose));
                m_anglePID.activate(m_targetAngle, ControlType.kPos);
                m_frontShootingController.activate(ShooterConsts.TARGET_RPS, ControlType.kVel);
                m_backShootingController.activate(ShooterConsts.TARGET_RPS * ShooterConsts.WHEELS_RATIO, ControlType.kVel);
                    break;
            case kStop:
                if (m_previousShooterState != ShooterState.kStop) {
                    m_frontShootingController.stop();
                    m_backShootingController.stop();
                    m_anglePID.stop();
                    m_frontMotor.stop();
                    m_backMotor.stop();
                    SwerveAngleController.getInstance().stop(); 
                }
                break;
            case kTest:
                frontShootingRpm(DELIVERY_RPS);
                backShootingRpm(DELIVERY_RPS * WHEELS_RATIO);
        }

        m_previousShooterState = m_shooterState;

        if (ShooterConsts.DEBUG_MODE) {
            log();
        }

    }

    public void log() {
        SmartDashboard.putNumber("Shooter Current Angle", m_angleEncoder.getPos());
        SmartDashboard.putNumber("Ball speed", m_predictedBallV0);

        SmartDashboard.putNumber("Front Wheel Shooting Speed", m_frontShootingEncoder.getVel());
        SmartDashboard.putNumber("Back Wheel Shooting Speed", m_backShootingEncoder.getVel());

        SmartDashboard.putNumber("Spar internal encoder angle", ShooterConsts.ANGLE_MOTOR.getControllerInstance().getEncoder().getPosition());

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
