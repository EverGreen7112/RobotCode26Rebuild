package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems.Consts;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
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

public class Shooter extends SubsystemBase implements Consts.ShooterConsts {

    public enum ShooterState {
        kStop,
        kScoring,
        kDelivery,
        kTrench
    }

    private static Shooter m_instance = new Shooter();

    private double m_targetAngle, m_targetSpeed, m_predictedBallV0;

    private EverMotorControllerGroup m_shootingMotors;

    private EverMotorController m_angleMotor;

    private EverPIDController m_anglePID, m_shootingPID;

    private EverEncoder m_angleEncoder, m_shootingEncoder;

    private Pose2d m_targetHub;

    private ShooterState m_shooterState, m_previousShooterState;

    private InterpolatingDoubleTreeMap m_shooterSpeedToPredictedBallV0;

    private DeltaTime m_deltaTime;

    private Shooter() {
        ShooterConsts.config();

        m_shootingMotors = new EverMotorControllerGroup(ShooterConsts.LEFT_MOTOR, ShooterConsts.RIGHT_MOTOR);
        m_angleMotor = ShooterConsts.ANGLE_MOTOR;
        m_angleEncoder = ShooterConsts.ANGLE_CAN_CODER;
        m_shootingEncoder = ShooterConsts.SHOOTING_ENCODER;

        m_targetSpeed = Funcs.convertRPMtoMS(ShooterConsts.WHEEL_RADIUS, ShooterConsts.TARGET_RPM);

        m_deltaTime = new DeltaTime();

        m_anglePID = ShooterConsts.ANGLE_PID_CONTROLLER;
        m_shootingPID = ShooterConsts.SHOOTING_PID_CONTROLLER;

        m_shooterState = ShooterState.kStop;
        m_previousShooterState = ShooterState.kStop;

        m_shooterSpeedToPredictedBallV0 = ShooterConsts.SHOOTER_TO_BALL_SPEED_TABLE;

        m_targetHub = ShooterConsts.BLUE_HUB_POSE; // default to blue hub, will be changed in teleopInit based on the
                                                   // alliance color
    }

    public static Shooter getInstance() {
        return m_instance;
    }

    public void setAllianceHub(boolean isBlue) {
        m_targetHub = isBlue ? ShooterConsts.BLUE_HUB_POSE : ShooterConsts.RED_HUB_POSE;
    }

    public void setShooterState(ShooterState shooterState) {
        m_shooterState = shooterState;
    }

    private double getShootingDistance() {
        double locX = SwerveLocalizer.getInstance().getCurrentPoint().getX();
        double locY = SwerveLocalizer.getInstance().getCurrentPoint().getY();
        return Math.sqrt(Math.pow(m_targetHub.getX() - locX, 2) + Math.pow(m_targetHub.getY() - locY, 2));
    }

    private double getHubAngle() {
        double locX = SwerveLocalizer.getInstance().getCurrentPoint().getX();
        double locY = SwerveLocalizer.getInstance().getCurrentPoint().getY();
        return Math.toDegrees(Math.atan2((m_targetHub.getY() - locY), (m_targetHub.getX() - locX)));
    }

    /**
     * calculates the speed at which we predict the shooter will be at when the ball
     * is fed into it
     * 
     * @return the predicted shooter speed in m/s
     */
    private double calcPredictedShooterSpeed() {
        double currentSpeed = m_shootingEncoder.getVel();
        double deltaSpeed = m_targetSpeed - currentSpeed;
        return currentSpeed + (deltaSpeed / m_deltaTime.get()) * 0.1; // TODO: add a const to store the feeding time
    }

    // TODO: need to consider which angle we take. the function return two angles +
    // and -
    /**
     * calculates the minimum shooting angle needed to shoot the ball to the hub
     * based on the current distance and hight of the shooter
     * the formula is based on the physics of projectile motion and it is derived
     * from the formula
     * 
     * @param shootingSpeed the speed at which the ball will be shot in m/s
     * @return the minimum shooting angle in degrees
     */
    private double calcShootingAngle(double shootingSpeed) {
        double verticalVelocity = Math.pow(shootingSpeed, 2) -
                Math.sqrt(Math.pow(shootingSpeed, 4)
                        - ShooterConsts.GRAVITY * (ShooterConsts.GRAVITY * Math.pow(getShootingDistance(), 2)
                                + 2 * ShooterConsts.SHOOTING_HEIGHT * Math.pow(shootingSpeed, 2)));

        double horizontalForce = ShooterConsts.GRAVITY * getShootingDistance();
        return Math.toDegrees(Math.atan(verticalVelocity / horizontalForce));
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
    // move this to a correct place
    public double calcRobotShootingOffsetAngle() {
        Pose2d pos = SwerveLocalizer.getInstance().getCurrentPoint();
        double x = m_targetHub.getX() - pos.getX();
        double y = m_targetHub.getY() - pos.getY();

        double robotsOffsetAngleFromHub = Math.toDegrees(Math.atan2(y, x));

        Vector2d robotVelocity = Swerve.getInstance().getRobotOrientedVelocity();

        return robotsOffsetAngleFromHub
                + Math.toDegrees(Math.atan2(robotVelocity.y + m_predictedBallV0, robotVelocity.x));
    }

    @Override
    public void periodic() {

        if ((m_angleEncoder.getPos() >= ShooterConsts.MAX_ANGLE && m_angleMotor.get() > 0)) {
            m_angleMotor.stop();
        } else if (m_angleEncoder.getPos() <= ShooterConsts.MIN_ANGLE && m_angleMotor.get() < 0) {
            m_angleMotor.stop();
        }

        switch (m_shooterState) {
            case kScoring:
                m_predictedBallV0 = m_shooterSpeedToPredictedBallV0.get(
                        MathUtil.clamp(calcPredictedShooterSpeed(), ShooterConsts.SHOOTER_SPEED[0], m_targetSpeed));
                m_targetAngle = MathUtil.clamp(calcShootingAngle(m_predictedBallV0), ShooterConsts.MIN_ANGLE,
                        ShooterConsts.MAX_ANGLE);
                m_shootingPID.activate(ShooterConsts.TARGET_RPM, ControlType.kVel);
                m_anglePID.activate(m_targetAngle, ControlType.kPos);
                SwerveAngleController.getInstance().start(getHubAngle());
                break;

            case kDelivery:
                m_shootingPID.activate(ShooterConsts.DELIVERY_RPM, ControlType.kVel);
                m_anglePID.activate(ShooterConsts.DELIVERY_ANGLE, ControlType.kPos);
                SwerveAngleController.getInstance().stop();
                break;

            case kStop:
                if (m_previousShooterState != ShooterState.kStop) {
                    m_shootingPID.stop();
                    m_anglePID.stop();
                    m_shootingMotors.stop();
                }
                SwerveAngleController.getInstance().stop();
                break;

            case kTrench:
                m_shootingPID.stop();
                m_anglePID.activate(ShooterConsts.MIN_ANGLE, ControlType.kPos);
                SwerveAngleController.getInstance().stop();
                break;
        }

        m_previousShooterState = m_shooterState;

        if (ShooterConsts.DEBUG_MODE) {
            log();
        }
    }

    public void log() {
        SmartDashboard.putNumber("Shooter Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Shooter Current Angle", m_angleEncoder.getPos());
        SmartDashboard.putNumber("Ball speed", m_predictedBallV0);
        SmartDashboard.putNumber("Shooting Speed", m_shootingEncoder.getVel());
    }

}
