package frc.robot.Subsystems;

import frc.robot.Subsystems.Swerve.SwerveModule;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverAnalogToDigitalLimitSwitch;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverDutyCycleEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalAbsEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkFlex;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverExternalMotorPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;
import frc.robot.Utils.Math.Vector2d;

public interface Consts {

        public interface SwerveConsts {

                // TODO: change all place holder to real values

                public static final boolean DEBUG_MODE = false;
                // speed values
                public static final double MAX_NORMAL_DRIVE_SPEED = 3.5; // m/s
                public static final double MAX_TURBO_DRIVE_SPEED = 5;
                public static final double MAX_SLOW_DRIVE_SPEED = 2;
                public static final double MAX_ANGULAR_SPEED = 180; // deg/s/
                public static final double MIN_SPEED = 0.4;

                public static final double GYRO_DIRECTION = -1; // decide the direction of the gyro(counter clock wise
                                                                // should be
                                                                // positive)

                public static final SwerveModule[] MODULES = new SwerveModule[4];

                // motor controllers
                public static final EverTalonFX 
                                TL_DRIVE_MOTOR = new EverTalonFX(3), // 7
                                TR_DRIVE_MOTOR = new EverTalonFX(1), // 5
                                DL_DRIVE_MOTOR = new EverTalonFX(2), // 1
                                DR_DRIVE_MOTOR = new EverTalonFX(0);// 3 TL - TR - DL -DR
                public static final EverSparkMax 
                                TL_STEER_MOTOR = new EverSparkMax(11), // 8
                                TR_STEER_MOTOR = new EverSparkMax(10), // 6
                                DL_STEER_MOTOR = new EverSparkMax(12), // 2
                                DR_STEER_MOTOR = new EverSparkMax(13);// 4

                public static final EverTalonFX[] DRIVE_MOTORS = { TL_DRIVE_MOTOR, TR_DRIVE_MOTOR, DL_DRIVE_MOTOR,
                                DR_DRIVE_MOTOR };
                public static final EverSparkMax[] STEER_MOTORS = { TL_STEER_MOTOR, TR_STEER_MOTOR, DL_STEER_MOTOR,
                                DR_STEER_MOTOR };

                // encoders
                public static final EverTalonFXInternalEncoder 
                                TL_DRIVE_ENCODER = new EverTalonFXInternalEncoder(TL_DRIVE_MOTOR),
                                TR_DRIVE_ENCODER = new EverTalonFXInternalEncoder(TR_DRIVE_MOTOR),
                                DL_DRIVE_ENCODER = new EverTalonFXInternalEncoder(DL_DRIVE_MOTOR),
                                DR_DRIVE_ENCODER = new EverTalonFXInternalEncoder(DR_DRIVE_MOTOR);

                public static final EverSparkInternalEncoder 
                                TL_STEER_ENCODER = new EverSparkInternalEncoder(TL_STEER_MOTOR),
                                TR_STEER_ENCODER = new EverSparkInternalEncoder(TR_STEER_MOTOR),
                                DL_STEER_ENCODER = new EverSparkInternalEncoder(DL_STEER_MOTOR),
                                DR_STEER_ENCODER = new EverSparkInternalEncoder(DR_STEER_MOTOR);

                public static final EverTalonFXInternalEncoder[] DRIVE_ENCODERS = { TL_DRIVE_ENCODER, TR_DRIVE_ENCODER,
                                DL_DRIVE_ENCODER, DR_DRIVE_ENCODER };
                public static final EverSparkInternalEncoder[] STEER_ENCODERS = { TL_STEER_ENCODER, TR_STEER_ENCODER,
                                DL_STEER_ENCODER, DR_STEER_ENCODER };

                // swerve module pid controllers
                public static final EverTalonFXPIDController TL_VELOCITY_CONTROLLER = new EverTalonFXPIDController(
                                TL_DRIVE_MOTOR),
                                TR_VELOCITY_CONTROLLER = new EverTalonFXPIDController(TR_DRIVE_MOTOR),
                                DL_VELOCITY_CONTROLLER = new EverTalonFXPIDController(DL_DRIVE_MOTOR),
                                DR_VELOCITY_CONTROLLER = new EverTalonFXPIDController(DR_DRIVE_MOTOR);

                public static final EverSparkMaxPIDController TL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(
                                TL_STEER_MOTOR),
                                TR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(TR_STEER_MOTOR),
                                DL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DL_STEER_MOTOR),
                                DR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DR_STEER_MOTOR);

                public static final EverTalonFXPIDController[] WHEEL_VELOCITY_CONTROLLERS = { TL_VELOCITY_CONTROLLER,
                                TR_VELOCITY_CONTROLLER, DL_VELOCITY_CONTROLLER, DR_VELOCITY_CONTROLLER };
                public static final EverSparkMaxPIDController[] WHEEL_ANGLE_CONTROLLERS = { TL_ANGLE_CONTROLLER,
                                TR_ANGLE_CONTROLLER, DL_ANGLE_CONTROLLER, DR_ANGLE_CONTROLLER };

                // chassis encoders
                public static final EverDutyCycleEncoder 
                                TL_ABS_ENCODER = new EverDutyCycleEncoder(3),
                                TR_ABS_ENCODER = new EverDutyCycleEncoder(1),
                                DL_ABS_ENCODER = new EverDutyCycleEncoder(4),
                                DR_ABS_ENCODER = new EverDutyCycleEncoder(2);

                public static final EverAbsEncoder[] ABS_ENCODERS = { TL_ABS_ENCODER, TR_ABS_ENCODER, DL_ABS_ENCODER,
                                DR_ABS_ENCODER };

                // swerve module velocity pidf values
                public static final double WHEEL_VELOCITY_KP = 0.1, WHEEL_VELOCITY_KI = 0.0, WHEEL_VELOCITY_KD = 0.00,
                                WHEEL_VELOCITY_KV = 1 / 8.5, WHEEL_VELOCITY_KS = 0;
                // swerve module wheel angle pid values
                public static final double WHEEL_ANGLE_KP = 0.01, WHEEL_ANGLE_KI = 0.0, WHEEL_ANGLE_KD = 0.000;

                // swerve dimensions
                public static final double CHASSIS_WIDTH = 0.67, CHASSIS_LENGTH = 0.67;
                public static final double BUMPERS_THICKNESS = 0.06;

                public static final double ROBOT_BOUNDING_CIRCLE_PERIMETER = Math.PI * Math.sqrt(
                                CHASSIS_WIDTH * CHASSIS_WIDTH + CHASSIS_LENGTH * CHASSIS_LENGTH);
                public static final double ROBOT_RADIUS = 0.5 * Math.sqrt(
                                CHASSIS_WIDTH * CHASSIS_WIDTH + CHASSIS_LENGTH * CHASSIS_LENGTH);
                public static final double WHEEL_PERIMETER = Math.PI * 0.09;

                // module gear ratios
                public static final double DRIVE_GEAR_RATIO = 1 / 6.75, STEER_GEAR_RATIO =  7.0 / 150.0;

                public static final double GYRO_OFFSET = 90;

                // swerve vectors
                public static final Vector2d TR = new Vector2d((CHASSIS_WIDTH / 2),
                                (CHASSIS_LENGTH / 2)).rotate(Math.toRadians(90)),
                                TL = new Vector2d(-(CHASSIS_WIDTH / 2),
                                                (CHASSIS_LENGTH / 2)).rotate(Math.toRadians(270)),
                                DR = new Vector2d(CHASSIS_WIDTH / 2,
                                                -(CHASSIS_LENGTH / 2)).rotate(Math.toRadians(270)),
                                DL = new Vector2d(-(CHASSIS_WIDTH / 2),
                                                -(CHASSIS_LENGTH / 2)).rotate(Math.toRadians(90));

                // array of physical module vectors
                public static final Vector2d[] modulesPositions = { 
                                TL,
                                TR,
                                DL,
                                DR
                };// array of vectors from robot center to swerves module

                public static void config() {

                        for (EverMotorController driveMotor : DRIVE_MOTORS) {
                                driveMotor.restoreFactoryDefaults();
                                driveMotor.setInverted(false);
                                driveMotor.setIdleMode(IdleMode.kCoast);
                        }
                        TL_DRIVE_MOTOR.setInverted(true);

                        for (EverMotorController steerMotor : STEER_MOTORS) {
                                steerMotor.restoreFactoryDefaults();
                                steerMotor.setIdleMode(IdleMode.kCoast);
                        }

                        for (EverEncoder driveEncoder : DRIVE_ENCODERS) {
                                driveEncoder.setVelConversionFactor(DRIVE_GEAR_RATIO * WHEEL_PERIMETER);
                                driveEncoder.setPosConversionFactor(DRIVE_GEAR_RATIO * WHEEL_PERIMETER);
                        }

                        for (EverEncoder steerEncoder : STEER_ENCODERS) {
                                steerEncoder.setPosConversionFactor(SwerveConsts.STEER_GEAR_RATIO * 360.0); // rotations
                                                                                                            // to
                                                                                                            // degrees
                        }

                        for (EverAbsEncoder absEncoder : ABS_ENCODERS) {
                                absEncoder.setPosConversionFactor(360.0);
                        }

                        ABS_ENCODERS[0].setOffset(69.9);
                        ABS_ENCODERS[1].setOffset(166.48);
                        ABS_ENCODERS[2].setOffset(52.311);
                        ABS_ENCODERS[3].setOffset(103.16);
                        for (EverTalonFXPIDController velocityController : WHEEL_VELOCITY_CONTROLLERS) {
                                Slot0Configs configs = new Slot0Configs();
                                configs.kP = WHEEL_VELOCITY_KP;
                                configs.kI = WHEEL_VELOCITY_KI;
                                configs.kD = WHEEL_VELOCITY_KD;
                                configs.kS = WHEEL_VELOCITY_KS;
                                configs.kV = WHEEL_VELOCITY_KV;
                                velocityController.setPID(configs);
                        }

                        for (EverSparkMaxPIDController angleController : WHEEL_ANGLE_CONTROLLERS) {
                                angleController.setPID(WHEEL_ANGLE_KP, WHEEL_ANGLE_KI, WHEEL_ANGLE_KD);
                        }

                        MODULES[0] = new SwerveModule(SwerveConsts.TL_VELOCITY_CONTROLLER, SwerveConsts.TL_DRIVE_MOTOR,
                                        SwerveConsts.TL_DRIVE_ENCODER, SwerveConsts.TL_ANGLE_CONTROLLER,
                                        SwerveConsts.TL_STEER_MOTOR,
                                        SwerveConsts.TL_STEER_ENCODER, SwerveConsts.ABS_ENCODERS[0]);
                        MODULES[1] = new SwerveModule(SwerveConsts.TR_VELOCITY_CONTROLLER, SwerveConsts.TR_DRIVE_MOTOR,
                                        SwerveConsts.TR_DRIVE_ENCODER, SwerveConsts.TR_ANGLE_CONTROLLER,
                                        SwerveConsts.TR_STEER_MOTOR,
                                        SwerveConsts.TR_STEER_ENCODER, SwerveConsts.ABS_ENCODERS[1]);
                        MODULES[2] = new SwerveModule(SwerveConsts.DL_VELOCITY_CONTROLLER, SwerveConsts.DL_DRIVE_MOTOR,
                                        SwerveConsts.DL_DRIVE_ENCODER, SwerveConsts.DL_ANGLE_CONTROLLER,
                                        SwerveConsts.DL_STEER_MOTOR,
                                        SwerveConsts.DL_STEER_ENCODER, SwerveConsts.ABS_ENCODERS[2]);
                        MODULES[3] = new SwerveModule(SwerveConsts.DR_VELOCITY_CONTROLLER, SwerveConsts.DR_DRIVE_MOTOR,
                                        SwerveConsts.DR_DRIVE_ENCODER, SwerveConsts.DR_ANGLE_CONTROLLER,
                                        SwerveConsts.DR_STEER_MOTOR,
                                        SwerveConsts.DR_STEER_ENCODER, SwerveConsts.ABS_ENCODERS[3]);
                }
        }

        public interface ShooterConsts {

                // TODO: change all place holder to real values

                public static final EverTalonFX FRONT_MOTOR = new EverTalonFX(5), BACK_MOTOR = new EverTalonFX(19);
                public static final EverSparkMax ANGLE_MOTOR = new EverSparkMax(3);

                public static final EverSparkInternalEncoder ANGLE_ENCODER = new EverSparkInternalEncoder(ANGLE_MOTOR);

                public static final EverTalonFXInternalEncoder FRONT_SHOOTING_ENCODER = new EverTalonFXInternalEncoder(FRONT_MOTOR);
                public static final EverTalonFXInternalEncoder BACK_SHOOTING_ENCODER = new EverTalonFXInternalEncoder(BACK_MOTOR);

                public static final double SHOOTER_ANGLE_GEAR_RATIO = 1/23.51351, SHOOTING_GEAR_RATIO = 1;

                public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.620, 4.03, new Rotation2d());
                public static final Pose2d RED_HUB_POSE = new Pose2d(11.920, 4.03, new Rotation2d(Math.toDegrees(Math.PI)));

                public static final Pose2d DELIVERY_POSES_BLUE = new Pose2d(1.723, 6.5, new Rotation2d(Math.toDegrees(Math.PI)));

                public static final Pose2d DELIVERY_POSES_RED = new Pose2d(15, 6.5, new Rotation2d());

                public static final double FRONT_WHEEL_RADIUS = 0.05 , BACK_WHEEL_RADIUS = 0.028, WHEELS_RATIO = FRONT_WHEEL_RADIUS / BACK_WHEEL_RADIUS,  // (in m)

                                GRAVITY = 9.81, // (in m/s^2)

                                HUB_HEIGHT = 1.8,
                                MECHANISM_HEIGHT = 0.351,// (in meters) // not exact

                                SHOOTING_HEIGHT = HUB_HEIGHT - MECHANISM_HEIGHT, // (in meters)

                                MAX_ANGLE = 22.0, // (in degrees)
                                MIN_ANGLE = 0.5, //  (in degrees)

                                TARGET_RPS = 33.3, // (in RPS)
                                DELIVERY_RPS = 30, // (in RPS)
                                
                                MIN_SHOOTING_DIST = 2.67, //(in meters)
                                SHOOTER_OFFSET_FROM_GROUND = 12.8; // (in degrees)

                public static final double 
                                FRONT_SPEED_KP = 0.3,//0.3
                                FRONT_SPEED_KI = 0.00009, 
                                FRONT_SPEED_KD = 0.001,
                                FRONT_SPEED_KV = 4.04 / (30.857421875), 

                                BACK_SPEED_KP = 1.7 * (1/WHEELS_RATIO), 
                                BACK_SPEED_KI = 0.00009, 
                                BACK_SPEED_KD = 0.0097 ,
                                BACK_SPEED_KV = (4.04 / 30.857421875) * (1/WHEELS_RATIO), 

                                ANGLE_KP = 0.68, 
                                ANGLE_KI = 0, 
                                ANGLE_KD = 0; 

                public static final EverSparkMaxPIDController ANGLE_PID_CONTROLLER = new EverSparkMaxPIDController(ANGLE_MOTOR);
                public static final EverTalonFXPIDController FRONT_SHOOTING_PID_CONTROLLER_ = new EverTalonFXPIDController(FRONT_MOTOR);
                public static final EverTalonFXPIDController BACK_SHOOTING_PID_CONTROLLER_ = new EverTalonFXPIDController(BACK_MOTOR);

                public static final boolean DEBUG_MODE = true;
                public static final double DELIVERY_ANGLE = 10; // (in degrees)

                public static final double[] BALL_V0_DATA = { 8.309, 12.62, 0, 0, 0 }; //place holder// initial velocity of the ball(in m/s)

                
                public static final double[] SHOOTER_SPEED = {3.830 ,4.787 ,5.426 ,6.224 ,7.022 ,7.979,8.618 ,9.256,9.735,10.533,10.638}; // Shooter speed (in m/s)
  
                public static final InterpolatingDoubleTreeMap SHOOTER_TO_BALL_SPEED_TABLE = new InterpolatingDoubleTreeMap();

                public static final SparkMaxConfig ANGLE_MOTOR_MOTION_CONFIG = new SparkMaxConfig();
                public static final SparkBaseConfig ANGLE_BASE_MOTOR_MOTION_CONFIG = new SparkMaxConfig();

                public static void config() {

                        ANGLE_ENCODER.setPos(0);
                        ANGLE_ENCODER.setPosConversionFactor(SHOOTER_ANGLE_GEAR_RATIO * 360);

                        ANGLE_MOTOR_MOTION_CONFIG.closedLoop.maxMotion.maxAcceleration(2);
                        ANGLE_MOTOR_MOTION_CONFIG.closedLoop.maxMotion.cruiseVelocity(2);
                        ANGLE_MOTOR_MOTION_CONFIG.closedLoop.maxMotion.allowedProfileError(0.001);

                        ANGLE_MOTOR.getControllerInstance().configure(ANGLE_MOTOR_MOTION_CONFIG, ResetMode.kNoResetSafeParameters, null);
                        FRONT_MOTOR.setInverted(true);
                        BACK_MOTOR.setInverted(true);
                        ANGLE_MOTOR.setInverted(false);
                        ANGLE_MOTOR.setIdleMode(IdleMode.kBrake);

                        ANGLE_PID_CONTROLLER.setPID(ANGLE_KP, ANGLE_KI, ANGLE_KD);
                        FRONT_SHOOTING_PID_CONTROLLER_.setPID(new Slot0Configs().withKP(FRONT_SPEED_KP).withKI(FRONT_SPEED_KI).withKD(FRONT_SPEED_KD).withKV(FRONT_SPEED_KV));
                        BACK_SHOOTING_PID_CONTROLLER_.setPID(new Slot0Configs().withKP(BACK_SPEED_KP).withKI(BACK_SPEED_KI).withKD(BACK_SPEED_KD).withKV(BACK_SPEED_KV));

                        for (int i = 0; i < BALL_V0_DATA.length; i++) {
                                SHOOTER_TO_BALL_SPEED_TABLE.put(SHOOTER_SPEED[i], BALL_V0_DATA[i]);
                        }
                }

                public static Pose2d STATIC_SHOOT_POSE_BLUE = new Pose2d(), STATIC_SHOOT_POSE_RED = new Pose2d();
        }

        public interface IntakeConsts {

                // TODO: change all place holder to real values

                public static final EverTalonFX PICKUP_MOTOR = new EverTalonFX(14);

                public static final EverTalonFX EXTENSION_MOTOR = new EverTalonFX(15);
                public static final EverTalonFXInternalEncoder EXTENSION_ENCODER = new EverTalonFXInternalEncoder(
                                EXTENSION_MOTOR);

                // public static final EverAnalogToDigitalLimitSwitch RETRACTION_LM = new EverAnalogToDigitalLimitSwitch(0),
                //                 EXTENSION_LM = new EverAnalogToDigitalLimitSwitch(3);

                public final double EXTENSION_SPEED = 0.25, PICKUP_SPEED = 0.6;
                public final boolean DEBUG_MODE = false;
                public final double MAX_EXTENDING_ROTATIONS = 0;

                public TalonFXConfiguration MOTOR_CONFIGS = new TalonFXConfiguration();

                public static void config(){
                        MOTOR_CONFIGS.MotionMagic.MotionMagicAcceleration = 100.0;
                        MOTOR_CONFIGS.MotionMagic.MotionMagicJerk = 50.0;
                        PICKUP_MOTOR.getControllerInstance().getConfigurator().apply(MOTOR_CONFIGS);
                }

        }

        public interface FeedAndConveyConsts {

                public final static boolean DEBUG_MOD = false;

                public final static double CONVEYING_TO_FEEDER_SPEED = -0.6, INTAKE_CONVEY_SPEED = -0.4; // motor power, place holder

                public final static EverTalonFX CONVEY_MOTOR = new EverTalonFX(17), FEEDING_MOTOR =
                 new EverTalonFX(16);

                public final static double FEEDER_MAX_STALL_TIME = 6, FEEDING_SPEED = 0.8; // motor power
                                                                                              

                public static final EverAnalogToDigitalLimitSwitch ENTER_LEFT_LM = new EverAnalogToDigitalLimitSwitch(0),
                                                 ENTER_RIGHT_LM = new EverAnalogToDigitalLimitSwitch(1);

                public static final double FEEDING_TIME = 0.15; // (in seconds)

        }

        public interface ClimbConst {

                // TODO: change all place holder to real values

                public static final double MAX_OPEN = 3000, OPEN_VEL = -0.8;

                public static final EverSparkMax CLIMB_MOTOR = new EverSparkMax(6);

                public static final EverSparkInternalEncoder CLIMB_ENCODER = new EverSparkInternalEncoder(CLIMB_MOTOR);

                public static final DigitalInput BOTTOM_LM = new DigitalInput(6);

                public static final boolean DEBUG_MODE = false;

                public static final Pose2d BLUE_CLIMB_POSE_TOP = new Pose2d(1.065, 4.857, new Rotation2d()),
                                BLUE_CLIMB_POSE_BOTTOM = new Pose2d(1.065, 3.679, new Rotation2d());
                public static final Pose2d RED_CLIMB_POSE_TOP = new Pose2d(15.49, 4.857, new Rotation2d()),
                                RED_CLIMB_POSE_BOTTOM = new Pose2d(15.49, 3.679, new Rotation2d());

        }

        public interface AutoConsts {

                public static final double LEFT_MIN_TRENCH_X = 3.793,
                                LEFT_MAX_TRENCH_X = 5.372,
                                RIGHT_MIN_TRENCH_X = 11.285,
                                RIGHT_MAX_TRENCH_X = 12.604,

                                RED_ALLIANCE_ZONE_X = 12.603,

                                BLUE_ALLIANCE_ZONE_X = 3.792;

        }
}