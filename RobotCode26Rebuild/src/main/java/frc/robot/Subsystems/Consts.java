package frc.robot.Subsystems;

import frc.robot.Subsystems.Swerve.SwerveModule;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalAbsEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkFlex;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;
import frc.robot.Utils.Math.Vector2d;

public interface Consts {

        public interface SwerveConsts {

                // TODO: change all place holder to real values

                public static final boolean DEBUG_MODE = false;
                // speed values
                public static final double MAX_NORMAL_DRIVE_SPEED = 6; // m/s
                public static final double MAX_TURBO_DRIVE_SPEED = 8;
                public static final double MAX_SLOW_DRIVE_SPEED = 0.45;
                public static final double MAX_ANGULAR_SPEED = 180; // deg/s/
                public static final double MIN_SPEED = 0.0;

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
                public static final EverSparkInternalAbsEncoder TL_ABS_ENCODER = new EverSparkInternalAbsEncoder(STEER_MOTORS[0]),
                                TR_ABS_ENCODER = new EverSparkInternalAbsEncoder(STEER_MOTORS[1]),
                                DL_ABS_ENCODER = new EverSparkInternalAbsEncoder(STEER_MOTORS[2]),
                                DR_ABS_ENCODER = new EverSparkInternalAbsEncoder(STEER_MOTORS[3]);

                public static final EverAbsEncoder[] ABS_ENCODERS = { TL_ABS_ENCODER, TR_ABS_ENCODER, DL_ABS_ENCODER,
                                DR_ABS_ENCODER };

                // swerve module velocity pidf values
                public static final double WHEEL_VELOCITY_KP = 0.05, WHEEL_VELOCITY_KI = 0.0, WHEEL_VELOCITY_KD = 0.00,
                                WHEEL_VELOCITY_KV = 0.75 / 2.81, WHEEL_VELOCITY_KS = 0;
                // swerve module wheel angle pid values
                public static final double WHEEL_ANGLE_KP = 0.01, WHEEL_ANGLE_KI = 0.0, WHEEL_ANGLE_KD = 0.000;

                // swerve dimensions
                public static final double CHASSIS_WIDTH = 0.75, CHASSIS_LENGTH = 0.75;
                public static final double BUMPERS_THICKNESS = 0.06;

                public static final double ROBOT_BOUNDING_CIRCLE_PERIMETER = Math.PI * Math.sqrt(
                                CHASSIS_WIDTH * CHASSIS_WIDTH + CHASSIS_LENGTH * CHASSIS_LENGTH);
                public static final double ROBOT_RADIUS = 0.5 * Math.sqrt(
                                CHASSIS_WIDTH * CHASSIS_WIDTH + CHASSIS_LENGTH * CHASSIS_LENGTH);
                public static final double WHEEL_PERIMETER = Math.PI * 0.09;

                // module gear ratios
                public static final double DRIVE_GEAR_RATIO = 1 / 6.75, STEER_GEAR_RATIO = 1 / 12.8;

                // swerve vectors
                public static final Vector2d TR = new Vector2d((CHASSIS_WIDTH / 2),
                                (CHASSIS_LENGTH / 2)),
                                TL = new Vector2d(-(CHASSIS_WIDTH / 2),
                                                (CHASSIS_LENGTH / 2)),
                                DR = new Vector2d(CHASSIS_WIDTH / 2,
                                                -(CHASSIS_LENGTH / 2)),
                                DL = new Vector2d(-(CHASSIS_WIDTH / 2),
                                                -(CHASSIS_LENGTH / 2));

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

                        for (EverMotorController steerMotor : STEER_MOTORS) {
                                steerMotor.restoreFactoryDefaults();
                                steerMotor.setIdleMode(IdleMode.kCoast);
                        }

                        for (EverEncoder driveEncoder : DRIVE_ENCODERS) {
                                driveEncoder.setVelConversionFactor(DRIVE_GEAR_RATIO * WHEEL_PERIMETER * (1.0 / 60.0));
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

                        ABS_ENCODERS[0].setOffset(81.474609375);// 80.507); //80.5078125
                        ABS_ENCODERS[1].setOffset(138.1640625);// 137.37); //-42.275394439697266
                        ABS_ENCODERS[2].setOffset(-150);// -149.150); //-149.50196838378906
                        ABS_ENCODERS[3].setOffset(-31.11328125);// -27.509); //-34.8046875

                        for (EverTalonFXPIDController velocityController : WHEEL_VELOCITY_CONTROLLERS) {
                                Slot0Configs configs = new Slot0Configs();
                                configs.kP = WHEEL_VELOCITY_KP;
                                configs.kI = WHEEL_VELOCITY_KI;
                                configs.kD = WHEEL_VELOCITY_KD;
                                configs.kS = WHEEL_VELOCITY_KS;
                                configs.kV = WHEEL_VELOCITY_KV;
                                velocityController.setPIDF(WHEEL_ANGLE_KP, WHEEL_ANGLE_KI, WHEEL_ANGLE_KD,
                                                WHEEL_VELOCITY_KV);
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

                public static final EverTalonFX LEFT_MOTOR = new EverTalonFX(5), RIGHT_MOTOR = new EverTalonFX(19);
                public static final EverSparkMax ANGLE_MOTOR = new EverSparkMax(3);

                public static final EverTalonFXInternalEncoder SHOOTING_ENCODER = new EverTalonFXInternalEncoder(LEFT_MOTOR);

                public static final double SHOOTER_ANGLE_GEAR_RATIO = 1/21.093, SHOOTING_GEAR_RATIO = 1;

                public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.620, 4.03, new Rotation2d());
                public static final Pose2d RED_HUB_POSE = new Pose2d(11.920, 4.03, new Rotation2d());

                public static final double WHEEL_RADIUS = 0.05, // (in m)

                                GRAVITY = 9.81, // (in m/s^2)

                                HUB_HEIGHT = 1.8,
                                MECHANISM_HEIGHT = 0.346,// (iN meters) //can be a bit off

                                SHOOTING_HEIGHT = HUB_HEIGHT - MECHANISM_HEIGHT,

                                MAX_ANGLE = 60, // change this for the real max angle of the shooter (in degrees)
                                MIN_ANGLE = 0, // change this for the real min angle of the shooter (in degrees)

                                TARGET_RPM = 2024, // change this for the real target shooting rpm
                                DELIVERY_RPM = 1000; // change this for the real delivery rpm

                public static final EverSparkInternalAbsEncoder ANGLE_ABS_ENCODER = new EverSparkInternalAbsEncoder(ANGLE_MOTOR);

                public static final double SPEED_KP = 0, // change this for the real KP of the shooter
                                SPEED_KI = 0, // change this for the real KI of the shooter
                                SPEED_KD = 0, // change this for the real KD of the shooter

                                ANGLE_KP = 0, // change this for the real KP of the shooter
                                ANGLE_KI = 0, // change this for the real KI of the shooter
                                ANGLE_KD = 0; // change this for the real KD of the shooter

                public static final EverSparkMaxPIDController ANGLE_PID_CONTROLLER = new EverSparkMaxPIDController(
                                ANGLE_MOTOR);
                public static final EverTalonFXPIDController SHOOTING_PID_CONTROLLER = new EverTalonFXPIDController(
                                LEFT_MOTOR);

                public static final boolean DEBUG_MODE = true;
                public static final double DELIVERY_ANGLE = 45;

                // TODO: make the cases for different shooting speeds and their corresponding
                // initial velocity of the ball
                // we need more cases in the 70 - 100% range of the taget speeding motor power
                // because that's where we will be doing most of the shooting, and we need to be
                // more precise in that range
                // low index values correspond to lower shooting speeds and low predicted ball
                // v0, high index values correspond to higher shooting speeds and high predicted
                // ball v0
                public static final double[] BALL_V0_DATA = { 0, 0, 0, 0, 0 }; // change this for the real data of the
                                                                               // initial
                                                                               // velocity of the ball(in m/s)
                public static final double[] SHOOTER_SPEED = { 0, 0, 0, 0, 0 }; // change this for the real data of the
                                                                                // Shooter
                                                                                // speed (in m/s)

                public static final InterpolatingDoubleTreeMap SHOOTER_TO_BALL_SPEED_TABLE = new InterpolatingDoubleTreeMap();

                public static void config() {

                        ANGLE_ABS_ENCODER.setPosConversionFactor(SHOOTER_ANGLE_GEAR_RATIO * 360);
                        ANGLE_ABS_ENCODER.setOffset(0);

                        SHOOTING_ENCODER.setVelConversionFactor((2 * Math.PI * WHEEL_RADIUS) / (SHOOTING_GEAR_RATIO * 60)); // convert to m/s

                        ANGLE_PID_CONTROLLER.setPID(ANGLE_KP, ANGLE_KI, ANGLE_KD);
                        SHOOTING_PID_CONTROLLER.setPID(SPEED_KP, SPEED_KI, SPEED_KD);

                        for (int i = 0; i < BALL_V0_DATA.length; i++) {
                                SHOOTER_TO_BALL_SPEED_TABLE.put(SHOOTER_SPEED[i], BALL_V0_DATA[i]);
                        }
                }

                public static Pose2d STATIC_SHOOT_POSE_BLUE = new Pose2d(), STATIC_SHOOT_POSE_RED = new Pose2d();
        }

        public interface IntakeConsts {

                // TODO: change all place holder to real values

                public static final EverTalonFX PICKUP_MOTOR = new EverTalonFX(14);

                public static final EverTalonFX EXTENSION_MOTOR = new EverTalonFX(1);
                public static final EverTalonFXInternalEncoder EXTENSION_ENCODER = new EverTalonFXInternalEncoder(
                                EXTENSION_MOTOR);

                public static final DigitalInput RETRACTION_LM = new DigitalInput(0),
                                EXTENSION_LM = new DigitalInput(3);

                public final double EXTENSION_SPEED = 0.25, PICKUP_SPEED = 0.6;
                public final boolean DEBUG_MODE = false;
                public final double MAX_EXTENDING_ROTATIONS = 0;

        }

        public interface FeedAndConveyConsts {

                public final static boolean DEBUG_MOD = false;

                public final static double CONVEYING_SPEED = -0.5; // motor power, place holder

                public final static EverTalonFX CONVEY_MOTOR = new EverTalonFX(17), FEEDING_MOTOR =
                 new EverTalonFX(16);

                public final static double FEEDER_MAX_STALL_TIME = 2, FEEDING_SPEED = 0.4; // motor power
                                                                                              

                public static final DigitalInput ENTER_LEFT_LM = new DigitalInput(2),
                                                 ENTER_RIGHT_LM = new DigitalInput(1);

                public static final double FEEDING_TIME = 0.15; // place holder (in seconds)

        }

        public interface ClimbConst {

                // TODO: change all place holder to real values

                public static final double MAX_OPEN = 3000, OPEN_VEL = 0.25;

                public static final EverTalonFX CLIMB_MOTOR = new EverTalonFX(6);

                public static final EverTalonFXInternalEncoder CLIMB_ENCODER = new EverTalonFXInternalEncoder(
                                CLIMB_MOTOR);

                public static final DigitalInput BOTTOM_LM = new DigitalInput(5), TOP_LM = new DigitalInput(4);

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