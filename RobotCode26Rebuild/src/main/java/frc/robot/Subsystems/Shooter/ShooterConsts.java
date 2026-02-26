package frc.robot.Subsystems.Shooter;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverMotorControllerGroup;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;
import frc.robot.Utils.Math.Funcs;

public interface ShooterConsts {

    public static final EverTalonFX LEFT_MOTOR = new EverTalonFX(0), RIGHT_MOTOR = new EverTalonFX(1);
    public static final EverSparkMax ANGLE_MOTOR = new EverSparkMax(2); 

    public static final EverTalonFXInternalEncoder SHOOTING_ENCODER = new EverTalonFXInternalEncoder(LEFT_MOTOR);

    public static final double SHOOTER_ANGLE_GEAR_RATIO = 0, SHOOTING_GEAR_RATIO = 0; 

    public static final Pose2d BLUE_HUB_POSE = new Pose2d(); // change this for the real location of the blue hub
    public static final Pose2d RED_HUB_POSE = new Pose2d(); // change this for the real location of the red hub

    
    public static final double 

        WHEEL_RADIUS = 0, // change this for the real radius of the shooter wheel (in meters)
        
        GRAVITY = 9.81, // m/s^2

        HUB_HEIGHT = 1.8,
        MECHANISM_HEIGHT = 0,
        
        SHOOTING_HIGHT = HUB_HEIGHT - MECHANISM_HEIGHT,// change this for the real hight of the hub (in meters)
        
        ANGLE_ERROR_MARGIN = 0.1, // change this for the real error margin of the shooter (in meters)

        MAX_ANGLE = 60, // change this for the real max angle of the shooter (in degrees)
        MIN_ANGLE = 0, // change this for the real min angle of the shooter (in degrees)

        TARGET_RPM = 2024, // change this for the real target shooting rpm 
        DELIVERY_RPM = 1000; // change this for the real delivery rpm

    public static final EverCANCoder ANGLE_CAN_CODER = new EverCANCoder(0);

    public static final double 
        SPEED_KP = 0, // change this for the real KP of the shooter
        SPEED_KI = 0, // change this for the real KI of the shooter
        SPEED_KD = 0, // change this for the real KD of the shooter  
        
        ANGLE_KP = 0, // change this for the real KP of the shooter
        ANGLE_KI = 0, // change this for the real KI of the shooter
        ANGLE_KD = 0; // change this for the real KD of the shooter
        
    public static final EverSparkMaxPIDController ANGLE_PID_CONTROLLER = new EverSparkMaxPIDController(ANGLE_MOTOR);
    public static final EverTalonFXPIDController SHOOTING_PID_CONTROLLER = new EverTalonFXPIDController(LEFT_MOTOR);
    
    public static final boolean DEBUG_MODE = false;
    public static final double DELIVERY_ANGLE = 45;

    public static void config(){
        
        ANGLE_CAN_CODER.setPosConversionFactor(SHOOTER_ANGLE_GEAR_RATIO * 360); 
        ANGLE_CAN_CODER.setOffset(0);

        SHOOTING_ENCODER.setVelConversionFactor((2 * Math.PI * WHEEL_RADIUS) / (SHOOTING_GEAR_RATIO * 60)); // convert from rpm to m/s

        ANGLE_PID_CONTROLLER.setPID(ANGLE_KP, ANGLE_KI, ANGLE_KD);
        SHOOTING_PID_CONTROLLER.setPID(SPEED_KP, SPEED_KI, SPEED_KD);

    }


} 
