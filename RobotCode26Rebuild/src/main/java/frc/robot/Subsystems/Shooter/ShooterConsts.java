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

public interface ShooterConsts {

    public enum ShooterState{
        kStop, kScoring, kDelivery
    }

    public static final EverMotorController 
        LEFT_MOTOR = new EverSparkMax(0),
        RIGHT_MOTOR = new EverSparkMax(1),
        ANGLE_MOTOR = new EverTalonFX(2);   

    
    public static final double SHOOTER_GEAR_RATIO = 0; 

    public static final Pose2d BLUE_HUB_POSE = new Pose2d(); // change this for the real location of the blue hub
    public static final Pose2d RED_HUB_POSE = new Pose2d(); // change this for the real location of the red hub
    
    public static final double 
        
        GRAVITY = 9.81, // m/s^2

        HUB_HIGHT = 1.8,
        MECHANISM_HIGHT = 0,
        
        SHOOTING_HIGHT = HUB_HIGHT - MECHANISM_HIGHT,// change this for the real hight of the hub (in meters)
        
        ANGLE_ERROR_MARGIN = 0.1, // change this for the real error margin of the shooter (in meters)

        MAX_ANGLE = 60, // change this for the real max angle of the shooter (in degrees)
        MIN_ANGLE = 0, // change this for the real min angle of the shooter (in degrees)

        TARGET_RPM = 2024; // change this for the real target shooting rpm 

    public static final EverCANCoder ANGLE_CAN_CODER = new EverCANCoder(0);

    public static final double 
        SPEED_KP = 0, // change this for the real KP of the shooter
        SPEED_KI = 0, // change this for the real KI of the shooter
        SPEED_KD = 0, // change this for the real KD of the shooter  
        
        ANGLE_KP = 0, // change this for the real KP of the shooter
        ANGLE_KI = 0, // change this for the real KI of the shooter
        ANGLE_KD = 0; // change this for the real KD of the shooter  
    
    public static final boolean DEBUG_MODE = false;

    public static void config(){
        
        ANGLE_CAN_CODER.setPosConversionFactor(SHOOTER_GEAR_RATIO * 360); 
        ANGLE_CAN_CODER.setOffset(0);




    }


} 
