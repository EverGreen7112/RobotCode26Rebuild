package frc.robot.Subsystems.Shooter;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverMotorControllerGroup;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;

public interface ShooterConsts {

    public static final EverMotorController 
        LEFT_MOTOR = new EverSparkMax(0),
        RIGHT_MOTOR = new EverSparkMax(1),
        ANGLE_MOTOR = new EverTalonFX(2);   

    
    public static final double SHOTER_GEAR_RATIO = 0; 
    
    public static final double 
        HUB_HIGHT = 0,
        MECHANISEM_HIGHT = 0,
        SHOOTING_HIGHT = HUB_HIGHT - MECHANISEM_HIGHT,// change this for the real hight of the hub (in meters)
        ANGLE_ERROR_MARGIN = 0.1, // change this for the real error margin of the shooter (in meters)

        MAX_ANGLE = 60, // change this for the real max angle of the shooter (in degrees)
        MIN_ANGLE = 0; // change this for the real min angle of the shooter (in degrees)

    public static final Pose2d HUB_POINT = new Pose2d(); // change this for the real location of the hubs

    public static final double 
        KP = 0, // change this for the real KP of the shooter
        KI = 0, // change this for the real KI of the shooter
        KD = 0; // change this for the real KD of the shooter   
    
    public static final boolean DEBUG_MODE = false;


} 
