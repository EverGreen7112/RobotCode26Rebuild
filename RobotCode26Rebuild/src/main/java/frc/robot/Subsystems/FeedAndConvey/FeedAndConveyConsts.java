package frc.robot.Subsystems.FeedAndConvey;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public interface FeedAndConveyConsts {

    public final static boolean DEBUG_MOD = false;
 
    public final static double CONVEYING_SPEED = 0.5; // motor power, place holder

    public final static EverTalonFX  CONVEY_MOTOR = new EverTalonFX(0), FEEDING_MOTOR = new EverTalonFX(1);

    public final static double FEEDER_MAX_STALL_TIME = 0.5, FEEDING_SPEED = 0.25; //motor power, place holder

    public static final DigitalInput ENTER_LEFT_LM = new DigitalInput(0), ENTER_RIGHT_LM = new DigitalInput(1);

                                     

}
