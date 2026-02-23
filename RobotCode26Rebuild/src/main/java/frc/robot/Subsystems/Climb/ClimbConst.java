package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

/**
 * Constants for the Climb subsystem.
 * Put hardware IDs, sensor ports and tuning constants here.
 */
public interface ClimbConst {
    
    public static final double MAX_OPEN = 3000, OPEN_VEL = 0.25; // place holder

    public static final EverTalonFX CLIMB_MOTOR = new EverTalonFX(0);

    public static final EverTalonFXInternalEncoder CLIMB_ENCODER = new EverTalonFXInternalEncoder(CLIMB_MOTOR);

    public static final DigitalInput BOTTOM_LM = new DigitalInput(0), TOP_RM = new DigitalInput(1);

    public static final boolean DEBUG_MODE = false;

}