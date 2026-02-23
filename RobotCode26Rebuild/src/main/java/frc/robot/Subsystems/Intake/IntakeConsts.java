package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkFlex;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public interface IntakeConsts {
    
        public static final EverSparkFlex PICKUP_MOTOR = new EverSparkFlex(0);
        
        public static final EverTalonFX EXTENSION_MOTOR = new EverTalonFX(1);


        public static final DigitalInput RETRACTION_LM = new DigitalInput(0), 
                                        EXTENSION_LM = new DigitalInput(1);

        public final double EXTENSION_SPEED = 0.25, PICKUP_SPEED = 0.6;
        public final boolean DEBUG_MODE = false;



}
