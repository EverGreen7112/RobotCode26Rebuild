package frc.robot.Commands.Shoter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Shot extends Command{
    
    private EverTalonFX m_motor;

    public Shot(){
        m_motor = new EverTalonFX(1);
        //m_motor.setInverted(true);
    }

    @Override
    public void initialize(){
        m_motor.set(1);
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_motor.set(0);
    }


}
