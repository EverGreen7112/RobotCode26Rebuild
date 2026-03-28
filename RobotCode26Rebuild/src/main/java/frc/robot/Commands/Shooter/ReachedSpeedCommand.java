package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Consts.ShooterConsts;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;

public class ReachedSpeedCommand extends Command implements ShooterConsts{

    @Override
    public void initialize() {
        
    }


    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return FRONT_SHOOTING_ENCODER.getVel() > 10;
    }
    
    
}
