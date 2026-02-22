package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConsts;

public class DeliverCommand extends Command {

        @Override
        public void initialize() {
            Shooter.getInstance().setShooterState(ShooterConsts.ShooterState.kDelivery);
        }
    
        @Override
        public void execute() {
            // delivery code here
        }
    
        @Override
        public void end(boolean interrupted) {
            // stop delivery code here
        }
    
        @Override
        public boolean isFinished() {
            return false;
        }
    
}
