package frc.robot.Utils;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Utils.EverKit.Periodic;

public class DeltaTime {

    private double m_lastTime;

    private Timer timer;

    private final double MIN_TIME = 0.001; // minimum delta time to avoid division by zero
    
    public DeltaTime(){
        timer = new Timer();
        m_lastTime = timer.getFPGATimestamp();
    }

    public void setNow(){
        m_lastTime = timer.getFPGATimestamp();
    }

    /**
     * calculates the time that has passed since the last time this method was called and updates the last time to the current time
     * need to update manually in periodic to get the time between each update and use it for calculations that need delta time
     * @return delta time in seconds
     */
    public double get(){
        double currentTime = timer.getFPGATimestamp();
        double deltaTime = (currentTime - m_lastTime); 
        return Math.max(deltaTime, MIN_TIME); // ensure minimum delta time is not zero  
    }

    
}
