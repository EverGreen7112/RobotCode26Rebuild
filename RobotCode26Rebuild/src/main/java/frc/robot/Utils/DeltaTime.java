package frc.robot.Utils;

import frc.robot.Utils.EverKit.Periodic;

public class DeltaTime {

    private double m_lastTime;

    private final double MIN_TIME = 0.001; // minimum delta time to avoid division by zero
    
    public DeltaTime(){
        m_lastTime = System.currentTimeMillis();
    }

    public void setNow(){
        m_lastTime = System.currentTimeMillis();
    }

    /**
     * calculates the time that has passed since the last time this method was called and updates the last time to the current time
     * need to update manually in periodic to get the time between each update and use it for calculations that need delta time
     * @return delta time in seconds
     */
    public double get(){
        double currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - m_lastTime) / 1000.0; // convert to seconds
        m_lastTime = currentTime;
        return Math.max(deltaTime, MIN_TIME); // ensure minimum delta time is not zero  
    }

    
}
