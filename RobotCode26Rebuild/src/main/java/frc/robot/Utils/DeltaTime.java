package frc.robot.Utils;

public class DeltaTime {

    private double m_lastTime;
    
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
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - m_lastTime) / 1000.0; // convert to seconds
        m_lastTime = currentTime;
        return deltaTime;
    }
    
}
