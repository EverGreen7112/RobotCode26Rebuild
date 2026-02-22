package frc.robot.Utils;

public class DeltaTime {

        private double m_lastTime;
    
        public DeltaTime(){
            m_lastTime = System.currentTimeMillis();
        }

        public void setNow(){
            m_lastTime = System.currentTimeMillis();
        }
    
        public double get(){
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - m_lastTime) / 1000.0; // convert to seconds
            m_lastTime = currentTime;
            return deltaTime;
        }
    
}
