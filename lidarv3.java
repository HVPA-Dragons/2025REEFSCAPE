package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.sensors.LidarLiteV3;

public class Robot extends TimedRobot {
      private LidarLiteV3 lidar;

      @Override
      public void robotInit() {
            lidar = new LidarLiteV3(I2C.Port.kOnboard); // Use onboard I2C port
      }


'''java
     @Override 
      public void robotPeriodic() {
             // Periodically log the distance
             int distance = lidar.getDistance();
             if (distance != -1) {
                 System.out.println("Lidar Distance: " + distance + " cm");
              } else {
                  System.out.println("Error reading Lidar sensor.");
              }
       }
}
