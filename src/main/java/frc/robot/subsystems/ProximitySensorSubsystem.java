package frc.robot;

import frc.robot.subsystems.ProximitySensorSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  
    private final ProximitySensorSubsystem proximitySensorSubsystem = new ProximitySensorSubsystem(0); // Assuming the sensor is on digital input port 0

    @Override
    public void teleopPeriodic() {
        // Check the sensor state during teleop
        if (proximitySensorSubsystem.isObjectDetected()) {
            System.out.println("Object detected!");
        } else {
            System.out.println("No object detected.");
        }
    }
}
