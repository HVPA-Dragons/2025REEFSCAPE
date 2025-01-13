package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;



public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;

        // SmartDashboard keys
        public static final String kDriveSensitivity = "Drive sensitivity";
        public static final String kTurnSensitivity = "Turn sensitivity";
    }
        // Swerve constants
    public static class SwerveDriveConstants {
        public static final LinearVelocity kMaxLinearVelocity = MetersPerSecond.of(3.0); 
        public static final AngularVelocity kMaxAngularVelocity = RadiansPerSecond.of(Math.PI);
        
    }

    public static class IntakeConstants {
        /* Intake Can IDs */
        public static final int kIntakeMotor1Port = 17;
        public static final int kIntakeMotor2Port = 18;
    }

    public static class ShooterConstants {
        /* Shooter Can IDs */
        public static final int kShooterMotor1Port = 15;
        public static final int kShooterMotor2Port = 16;

    }

}