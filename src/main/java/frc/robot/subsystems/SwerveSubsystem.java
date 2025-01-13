package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.parser.*;

/* TODO: Implement interaction w/ swerve modules, gyro, and path planner*/
 



public class SwerveSubsystem extends SubsystemBase {
    public static final LinearVelocity kMaxLinearVelocity = frc.robot.Constants.SwerveDriveConstants.kMaxLinearVelocity;
    public static final AngularVelocity kMaxAngularVelocity = frc.robot.Constants.SwerveDriveConstants.kMaxAngularVelocity;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // TODO Get wheel locations
            new Translation2d(0.381, 0.381),
            new Translation2d(0.381, -0.381),
            new Translation2d(-0.381, 0.381),
            new Translation2d(-0.381, -0.381)
        );





    public SwerveSubsystem() {
        
    }

    public void drive(LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Rotation2d heading) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds (vx, vy, omega, heading);
        var states = kinematics.toSwerveModuleStates(speeds);
        
    }

    public LinearVelocity getMaximumVelocity() {
        return kMaxLinearVelocity;
    }

    public  AngularVelocity getMaximumAngularVelocity() {
        return kMaxAngularVelocity;
    }

    public void ZeroGyro() {
        // Implementation for ZeroGyro
    }

    public Command cZeroGyro() {
        return this.runOnce(this::ZeroGyro);}

    public Rotation2d getHeading() {
        // Implementation for getHeading
        return null;
    }

    public void stop() {
        // Implementation for stop
        
    }
    }
