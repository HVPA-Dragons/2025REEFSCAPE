package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.parser.*;



public class SwerveSubsystem extends SubsystemBase {
    public static final double kMaxLinearVelocity = frc.robot.Constants.SwerveDriveConstants.kMaxLinearVelocity;
    public static final double kMaxAngularVelocity = frc.robot.Constants.SwerveDriveConstants.kMaxAngularVelocity;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // TODO Get wheel locations
            new Translation2d(0.381, 0.381),
            new Translation2d(0.381, -0.381),
            new Translation2d(-0.381, 0.381),
            new Translation2d(-0.381, -0.381)
        );





    public SwerveSubsystem() {
        
    }

    public void drive(double fwd, double str, double rot, Rotation2d heading) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, heading);
        var states = kinematics.toSwerveModuleStates(speeds);
        // Further module control logic goes here
    }
}