package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.math.util.Units.feetToMeters;
import static edu.wpi.first.math.util.Units.inchesToMeters;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive m_swerveDrive;

    /**
     * Creates a new SwerveSubsystem.
     * 
     * @param visionSubsystem The vision subsystem to use for pose estimation.
     * @throws IOException If the swerve module configuration file cannot be read.
     */
    public SwerveSubsystem() throws IOException {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        String swerveDir = "7718";
        SwerveParser parser = new SwerveParser(new File(Filesystem.getDeployDirectory(), swerveDir));

        // https://www.swervedrivespecialties.com/products/mk4-swerve-module
        // L3 free speed is around 19.3-19.5 ft/s
        double maxSpeed = feetToMeters(19.3);
        // Steering gear ratio of the MK4 is 12.8:1
        double angleMotorConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(21.428571429);
        // Drive gear ratio for the L3 is 6.12:1
        double driveMotorConversion = SwerveMath.calculateMetersPerRotation(inchesToMeters(4), 6.12);
        m_swerveDrive = parser.createSwerveDrive(maxSpeed, angleMotorConversionFactor, driveMotorConversion);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getVelocity,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5,
                        m_swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
                this);

    }

    @Override
    public void periodic() {
    }

    /**
     * Returns the maximum velocity of the swerve drive.
     * 
     * @reture A measure of the maximum velocity of the swerve drive.
     */
    public LinearVelocity getMaximumVelocity() {
        return Units.MetersPerSecond.of(m_swerveDrive.getMaximumVelocity());
    }

    /**
     * Returns the maximum angular velocity of the swerve drive.
     * 
     * @return A measure of the maximum angular velocity of the swerve drive.
     */
    public AngularVelocity getMaximumAngularVelocity() {
        return Units.RadiansPerSecond.of(m_swerveDrive.getMaximumAngularVelocity());
    }

    /**
     * Field-relative swerve drive.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        m_swerveDrive.driveFieldOriented(chassisSpeeds);
    }

    /**
     * Robot-relative swerve drive.
     */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        m_swerveDrive.drive(chassisSpeeds);
    }

    /**
     * Gets the current pose of the robot.
     * 
     * @return The estimated position (rotation + translation) of the robot.
     */
    public Pose2d getPose() {
        return m_swerveDrive.getPose();
    }

    /**
     * Returns the current gyro angle.
     * 
     * @return The current gyro angle as a Rotation2d.
     */
    public Rotation2d getHeading() {
        return m_swerveDrive.getYaw();
    }

    /**
     * Resets the robot's position on the field.
     * 
     * @param pose The new position of the robot.
     */
    public void resetPose(Pose2d pose) {
        m_swerveDrive.resetOdometry(pose);
    }

    /**
     * Stops the swerve drive
     */
    public void stop() {
        m_swerveDrive.drive(new ChassisSpeeds());
    }

    /**
     * Zeros the gyroscope
     */
    public Command cZeroGyro() {
        return this.runOnce(() -> {
            m_swerveDrive.zeroGyro();
        });
    }
    /* Supplier for ChassisSpeeds */

    public ChassisSpeeds getVelocity() {

        return m_swerveDrive.getRobotVelocity();

    }

}
