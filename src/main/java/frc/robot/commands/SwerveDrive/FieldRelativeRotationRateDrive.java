package frc.robot.commands.SwerveDrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to drive a swerve drive robot with field relative translation and a
 * rotation rate.
 */
public class FieldRelativeRotationRateDrive extends Command {
    private final SwerveSubsystem drive;
    private final LinearVelocity MAX_SPEED;
    private final AngularVelocity MAX_OMEGA;
    private final DoubleSupplier omegaSupplier;
    private final Supplier<Translation2d> translationSupplier;

    /**
     * Creates a new FieldRelativeRotationRateDrive.
     * 
     * @param drive                The drive subsystem this command will run on
     * @param translationSupplier  The translation as a Supplier (typically from the joystick input)
     * @param omegaSupplier        The omega (rotation) value as a DoubleSupplier (typically from the joystick input)
     */
    public FieldRelativeRotationRateDrive(SwerveSubsystem drive, Supplier<Translation2d> translationSupplier,
            DoubleSupplier omegaSupplier) {
        this.drive = drive;
        this.translationSupplier = translationSupplier;
        this.omegaSupplier = omegaSupplier;

        // Set maximum speeds for translation and rotation
        MAX_SPEED = drive.getMaximumVelocity(); 
        MAX_OMEGA = drive.getMaximumAngularVelocity();

        // Add drive subsystem as a requirement for the command
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        // No initialization necessary, but this method can be used if needed
    }

    @Override
    public void execute() {
        // Get the drive and turn sensitivities from the SmartDashboard or use default values
        double driveSensitivity = SmartDashboard.getNumber(OperatorConstants.kDriveSensitivity, 1.0);
        double turnSensitivity = SmartDashboard.getNumber(OperatorConstants.kTurnSensitivity, 1.0);

        // Get the translation values (X and Y) and omega (rotation rate) from the suppliers
        Translation2d translation = translationSupplier.get();
        
        // Apply sensitivity scaling and convert to appropriate units for velocity
        LinearVelocity vx = MAX_SPEED.times(translation.getX() * driveSensitivity);
        LinearVelocity vy = MAX_SPEED.times(translation.getY() * driveSensitivity);
        AngularVelocity omega = MAX_OMEGA.times(omegaSupplier.getAsDouble() * turnSensitivity);

    

        // Drive the robot using the calculated speeds 
        drive.drive(
            vx,
            vy,
            omega,
            drive.getHeading()
        );
    }

    @Override
    public boolean isFinished() {
        // This command should run continuously, so it's never finished
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot if the command is interrupted or finishes
        drive.stop();
    }
}