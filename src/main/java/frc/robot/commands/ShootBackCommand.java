package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// This command exists only to shoot a disc back into the intake system, used to spool up the shooter system / to readjust the disc in the intake
public class ShootBackCommand extends Command {
    private final ShooterClimberSubsystem shooterClimberSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ShootBackCommand(ShooterClimberSubsystem shooterClimberSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterClimberSubsystem = shooterClimberSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterClimberSubsystem);
        addRequirements(intakeSubsystem);

    }

    public void execute() {
        shooterClimberSubsystem.shootBack();
        intakeSubsystem.BackTake();
    }

    public void end(boolean interrupted) {
        shooterClimberSubsystem.stopShooter();
        intakeSubsystem.stopIntake();
    }
}
