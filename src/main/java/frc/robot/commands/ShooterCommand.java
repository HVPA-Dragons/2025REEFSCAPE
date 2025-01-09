package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterClimberSubsystem;

public class ShooterCommand extends Command {

    private final ShooterClimberSubsystem shooterClimberSubsystem;

    public ShooterCommand(ShooterClimberSubsystem shooterClimberSubsystem) {
        this.shooterClimberSubsystem = shooterClimberSubsystem;
        addRequirements(shooterClimberSubsystem);
    }

    @Override
    public void execute() {
        shooterClimberSubsystem.shootOnSpeaker();

    }

    @Override
    public void end(boolean interrupted) {
        shooterClimberSubsystem.stopShooter();
    }
}