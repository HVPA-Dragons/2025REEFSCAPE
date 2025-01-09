package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SensorSuiteSubsystem;

public class IRRead extends Command {
    private SensorSuiteSubsystem SensorSuiteSubsystem;

    public IRRead(SensorSuiteSubsystem sensorSuiteSubsystem) {
        this.SensorSuiteSubsystem = sensorSuiteSubsystem;

        addRequirements(SensorSuiteSubsystem);
    }

    @Override
    public void execute() {
        SensorSuiteSubsystem.ReadIRSensor();
    }

    @Override
    public void end(boolean interrupted) {
        SensorSuiteSubsystem.StopRead();

    }
}
