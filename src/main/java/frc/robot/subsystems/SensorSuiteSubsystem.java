package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorSuiteSubsystem extends SubsystemBase {

    private final DigitalInput IRSensor;

    public SensorSuiteSubsystem() {
        // Port for the sensors
        IRSensor = new DigitalInput(0);

    }

    public void ReadIRSensor() {
        // TODO use the sensor for stuff
        System.out.println(IRSensor.get());

    }

    public void StopRead() {
    }

    public Command ColorReadCommand() {
        return this.runEnd(this::ReadIRSensor, this::StopRead);
    }
}
/* Runs intake motors at half speed and runs back if a note is inserted */
