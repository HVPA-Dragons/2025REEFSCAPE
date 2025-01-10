package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor1;
    private final SparkMax intakeMotor2;
    private final DigitalInput intakeStopSensor;

    /* Can IDs */
    public IntakeSubsystem() {
        intakeMotor1 = new SparkMax(Constants.IntakeConstants.kIntakeMotor1Port,
                MotorType.kBrushless);
        intakeMotor2 = new SparkMax(Constants.IntakeConstants.kIntakeMotor2Port,
                MotorType.kBrushless);
        intakeStopSensor = new DigitalInput(2);

    }

    /* Runs the intake. */
    public void intake() {
        intakeMotor1.set(-1);
        intakeMotor2.set(-1);
    }

    /* Stops the intake */
    public void stopIntake() {
        intakeMotor1.set(0);
        intakeMotor2.set(0);
    }

    /* Runs the intake motors in reverse. */
    public void BackTake() {

        intakeMotor1.set(1);
        intakeMotor2.set(1);
    }

}
