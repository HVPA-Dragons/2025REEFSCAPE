package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.*;

public class ShooterClimberSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor1;
    private final CANSparkMax shooterMotor2;

    public ShooterClimberSubsystem() {
        shooterMotor1 = new CANSparkMax(Constants.ShooterConstants.kShooterMotor1Port, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(Constants.ShooterConstants.kShooterMotor2Port, MotorType.kBrushless);
        shooterMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }

    public void shootOnSpeaker() {
        shooterMotor1.set(1);
        shooterMotor2.set(1);
    }

    public void shootOnAmp() {
        shooterMotor1.set(1);
        shooterMotor2.set(1);
    }

    public void stopShooter() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);

    }

    public void shootBack() {
        shooterMotor1.set(-1);
        shooterMotor2.set(-1);

    }

    public Command cShootOnSpeaker() {
        return this.runEnd(this::shootOnSpeaker, this::stopShooter);
    }

    public Command cShootOnAmp() {
        return this.runEnd(this::shootOnAmp, this::stopShooter);
    }

    public Command cShootBack() {
        return this.runEnd(this::shootBack, this::stopShooter);
    }

}
