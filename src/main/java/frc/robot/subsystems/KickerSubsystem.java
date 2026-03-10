package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {

    private final SparkMax KickerMotor =
        new SparkMax(Constants.IDConstants.KickerMotor_ID, MotorType.kBrushless);
 
         private final SparkMax KickerMotor2 =
        new SparkMax(Constants.IDConstants.KickerMotor_ID2, MotorType.kBrushless);
public KickerSubsystem() {


        // Leader motor config
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        KickerMotor.configure(
            leaderConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        // Follower motor config (inverted)
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(KickerMotor, true); 

        KickerMotor2.configure(
            
            followerConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters

        );
}
    // Run kicker motor
    public void runKicker(double speed) {
        KickerMotor.set(speed);
        KickerMotor2.set(speed);
    }

    // Stop kicker motor
    public void stopKicker() {
        KickerMotor.set(0);
        KickerMotor2.set(0);
    }

    // Command to run kicker at a speed
    public Command runKickerCommand(double speed) {
        return run(() -> runKicker(speed));
    }

    // Command to stop kicker
    public Command stopKickerCommand() {
        return run(this::stopKicker);
    }

    //Gimme all that telemtry 
     @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "KickerRPM",
            KickerMotor.getEncoder().getVelocity()
        );

         SmartDashboard.putNumber(
            "Kicker2RPM",
            KickerMotor2.getEncoder().getVelocity()
        );

    }

    public double getRPM() {
    return KickerMotor.getEncoder().getVelocity();
}
}