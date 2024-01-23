package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class LimelightCommand extends Command {
    private Drivetrain m_drivetrain = new Drivetrain();
    private Limelight m_limelight = new Limelight();
    private TalonFX m_leftSide = m_drivetrain.getLeftMotors();
    private TalonFX m_rightSide = m_drivetrain.getRightMotors();
    double Kp = -0.1;
    double minimum = 0.05;
    double tx;

    public void aim() {
        double error = -tx; 
        double adjust = 0.0;

        if (Math.abs(error) > 1.0) {
            if (error < 0) {
                adjust = Kp * error + minimum;
            } else {
                adjust = Kp * error - minimum;
            }
        }

        m_leftSide.set(adjust);
        m_rightSide.set(-adjust);
    }

    @Override
    public void execute() {
        tx = m_limelight.getTX().getDouble(0.0);
    }

    public void end() {
        m_leftSide.set(0);
        m_rightSide.set(0);
    }

}
