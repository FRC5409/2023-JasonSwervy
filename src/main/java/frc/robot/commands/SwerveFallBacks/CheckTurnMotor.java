package frc.robot.commands.SwerveFallBacks;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kFallBack;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class CheckTurnMotor extends CommandBase {

    private final Drivetrain m_drive;
    private final SwerveModule[] m_modules;

    private final Timer timer;

    private double[] desiredPos;

    public CheckTurnMotor(Drivetrain drivetrain) {
        m_drive = drivetrain;
        m_modules = m_drive.getModules();
        desiredPos = new double[m_drive.getModules().length];

        timer = new Timer();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.resetAllEncoders();

        for (int i = 0; i < m_modules.length; i++) {
            desiredPos[i] = (m_modules[i].getAbsoluteTurnEncoderPositionDegrees() + 180) % 360;
            
            m_modules[i].setTurnDegrees(desiredPos[i]);

            System.out.println("Turn motor: " + i + " set to: " + desiredPos[i] + " degrees");
        }

        timer.start();
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        int num = -1;

        for (int i = 0; i < m_modules.length; i++) {
            double pos = m_modules[i].getAbsoluteTurnEncoderPositionDegrees();
            if (Math.abs(pos - desiredPos[i]) > kFallBack.turnTolerance) {
                m_modules[i].isTurnStuck(true);
                num = i;
                System.out.println("Module: " + i + " seems to have gotten stuck at: " + pos + " degrees");
            }
        }

        if (num != -1) {
            double pos = m_modules[num].getAbsoluteTurnEncoderPositionDegrees();

            System.out.println("Set all motors turning direction to be: " + pos + " degrees");

            for (int i = 0; i < m_modules.length; i++) {
                if (i == num) continue;

                m_modules[i].setTurnDegrees(pos);

                m_drive.isTank(true);
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.advanceIfElapsed(1);
    }

}
