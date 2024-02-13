package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class Retract extends Command {
    IntakeSubsystem m_intake= IntakeSubsystem.getInstance();
    public  Retract(){
        addRequirements(m_intake);
    }
        @Override
        public void initialize() {
          m_intake.setpostionmode(true);
          m_intake.setTargetPosition(m_intake.getencoder() + Constants.Intake.kRetractDistance);
        }

        @Override
        public boolean isFinished() {
          return m_intake.isNearTarget();
        }


    }

