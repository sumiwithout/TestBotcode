package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;
public class ArmDown extends Command{
    private Timer m_timer;
    ArmSubsystem m_arm = ArmSubsystem.getInstance();
    // help
    public ArmDown(){
      
        addRequirements(m_arm);
    }
    @Override
          public void initialize() {            
          }

          @Override
          public void execute() {
            m_arm.setTargetPosition(Constants.Arm.kIntakePosition);
            m_arm.runAutomatic();
          }

          @Override
          public boolean isFinished() {
           return m_arm.isFinished();
         
          }

          @Override
          public void end(boolean interrupted) {
          }
}