package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;
public class ArmUp extends Command{
    private Timer m_timer;
    ArmSubsystem m_arm = ArmSubsystem.getInstance();
    // help
    public ArmUp(){
      
        addRequirements(m_arm);
    }
    @Override
          public void initialize() {            
          }

          @Override
          public void execute() {
            m_arm.setTargetPosition(Constants.Arm.kScoringPosition);
            m_arm.runAutomatic();
          }

          @Override
          public boolean isFinished() {
         return   m_arm.isNearTarget();
          }

          @Override
          public void end(boolean interrupted) {
          }
}