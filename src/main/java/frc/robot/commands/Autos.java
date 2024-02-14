package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class maddie {
  DriveSubsystem m_driveSubsystem  = DriveSubsystem.getInstance();
      private final SendableChooser<Command> autonChooser;
      public maddie(){
AutoBuilder.configureHolonomic(
      m_driveSubsystem::getPose, // Robot pose supplier
      m_driveSubsystem::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      m_driveSubsystem::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      m_driveSubsystem::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
              4.5, // Max module speed, in m/s
              0.4, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
      m_driveSubsystem // Reference to this subsystem to set requirements
);



        autonChooser = new SendableChooser<Command>();
         autonChooser.setDefaultOption("No-op", null);
         buildAuto("route");
      
       SmartDashboard.putData("Auton Chooser", autonChooser);
      
       
      }
      public Command getSelected() {
        return autonChooser.getSelected();
      }
    
        private void buildAuto(String autoName) {
            Command autoCommand = AutoBuilder.buildAuto(autoName);
            autonChooser.addOption(autoName, autoCommand);
          }
}
