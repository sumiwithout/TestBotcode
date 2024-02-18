package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hang extends SubsystemBase {
    private CANSparkMax m_hang;
    private boolean isiton;
    private boolean goback;

    private static final Hang m_hangleft = new Hang(Constants.hang.leftcanID,false);
private static final Hang m_righthang = new Hang(Constants.hang.rightcanID, true);
    public static Hang getleftInstance(){
        return m_hangleft;
    }
    public static Hang getrightInstance(){
        return m_righthang;
    }
    /**
     * @param canID The CANID for the motor connected to the actuator
     * @param ifreverse Since they will be side by side one will be reversed 
     */
    public Hang(int canID, boolean ifreverse ){
m_hang = new CANSparkMax(canID, MotorType.kBrushless);
m_hang.setInverted(ifreverse);
m_hang.setSmartCurrentLimit(Constants.hang.kCurrentLimit);
m_hang.setIdleMode(IdleMode.kBrake);
m_hang.burnFlash();
isiton = false;
goback = false;
    }

    public void turnon(){
        isiton = true;
    }
    public void stopoff(){
        isiton = false;

    }
    public void goback(){
        goback = true;
    }
    
    
    @Override
    public void periodic(){
        if(isiton){
            m_hang.set(Constants.hang.speedup);
        }
        else if(goback){
             m_hang.set(Constants.hang.speeddown);
        }
        else{
            m_hang.set(0);
        }
    }
}
