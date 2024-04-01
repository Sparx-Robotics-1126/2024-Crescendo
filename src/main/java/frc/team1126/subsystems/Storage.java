package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.StorageConstants;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.CANdleSubsystem.LEDState;

public class Storage extends SubsystemBase {
    
    //neo 550 to power both acquisition wheels and storage belts
    private CANSparkMax m_storageWheels;
    private DigitalInput m_noteSensor;
    private boolean m_hasNote;

    public Storage() {

        m_storageWheels = new CANSparkMax(StorageConstants.ACQ_WHEELS_ID, CANSparkLowLevel.MotorType.kBrushless); // shouldn't need to specify that this is a neo 550 in code, but keep this in mind
        
        m_noteSensor = new DigitalInput(StorageConstants.LIGHT_SENSOR);
        m_hasNote = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Light Sensor Limit",  !m_noteSensor.get());
        SmartDashboard.putNumber("Storage Wheels Speed",getStorageWheelsSpeed());
    }

    public void setStorageWheels(double speed) {
        // if (noteSensor.get()) {
        //     storageWheels.set(0);
        // } else {
            m_storageWheels.set(speed);
        // }
    }

    public double getStorageWheelsSpeed() {
        return m_storageWheels.get();
    }

    public boolean hasNote() {
        return !m_noteSensor.get();
    }

    public void setHasNote(){
        m_hasNote = true;
    }

    public void resetNote(){
        m_hasNote = false;
    }

    public boolean getHasNote() {
        return m_hasNote;
    }

     //for the greece allignment code
    public boolean isBeamBroken() {
    return m_noteSensor.get() == false;
  }
}
