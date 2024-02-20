package frc.team1126.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.StorageConstants;

public class Storage extends SubsystemBase {
    
    //neo 550 to power both acquisition wheels and storage belts
    private CANSparkMax storageWheels;
    private RelativeEncoder storageEncoder;
    private DigitalInput noteSensor;
    private boolean hasNote;

    public Storage() {

        storageWheels = new CANSparkMax(StorageConstants.ACQ_WHEELS_ID, CANSparkLowLevel.MotorType.kBrushless); // shouldn't need to specify that this is a neo 550 in code, but keep this in mind
        storageEncoder = storageWheels.getEncoder();
        
        noteSensor = new DigitalInput(StorageConstants.LIGHT_SENSOR);
        hasNote = false;
    }

    @Override
    public void periodic() {
        var lightSensorState = noteSensor.get();
        SmartDashboard.putBoolean("Light Sensor Limit",  !noteSensor.get());
        SmartDashboard.putNumber("Storage Wheels Speed",getStorageWheelsSpeed());
    }

    public void setStorageWheels(double speed) {
        // if (noteSensor.get()) {
        //     storageWheels.set(0);
        // } else {
            storageWheels.set(speed);
        // }
    }

    public double getStorageWheelsSpeed() {
        return storageWheels.get();
    }

    public boolean hasNote() {
        return !noteSensor.get();
    }

    public void setHasNote(){
        hasNote = true;
    }

    public void resetNote(){
        hasNote = false;
    }

    public boolean getHasNote() {
        return hasNote;
    }
}
