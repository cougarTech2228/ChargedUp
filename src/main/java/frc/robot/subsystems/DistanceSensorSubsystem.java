package frc.robot.subsystems;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class DistanceSensorSubsystem {

    private Rev2mDistanceSensor m_ElevatorDistMxp;
    private Rev2mDistanceSensor m_ExtendoArmDistMxp;

    // private final int kI2CAddress;
    // private final I2C m_i2c;
    private final static int muxI2cAddr = 0x70;

   
    public void selectMuxPort(int ports){
        I2C i2c = new I2C(I2C.Port.kMXP, muxI2cAddr);

        i2c.write(muxI2cAddr, ports);

        byte[] result = new byte[1];
        i2c.readOnly(result, 1);

        System.out.println("enabledBuses: " + result[0]);
        i2c.close();
    }


    public DistanceSensorSubsystem(){
        // DigitalOutput pin = new DigitalOutput(9);

        // //Set Elevator Address
        // pin.set(false);

        selectMuxPort(0x01);
        
        setRev2Addr(0x29, Constants.ELEVATOR_DISTANCE_SENSOR_ADDRESS);

        selectMuxPort(0x02);
        
        setRev2Addr(0x29, Constants.EXTENDO_ARM_DISTANCE_SENSOR_ADDRESS);
        
        selectMuxPort(0x03);
        //Set Extendo Arm Address
        // pin.set(true);
        // try {
        //     Thread.sleep(50);
        // } catch (InterruptedException e) {
        //     e.printStackTrace();
        // }
        // setRev2Addr(0x29, Constants.EXTENDO_ARM_DISTANCE_SENSOR_ADDRESS);

        m_ElevatorDistMxp = new Rev2mDistanceSensor(Port.kMXP, Constants.ELEVATOR_DISTANCE_SENSOR_ADDRESS);
        m_ElevatorDistMxp.setEnabled(true);
        // m_ElevatorDistMxp.setAutomaticMode(true);

        m_ExtendoArmDistMxp = new Rev2mDistanceSensor(Port.kMXP, Constants.EXTENDO_ARM_DISTANCE_SENSOR_ADDRESS);
        m_ExtendoArmDistMxp.setEnabled(true);
        m_ExtendoArmDistMxp.setAutomaticMode(true);
        // pin.close();
    }

    private final static int VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS = 0x008a;
    public void setRev2Addr(int oldAddr, int newAddr) {
        System.out.println(String.format("setRev2Addr(0x%02x, 0x%02x)", oldAddr, newAddr));
        I2C i2c = new I2C(I2C.Port.kMXP, oldAddr);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        byte[] data = new byte[1];
        if (i2c.read(VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, 1, data)) {
            System.out.println("Failed to read addr register");
        }
        System.out.println(String.format("read addr: 0x%02x", data[0]));
        
        if (i2c.write(VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, (0xFF & (newAddr / 2)))) {
            System.out.println("i2c.write returned true");
        }
        i2c.close();
    }

    public Rev2mDistanceSensor getElevatorSensor(){
        return m_ElevatorDistMxp;
    }

    public Rev2mDistanceSensor getExtendoArmSensor(){
        return m_ExtendoArmDistMxp;
    }
}
