package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
    private AddressableLED LED = new AddressableLED(1);
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(4*LEDConstants.numLEDsPerStrip);

    public LEDs(){
        LED.setLength(LEDBuffer.getLength());
        setLEDColor(255, 0, 0);
        LED.start();
    }

    public void setLEDColor(int red, int green, int blue){
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, red, green, blue);
        }
        LED.setData(LEDBuffer);
    }

    public void setRed(){
        setLEDColor(255, 0, 0);
    }

    public void setGreen(){
        setLEDColor(0,255,0);
    }

    public void setBlue(){
        setLEDColor(0, 0, 255);
    }

}
