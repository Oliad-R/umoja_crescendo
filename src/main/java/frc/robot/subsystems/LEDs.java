package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Colors;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
    private AddressableLED LED = new AddressableLED(1);
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(4*LEDConstants.numLEDsPerStrip);

    private Color currColor;
    private int index = 0;

    public LEDs(){
        LED.setLength(LEDBuffer.getLength());
        setLEDColor(Colors.red);
        LED.start();
    }

    public void setLEDColor(Color color){
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, (int)color.red*255, (int)color.green*255, (int)color.blue*255);
        }
        LED.setData(LEDBuffer);
    }

    public void setUmojaColors(){
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            currColor = Colors.uColors[index];
            if(i%3==0){
                index++;
                if(index>=Colors.uColors.length){
                    index = 0;
                }
            }
            LEDBuffer.setRGB(i, (int)currColor.red*255, (int)currColor.green*255, (int)currColor.blue*255);
        }
        LED.setData(LEDBuffer);
    }

    // public void rotateColors() {
        
    //     for (int i = 0; i < LEDBuffer.getLength(); i++) {
    //         currColor = Colors.uColors[index];
    //         if(i%3==0){
    //             index++;
    //             if(index>=Colors.uColors.length){
    //                 index = 0;
    //             }
    //         }
    //         LEDBuffer.setRGB(i, (int)currColor.red*255, (int)currColor.green*255, (int)currColor.blue*255);
    //     }
    //     LED.setData(LEDBuffer);
    // }
}