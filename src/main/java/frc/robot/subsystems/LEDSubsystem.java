// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.system.Timer;
import org.wpilib.util.Color;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.WaitCommand;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LimitSwitch. */
  private AddressableLED led1 = new AddressableLED(Constants.LEDConstants.LEDPort);
  private AddressableLEDBuffer buffer1 = new AddressableLEDBuffer(Constants.LEDConstants.BufferLength);
  private int m_rainbowFirstPixelHue = 0;
  private int m_bagelbow_first_index = 0;

  public LEDSubsystem() {
    led1.setLength(Constants.LEDConstants.BufferLength);
    led1.setData(buffer1);
   // bagelbow();
  }
  
   public void setColor(Color color) {
    for(int i = 0; i < buffer1.getLength(); ++i) {
        buffer1.setLED(i, color);
    }
    led1.setData(buffer1);}

    public void DoTheRainbow(boolean val) {
    
   }

  //  public void bagelbow() {
   
  //   for(var i = 0; i < 8; i++) {


      
  //     buffer1.setLED((m_bagelbow_first_index + i)%16 , Color.BLUE);
  //     //buffer1.setLED((m_bagelbow_first_index + i) , Color.BLUE);
  //     buffer1.setLED((m_bagelbow_first_index + i + 8)%16 , Color.YELLOW);
  //     //buffer1.setLED((m_bagelbow_first_index + i + 8), Color.YELLOW);
      
  //   }


    
  
  //   m_bagelbow_first_index = (int)(Timer.getTimestamp()* 8) % 16;
  //   led1.setData(buffer1);

  //  }

   public void LEDScroll(int length, int start, boolean isForwards){//technically doesn't actually scroll
    if(isForwards){
        for(var i = start; i < length/2 + start; i++) {
          buffer1.setLED(((m_bagelbow_first_index%length) + i) % length + start, Color.BLUE);
          buffer1.setLED(((m_bagelbow_first_index%length) + i + length/2) % length + start, Color.YELLOW);
        }
    }else{
      for(var i = length + start ; i >= length/2 + start; i--) {
        buffer1.setLED((-m_bagelbow_first_index%length + i) % length + start, Color.BLUE);
        buffer1.setLED((-m_bagelbow_first_index%length + i + length/2) % length + start, Color.YELLOW);
      }
    }
    m_bagelbow_first_index = (int)(Timer.getMonotonicTimestamp()* 8) % 16;
    led1.setData(buffer1);
   }
 
   public void rainbow(){
    // For every pixel
    for (var i = 0; i < 14; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / 16)) % 180;
      // Set the value
      buffer1.setHSV(i, hue, 255, 128);
      buffer1.setHSV(i+14, hue, 255, 128);
    }
    led1.setData(buffer1);
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
