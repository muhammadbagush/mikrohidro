//-------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//-------------------------------                                     Current Sensor Measure Module                                      ----------------------------//
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------//

//#define DEBUG
#define RELEASE

#define AvgSize 50
uint8_t index0 = 0;uint8_t index1 = 0;uint8_t index2 = 0;
float sum0;float sum1;float sum2;
float reading0[AvgSize];float reading1[AvgSize];float reading2[AvgSize];
float MvAvg0;float MvAvg1;float MvAvg2;

#include "EmonLib.h"                                                                // Include Emon Library 
EnergyMonitor emon0;                                                                // Create an instance emon0
EnergyMonitor emon1;                                                                // Create an instance emon1
EnergyMonitor emon2;                                                                // Create an instance emon2
EnergyMonitor emon3;                                                                // Create an instance emon3
EnergyMonitor emon4;                                                                // Create an instance emon4
EnergyMonitor emon5;                                                                // Create an instance emon5
EnergyMonitor emon6;                                                                // Create an instance emon6
EnergyMonitor emon7;                                                                // Create an instance emon7
EnergyMonitor emon8;                                                                // Create an instance emon8

float R1 = 4700.0;                                                                  // Value resistor 1 in voltage divider circuit                                     
float R2 = 1200.0;                                                                  // Value resistor 2 in voltage divider circuit 
float ACVoltageRefference = 220.0;                                                  // AC Voltage refference for data measurement 
float inputVoltageRefference = 14.6;                                                // Input Voltage refference for data measurement (voltage output from diode bridge)

float adc0;float adc1;float adc2;                                                   // Define variable adc 0, 1, & 2                   
float voltage0;float voltage1;float voltage2;                                       // Define variable voltage 0, 1, & 2 from adc (voltage = adc * Vreff/1023)
float inputVoltage0;float inputVoltage1;float inputVoltage2;                        // Define variable voltage divider 0, 1, & 2 - fromula (Vout = R2/(R1+R2)*Vin)
float ACVoltage0;float ACVoltage1; float ACVoltage2;                                // Define varibale ACVoltage 0, 1, & 2 - formula comparison to measure AC Voltage (AC Voltage = ACVoltage/InputVoltage)
float CallibrationFactor = 1.27;//1.19;                                                    // Callibration factor
float CallibrationFactorVmes0 = 1.95;
float CallibrationFactorVmes2 = 0.98;

float Wpress;                                                                       // Define variable Water Pressure of Turbine
float vadc;                                                                         // Define variable Vadc 
float Wpress_Bar;
//float avgGrad = 0.88780097;
//float avgGrad = 0.767796195;
//float avgGrad = 0.753783624;
float c = 0.5;                                                                      // Define constant
float avgGrad = 0.846960854;                                                        // Calibration Factor
//float avgGrad = 0.7645556;            

String inputString = "";                                                            // a String to hold incoming data
bool stringComplete = false;                                                        // whether the string is complete

int64_t oldtime;                                                                    // Millis Oldtime

void setup() {
    oldtime = millis();
    Serial.begin(9600);                                                             // Serial begin initialization
    Serial1.begin(9600);                                                            // Serial begin initialization

    pinMode(13, OUTPUT);                                                            // Initialization digital pin relay as output
    digitalWrite(13, HIGH);                                                         // Initial condition value digital pin relay

    emon0.current(0, 47);                                                           // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon1.current(1, 47);                                                           // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon2.current(2, 47);                                                           // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon3.current(3, 47);                                                           // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon4.current(4, 47);                                                           // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon5.current(5, 47);                                                           // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon6.current(6, 47);                                                           // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon7.current(7, 47);                                                           // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
    emon8.current(8, 47);                                                           // Current: input pin, calibration. Cur Const= Ratio/BurdenR. 1800/62(in resistor) + 100(ex resistor) = 47.
 } 

void loop() {
    // voltage0 measure
    adc0 = analogRead(A9);                                                      // Read adc
    voltage0 = (adc0 * 5.0) * CallibrationFactorVmes0 / 1023.0;                      // Formula read voltage from adc (voltage = adc * Vreff/1023)
    inputVoltage0 = voltage0 * (R1+R2)/R2;                                      // Voltage divider fromula (Vout = R2/(R1+R2)*Vin)
    ACVoltage0 = inputVoltage0 * (ACVoltageRefference/inputVoltageRefference);  // Formula comparison to measure AC Voltage (AC Voltage = ACVoltage/InputVoltage)
    
    // movingAvg0 method
    sum0 = sum0 - reading0[index0];
    reading0[index0] = ACVoltage0;
    sum0 = sum0 + reading0[index0];
    index0 = index0 + 1;
    if(index0 >= AvgSize){
       index0 = 0;
    }
    
    // voltage1 measure
    adc1 = analogRead(A10);                                                     // Read adc
    voltage1 = (adc1 * 5.0) * CallibrationFactor / 1023.0;                      // Formula read voltage from adc (voltage = adc * Vreff/1023)
    inputVoltage1 = voltage1 * (R1+R2)/R2;                                      // Voltage divider fromula (Vout = R2/(R1+R2)*Vin)
    ACVoltage1 = inputVoltage1 * (ACVoltageRefference/inputVoltageRefference);  // Formula comparison to measure AC Voltage (AC Voltage = ACVoltage/InputVoltage)
    
    // movingAvg1 method
    sum1 = sum1 - reading1[index1];
    reading1[index1] = ACVoltage1;
    sum1 = sum1 + reading1[index1];
    index1 = index1 + 1;
    if(index1 >= AvgSize){
       index1 = 0;
    }

    // voltage2 measure
    adc2 = analogRead(A11);                                                     // Read adc
    voltage2 = (adc2 * 5.0) * CallibrationFactorVmes2 / 1023.0;                      // Formula read voltage from adc (voltage = adc * Vreff/1023)
    inputVoltage2 = voltage2 * (R1+R2)/R2;                                      // Voltage divider fromula (Vout = R2/(R1+R2)*Vin)
    ACVoltage2 = inputVoltage2 * (ACVoltageRefference/inputVoltageRefference);  // Formula comparison to measure AC Voltage (AC Voltage = ACVoltage/InputVoltage)
    // movingAvg2 method
    sum2 = sum2 - reading2[index2];
    reading2[index2] = ACVoltage2;
    sum2 = sum2 + reading2[index2];
    index2 = index2 + 1;
    if(index2 >= AvgSize){
       index2 = 0;
    }

    if((millis()-oldtime)>=10000){
        oldtime = millis();
        // current 0 - 8 measure
        float Irms0 = emon0.calcIrms(1800);                                         // Calculate Irms0 only 
        float Irms1 = emon1.calcIrms(1800);                                         // Calculate Irms1 only   
        float Irms2 = emon2.calcIrms(1800);                                         // Calculate Irms2 only   
        float Irms3 = emon3.calcIrms(1800);                                         // Calculate Irms3 only   
        float Irms4 = emon4.calcIrms(1800);                                         // Calculate Irms4 only   
        float Irms5 = emon5.calcIrms(1800);                                         // Calculate Irms5 only   
        float Irms6 = emon6.calcIrms(1800);                                         // Calculate Irms6 only   
        float Irms7 = emon7.calcIrms(1800);                                         // Calculate Irms7 only   
        float Irms8 = emon8.calcIrms(1800);                                         // Calculate Irms8 only      

        // voltage 0 - 2 measure MvAvg
        MvAvg0 = sum0/AvgSize;                                                      // calculate MvAvg0 ACVoltage1
        MvAvg1 = sum1/AvgSize;                                                      // calculate MvAvg0 ACVoltage1
        MvAvg2 = sum2/AvgSize;                                                      // calculate MvAvg0 ACVoltage1

        // water pressure calculation
        vadc = (analogRead(12) * 5.00) / 1023;                                      // Read adc
        Wpress = avgGrad * (250*(vadc / 5) - 25) + c;                               // Formula read pressure
        if ( Wpress < 0){                                                           // Decision pressure < 0
            Wpress = 0;                                                             // Default pressure = 0
        }
        
        Wpress_Bar = Wpress / 14.513;
        
        // output data for DEBUG and correction
        #ifdef DEBUG
        Serial.print("|Irms0=");Serial.print(Irms0);                                // Irms0  
        Serial.print("|Irms1=");Serial.print(Irms1);                                // Irms1 
        Serial.print("|Irms2=");Serial.print(Irms2);                                // Irms2 
        Serial.print("|Irms3=");Serial.print(Irms3);                                // Irms3 
        Serial.print("|Irms4=");Serial.print(Irms4);                                // Irms4 
        Serial.print("|Irms5=");Serial.print(Irms5);                                // Irms5 
        Serial.print("|Irms6=");Serial.print(Irms6);                                // Irms6 
        Serial.print("|Irms7=");Serial.print(Irms7);                                // Irms7 
        Serial.print("|Irms8=");Serial.println(Irms8);                              // Irms8  
        Serial.println("----------------------------------------------------------------------------------------");
        Serial.print("|adc0=");Serial.print(adc0);                                
        Serial.print("|adcVoltage0=");Serial.print(voltage0);
        Serial.print("|inputVoltage0=");Serial.print(inputVoltage0);
        Serial.print("|ACVoltage0=");Serial.println(MvAvg0);                      
        Serial.print("|adc1=");Serial.print(adc1);
        Serial.print("|adcVoltage1=");Serial.print(voltage1);
        Serial.print("|inputVoltage1=");Serial.print(inputVoltage1);
        Serial.print("|ACVoltage1=");Serial.println(MvAvg1);
        Serial.print("|adc2=");Serial.print(adc2);
        Serial.print("|adcVoltage2=");Serial.print(voltage2);
        Serial.print("|inputVoltage2=");Serial.print(inputVoltage2);
        Serial.print("|ACVoltage2=");Serial.println(MvAvg2);
        Serial.println("----------------------------------------------------------------------------------------");
        Serial.print("adc=");Serial.println(vadc);
        Serial.print("WaterPressure=");Serial.println(Wpress_Bar);
        Serial.println("----------------------------------------------------------------------------------------");
        #endif

        // output data send to Rpi
        #ifdef RELEASE
        Serial1.print("$");
        Serial1.print(Irms0);Serial1.print(";");
        Serial1.print(Irms1);Serial1.print(";");
        Serial1.print(Irms2);Serial1.print(";");
        Serial1.print(Irms3);Serial1.print(";");
        Serial1.print(Irms4);Serial1.print(";");
        Serial1.print(Irms5);Serial1.print(";");
        Serial1.print(Irms6);Serial1.print(";");
        Serial1.print(Irms7);Serial1.print(";");
        Serial1.print(Irms8);Serial1.print(";");
        Serial1.print(MvAvg0);Serial1.print(";");
        Serial1.print(MvAvg1);Serial1.print(";");
        Serial1.print(MvAvg2);Serial1.print(";");
        Serial1.print(Wpress_Bar);Serial1.print(";");
        Serial1.print("\n");
        #endif
    } else {
        if((millis()-oldtime)<=0){
            oldtime=0;                                                              // Reset variable oldtime to zero;              
        }
    }

    // Drive relay for Wifi indicator
    if (stringComplete){
        Serial.println(inputString); 
        // clear the string:
        if (inputString == "w0nl1n3\n"){
            digitalWrite(13, LOW);
            inputString = "";
            stringComplete = false; 
        }else{ 
            if (inputString == "w0ffl1n3\n"){
                digitalWrite(13, HIGH);
                inputString = "";
                stringComplete = false; 
            }
        }
    }
}


// SerialEvent
void serialEvent1() {
while (Serial.available()) {
// get the new byte:
  char inChar = (char)Serial.read();
  // add it to the inputString:
  inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
