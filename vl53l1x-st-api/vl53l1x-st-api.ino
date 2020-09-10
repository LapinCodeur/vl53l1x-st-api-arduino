/*******************************************************************************

This sketch file is derived from an example program
(Projects\Multi\Examples\VL53L1X\SimpleRangingExamples\Src\main.c) in the
X-CUBE-53L1A1 Long Distance Ranging sensor software expansion for STM32Cube
from ST, available here:

http://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube-expansion-software/stm32-ode-sense-sw/x-cube-53l1a1.html

The rest of the files in this sketch are from the STSW-IMG007 VL53L1X API from
ST, available here:

http://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img007.html

It is also using fonctions from the ultra lite driver and implement STSW-IMG017 following the AN5191 application note.

********************************************************************************

COPYRIGHT(c) 2017 STMicroelectronics
COPYRIGHT(c) 2018 Pololu Corporation

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of STMicroelectronics nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include <Wire.h>
#include "vl53l1_api.h"


// Timing budget set through VL53L1_SetMeasurementTimingBudgetMicroSeconds().
#define MEASUREMENT_BUDGET_MS 20//20

// Interval between measurements, set through
// VL53L1_SetInterMeasurementPeriodMilliSeconds(). According to the API user
// manual (rev 2), "the minimum inter-measurement period must be longer than the
// timing budget + 4 ms."
#define INTER_MEASUREMENT_PERIOD_MS 24//33
#define TotalWidthOfSPADS           16
#define WidthOfSPADsPerZone           4
#define NumOfSPADsShiftPerZone          1
#define HorizontalFOVofSensor         19.09
#define SingleSPADFOV             (HorizontalFOVofSensor/TotalWidthOfSPADS)
#define NumOfZonesPerSensor           (((TotalWidthOfSPADS - WidthOfSPADsPerZone) / NumOfSPADsShiftPerZone) + 1)
#define StartingZoneAngle           (WidthOfSPADsPerZone / 2 * SingleSPADFOV)
#define ZoneFOVChangePerStep          (SingleSPADFOV * NumOfSPADsShiftPerZone)

VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;
uint16_t zone_center[]={247,239,231,223,215,207,199,191,183,175,167,159,151};
float    LidarAngle[13];
uint16_t LidarDistance[13];
int status;

void setup()
{
  uint8_t byteData;
  uint16_t wordData;

  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  delay(2000);

  // This is the default 8-bit slave address (including R/W as the least
  // significant bit) as expected by the API. Note that the Arduino Wire library
  // uses a 7-bit address without the R/W bit instead (0x29 or 0b0101001).
  Dev->I2cDevAddr = 0x52;

  VL53L1_software_reset(Dev);

  VL53L1_RdByte(Dev, 0x010F, &byteData);
  Serial.print(F("VL53L1X Model_ID: "));
  Serial.println(byteData, HEX);
  VL53L1_RdByte(Dev, 0x0110, &byteData);
  Serial.print(F("VL53L1X Module_Type: "));
  Serial.println(byteData, HEX);
  VL53L1_RdWord(Dev, 0x010F, &wordData);
  Serial.print(F("VL53L1X: "));
  Serial.println(wordData, HEX);

  Serial.println(F("Autonomous Ranging Test"));
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  status = VL53L1_StaticInit(Dev);
  Serial.print(F("VL53L1_SetDistanceMode : "));
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);
  Serial.println(status);
  Serial.print(F("VL53L1_SetMeasurementTimingBudgetMicroSeconds : "));
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, MEASUREMENT_BUDGET_MS * 1000);
  Serial.println(status);
  Serial.print(F("VL53L1_SetInterMeasurementPeriodMilliSeconds : "));
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, INTER_MEASUREMENT_PERIOD_MS);
  Serial.println(status);
  Serial.print(F("VL53L1_SetROI : "));
  status = VL53L1_SetROI(Dev, WidthOfSPADsPerZone, TotalWidthOfSPADS);
  Serial.println(status);
  Serial.print(F("VL53L1_SetROICenter : "));
  status = VL53L1_SetROICenter(Dev, zone_center[0]);
  Serial.println(status);
  Serial.print(F("VL53L1_StartMeasurement : "));
  status = VL53L1_StartMeasurement(Dev);
  Serial.println(status);

  if(status)
  {
    Serial.println(F("VL53L1_StartMeasurement failed"));
    while(1);
  }
}

void loop()
{
  int exectime = millis();
  int coord[13][2];
  
  for(int Zone = 0; Zone < NumOfZonesPerSensor; Zone)
  {
    VL53L1_ClearInterrupt(Dev);
    status = VL53L1_SetROI(Dev, WidthOfSPADsPerZone, TotalWidthOfSPADS);
    status = VL53L1_SetROICenter(Dev, zone_center[Zone]);
    /*Serial.print(F("VL53L1_SetROICenter : "));
    uint8_t ROIcenter;
    VL53L1_GetROICenter(Dev, &ROIcenter);
    Serial.println(ROIcenter);
    Serial.print(F("VL53L1_SetROI : "));
    uint16_t ROI_x;
    uint16_t ROI_y;
    VL53L1_GetROI_XY(Dev, &ROI_x, &ROI_y);
    Serial.print(ROI_x);
    Serial.print("\t");
    Serial.println(ROI_y);*/
    
    // blocking wait for data ready
    status = VL53L1_WaitMeasurementDataReady(Dev);

    if(!status)
    {
      //printRangingData();
      stockData(Zone);
      Zone++;
    }
    else
    {
      Serial.print(F("Error waiting for data ready: "));
      Serial.println(status);
    }
  }
  
  getXYpos(coord);
  
  for(int zonetoprint = 0; zonetoprint < NumOfZonesPerSensor; zonetoprint++)
  {
    Serial.print(coord[zonetoprint][0]);
    Serial.print(" , ");
    Serial.print(coord[zonetoprint][1]);
    Serial.print("\t");
  }
  exectime = millis() - exectime;
  Serial.print("time : ");
  Serial.print(exectime);
  Serial.println(" ms");
  Serial.println();
}

//Acquire angle and distance for the current zone
void stockData(uint8_t CurrentZone)
{
  static VL53L1_RangingMeasurementData_t RangingData;
  double PartZoneAngle;
  unsigned int Distance = 0;

  status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
  if(!status)
  {
    Distance = RangingData.RangeMilliMeter;

    if (Distance > 60000)
    {
      Distance = 0;
    }
    PartZoneAngle = (StartingZoneAngle + ZoneFOVChangePerStep*CurrentZone) - (HorizontalFOVofSensor / 2.0);
    LidarAngle[CurrentZone] = PartZoneAngle;
    LidarDistance[CurrentZone] = Distance;
  }
}

//Convert angular and distance data into cartesian data
void getXYpos(int coord[][2])
{
  for(int Zone = 0; Zone < NumOfZonesPerSensor; Zone++)
  {
    coord[Zone][0] = cos((LidarAngle[Zone] / 180) * PI) * LidarDistance[Zone];
    coord[Zone][1] = sin((LidarAngle[Zone] / 180) * PI) * LidarDistance[Zone];
  }
}
