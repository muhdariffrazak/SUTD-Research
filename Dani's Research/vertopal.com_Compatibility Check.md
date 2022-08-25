Compatibility Check

Thursday, 25 August 2022

11:28 am

Compatibility check for the suggested Servo to be used to upscale the
size of the project, to support the weight of the additional sensors

<table>
<thead>
<tr class="header">
<th>Item</th>
<th>CM_530</th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td>Weight</td>
<td>54g</td>
</tr>
<tr class="even">
<td>CPU</td>
<td>STM32F103RE</td>
</tr>
<tr class="odd">
<td>Voltage Supply</td>
<td><p>Range : 6V ~ 15V</p>
<p>Recommended : 11.1V (3S1P Li-Po)</p></td>
</tr>
<tr class="even">
<td>Current Consumption</td>
<td><p>Standby : 50mA</p>
<p>External I/O Max Current : 300mA</p>
<p>Total Max Current : 10A(Fuse)</p></td>
</tr>
<tr class="odd">
<td>Internal IO Device</td>
<td><p>Button x 5(Reset 1, Port 5)</p>
<p>MIC for sound detection x 1</p>
<p>Voltage Sensor x 1</p></td>
</tr>
<tr class="even">
<td>External IO Device</td>
<td><p>ROBOTIS 5-Pin Port x 6</p>
<p>AX/MX Series DYNAMIXEL Connector x 5</p></td>
</tr>
</tbody>
</table>

 

**Connection to servos** for the CM-530 is done in a DAISY CHAIN METHOD
be it the AX/MX Series

>  

**Area of Concern** It is used to set or test the operations of CM-530
and AX-12A using [RoboPlus
Manager](https://emanual.robotis.com/docs/en/software/rplus1/manager/).
Are we planning to use or using the RoboPlus Manager to test AX-12A for
our project?

* *

>  

**Similarities of AX & MX series;**

-   Both run on the same communication protocol, DYNAMIXEL Protocol 1.0

-   Both use 3pin Molex Connector for their physical connection

 

 

**Conclusion:**

-   Both the motors are compatible with the CM\_530 controller.

-   Need to actually test out if the AX-12A(OLD) can run alongside
    MX-64T (IN LAB)

-   No existing documentation if the CM\_530 can run both AX-12A and
    MX-64T at the same time

 

 

 

 **References:**

 

Documentation for CM-530:

<https://emanual.robotis.com/docs/en/parts/controller/cm-530/>

 

Harness compatibility guide:

<https://emanual.robotis.com/docs/en/popup/cable_compatibility/>

 

DYNAMIXEL product compatibility guide:

<https://emanual.robotis.com/docs/en/faq/faq_dynamixel/>

 

DYNAMIXEL robotis Products Compatibility Table:

<https://emanual.robotis.com/docs/en/popup/faq_protocol_compatibility_table/>  
  
 

 
