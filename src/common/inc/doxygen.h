/*******************************************************************************
 * @file 	doxygen.h
 * @brief 	front page of the doxygen
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/


#ifndef DOXYGEN_H_
#define DOXYGEN_H_

/**
\mainpage CrossRFID &reg;
\section overview
This project is the firmware of the CrossRFID &reg; device coupled with a SPI accelerometer. it includes an RTOS (FreeRTOS) manages 3 tasks :
	- RF task
	- serial task
	- Kernel

The RF task manages the communications with the RF front end. <br>
The serial interface manages the communications with the serial devices. <br>
The kernel is at the center of the OS. it receives the messages from the others tasks and forward them to the right task. <br>

The firmware support several applications:

<table>
<caption id="multi_row">Firmware applications</caption>
<tr><th>Application<th>Flag for the compilation
<tr><td>Activity Counter<td>1
<tr><td>Gluepot<td>2
<tr><td>Nanolike<td>3
</table>

\section version
<table>
<caption id="multi_row">version</caption>
<tr><th>version                      <th>change        <th>date
<tr><td>1.0<td>initial release<td>february 2016
<tr><td>1.1<td> add the management of the alarm <td>
<tr><td>   <td> add the calander feature <td>
<tr><td>1.2<td>  <td>

</table>

see the CHANGELOG.md
\section license

\section sensor
The accelerometer is an ADXL345 from analog devices. It works as a SPI slave. <br>
see its datasheet for further details<br>
http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf

\section RTOS
The RTOS is FreeRTOS. please refer the following web site for description and support<br>
http://www.freertos.org/index.html



@page Description of the tasks
@section overview
the application contains 3 tasks.
 - Kernel : broadcast the message between the others tasks
 - Sensor Interface Task (SIT) : manage the communication with the sensor
 - Sensor Interface Task (SIT) : manage the communication with the serial interface
 - RF task  : manage the RF task
<br>
@section S1 sensor measurement finished
When the sensor sends a DataReady Irq the following messages will be exchanged between the different tasks
\msc
  Sensor,SIT,Kernel,RFtask;
  Sensor->SIT [label="DataReady Irq"];
  SIT->Sensor [label="read measurement"];
  SIT=>SIT [label="lock buffer"];
  SIT->Kernel [label="datareceived"];
  Kernel->RFtask [label="datareceived"];
  Kernel=>Kernel [label="take mutex"];
  Kernel->RFtask [label="datareceived"];
  RFtask=>RFtask [label="write in the RFtask's user memory"];
  RFtask->Kernel [label="releasebuffer"];
  Kernel=>Kernel [label="give mutex"];
  Kernel->SIT [label="releasebuffer"];
  SIT=>SIT [label="unlock buffer"];
\endmsc

@section S2 when RSSI > threshold
When the RF field strength (RSSI) is above the threshold the following messages will be exchanged
\msc
  LNAtask,Kernel,RFtask;
  LNAtask->Kernel [label="RFON"];
  Kernel=>Kernel [label="save RF field status"];
  Kernel->RFtask [label="RFON"];
  RFtask=>RFtask [label="enable RFtask device"];
\endmsc
@section S3 when RSSI < threshold
When the RF field strength (RSSI) is below the threshold the following messages will be exchanged
\msc
  LNAtask,Kernel,RFtask;
  LNAtask->Kernel [label="RFRESET"];
  Kernel->RFtask [label="RFRESET"];
  RFtask=>RFtask [label="reset RFtask"];
  Kernel->RFtask [label="load new config"];
  LNAtask->Kernel [label="RFOFF"];
  Kernel=>Kernel [label="save RF field status"];
  Kernel->RFtask [label="RFROFF"];
  RFtask=>RFtask [label="disable RFtask  device"];
\endmsc
 <OL>
 <LI>
 </OL>
*/

/******** THE REST OF THE FILE IS DOCUMENTATION ONLY !**********************//**
 * @{

@page System_file
	Overview:

	@li @ref Introduction
	@li @ref Mapping
	@li @ref Data integrity
	@li @ref Access to the memory

@n @section System_file introduction

the system file gives is used to the data exchnge betwenn the RF front end and the MCU

@n @section System_file mapping

The system file is an memory area mapped in the MCU shared betwen the different interfaces

<table>
<caption id="multi_row">24LC64 Mapping</caption>
<tr><th> Byte address <th> Word description  <th> right access
<tr><td> 0x00 <td> WhoAreYou?  	<td> R/W
<tr><td> 0x01 <td> HowAreyou? <td> R
<tr><td> 0x02 <td>
<tr><td> 0x03 <td> HowWarmIsIt? <td> R
<tr><td> 0x04 <td> Whattimeisit? #1  <td> R
<tr><td> 0x05 <td> Whattimeisit? #2  <td> R
<tr><td> 0x06 <td> Whatdateisit? #1 <td> R
<tr><td> 0x07 <td> Whatdateisit? #2 <td> R
<tr><td> 0x08 <td> ACQMode <td> R/W
<tr><td> 0x09 <td> ACQPeriod <td> R/W
<tr><td> 0x0A <td> ACQBegin <td> R/W
<tr><td> 0x0B <td> ACQEnd <td> R/W
<tr><td> 0x0C <td> ACQThresholds  <td> R/W
<tr><td> 0x0D <td> EnableAlarm <td> R/W
<tr><td> 0x0E <td> Areyouwarningme?  <td> R
<tr><td> 0x0F <td> ResetAlarm <td> R/W
<tr><td> 0x10 <td> DateTimeAlarm <td> R/W
<tr><td> 0x11 <td>  <td> R/W
<tr><td> 0x12 <td>  <td> R/W
<tr><td> 0x13 <td>  <td> R/W
<tr><td> 0x14 <td> Comebacktolife <td> W
<tr><td> 0x15 <td> IsDataAvailable?  <td> R/W
<tr><td> 0x16 <td> GiveMeSamples  <td> R
<tr><td> 0x15 <td> IdxSample  <td> R/W
<tr><td> 0x15 <td> NbSamplesToGive <td> R/W
<tr><td> 0x15 <td> SelectMemory  <td> R/W
<tr><td> 0x15 <td>   <td> R/W


</table>

@n @section tempdrv_crc Data integrity

The stored data in the memory are protected by a CRC CCITT16bit.<br>
It is computed by the GPCRC module of the microcontroller.<br>
The polynomial is 0x1021.<br>

@n @section tempdrv_access Access to the memory

Serial is the task that interacts with the 24LC64 via the I2C 0.<br>
The firmware always checks the CRC after reading the memory.<br>
Similarly, the crc is computed to replace the old crc when data must be written.<br>
Write cycle time (byte or page): 2ms (typical), 5ms (max)<br>

 * @}**************************************************************************/




#endif /* DOXYGEN_H_ */
