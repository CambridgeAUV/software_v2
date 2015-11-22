#include "commandsCalib.h"
#include "../sbgCom.h"

//----------------------------------------------------------------------//
//  Calibration commands                                                //
//----------------------------------------------------------------------//

/*!
 *	Command used to start/stop/save a magnetometer calibration procedure.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	argument				Define the action we would like to execute.<br>
 *										Possible values are:<br>
 *										- SBG_CALIB_LOAD_DEFAULT<br>
 *										- SBG_CALIB_START_2D<br>
 *										- SBG_CALIB_START_3D<br>
 *										- SBG_CALIB_STOP<br>
 *										- SBG_CALIB_SAVE<br>
 *	\return								SBG_NO_ERROR if the action has been executed sucessfully.
 */
SbgErrorCode sbgCalibMagnetometers(SbgProtocolHandle handle, SbgCalibAction argument)
{
	uint8 buffer;
	SbgErrorCode error;

	//
	// Check argument before sending it
	//
	if ((argument >= SBG_CALIB_LOAD_DEFAULT) && (argument <= SBG_CALIB_SAVE) )
	{
		//
		// Build our frame
		//
		buffer = argument;

		//
		// Send the command used to save current gyro bias
		//
		error = sbgProtocolSend(handle, SBG_CALIB_MAG, &buffer, sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			//
			// We should receive an ACK, wait for 2 seconds to let the device compute the calibration
			//
			error = sbgWaitForAck(handle, 2000);
		}
	}
	else
	{
		error = SBG_INVALID_FRAME;
	}

	return error;
}

/*!
 *	Define our current magnetometer calibration parameters.<br>
 *	This command is used, for example, by the sbgCenter when the magnetometer calibration is exectured on the computer.<br> 
 *	When this command is called, the new settings are only stored in volatile memory.<br>
 *	To save this calibration permanently, use sbgCalibMagnetometers with SBG_CALIB_SAVE argument.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	offset					X,Y,Z offset of the magnetic field.
 *	\param[in]	crossAxis				3x3 matrix containing the rotation and deformation of the magnetic field.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCalibMagnetometersSetTransformations(SbgProtocolHandle handle, const float offset[3], const float crossAxis[9])
{
	uint8 buffer[3*sizeof(uint32) + 9*sizeof(uint32)];
	SbgErrorCode error = SBG_NO_ERROR;
	uint32 *outOffset;
	uint32 *outCrossAxis;
	uint16 i;

	//
	// Build our buffer, offset part
	//
	outOffset = (uint32*)(buffer);

	outOffset[0] = sbgHostToTargetFloat(handle->targetOutputMode, offset[0]);
	outOffset[1] = sbgHostToTargetFloat(handle->targetOutputMode, offset[1]);
	outOffset[2] = sbgHostToTargetFloat(handle->targetOutputMode, offset[2]);

	//
	// Build our buffer, cross-axis matrix part
	//
	outCrossAxis = (uint32*)(buffer + 3*sizeof(uint32));
	for (i=0;i<9;i++)
	{
		outCrossAxis[i] = sbgHostToTargetFloat(handle->targetOutputMode, crossAxis[i]);
	}

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_CALIB_MAG_SET_MANUAL, buffer, 3*sizeof(uint32) + 9*sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// We should receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

/*!
 *	Returns the current magnetometer calibration parameters.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	offset					X,Y,Z offset of the magnetic field.
 *	\param[out]	crossAxis				3x3 matrix containing the rotation and deformation of the magnetic field.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCalibMagnetometersGetTransformations(SbgProtocolHandle handle, float offset[3], float crossAxis[9])
{
	uint8 cmd;
	uint16 size;
	uint8 buffer[3*sizeof(uint32) + 9*sizeof(uint32)];
	SbgErrorCode error;
	uint32 i;

	uint32 *magOffset;
	uint32 *magCrossAxis;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_CALIB_MAG_GET_TRANSFORMATIONS, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, buffer, &size, 3*sizeof(uint32) + 9*sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_CALIB_MAG_RET_TRANSFORMATIONS) && (size == 3*sizeof(uint32) + 9*sizeof(uint32)) )
			{
				//
				// Returns, if possible, the offset vector
				//
				if (offset)
				{
					magOffset = (uint32*)buffer;
					
					offset[0] = sbgTargetToHostFloat(handle->targetOutputMode, magOffset[0]);
					offset[1] = sbgTargetToHostFloat(handle->targetOutputMode, magOffset[1]);
					offset[2] = sbgTargetToHostFloat(handle->targetOutputMode, magOffset[2]);
				}

				//
				// Returns, if possible, the cross axis matrix
				//
				if (crossAxis)
				{
					magCrossAxis = (uint32*)(buffer + 3*sizeof(uint32));

					for (i=0;i<9;i++)
					{
						crossAxis[i] = sbgTargetToHostFloat(handle->targetOutputMode, magCrossAxis[i]);
					}
				}
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}

/*!
 *	Acquiere and save the current gyroscope bias value.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	argument				Define the action we would like to execute.<br>
 *										Possible values are:<br>
 *										- SBG_CALIB_LOAD_DEFAULT<br>
 *										- SBG_CALIB_MEASURE<br>
 *										- SBG_CALIB_SAVE<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCalibGyroBias(SbgProtocolHandle handle, SbgCalibAction argument)
{
	uint8 data;
	uint32 timeOut = SBG_FRAME_RECEPTION_TIME_OUT;
	SbgErrorCode error = SBG_NO_ERROR;

	//
	// Check the time to receive the answer depending on the parameter.
	// Put an error if an unknown parameter is set
	//
	switch (argument)
	{
		case SBG_CALIB_LOAD_DEFAULT:
		case SBG_CALIB_SAVE:
		case SBG_CALIB_MEASURE_COARSE:
			timeOut = SBG_FRAME_RECEPTION_TIME_OUT;
			break;

		case SBG_CALIB_MEASURE_MEDIUM:
			timeOut = 1500;
			break;

		case SBG_CALIB_MEASURE_FINE:
			timeOut = 3500;
			break;

		default:
			error = SBG_INVALID_PARAMETER;
			break;
	}
	
	if (error == SBG_NO_ERROR)
	{
		//
		// Cast our action into a uint8 before transmitting it
		//
		data = (uint8)argument;

		//
		// Send the command used to load/measure/save current gyro bias
		//
		error = sbgProtocolSend(handle, SBG_CALIB_GYRO_BIAS, &data, sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			//
			// We should receive an ACK
			//
			error = sbgWaitForAck(handle, timeOut);
		}
	}

	return error;
}
