#include "commandsFilter.h"
#include "../sbgCom.h"

/*!
 *	Defines our Kalman Filter errors coefficiens.<br>
 *	Using this command, you can balance between accelerometers/gyroscopes and magnetometers/gyroscopes<br>
 *	as the attitude estimation is concerned.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	accelerometersError		Errors between 0.1 to 10.0 associated with our accelerometers.<br>
 *										If this settings is set to 0.1, the device will be more accurate in static conditions but less in dynamic one.<br>
 *										With a setting near 10.0, the device will be less sensitive to transients accelerations in dynamic conditions<br>
 *	\param[in]	magnetometersError		Errors between 0.1 to 10.0 associated with our magnetometers.<br>
 *										If this settings is set to 0.1, the device will be more accurate in static conditions but less in dynamic one.<br>
 *										With a setting near 10.0, the device will be less sensitive to magnetic disturbances and in dynamic conditions<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterAttitudeCoef(SbgProtocolHandle handle, bool permanent, float accelerometersError, float magnetometersError)
{
	uint8 buffer[sizeof(uint8) + 2*sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build our arguments buffer
	//
	*buffer = permanent;
	*(uint32*)(buffer + sizeof(uint8)) = sbgHostToTargetFloat(handle->targetOutputMode,accelerometersError);
	*(uint32*)(buffer + sizeof(uint8) + sizeof(uint32)) = sbgHostToTargetFloat(handle->targetOutputMode,magnetometersError);

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_SET_FILTER_ATTITUDE_SENSORS_ERR, buffer, sizeof(uint8) + 2*sizeof(uint32));

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
 *	Retreives our Kalman Filter errors coefficiens.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pAccelerometersError	Errors between 0.1 to 10.0 associated with our accelerometers.<br>
 *										If this settings is set to 0.1, the device will be more accurate in static conditions but less in dynamic one.<br>
 *										With a setting near 10.0, the device will be less sensitive to transients accelerations in dynamic conditions<br>
 *	\param[out]	pMagnetometersError		Errors between 0.1 to 10.0 associated with our magnetometers.<br>
 *										If this settings is set to 0.1, the device will be more accurate in static conditions but less in dynamic one.<br>
 *										With a setting near 10.0, the device will be less sensitive to magnetic disturbances and in dynamic conditions<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterAttitudeCoef(SbgProtocolHandle handle, float *pAccelerometersError, float *pMagnetometersError)
{
	uint8 cmd;
	uint16 size;
	uint32 settings[2];
	SbgErrorCode error;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_GET_FILTER_ATTITUDE_SENSORS_ERR, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, (uint8*)settings, &size, 2*sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_FILTER_ATTITUDE_SENSORS_ERR) && (size == 2*sizeof(uint32)) )
			{
				//
				// Returns, if possible, our accelerometers error param.
				//
				if (pAccelerometersError)
				{
					*pAccelerometersError = sbgTargetToHostFloat(handle->targetOutputMode, settings[0]);
				}

				//
				// Returns, if possible, our magnetomters error param.
				//
				if (pMagnetometersError)
				{
					*pMagnetometersError = sbgTargetToHostFloat(handle->targetOutputMode, settings[1]);
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
 *	Defines some options regarding the Kalman Filter.<br>
 *	It's possible, for example, to enable/disable the attitude computation or to enable/disable gyro-bias estimation.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	filterOptions			Our Kalman filter options mask.<br>
 *										Possible values are:<br>
 *										- #SBG_FILTER_OPTION_FILTER_ACCELERATIONS
 *										- #SBG_FILTER_OPTION_FILTER_MAG_DISTORTIONS
 *										- #SBG_FILTER_OPTION_ESTIMATE_GYRO_BIAS
 *										- #SBG_FILTER_OPTION_ESTIMATE_ACCEL_BIAS
 *										- #SBG_FILTER_OPTION_ENABLE_ATTITUDE
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterAttitudeOptions(SbgProtocolHandle handle, bool permanent, uint32 kalmanMode)
{
	uint8 buffer[sizeof(uint8) + sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build our arguments buffer
	//
	*buffer = permanent;
	*(uint32*)(buffer + sizeof(uint8)) = sbgHostToTarget32(handle->targetOutputMode,kalmanMode);

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_SET_FILTER_ATTITUDE_OPTIONS, buffer, sizeof(uint8) + sizeof(uint32));

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
 *	Retreives some options regarding the Kalman Filter.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pFilterOptions			Our Kalman filter options mask.<br>
 *										Possible values are:<br>
 *										- #SBG_FILTER_OPTION_FILTER_ACCELERATIONS
 *										- #SBG_FILTER_OPTION_FILTER_MAG_DISTORTIONS
 *										- #SBG_FILTER_OPTION_ESTIMATE_GYRO_BIAS
 *										- #SBG_FILTER_OPTION_ESTIMATE_ACCEL_BIAS
 *										- #SBG_FILTER_OPTION_ENABLE_ATTITUDE
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterAttitudeOptions(SbgProtocolHandle handle, uint32 *pFilterOptions)
{
	uint8 cmd;
	uint16 size;
	uint32 buffer;
	SbgErrorCode error;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_GET_FILTER_ATTITUDE_OPTIONS, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, (uint8*)&buffer, &size, sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_FILTER_ATTITUDE_OPTIONS) && (size == sizeof(uint32)) )
			{
				//
				// Returns, if possible, our Kalman filter options
				//
				if (pFilterOptions)
				{
					*pFilterOptions = sbgTargetToHost32(handle->targetOutputMode, buffer);
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
 *	Defines our sensors filter cut-off frequencies and the update rate for the Kalman Filter.<br>
 *	If you set a setting to 0.0, the value will remain unchanged.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	gyroAccelsSampling		Reserved for backward compatibility. Leave to 0.
 *	\param[in]	cutoffGyro				Gyroscopes low-pass filter cut-off frequency in Hz.
 *	\param[in]	cutoffAccel				Accelerometers low-pass filter cut-off frequency in Hz.
 *	\param[in]	cutoffMagneto			Magnetometers low-pass filter cut-off frequency in Hz.
 *	\param[in]	kalmanFreq				Our Kalman filter refresh rate.<br>
 *										Max 100 Hz for IG-500N and 160 Hz for IG-500A.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterFrequencies(SbgProtocolHandle handle, bool permanent, float gyroAccelsSampling, float cutoffGyro, float cutoffAccel, float cutoffMagneto, float kalmanFreq)
{
	uint8 buffer[sizeof(uint8) + 5*sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build our arguments buffer
	//
	*buffer = permanent;
	*(uint32*)(buffer + sizeof(uint8))                    = sbgHostToTargetFloat(handle->targetOutputMode, gyroAccelsSampling);
	*(uint32*)(buffer + sizeof(uint8) +   sizeof(uint32)) = sbgHostToTargetFloat(handle->targetOutputMode, cutoffGyro);
	*(uint32*)(buffer + sizeof(uint8) + 2*sizeof(uint32)) = sbgHostToTargetFloat(handle->targetOutputMode, cutoffAccel);
	*(uint32*)(buffer + sizeof(uint8) + 3*sizeof(uint32)) = sbgHostToTargetFloat(handle->targetOutputMode, cutoffMagneto);
	*(uint32*)(buffer + sizeof(uint8) + 4*sizeof(uint32)) = sbgHostToTargetFloat(handle->targetOutputMode, kalmanFreq);

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_SET_FILTER_FREQUENCIES, buffer, sizeof(uint8) + 5*sizeof(uint32));

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
 *	Retrives our sensors filter cut-off frequencies and the update rate for the Kalman Filter.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pGyroAccelsSampling		Our accelerometers and gyroscopes sampling frequency in Hz.
 *	\param[out]	pCutoffGyro				Gyroscopes low-pass filter cut-off frequency in Hz.
 *	\param[out]	pCutoffAccel			Accelerometers low-pass filter cut-off frequency in Hz.
 *	\param[out]	pCutoffMagneto			Magnetometers low-pass filter cut-off frequency in Hz.
 *	\param[out]	pKalmanFreq				Our Kalman filter refresh rate.<br>
 *										Max 100 Hz for IG-500N and 160 Hz for IG-500A.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterFrequencies(SbgProtocolHandle handle, float *pGyroAccelsSampling, float *pCutoffGyro, float *pCutoffAccel, float *pCutoffMagneto, float *pKalmanFreq)
{
	uint8 cmd;
	uint16 size;
	uint32 frequencies[5];
	SbgErrorCode error;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_GET_FILTER_FREQUENCIES, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, (uint8*)frequencies, &size, 5*sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_FILTER_FREQUENCIES) && (size == 5*sizeof(uint32)) )
			{
				//
				// Returns, if possible, our frequencies settings
				//
				if (pGyroAccelsSampling)
				{
					*pGyroAccelsSampling = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[0]);
				}
				if (pCutoffGyro)
				{
					*pCutoffGyro = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[1]);
				}
				if (pCutoffAccel)
				{
					*pCutoffAccel = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[2]);
				}
				if (pCutoffMagneto)
				{
					*pCutoffMagneto = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[3]);
				}
				if (pKalmanFreq)
				{
					*pKalmanFreq = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[4]);
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
 *	Defines the kalman filter source for heading estimate.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	source					Source used in heading calculation.<br>
 *										Possible values are:<br>
 *										- SBG_HEADING_SOURCE_NONE, unavailable for IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_MAGNETOMETERS<br>
 *										- SBG_HEADING_SOURCE_GPS, only for IG-30G and IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_EXTERNAL<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterHeadingSource(SbgProtocolHandle handle, bool permanent, SbgHeadingSource source)
{
	uint8 buffer[sizeof(uint8) + sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build our arguments buffer
	//
	buffer[0] = permanent;
	buffer[1] = (uint8)source;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_SET_FILTER_HEADING_SOURCE, buffer, 2*sizeof(uint8));

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
 *	Retrives the kalman filter source for heading estimate.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pSource					Source used in heading calculation.<br>
 *										Possible values are:<br>
 *										- SBG_HEADING_SOURCE_NONE, unavailable for IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_MAGNETOMETERS<br>
 *										- SBG_HEADING_SOURCE_GPS, only for IG-30G and IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_EXTERNAL<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterHeadingSource(SbgProtocolHandle handle, SbgHeadingSource *pSource)
{
	uint8 cmd;
	uint16 size;
	uint8 buffer;
	SbgErrorCode error;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_GET_FILTER_HEADING_SOURCE, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &buffer, &size, sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_FILTER_HEADING_SOURCE) && (size == sizeof(uint8)) )
			{
				//
				// Returns, if possible, our heading source
				//
				if (pSource)
				{
					*pSource = (SbgHeadingSource)buffer;
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
 *	Defines the magnetic declination in radians.<br>
 *	The declination is important for good results in navigation estimation with IG-500N devices.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	permanent				If true, stores this settings in flash memory.
 *	\param[in]	declination				The local magnetic declination in radians from -PI to +PI.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetMagneticDeclination(SbgProtocolHandle handle, bool permanent, float declination)
{
	uint8 buffer[sizeof(uint8) + sizeof(uint32)];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build our arguments buffer
		//
		*(uint8*)(buffer)					= permanent;
		*(uint32*)(buffer + sizeof(uint8))	= sbgHostToTargetFloat(handle->targetOutputMode, declination);

		//
		// Send the command used to define the magnectic declination
		//
		error = sbgProtocolSend(handle, SBG_SET_MAGNETIC_DECLINATION, buffer, sizeof(uint8)+sizeof(uint32));

		//
		// Check if we have sent our frame successfully
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// We should receive an ACK
			//
			error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Returns the magnetic declination in radians.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pDeclination			The local magnetic declination in radians from -PI to +PI.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetMagneticDeclination(SbgProtocolHandle handle, float *pDeclination)
{
	uint8 cmd;
	uint16 size;
	uint32 buffer;
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Send the command used to get the current magnetic declination
		//
		error = sbgProtocolSend(handle, SBG_GET_MAGNETIC_DECLINATION, NULL, 0);

		//
		// Check if we have sent our frame successfully
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Try to read our answer
			//
			error = sbgProtocolReceiveTimeOut(handle, &cmd, (uint8*)&buffer, &size, sizeof(uint32));

			//
			// Check if we have received our frame successfully
			//
			if (error == SBG_NO_ERROR)
			{
				if ( (cmd == SBG_RET_MAGNETIC_DECLINATION) && (size == sizeof(uint32)) )
				{
					//
					// Returns, if possible, our magnetic declination
					//
					if (pDeclination)
					{
						*pDeclination = sbgTargetToHostFloat(handle->targetOutputMode, buffer);
					}
				}
				else
				{
					error = SBG_INVALID_FRAME;
				}
			}
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Send a new heading inforamtion to our Kalman filter.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	heading					The new heading in radians.
 *	\param[in]	accuracy				Our heading accuracy in radians.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendFilterHeading(SbgProtocolHandle handle, float heading, float accuracy)
{
	uint32 buffer[2];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build our arguments buffer
		//
		buffer[0] = sbgHostToTargetFloat(handle->targetOutputMode, heading);
		buffer[1] = sbgHostToTargetFloat(handle->targetOutputMode, accuracy);

		//
		// Send the command used to inform the device we have a new heading data
		//
		error = sbgProtocolSend(handle, SBG_SEND_FILTER_HEADING, buffer, 2*sizeof(uint32));

		//
		// Check if we have sent our frame successfully
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// We should receive an ACK
			//
			error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}
