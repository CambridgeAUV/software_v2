#include "commandsNav.h"
#include "protocolOutputMode.h"
#include "commands.h"
#include <string.h>

/*!
 *	Get the advanced information about Space Vehicles seen by GPS. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	pSvInfo				Pointer to an array of SbgGpsSVInfo structure that will contain SV information (std size 16 channels)
 *	\param[out]	pNbSV				Pointer to the number of satellites managed
 *	\param[in]	maxNbSv				Maximum number of nbSv that can be contained in our pSvInfo list.
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgGetSvInfo(SbgProtocolHandle handle, SbgGpsSVInfo *pSvInfo, uint8 *pNbSV, uint8 maxNbSv)
{
	uint8			cmd;
	uint16			size;
	uint8			buffer[1+ 50 * sizeof(SbgGpsSVInfo)];
	SbgErrorCode	error;
	uint8			nbSatellites;
	SbgGpsSVInfo	*svBlocks;

	//
	// Send the command used to get space vehicles information
	//
	error = sbgProtocolSend(handle, SBG_GET_GPS_SVINFO, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, buffer, &size, 1+ 50 * sizeof(SbgGpsSVInfo));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_GPS_SVINFO) && (((size-1)%sizeof(SbgGpsSVInfo)) == 0))
			{
				//
				// Get the number of satellites
				//
				nbSatellites = buffer[0];

				//
				// Calculate the real number of satellites
				//
				if (nbSatellites>maxNbSv)
				{
					nbSatellites = maxNbSv;
					error = SBG_BUFFER_OVERFLOW;
				}			

				//
				// Returns, if possible, the number of SV
				//
				if (pNbSV)
				{
					*pNbSV = nbSatellites;
				}

				//
				// Returns, if possible, the list of SV
				//
				if (pSvInfo)
				{
					svBlocks = (SbgGpsSVInfo*)(&(buffer[1]));

					//
					// Copy output in user buffer 
					// all data is in (u)int8 format, so no data conversion is needed
					//
					memcpy(pSvInfo, svBlocks, sizeof(SbgGpsSVInfo)*nbSatellites);
				}
			}
			else if ((cmd == SBG_ACK) && (size == sizeof(uint8)))
			{
				//
				// Get the error code contained in the ACK frame
				//
				error = (SbgErrorCode)(buffer[0]);

				//
				// We should receive an NACK so check if it's the case and return the error
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// We have received an invalid frame, maybe a question/answer desync!
					//
					error = SBG_INVALID_FRAME;
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
 *	Defines which source to use to aid the velocity in our navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	aidingSrc			The aiding source to use in our navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetNavVelocitySrc(SbgProtocolHandle handle, bool permanent, SbgAidingVelSrc aidingSrc)
{
	uint8 buffer[sizeof(uint8)*2];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build our arguments buffer
		//
		*buffer = permanent;
		*(buffer+sizeof(uint8)) = (uint8)aidingSrc;

		//
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_SET_NAV_VELOCITY_SRC, buffer, sizeof(uint8)*2);

		//
		// Check if our command has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Wait for an answer
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
 *	Returns which source is used to aid the velocity in our navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pAidingSrc			The aiding source used in our navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetNavVelocitySrc(SbgProtocolHandle handle, SbgAidingVelSrc *pAidingSrc)
{
	uint8 cmd;
	uint16 size;
	uint8 aidingSource;
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_GET_NAV_VELOCITY_SRC, NULL, 0);

		//
		// Check if our frame has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Try to read our answer
			//
			error = sbgProtocolReceiveTimeOut(handle, &cmd, &aidingSource, &size, sizeof(uint8));

			//
			// Check if we have recieved a frame
			//
			if (error == SBG_NO_ERROR)
			{
				//
				// Check if we have the right command and data size
				//
				if ( (cmd == SBG_RET_NAV_VELOCITY_SRC) && (size == sizeof(uint8)) )
				{
					//
					// Returns, if possible, our aiding source
					//
					if (pAidingSrc)
					{
						*pAidingSrc = (SbgAidingVelSrc)aidingSource;
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
 *	Defines which source to use to aid the position in our navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	aidingSrc			The aiding source to use in our navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetNavPositionSrc(SbgProtocolHandle handle, bool permanent, SbgAidingPosSrc aidingSrc)
{
	uint8 buffer[sizeof(uint8)*2];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build our arguments buffer
		//
		*buffer = permanent;
		*(buffer+sizeof(uint8)) = (uint8)aidingSrc;

		//
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_SET_NAV_POSITION_SRC, buffer, sizeof(uint8)*2);

		//
		// Check if our command has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Wait for an answer
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
 *	Returns which source is used to aid the position in our navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pAidingSrc			The aiding source used in our navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetNavPositionSrc(SbgProtocolHandle handle, SbgAidingPosSrc *pAidingSrc)
{
	uint8 cmd;
	uint16 size;
	uint8 aidingSource;
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_GET_NAV_POSITION_SRC, NULL, 0);

		//
		// Check if our frame has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Try to read our answer
			//
			error = sbgProtocolReceiveTimeOut(handle, &cmd, &aidingSource, &size, sizeof(uint8));

			//
			// Check if we have recieved a frame
			//
			if (error == SBG_NO_ERROR)
			{
				//
				// Check if we have the right command and data size
				//
				if ( (cmd == SBG_RET_NAV_POSITION_SRC) && (size == sizeof(uint8)) )
				{
					//
					// Returns, if possible, our aiding source
					//
					if (pAidingSrc)
					{
						*pAidingSrc = (SbgAidingPosSrc)aidingSource;
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
 *	Defines the GPS lever arm. (IG-500N only).<br>
 *	Use this command to specifiy the vector between the device and the GPS antenna.<br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	gpsLeverArm			X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetGpsLeverArm(SbgProtocolHandle handle, bool permanent, const float gpsLeverArm[3])
{
	uint8 buffer[sizeof(uint8)+sizeof(uint32)*3];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build our arguments buffer
		//
		*buffer = permanent;
		*(uint32*)(buffer + sizeof(uint8) + sizeof(uint32)*0)	= sbgHostToTargetFloat(handle->targetOutputMode, gpsLeverArm[0]);
		*(uint32*)(buffer + sizeof(uint8) + sizeof(uint32)*1)	= sbgHostToTargetFloat(handle->targetOutputMode, gpsLeverArm[1]);
		*(uint32*)(buffer + sizeof(uint8) + sizeof(uint32)*2)	= sbgHostToTargetFloat(handle->targetOutputMode, gpsLeverArm[2]);

		//
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_SET_GPS_LEVER_ARM, buffer, sizeof(uint8)+sizeof(uint32)*3);

		//
		// Check if our command has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Wait for an answer
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
 *	Returns the GPS lever arm. (IG-500N only).<br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	gpsLeverArm			X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetGpsLeverArm(SbgProtocolHandle handle, float gpsLeverArm[3])
{
	uint8 cmd;
	uint16 size;
	uint32 buffer[3];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_GET_GPS_LEVER_ARM, NULL, 0);

		//
		// Check if our frame has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Try to read our answer
			//
			error = sbgProtocolReceiveTimeOut(handle, &cmd, buffer, &size, sizeof(uint32)*3);

			//
			// Check if we have recieved a frame
			//
			if (error == SBG_NO_ERROR)
			{
				//
				// Check if we have the right command and data size
				//
				if ( (cmd == SBG_RET_GPS_LEVER_ARM) && (size == sizeof(uint32)*3) )
				{
					//
					// Returns, if possible, our gps lever arm
					//
					if (gpsLeverArm)
					{
						gpsLeverArm[0] = sbgTargetToHostFloat(handle->targetOutputMode, buffer[0]);
						gpsLeverArm[1] = sbgTargetToHostFloat(handle->targetOutputMode, buffer[1]);
						gpsLeverArm[2] = sbgTargetToHostFloat(handle->targetOutputMode, buffer[2]);
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
 *	Defines the local gravity magnitude. (IG-500N only).
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	gravityMagnitude	The local gravity magnitude in m.s^-2.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetGravityMagnitude(SbgProtocolHandle handle, bool permanent, float gravityMagnitude)
{
	uint8 buffer[sizeof(uint8)+sizeof(uint32)];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build our arguments buffer
		//
		*buffer = permanent;
		*(uint32*)(buffer + sizeof(uint8))	= sbgHostToTargetFloat(handle->targetOutputMode, gravityMagnitude);
		
		//
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_SET_GRAVITY_MAGNITUDE, buffer, sizeof(uint8)+sizeof(uint32));

		//
		// Check if our command has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Wait for an answer
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
 *	Returns the local gravity magnitude. (IG-500N only).<br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pGravityMagnitude	The local gravity magnitude in m.s^-2.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetGravityMagnitude(SbgProtocolHandle handle, float *pGravityMagnitude)
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
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_GET_GRAVITY_MAGNITUDE, NULL, 0);

		//
		// Check if our frame has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Try to read our answer
			//
			error = sbgProtocolReceiveTimeOut(handle, &cmd, &buffer, &size, sizeof(uint32));

			//
			// Check if we have recieved a frame
			//
			if (error == SBG_NO_ERROR)
			{
				//
				// Check if we have the right command and data size
				//
				if ( (cmd == SBG_RET_GRAVITY_MAGNITUDE) && (size == sizeof(uint32)) )
				{
					//
					// Returns, if possible, our gravity magnitude
					//
					if (pGravityMagnitude)
					{
						*pGravityMagnitude = sbgTargetToHostFloat(handle->targetOutputMode, buffer);
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
 *	Defines the constraints applied on velocity. If an axis is constrained, its velocity will be recentered to 0 by the kalman filter
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	constraints			Bitmask for the constraints to apply: CONSTRAINT_X, CONSTRAINT_Y and CONSTRAINT_Z can be set
 *  \param[in]	errorLevel			Error factor on the constraint applied: SBG_CONSTRAINT_STRICT, SBG_CONSTRAINT_NORMAL or SBG_CONSTRAINT_LOOSE
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetVelocityConstraints(SbgProtocolHandle handle, bool permanent, uint16 constraints, SbgConstraintError errorLevel)
{
	uint8 buffer[sizeof(uint8)+sizeof(uint16)+sizeof(uint8)];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build our arguments buffer
		//
		*buffer												= permanent;
		*(uint16*)(buffer + sizeof(uint8))					= sbgHostToTarget16(handle->targetOutputMode, constraints);
		*(uint8*)(buffer + sizeof(uint8) + sizeof(uint16))	= (uint8)errorLevel;

		//
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_SET_VELOCITY_CONSTRAINTS, buffer, sizeof(uint8)+sizeof(uint16)+sizeof(uint8));

		//
		// Check if our command has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Wait for an answer
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
 *	Returns the Constraints applied on velocity. If an axis is constrained, its velocity will be recentered to 0 by the kalman filter
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pConstraints		Bitmask for the constraints to apply: CONSTRAINT_X, CONSTRAINT_Y and CONSTRAINT_Z can be set
 *  \param[out]	pErrorLevel			Error factor on the constraint applied: SBG_CONSTRAINT_STRICT, SBG_CONSTRAINT_NORMAL or SBG_CONSTRAINT_LOOSE
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetVelocityConstraints(SbgProtocolHandle handle, uint16 *pConstraints, SbgConstraintError *pErrorLevel)
{
	uint8 cmd;
	uint16 size;
	uint8 buffer[sizeof(uint16)+sizeof(uint8)];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Send our command
		//
		error = sbgProtocolSend(handle, SBG_GET_VELOCITY_CONSTRAINTS, NULL, 0);

		//
		// Check if our frame has been sent
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Try to read our answer
			//
			error = sbgProtocolReceiveTimeOut(handle, &cmd, &buffer, &size, sizeof(uint16)+sizeof(uint8));

			//
			// Check if we have recieved a frame
			//
			if (error == SBG_NO_ERROR)
			{
				//
				// Check if we have the right command and data size
				//
				if ( (cmd == SBG_RET_VELOCITY_CONSTRAINTS) && (size == sizeof(uint16)+sizeof(uint8)) )
				{
					//
					// Returns, if possible, our velocity constraints mask
					//
					if (pConstraints)
					{
						*pConstraints = sbgTargetToHost16(handle->targetOutputMode, *(uint16*)buffer);
					}

					//
					// Returns, if possible, our constraints error
					//
					if (pErrorLevel)
					{
						*pErrorLevel = (SbgConstraintError)(*(buffer+sizeof(uint16)));
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
 *	Send a new velocity information to our Navigation filter.
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	velocity			The new X,Y,Z velocity that should be used by the navigation filter in m/s.
 *	\param[in]	accuracy			The velocity accuracy in m/s.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendNavVelocity(SbgProtocolHandle handle, const float velocity[3], float accuracy)
{
	uint32 buffer[4];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build our arguments buffer
		//
		buffer[0] = sbgHostToTargetFloat(handle->targetOutputMode, velocity[0]);
		buffer[1] = sbgHostToTargetFloat(handle->targetOutputMode, velocity[1]);
		buffer[2] = sbgHostToTargetFloat(handle->targetOutputMode, velocity[2]);
		buffer[3] = sbgHostToTargetFloat(handle->targetOutputMode, accuracy);

		//
		// Send the command
		//
		error = sbgProtocolSend(handle, SBG_SEND_NAV_VELOCITY, buffer, 4*sizeof(uint32));

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
 *	Send a new position information to our Navigation filter.
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	position			The new WGS84 position : latitude, longitude and altitude (above ellipsoid) in [deg, deg, meters].
 *	\param[in]	hAccuracy			The horizontal accuracy in meters.
 *	\param[in]	vAccuracy			The vertical accuracy in meters.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendNavPosition(SbgProtocolHandle handle, const double position[3], float hAccuracy, float vAccuracy)
{
	uint8 buffer[sizeof(uint64)*3+sizeof(uint32)*2];
	SbgErrorCode error;

	//
	// Check if our handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build our arguments buffer
		//
		*(uint64*)(buffer+sizeof(uint64)*0) = sbgHostToTargetDouble(handle->targetOutputMode, position[0]);
		*(uint64*)(buffer+sizeof(uint64)*1) = sbgHostToTargetDouble(handle->targetOutputMode, position[1]);
		*(uint64*)(buffer+sizeof(uint64)*2) = sbgHostToTargetDouble(handle->targetOutputMode, position[2]);

		*(uint32*)(buffer+sizeof(uint64)*3+sizeof(uint32)*0) = sbgHostToTargetFloat(handle->targetOutputMode, hAccuracy);
		*(uint32*)(buffer+sizeof(uint64)*3+sizeof(uint32)*1) = sbgHostToTargetFloat(handle->targetOutputMode, vAccuracy);

		//
		// Send the command
		//
		error = sbgProtocolSend(handle, SBG_SEND_NAV_POSITION, buffer, sizeof(uint64)*3+sizeof(uint32)*2);

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
