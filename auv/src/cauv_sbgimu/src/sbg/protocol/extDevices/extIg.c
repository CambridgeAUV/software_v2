#include "extIg.h"
#include "../commandsExt.h"
#include "../protocolOutputMode.h"
#include <string.h>

/*!
 *	Set an automatic orientation offset between the IG-500E and the IG-Device to realign them
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	type				Offset type. Value allowed is:
 *									- SBG_IG_AUTO_OFFSET_XY
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgSetAutoOffset(SbgProtocolHandle handle, bool permanent, uint8 type)
{
	uint8 command[3];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_EXT_CMD_IG_SET_AUTO_OFFSET;
	command[1] = permanent;
	command[2] = type;

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgExtDeviceConfig(handle,command,3*sizeof(uint8),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually an acknowledge
		//
		if ((answerSize == 2) && (answer[0] == SBG_EXT_CMD_IG_ACK))
		{
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Set a manual orientation offset between the IG-500E and remote device, in a matrix form
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *	\param[in]	matrixOffset		orientation offset between the IG-Device and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgSetMatrixOffset(SbgProtocolHandle handle, bool permanent, const float matrixOffset[9])
{
	uint8 command[2*sizeof(uint8)+9*sizeof(uint32)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;
	int8 i;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_EXT_CMD_IG_SET_MATRIX_OFFSET;
	command[1] = permanent;
	for (i=0; i<9; i++)
	{
		*((uint32*)(command+2*sizeof(uint8)+i*sizeof(uint32))) = sbgHostToTargetFloat(handle->targetOutputMode, matrixOffset[i]);
	}

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgExtDeviceConfig(handle, command, 2*sizeof(uint8)+9*sizeof(uint32), answer, &answerSize, 256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually an acknowledge
		//
		if ((answerSize == 2) && (answer[0] == SBG_EXT_CMD_IG_ACK))
		{
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Get the offset between the IG-500E orientation and the IG-Device orientation. <br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	matrixOffset		Orientation offset between the remote IG-Device and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgGetMatrixOffset(SbgProtocolHandle handle, float matrixOffset[9])
{
	uint8 command[sizeof(uint8)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;
	uint32* matrix;
	int8 i;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_EXT_CMD_IG_GET_MATRIX_OFFSET;	

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgExtDeviceConfig(handle,command,sizeof(uint8),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually a matrix output
		// Handle acknowledge received to make an error
		//
		if ((answer[0] == SBG_EXT_CMD_IG_RET_MATRIX_OFFSET) && (answerSize == (sizeof(uint8)+9*sizeof(uint32))))
		{
			if (matrixOffset)
			{
				matrix = (uint32*)(answer + sizeof(uint8));
				for (i=0; i<9; i++)
				{
					matrixOffset[i] = sbgTargetToHostFloat(handle->targetOutputMode, matrix[i]);
				}
			}
		}
		else if ((answerSize == 2) && (answer[0] == SBG_EXT_CMD_IG_ACK) && (answer[1] != SBG_NO_ERROR))
		{
			//
			// We received an error
			//
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Send an encapsulated frame to the remote IG-Device, and return its answer
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	cmdIn				Command to be sent to the remote IG-Device
 *  \param[in]	sizeIn				Size of the frame to be sent
 *  \param[in]	pDataIn				Pointer to the data to be sent to the IG-Device
 *  \param[out]	pCmdOut				Pointer to the command returned by the Remote IG-Device
 *  \param[out]	pSizeOut			Pointer to the actual data size returned by the remote device
 *  \param[out]	pDataOut			Pointer to the data output buffer returned by the remote IG-Device
 *  \param[in]	maxSizeOut			Maximum output size allowed in pDataOut field
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgEncapsulatedFrame(SbgProtocolHandle handle, uint8 cmdIn, uint16 sizeIn, const void *pDataIn, 
									   uint8 *pCmdOut, uint16 *pSizeOut, void *pDataOut, uint16 maxSizeOut)
{
	uint8 command[512];
	uint8 answer[512];
	uint16 answerSize;
	SbgErrorCode error;

	
	//
	// Check input parameters
	// The pDataIn pointer can be null only if the sizeIn param is also null. Otherwise, the pointer must be provided
	//
	if ((pCmdOut) && ((!sizeIn) || ((sizeIn) && (pDataIn))))
	{
		//
		// Build up the external device command buffer
		//
		command[0] = SBG_EXT_CMD_IG_SEND_REMOTE_FRAME;
		command[1] = cmdIn;
		if (sizeIn)
		{
			memcpy(command+2*sizeof(uint8),pDataIn,sizeIn);
		}

		//
		// Send the external module configuration, and wait for the answer
		//
		error = sbgExtDeviceConfig(handle, command, 2*sizeof(uint8)+sizeIn, answer, &answerSize, 512);

		//
		// Check if the answer is actually a remote frame return
		// Handle acknowledge received to make an error
		//
		if ((answer[0] == SBG_EXT_CMD_IG_RET_REMOTE_FRAME) && (answerSize > 1))
		{
			//
			// We received an answer, so return the parameters
			//
			if (pCmdOut)
			{
				*pCmdOut = answer[1];
			}

			// Check if we can store the whole result or if we have to cut it
			if ((answerSize) &&  (maxSizeOut) && (pDataOut))
			{
				if (maxSizeOut < (answerSize - 2))
				{
					answerSize = maxSizeOut;
				}
				memcpy(pDataOut,answer+(2*sizeof(uint8)),answerSize);
			}

			if (pSizeOut)
			{
				*pSizeOut = answerSize;
			}
		}
		else if ((answerSize == 2) && (answer[0] == SBG_EXT_CMD_IG_ACK) && (answer[1] != SBG_NO_ERROR))
		{
			//
			// We received an error
			//
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;

}