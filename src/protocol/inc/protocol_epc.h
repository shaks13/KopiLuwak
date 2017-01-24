/*******************************************************************************
 * @file protocol_epc.h
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef PROTOCOL_EPC_H
#define PROTOCOL_EPC_H
#include "common_library.h"


#define EPC_LOCK_PERMALOCK 							(1)
#define EPC_READ_PERMALOCK 							(0)

#define EPC_NB_BLOCK_PER_UNIT 						(16)
/*****************************************
		Interogator Commands
******************************************/
//BlockPermalock
#define RCMD_READ 										(0xC2)
//BlockPermalock
#define RCMD_BLOCK_PERMALOCK 							(0xC9)
//Test Write
#define RCMD_TEST_WRITE 								(0xE1C3)


#define TID_BANK_SIZE_BYTE 								(16)
#define SECRET_BANK_SIZE_WORDS 							(58)

#define MAX_NUM_WORDS_PER_WRITE 						(512)
#define MAX_NUM_WORDS_PER_PERMALOCK 					(MAX_NUM_WORDS_PER_WRITE)

#define FIDJI_BLOCK_SIZE 								(4)
//Bliss can only Permalock one range
#define FIDJI_BLOCK_PERMALOCK_RANGE						(1)

#define PROTOCOL_HEADER_NBBIT 								(1)
#define PROTOCOL_HANDLE_NBBIT 								(16)
#define PROTOCOL_CRCON16BITS								(16)
#define PROTOCOL_CRCON5BITS									(5)
#define PROTOCOL_NOCRC										(0)

// wordcount is limited to MAX_NUM_WORDS_PER_READ
// read words are stored into read_words_global table
#define MAX_NUM_WORDS_PER_READ 								(512)

/* the following constant is the command code of the EPCgen 2 command*/
#define PROTOCOL_EPC_CMDCODE_SELECT							(0b1010)
#define PROTOCOL_EPC_CMDCODE_QUERY							(0b1000)
#define PROTOCOL_EPC_CMDCODE_QUERYADJ						(0b1001)
#define PROTOCOL_EPC_CMDCODE_QUERYREP						(0b00)
#define PROTOCOL_EPC_CMDCODE_ACK							(0b01)
#define PROTOCOL_EPC_CMDCODE_NACK							(0b11000000)
#define PROTOCOL_EPC_CMDCODE_REQRN							(0b11000001)
#define PROTOCOL_EPC_CMDCODE_READ							(0b11000010)
#define PROTOCOL_EPC_CMDCODE_WRITE							(0b11000011)
#define PROTOCOL_EPC_CMDCODE_KILL							(0b11000100)
#define PROTOCOL_EPC_CMDCODE_LOCK							(0b11000101)
#define PROTOCOL_EPC_CMDCODE_ACCESS							(0b11000110)
#define PROTOCOL_EPC_CMDCODE_BLOCKWRITE						(0b11000111)
#define PROTOCOL_EPC_CMDCODE_BLOCKERASE						(0b11001000)
#define PROTOCOL_EPC_CMDCODE_BLOCKPERMALOCK					(0b11001001)
#define PROTOCOL_EPC_CMDCODE_AUTHENTICATE					(0b11010101)
#define PROTOCOL_EPC_CMDCODE_AUTHCOMM						(0b11010111)
#define PROTOCOL_EPC_CMDCODE_SECURECOMM						(0b11010110)
#define PROTOCOL_EPC_CMDCODE_KEYUPDATE						(0b1110001000000010)
#define PROTOCOL_EPC_CMDCODE_TAGPRIVILEGE					(0b1110001000000011)
#define PROTOCOL_EPC_CMDCODE_READBUFFER						(0b11010010)
#define PROTOCOL_EPC_CMDCODE_UNTRACEABLE					(0b1110001000000000)
#define PROTOCOL_EPC_CMDCODE_FILEOPEN						(0b11010011)
#define PROTOCOL_EPC_CMDCODE_FILELIST						(0b1110001000000001)
#define PROTOCOL_EPC_CMDCODE_FILEPRIVILEGE					(0b1110001000000100)
#define PROTOCOL_EPC_CMDCODE_FILESETUP						(0b1110001000000101)

/* the following constant is the size of the command code of the EPCgen 2 commands*/
#define PROTOCOL_EPC_CMDCODESIZE_SELECT						(4)
#define PROTOCOL_EPC_CMDCODESIZE_QUERY						(4)
#define PROTOCOL_EPC_CMDCODESIZE_QUERYADJ					(4)
#define PROTOCOL_EPC_CMDCODESIZE_QUERYREP					(2)
#define PROTOCOL_EPC_CMDCODESIZE_ACK						(2)
#define PROTOCOL_EPC_CMDCODESIZE_NACK						(8)
#define PROTOCOL_EPC_CMDCODESIZE_REQRN						(8)
#define PROTOCOL_EPC_CMDCODESIZE_READ						(8)
#define PROTOCOL_EPC_CMDCODESIZE_WRITE						(8)
#define PROTOCOL_EPC_CMDCODESIZE_KILL						(8)
#define PROTOCOL_EPC_CMDCODESIZE_LOCK						(8)
#define PROTOCOL_EPC_CMDCODESIZE_ACCESS						(8)
#define PROTOCOL_EPC_CMDCODESIZE_BLOCKWRITE					(8)
#define PROTOCOL_EPC_CMDCODESIZE_BLOCKERASE					(8)
#define PROTOCOL_EPC_CMDCODESIZE_BLOCKPERMALOCK				(8)
#define PROTOCOL_EPC_CMDCODESIZE_AUTHENTICATE				(8)
#define PROTOCOL_EPC_CMDCODESIZE_AUTHCOMM					(8)
#define PROTOCOL_EPC_CMDCODESIZE_SECURECOMM					(8)
#define PROTOCOL_EPC_CMDCODESIZE_KEYUPDATE					(16)
#define PROTOCOL_EPC_CMDCODESIZE_TAGPRIVILEGE				(16)
#define PROTOCOL_EPC_CMDCODESIZE_READBUFFER					(8)
#define PROTOCOL_EPC_CMDCODESIZE_UNTRACEABLE				(16)
#define PROTOCOL_EPC_CMDCODESIZE_FILEOPEN					(8)
#define PROTOCOL_EPC_CMDCODESIZE_FILELIST					(16)
#define PROTOCOL_EPC_CMDCODESIZE_FILEPRIVILEGE				(16)
#define PROTOCOL_EPC_CMDCODESIZE_FILESETUP					(16)

/* the following constant gives the numer of symbol of the parameter field (without commmand code and CRC)*/
#define PROTOCOL_EPC_NBPARAMETER_SELECT(x,y)				(3+3+2+8*x+8+y+1) /* x is the pointer (EBV) and y id the mask*/
#define PROTOCOL_EPC_NBPARAMETER_QUERY						(1+2+1+2+2+1+4)
#define PROTOCOL_EPC_NBPARAMETER_QUERYADJ					(2+3)
#define PROTOCOL_EPC_NBPARAMETER_QUERYREP					(2)
#define PROTOCOL_EPC_NBPARAMETER_ACK						(16)
#define PROTOCOL_EPC_NBPARAMETER_NACK						(0)
#define PROTOCOL_EPC_NBPARAMETER_REQRN						(16)
#define PROTOCOL_EPC_NBPARAMETER_READ(x)					(2+8*x+8+16) 	/* x is the word pointer (EBV)*/
#define PROTOCOL_EPC_NBPARAMETER_WRITE(x)					(2+8*x+8+16) 	/* x is the word pointer (EBV)*/
#define PROTOCOL_EPC_NBPARAMETER_KILL						(16+3+16)
#define PROTOCOL_EPC_NBPARAMETER_LOCK						(20+16)
#define PROTOCOL_EPC_NBPARAMETER_ACCESS						(16+16)
#define PROTOCOL_EPC_NBPARAMETER_BLOCKWRITE(x,y)			(2+8*x+8+y)			/* x is the word pointer (EBV) and y the number of bit to write */
#define PROTOCOL_EPC_NBPARAMETER_BLOCKERASE(x)				(2+8*x+8)			/* x is the word pointer (EBV) and y the number of bit to write */
#define PROTOCOL_EPC_NBPARAMETER_BLOCKPERMALOCK(x,y)		(8+1+2+8*x+8+y+16) 	/* x is the word pointer (EBV) and y the number of bit of the mask */
#define PROTOCOL_EPC_NBPARAMETER_AUTHENTICATE(x)			(2+1+1+8+12+x+16)  	/* x is the number of bit of the message */
#define PROTOCOL_EPC_NBPARAMETER_AUTHCOMM(x)				(2+1+x+16)			/* x is the number of bit of the message */
#define PROTOCOL_EPC_NBPARAMETER_SECURECOMM(x)				(2+1+1+12+x+16)		/* x is the number of bit of the message */
#define PROTOCOL_EPC_NBPARAMETER_KEYUPDATE(x)				(2+1+1+8+12+x+16)	/* x is the number of bit of the message */
#define PROTOCOL_EPC_NBPARAMETER_TAGPRIVILEGE				(2+1+1+1+1+8+16+16)
#define PROTOCOL_EPC_NBPARAMETER_READBUFFER					(2+12+12+16)
#define PROTOCOL_EPC_NBPARAMETER_UNTRACEABLE				(2+1+6+2+1+2+16)
#define PROTOCOL_EPC_NBPARAMETER_FILEOPEN					(2+10+16)
#define PROTOCOL_EPC_NBPARAMETER_FILELIST					(2+1+1+10+8+16)
#define PROTOCOL_EPC_NBPARAMETER_FILEPRIVILEGE				(2+1+1+3+8+4+16)
#define PROTOCOL_EPC_NBPARAMETER_FILESETUP					(2+1+1+8+10+16)

/* Max number of bytes in parameters delimited by EBV */
#define PROTOCOL_EPC_NBMAXBYTE_EBVPARAMETER				(3)

/**
 *  @enum 	protocol_epccommandId_enum
 *  @brief 	this enum contains the Id of the epc command according to the epc specification v2
 */
typedef enum {
  PROTOCOL_EPC_CMDID_SELECT=0x00,				/*!< the Id of the select command*/
  PROTOCOL_EPC_CMDID_CHALLENGE,					/*!< the Id of the challenge command*/
  PROTOCOL_EPC_CMDID_QUERY,						/*!< the Id of the query command*/
  PROTOCOL_EPC_CMDID_QUERYADJ,					/*!< the Id of the queryAdj command*/
  PROTOCOL_EPC_CMDID_QUERYREP,					/*!< the Id of the queryRep command*/
  PROTOCOL_EPC_CMDID_ACK,						/*!< the Id of the ack command*/
  PROTOCOL_EPC_CMDID_NACK,						/*!< the Id of the nack command*/
  PROTOCOL_EPC_CMDID_REQRN,						/*!< the Id of the reqrn command*/
  PROTOCOL_EPC_CMDID_READ,						/*!< the Id of the read command*/
  PROTOCOL_EPC_CMDID_WRITE,						/*!< the Id of the write command*/
  PROTOCOL_EPC_CMDID_KILL,						/*!< the Id of the kill command*/
  PROTOCOL_EPC_CMDID_LOCK,						/*!< the Id of the lock command*/
  PROTOCOL_EPC_CMDID_ACCESS,					/*!< the Id of the access command*/
  PROTOCOL_EPC_CMDID_BLOCKWRITE,				/*!< the Id of the blockWrite command*/
  PROTOCOL_EPC_CMDID_BLOCKERASE,				/*!< the Id of the BlockErase command*/
  PROTOCOL_EPC_CMDID_BLOCKPERMALOCK,			/*!< the Id of the BlockPermalock command*/
  PROTOCOL_EPC_CMDID_AUTHENTICATE,				/*!< the Id of the Authenticate command*/
  PROTOCOL_EPC_CMDID_AUTHCOMM,			/*10*/	/*!< the Id of the AuthComm command*/
  PROTOCOL_EPC_CMDID_SECURECOMM,				/*!< the Id of the SecureComm command*/
  PROTOCOL_EPC_CMDID_KEYUPDATE,					/*!< the Id of the Keyupdate command*/
  PROTOCOL_EPC_CMDID_TAGPRIVILEGE,				/*!< the Id of the TagPrivilege command*/
  PROTOCOL_EPC_CMDID_READBUFFER,				/*!< the Id of the ReadBuffer command*/
  PROTOCOL_EPC_CMDID_UNTRACEABLE,				/*!< the Id of the Untraceable command*/
  PROTOCOL_EPC_CMDID_FILEOPEN,					/*!< the Id of the FileOpen command*/
  PROTOCOL_EPC_CMDID_FILELIST,					/*!< the Id of the FileList command*/
  PROTOCOL_EPC_CMDID_FILEPRIVILEGE,				/*!< the Id of the FilePrivilege command*/
  PROTOCOL_EPC_CMDID_FILESETUP,					/*!< the Id of the FileSetup command*/
  PROTOCOL_EPC_CMDID_UNKNOWN					/*!< the Id of the unknown command*/
} protocol_epccommandId_enum;

/**
 *  @enum 	membank_t
 *  @brief 	this enum contains the Id of the memory bank according to the epc standard
 */
typedef enum {
  MEMBANK_RESERVED,		/*!< the Id of the reserved bank*/
  MEMBANK_EPC,			/*!< the Id of the EPC bank*/
  MEMBANK_TID,			/*!< the Id of the TID bank*/
  MEMBANK_USER,			/*!< the Id of the USER bank*/
} membank_t;

/**
 *  @enum 	protocol_encoding_enum
 *  @brief 	this enum contains the different coding of the tag response
 */
typedef enum {
  PROTOCOL_ENCODING_FM0 = 0,		/*!< the tag response uses the FM0 coding*/
  PROTOCOL_ENCODING_MILLER2,		/*!< the tag response uses the Miller 2 coding*/
  PROTOCOL_ENCODING_MILLER4,		/*!< the tag response uses the Miller 4 coding*/
  PROTOCOL_ENCODING_MILLER8,		/*!< the tag response uses the Miller 8 coding*/
} protocol_encoding_enum;


/** @struct protocol_preamble_struct
 *  @brief this structure contains the fields of the preamble
 *  @var protocol_preamble_struct::uDelimiter
 *  Member 'uTari' is the Tari value (counted as BLF period)
 *  @var protocol_preamble_struct::uTari
 *  Member 'uTari' is the Tari value (counted as BLF period)
 *  @var protocol_preamble_struct::uRtCal
 *  Member 'uRtCal' is the RtCal value (counted as BLF period)
 *  @var protocol_preamble_struct::uTrCal
 *  Member 'uTrCal' is the TrCal value (counted as BLF period)
 */
typedef struct
{
	uint8_t uDelimiter;
	uint8_t uTari;
	uint8_t uRtCal;
	uint16_t uTrCal;

}protocol_preamble_struct;

/** @struct protocol_response_struct
 *  @brief this structure contains the fields of the tag response
 *  @var protocol_response_struct::bIsResPonseexpected
 *  Member 'bIsResPonseexpected' is set to true when the received EPC command expects a response
 *  @var protocol_response_struct::bIsPilotTone
 *  Member 'bIsPilotTone' is set to true when the pilot tone should be addded
 *  @var protocol_response_struct::bIsHandleFound
 *  Member 'bIsHandleFound' is set to true when the value abHandle is relevant
 *  @var protocol_response_struct::abHandle
 *  Member 'abHandle' contain the handle (16 bits)
 *  @var protocol_response_struct::uTpri
 *  Member 'uTpri' is the Tpri multiplied by 10
 *  @var protocol_response_struct::uNbBit
 *  Member 'uNbBit' is the is the number of bit if the tag response (CRC included)
 *  @var protocol_response_struct::pResponse
 *  Member 'pResponse' is the pointer of the response. the header, CRC and dummy bit should be appended
 */
typedef struct
{
	bool bIsResPonseExpected;
	bool bIsImmediateReplyExpected;
	bool bIsPilotTone;
	protocol_encoding_enum protocol_encoding;
	bool bIsHandleFound;
	bool abHandle [16];
	uint8_t uTpri;
	uint8_t uNbBit;
	bool *pResponse;
}protocol_response_struct;

extern uint16_t read_words_global[MAX_NUM_WORDS_PER_READ];

extern char general_purpose_buffer[MAX_NUM_WORDS_PER_READ]; // TODO cleaning


#endif // #ifndef EPC_H
