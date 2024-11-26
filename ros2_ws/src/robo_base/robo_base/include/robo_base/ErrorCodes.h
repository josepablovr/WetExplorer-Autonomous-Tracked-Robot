#ifndef __ErrorCodes_H_
#define __ErrorCodes_H_

#define RQ_INVALID_HANDLE        -1
#define RQ_SUCCESS               0
#define RQ_ERR_OPEN_PORT         1
#define RQ_ERR_NOT_CONNECTED     2
#define RQ_ERR_TRANSMIT_FAILED   3
#define	RQ_ERR_SERIAL_IO         4
#define	RQ_ERR_SERIAL_RECEIVE    5
#define RQ_INVALID_RESPONSE      6
#define RQ_UNRECOGNIZED_DEVICE   7
#define RQ_UNRECOGNIZED_VERSION  8
#define RQ_INVALID_CONFIG_ITEM   9
#define RQ_INVALID_OPER_ITEM     10
#define RQ_INVALID_COMMAND_ITEM  11
#define RQ_INDEX_OUT_RANGE       12
#define RQ_SET_CONFIG_FAILED     13
#define RQ_GET_CONFIG_FAILED     14
#define RQ_GET_VALUE_FAILED      15
#define RQ_SET_COMMAND_FAILED    16
#define RQ_ERR_READING_FILE      17
#define RQ_EMPTY_SCRIPT_HEX		 18
#define RQ_DOWNLOAD_TIMEOUT		 19
#define RQ_SCRIPT_STORE_ERR		 20
#define RQ_INVALID_NODE			 21

#endif
