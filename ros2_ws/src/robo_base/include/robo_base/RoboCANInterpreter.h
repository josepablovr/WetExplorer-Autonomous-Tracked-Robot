#ifndef __RoboCANInterpreter_H_
#define __RoboCANInterpreter_H_

#include <cstdint>

using namespace std;

#define ROBOCAN_DEFAULT_RATE	50

struct CANMessage 
{
	int CANID;
	int DataLength;
	uint8_t Data[8];
};

enum MessageStatus {
	STATUS_INVALID = -1,
	STATUS_HEARTBEAT = 0,

	STATUS_MAINTENANCE = 1,
	STATUS_HISTORY = 2,
	STATUS_CONFIG_SET = 3,
	STATUS_CONFIG_GET = 4,
	STATUS_RUNTIME_QUERY = 5,
	STATUS_RUNTIME_COMMAND = 6,
	STATUS_RUNTIME_1ARGCMD = 7,
};

struct MessageInfo {
	MessageStatus Status;
	int SourceNodeID;
	int TargetNodeID;
	bool IsResponse;
	int Item;
	int RefreshRate;
	int Index;
	int Value;
};

typedef enum {
	CMD_GROUP_MAINTENANCE		= 1,
	CMD_GROUP_HISTORY			= 2,
	CMD_GROUP_CONFIG_SET		= 3,
	CMD_GROUP_CONFIG_GET		= 4,
	CMD_GROUP_RUNTIME_QUERY		= 5,
	CMD_GROUP_RUNTIME_COMMAND	= 6,
	CMD_GROUP_RUNTIME_1ARGCMD	= 7
} CMD_GROUP;


void PrintCANMessage(CANMessage message);
void PrintMessageInfo(MessageInfo info);

class RoboCANInterpreter
{
private:
	int nodeID;

protected:

public:
	void SetNodeID(int nid);
	int GetNodeID();

	CANMessage EncodeHeartbeat();
	//CANMessage EncodeCommand(int targetNodeID, int commandItem, int value);
	CANMessage EncodeCommand(int targetNodeID, int commandItem, int index, int value);
	CANMessage EncodeConfigSet(int targetNodeID, int configItem, int index, int value);
	CANMessage EncodeConfigGet(int targetNodeID, int configItem, int index);
	CANMessage EncodeQuery(int targetNodeID, int operatingItem, int index, int refreshRate);

	MessageInfo Decode(CANMessage message);

	RoboCANInterpreter(int nid);
};

#endif
