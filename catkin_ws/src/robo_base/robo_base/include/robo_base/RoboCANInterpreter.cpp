#include <iostream>
#include <iomanip>

#include "RoboCANInterpreter.h"

using namespace std;

void PrintCANMessage(CANMessage message)
{
	cout << setfill('0') << setw(3) << hex << uppercase << message.CANID << "h\t";
	cout << message.DataLength << "\t";

	for (size_t i = 0; i < message.DataLength; i++)
	{
		cout << setfill('0') << setw(2) << hex << uppercase << (int) message.Data[i] << " ";
	}

	cout << endl << dec;
}
void PrintMessageInfo(MessageInfo info)
{
	switch (info.Status)
	{
	case STATUS_INVALID:
		cout << "Invalid RoboCAN message." << endl;
		break;
		
	case STATUS_HEARTBEAT:
		cout << "Heartbeat from node " << info.SourceNodeID << "." << endl;
		break;

	case STATUS_RUNTIME_COMMAND:
		cout << "Node " << info.SourceNodeID << ": SetCommand(" << info.Item << ", " << info.Index << ", " << info.Value << ") to node " << info.TargetNodeID << "." << endl;
		break;

	case STATUS_CONFIG_SET:
		cout << "Node " << info.SourceNodeID << ": SetConfig(" << info.Item << ", " << info.Index << ", " << info.Value << ") to node " << info.TargetNodeID << "." << endl;
		break;

	case STATUS_CONFIG_GET:
		if (info.IsResponse)
			cout << "Node " << info.SourceNodeID << " replied to GetConfig(" << info.Item << ", " << info.Index << ") with value " << info.Value << " to node " << info.TargetNodeID << "." << endl;
		else
			cout << "Node " << info.SourceNodeID << " requests GetConfig(" << info.Item << ", " << info.Index << ") from node " << info.TargetNodeID << "." << endl;
		break;

	case STATUS_RUNTIME_QUERY:
		if (info.IsResponse)
			cout << "Node " << info.SourceNodeID << " replied to GetValue(" << info.Item << ", " << info.Index << ") with value " << info.Value << " to node " << info.TargetNodeID << "." << endl;
		else
			cout << "Node " << info.SourceNodeID << " requests GetValue(" << info.Item << ", " << info.Index << ") from node " << info.TargetNodeID << " with refresh rate of " << info.RefreshRate << "ms." << endl;
		break;

	default:
		cout << "Unhandled status " << info.Status << "." << endl;
		break;
	}
}


void RoboCANInterpreter::SetNodeID(int nid)
{
	nodeID = nid;
}

int RoboCANInterpreter::GetNodeID()
{
	return nodeID;
}

CANMessage RoboCANInterpreter::EncodeHeartbeat()
{
	CANMessage message;
	message.CANID = 0;
	message.DataLength = 2;
	message.Data[0] = nodeID & 0x7F;
	message.Data[1] = 0;

	return message;
}

//CANMessage RoboCANInterpreter::EncodeCommand(int targetNodeID, int commandItem, int value)
//{
//	CANMessage message;
//	message.CANID = ((targetNodeID & 0x7F) << 4) | CMD_GROUP_RUNTIME_1ARGCMD;
//	message.DataLength = 8;
//	message.Data[0] = nodeID & 0x7F;
//	message.Data[1] = commandItem;
//	message.Data[2] = ROBOCAN_DEFAULT_RATE; //rate - not used
//	message.Data[3] = 1;
//	message.Data[4] = (value)& 0xFF;
//	message.Data[5] = (value >> 8) & 0xFF;
//	message.Data[6] = (value >> 16) & 0xFF;
//	message.Data[7] = (value >> 24) & 0xFF;
//
//	return message;
//}

CANMessage RoboCANInterpreter::EncodeCommand(int targetNodeID, int commandItem, int index, int value)
{
	CANMessage message;
	message.CANID = ((targetNodeID & 0x7F) << 4) | CMD_GROUP_RUNTIME_COMMAND;
	message.DataLength = 8;
	message.Data[0] = nodeID & 0x7F;
	message.Data[1] = commandItem;
	message.Data[2] = ROBOCAN_DEFAULT_RATE; //rate - not used
	message.Data[3] = index;
	message.Data[4] = (value) & 0xFF;
	message.Data[5] = (value >> 8) & 0xFF;
	message.Data[6] = (value >> 16) & 0xFF;
	message.Data[7] = (value >> 24) & 0xFF;

	return message;
}

CANMessage RoboCANInterpreter::EncodeConfigSet(int targetNodeID, int configItem, int index, int value)
{
	CANMessage message;
	message.CANID = ((targetNodeID & 0x7F) << 4) | CMD_GROUP_CONFIG_SET;
	message.DataLength = 8;
	message.Data[0] = nodeID & 0x7F;
	message.Data[1] = configItem;
	message.Data[2] = ROBOCAN_DEFAULT_RATE; //rate - not used
	message.Data[3] = index;
	message.Data[4] = (value)& 0xFF;
	message.Data[5] = (value >> 8) & 0xFF;
	message.Data[6] = (value >> 16) & 0xFF;
	message.Data[7] = (value >> 24) & 0xFF;

	return message;
}

CANMessage RoboCANInterpreter::EncodeConfigGet(int targetNodeID, int configItem, int index)
{
	CANMessage message;
	message.CANID = ((targetNodeID & 0x7F) << 4) | CMD_GROUP_CONFIG_GET;
	message.DataLength = 4;
	message.Data[0] = nodeID & 0x7F;
	message.Data[1] = configItem;
	message.Data[2] = ROBOCAN_DEFAULT_RATE; //rate - not used
	message.Data[3] = index;

	return message;
}

CANMessage RoboCANInterpreter::EncodeQuery(int targetNodeID, int operatingItem, int index, int refreshRate)
{
	CANMessage message;
	message.CANID = ((targetNodeID & 0x7F) << 4) | CMD_GROUP_RUNTIME_QUERY;
	message.DataLength = 4;
	message.Data[0] = nodeID & 0x7F;
	message.Data[1] = operatingItem;
	message.Data[2] = refreshRate;
	message.Data[3] = index;

	return message;
}

MessageInfo RoboCANInterpreter::Decode(CANMessage message)
{
	MessageInfo info;
	info.Status = STATUS_INVALID;

	if (message.DataLength <= 0)
		return info;

	info.TargetNodeID = message.CANID >> 4;
	info.SourceNodeID = message.Data[0];

	if (info.SourceNodeID > 126)
		return info;

	if (message.CANID == 0 && message.DataLength == 2) {
		info.Status = STATUS_HEARTBEAT;_V
		return info;
	}

	if (message.DataLength != 4 && message.DataLength != 8) {
		info.Status = STATUS_INVALID;
		return info;
	}

	int cmdGroup = message.CANID & 0x07;
	if (cmdGroup < CMD_GROUP_MAINTENANCE || cmdGroup > CMD_GROUP_RUNTIME_1ARGCMD)
		return info;

	info.Status = (MessageStatus)cmdGroup;
	info.IsResponse = (message.CANID & 0x08) != 0;

		/*STATUS_CONFIG_SET = 3,
		STATUS_CONFIG_GET = 4,
		STATUS_RUNTIME_QUERY = 5,
		STATUS_RUNTIME_COMMAND = 6,
		STATUS_RUNTIME_1ARGCMD = 7,*/


	if (info.Status == STATUS_CONFIG_GET || info.Status == STATUS_RUNTIME_QUERY) {
		if (info.IsResponse && message.DataLength != 8 || !info.IsResponse && message.DataLength != 4) {
			info.Status = STATUS_INVALID;
			return info;
		}
	}
	else if (message.DataLength != 8) {
		info.Status = STATUS_INVALID;
		return info;
	}

	info.Item = message.Data[1];
	info.RefreshRate = message.Data[2];
	info.Index = message.Data[3];

	if (message.DataLength == 8) {
		info.Value = message.Data[7];
		info.Value = (info.Value << 8) | message.Data[6];
		info.Value = (info.Value << 8) | message.Data[5];
		info.Value = (info.Value << 8) | message.Data[4];
	}

	return info;
}

RoboCANInterpreter::RoboCANInterpreter(int nid)
{
	nodeID = nid;
}

