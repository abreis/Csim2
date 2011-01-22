#ifndef VANETCSIM_H_
#define VANETCSIM_H_

#include <iostream>
#include <iomanip>
#include <vector>	// contiguous, no insertion of events in the middle
#include <list>		// fast insertion anywhere, sorting
#include <boost/random.hpp>

using namespace std;

// sim parameters
int g_simTime=0;			// in ms
int g_simEndTime=2000*1000;	// in ms
int g_step=100;				// 100ms step
unsigned int g_BestCaseCount=0;
unsigned int g_WorstCaseCount=0;
int g_PacketStartVID=0;

// highway parameters
float g_speed=30.0;		// 30 m/s
float g_rrange=250.0;	// 250m radio range
float g_lambda=0.0039;	// vehicle density
float g_length=6000;	// highway length
unsigned int g_vID=0;	// global vehicle ID counter

class VanetVehicle {
public:
	int vehicleID;
	float position;
	char direction;
	vector<int> packetList; 	// array of packets
	VanetVehicle(int vid, char dir, float pos) {vehicleID=vid; direction=dir; position=pos;}
	VanetVehicle(){vehicleID=0;position=0;direction='F';}
};

class simEvent {
public:
	char eventType; 	// 'V' new vehicle 'P' new packet
	int time;			// in ms
	float position;		// V
	char direction;		// V
	int vehicleID;		// V, P
	int packetID;		// P
	simEvent(char type, int t, float pos, char dir, int vid) {eventType=type; time=t; position=pos; direction=dir; vehicleID=vid; packetID=0;}
	simEvent(char type, int t, int vid, int pid) {eventType=type; time=t; vehicleID=vid; packetID=pid; direction='P'; position=0.0;}
};

bool eventCompare (simEvent event1, simEvent event2);
bool vehiclePosCompare (VanetVehicle veh1, VanetVehicle veh2);
void AdvanceVehicles(list<VanetVehicle> *Vehicles, unsigned int delta);
void AddPacket (list<VanetVehicle> *Vehicles, int vehicleID, int packetID);
void BroadcastPacket(list<VanetVehicle> *Vehicles, int srcVehicleID, int packetID, bool needsSort);
void ReBroadcastPackets(list<VanetVehicle> *Vehicles);

#endif /* VANETCSIM_H_ */
