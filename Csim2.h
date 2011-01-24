#ifndef VANETCSIM_H_
#define VANETCSIM_H_

#include <iostream>
#include <iomanip>
#include <vector>	// contiguous, no insertion of events in the middle
#include <list>		// fast insertion anywhere, sorting
#include <boost/random.hpp>

using namespace std;

// simulation parameters
int g_simTime=0;			// in ms
int g_simEndTime=UINT_MAX;	// in ms
int g_step=100;				// 100ms step

// highway parameters
float g_speed=30.0;		// 30 m/s
float g_rrange=250.0;	// 250m radio range
float g_lambda=0.0039;	// vehicle density
float g_length=10000;	// highway length
float g_margin=3000.0;	// side margin on highway length
unsigned int g_vID=0;	// global vehicle ID counter
unsigned int g_PacketStartVID=0;	// ID of vehicle that generates the packet
unsigned int g_PacketEndVID=0;		// ID of destination vehicle
unsigned int g_seed=1;				// RV seed
unsigned int g_rsuDensity=1000;		// meters per RSU

class VanetVehicle {
public:
	int vehicleID;
	float position;
	char direction;
	vector<int> packetList; 	// array of packets
	VanetVehicle(int vid, char dir, float pos) {vehicleID=vid; direction=dir; position=pos;}
	VanetVehicle(){vehicleID=0; position=0; direction='F';}
};

class simEvent {
public:
	char eventType; 	// 'V' new vehicle; 'P' new packet
	int time;			// in ms
	float position;		// V only
	char direction;		// V only
	int vehicleID;		// V & P
	int packetID;		// P only
	simEvent(char type, int t, float pos, char dir, int vid) {eventType=type; time=t; position=pos; direction=dir; vehicleID=vid; packetID=0;}
	simEvent(char type, int t, int vid, int pid) {eventType=type; time=t; vehicleID=vid; packetID=pid; direction='P'; position=0.0;}
};

class reHealingTime {
public:
	char type;		// 'B' for best, 'W' for worst
	int startTime;
	int startVID;
	int endVID;
	int delay;
	reHealingTime(char ty, int st, int sVID, int eVID) {type=ty; startTime=st; startVID=sVID; endVID=eVID; delay=INT_MIN;}
};

void AdvanceVehicles(list<VanetVehicle> *Vehicles, unsigned int delta);
void AddPacket (list<VanetVehicle> *Vehicles, int vehicleID, int packetID);
void BroadcastPacket(list<VanetVehicle> *Vehicles, int srcVehicleID, int packetID, bool needsSort);
void ReBroadcastPackets(list<VanetVehicle> *Vehicles);
void DoStatistics(list<VanetVehicle> *Vehicles, int srcVehicleID, int packetID);
// class sorting functions
bool eventCompare (simEvent event1, simEvent event2);
bool vehiclePosCompare (VanetVehicle veh1, VanetVehicle veh2);

#endif /* VANETCSIM_H_ */
