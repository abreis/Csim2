#include "Csim2.h"
using namespace std;

// global variable definition
list<reHealingTime> statList;
list<VanetVehicle> Vehicles;

int main(int argc, char* argv[])
{
	// list of events is local to the main loop
	list<simEvent> eventList;

	// usage info
	if(argc==1)
		{ cout << "usage: " << argv[0] << " seed [length] [lambda] [rsu density]" << endl; exit(1); }

	// get RSU density from CLI
	if(argc>=5) g_rsuDensity=atoi(argv[4]);
	// get lambda from CLI
	if(argc>=4) g_lambda=atof(argv[3]);
	// highway length from CLI
	if(argc>=3) g_length=(float)atoi(argv[2]) + g_margin;	// 1500m buffer on each side
	// random variable seed from CLI
	if(argc>=2) g_seed = atoi(argv[1]);

	// simulation time from time to fill road plus enough to get message across
	g_simEndTime = (int)( (g_length/g_speed)*1000*1.10 );	// ten percent more
	g_simEndTime += (int)(80*g_length); 	// conservative 80 seconds per kilometer [s/km]*[m] = ms
	g_simEndTime = (int)(g_simEndTime*(0.0039/g_lambda));	// factor density

	// Packet start time must be enough for the road to fill
	g_packetStart = (unsigned int)( ((g_length)/5000)*200*1000 );

	cout << fixed << setprecision(4);

	cout << "Simulation Start" << '\n';
	cout << "Parameters:" <<
			"\n\tSpeed \t" << g_speed <<
			"\n\tRange \t" << g_rrange <<
			"\n\tLambda \t" << g_lambda <<
			"\n\tStep \t" << g_step <<
			"\n\tStart \t" << g_packetStart <<
			"\n\tEnd \t" << g_simEndTime <<
			"\n\tSeed \t" << g_seed <<
			'\n';

	cout << fixed << setprecision(1);

	// Random variable init
	boost::mt19937 rng(g_seed);
	boost::exponential_distribution<float> exp( g_lambda*g_speed/1000 );
	boost::variate_generator< boost::mt19937, boost::exponential_distribution<float> > expgen(rng,exp) ;
		// now call expgen() to get a number
	// Uniform distribution
	boost::uniform_smallint<int> unif(0, g_rsuDensity);
	boost::variate_generator< boost::mt19937, boost::uniform_smallint<int> > unifgen(rng, unif);
		// now call unifgen() to get an int


	// Initial vehicle, RSU, packet setup
	{
		// randomize first RSU position with a Uniform distribution
		float startingRSUpos = unifgen();
		cout << "\tRSU1\t" << startingRSUpos << '\n';

		// place RSUs
		if(g_rsuDensity!=0)
		{
			// one RSU every 1km
			int nRSU = (int)(g_length)/g_rsuDensity;
			// schedule RSU placement event
			for(int i=0; i<nRSU; i++)
			{
				simEvent eventRSU ('R', 0, startingRSUpos+(float)i*g_rsuDensity, 'R', ++g_vID);	// type, time, position, direction, vID
				eventList.push_back(eventRSU);
			}
		}
	}

	// Exponential vehicle generation event loop
	{
		// fill list with WEST-bound vehicles
		{
			int accum = 0;
			while(accum < g_simEndTime) // fill event list with enough events
			{
				int expvalue = (int)expgen();	// get exponential value

				cout << "INFO exponential W " <<  expvalue << '\n';

				// min jump is 100; expvalue<100 occurs very rarely
				if(expvalue<100) expvalue=100;
				// crop expgen() to a maximum bound
				if(expvalue>=5*(1000/(g_lambda*g_speed))) expvalue=(int)(5*(1000/(g_lambda*g_speed)) );

				// increment the 'total time covered by events' accumulator
				accum+= (int) expvalue;

				simEvent eventWestVehicle ('V', accum, 0.0, 'W', ++g_vID);	// type, time, position, direction, vID
				eventList.push_back(eventWestVehicle);
			}
		}

		// no eastbound vehicles for the connected RSU model
	}

	// Sort event list
	eventList.sort(eventCompare);


	// Find the vehicles nearest to gmargin/2 (end) and (glength-gmargin/2) (start)
	{
		// times
			// at time=g_packetStart, the vehicle closest to gmargin/2 is the one created at
		int dstPacketTimeMark=g_packetStart-1000*(g_margin/2)/g_speed;
		int srcPacketTimeMark=g_packetStart-1000*(g_length-g_margin/2)/g_speed;
		int srcPacketPosition=0, dstPacketPosition=0;

		{
			list<simEvent>::iterator itMark = eventList.begin();
			// move to the first vehicle @ time >=dstPacketTimeMark
			for(; itMark != eventList.end() && itMark->time<dstPacketTimeMark; itMark++);

			if(itMark->direction!='W')
			{
				int jump=1, power=2;
				while(itMark->direction!='W')
				{
					// now jump +1,-1,+2,-2,... until direction=='W'
					if(jump>0) for(int i=0; i!=jump && itMark!=eventList.end(); i++) itMark++;
					if(jump<0) for(int i=0; i!=jump && itMark!=eventList.begin(); i--) itMark--;

					jump=power*pow(-1.0,power+1); power++;
				}
			}
			g_PacketEndVID=itMark->vehicleID;	// this vehicle is the packet's destination
			dstPacketPosition=g_speed*(g_packetStart - itMark->time)/1000;
		}

		{
			list<simEvent>::iterator itMark = eventList.begin();
			// move to the first vehicle @ time >=srcPacketTimeMark
			for(; itMark != eventList.end() && itMark->time<srcPacketTimeMark; itMark++);

			if(itMark->direction!='W')
			{
				int jump=1, power=2;
				while(itMark->direction!='W')
				{
					// now jump +1,-1,+2,-2,... until direction=='W'
					if(jump>0) for(int i=0; i!=jump; i++) itMark++;
					if(jump<0) for(int i=0; i!=jump; i--) itMark--;

					jump=power*pow(-1.0,power+1); power++;
				}
			}
			g_PacketStartVID=itMark->vehicleID;	// this vehicle is the packet source
			srcPacketPosition=g_speed*(g_packetStart - itMark->time)/1000;
		}

		cout << "\tDEBUG dstTime " << dstPacketTimeMark << " vehicle picked " << g_PacketEndVID << '\n';
		cout << "\tDEBUG srcTime " << srcPacketTimeMark << " vehicle picked " << g_PacketStartVID << '\n';
		cout << "\tDEBUG distance betweet SRC and DST: " << (srcPacketPosition-dstPacketPosition) << '\n';

		// Schedule creation of first packet
		simEvent packetEvent1 ('P', g_packetStart, g_PacketStartVID, 1); 	// type, time, vID, pID
		eventList.push_back(packetEvent1);
	}

	// Re-sort event list (new entry added)
	eventList.sort(eventCompare);

	// Print event list
	cout << "Event List:" << '\n';
	for( list<simEvent>::iterator it = eventList.begin(); it != eventList.end(); ++it )
	{
	    cout << '\t' << it->eventType << ' ' << it->time;
	    if(it->eventType=='V') cout << " pos " << it->position << " dir " << it->direction << " vID " << it->vehicleID << '\n';
	    if(it->eventType=='R') cout << " pos " << it->position << " vID " << it->vehicleID << '\n';
	    if(it->eventType=='P') cout << " vID " << it->vehicleID << " pID " << it->packetID << '\n';
	}


	// MAIN LOOP
	for (g_simTime=0; g_simTime<=g_simEndTime; g_simTime+=g_step)
	{
		// Advance all vehicles on each step
		if(g_simTime!=0)
			AdvanceVehicles(g_step);

		// Process events
		{
			while( eventList.front().time <= g_simTime)
			{	// fetch event to process
				simEvent tempEvent = eventList.front();

				// log event
				{
					cout << "INFO Processing event " << tempEvent.eventType << ' ' << tempEvent.time;
					if(tempEvent.eventType=='V') cout << " pos " << tempEvent.position << " dir " << tempEvent.direction << " vID " << tempEvent.vehicleID << '\n';
					if(tempEvent.eventType=='R') cout << " pos " << tempEvent.position << " vID " << tempEvent.vehicleID << '\n';
					if(tempEvent.eventType=='P') cout << " vID " << tempEvent.vehicleID << " pID " << tempEvent.packetID << '\n';
				}

				// process event
				{
					switch(tempEvent.eventType)
					{
						case 'V':
						{
							VanetVehicle tempVeh(tempEvent.vehicleID, tempEvent.direction, tempEvent.position);	// vID, dir, pos, isRSU
							Vehicles.push_back(tempVeh);
							cout << "LOG " << ((float)g_simTime)/1000.0 << " V " << tempEvent.vehicleID << ' ' << tempEvent.position << ' ' << tempEvent.direction << '\n';
							break;
						}
						case 'R':
						{
							VanetVehicle tempRSU(tempEvent.vehicleID, 'R', tempEvent.position);	// vID, dir, pos, isRSU
							Vehicles.push_back(tempRSU);
							cout << "LOG " << ((float)g_simTime)/1000.0 << " R " << tempEvent.vehicleID << ' ' << tempEvent.position << '\n';
							break;
						}
						case 'P':
							AddPacket(tempEvent.vehicleID, tempEvent.packetID, true);
							break;
						default:
							break;
					}
				}
				// erase event
				eventList.pop_front();
			}	// end process event
		}

		// propagate packets on all vehicles
		ReBroadcastPackets();

	}	// end MAIN_LOOP

	// should never get here
	return 1;
}

void AdvanceVehicles(unsigned int delta)
{
	// Advance all Vehicles, skip RSUs
    for( list<VanetVehicle>::iterator iter = Vehicles.begin();
    		iter != Vehicles.end();
    			++iter )
    {
    	if(iter->direction=='W')
    		iter->position += ( (float)delta/1000 * g_speed ); 		// time is in ms
    	else if(iter->direction=='E')
        	iter->position -= ( (float)delta/1000 * g_speed ); 		// time is in ms

    	if(g_simTime%(10*1000)==0)	// log 'C' every x*1000 seconds
    		cout << "LOG " << ((float)g_simTime)/1000.0 << " C " << iter->vehicleID << ' ' << iter->position << '\n';
    }
}

void AddPacket (unsigned int vehicleID, int packetID, bool needsSort)
{
	// sort list of vehicles
	if(needsSort) Vehicles.sort(vehiclePosCompare);

	// move iter to desired vehicle ID
	list<VanetVehicle>::iterator iter = Vehicles.begin();
	while( iter->vehicleID != vehicleID ) iter++;

	// see if the vehicle already has the packet
	bool hasPacket=false; 	// hasPacket flag
	for(vector<int>::iterator packetIter = iter->packetList.begin();
			packetIter != iter->packetList.end();
				++packetIter )
				{
					if(*packetIter == packetID ) { hasPacket=true; break; }
				}

	// add a packet to a vehicle's vector of packets, if it is not present
    if(!hasPacket)
    {
    	iter->packetList.push_back(packetID);

    	// log 'P' event
    	cout << "LOG " << ((float)g_simTime)/1000.0 << " P " << iter->vehicleID << ' ' << iter->position << ' ' << packetID << '\n';

    	if(iter->direction=='R')
    		for(list<VanetVehicle>::iterator iterRSU = Vehicles.begin(); iterRSU!=Vehicles.end(); iterRSU++ )
    		{
    			if(iterRSU->direction=='R' && iterRSU->vehicleID!=iter->vehicleID)
    				AddPacket(iterRSU->vehicleID, packetID, false);
    		}


    	// see if it is a special case, and count
    	DoStatistics(vehicleID, packetID);

    	// if is new, then instruct vehicle to rebroadcast it immediately
    	BroadcastPacket(vehicleID, packetID, true);
    }
}

void BroadcastPacket(unsigned int srcVehicleID, int packetID, bool needsSort)
{
	// sort list of vehicles
	if(needsSort) Vehicles.sort(vehiclePosCompare);

	// move iter to desired vehicle ID
	list<VanetVehicle>::iterator iter = Vehicles.begin();
	while( iter->vehicleID != srcVehicleID ) iter++;

	// tag position
	float srcPosition = iter->position;

	// we'll need these to be mutable
	list<VanetVehicle>::iterator iterFW = iter;
	list<VanetVehicle>::iterator iterBW = iter;

	// broadcast forward
	if( iterFW != Vehicles.end()) iterFW++;
	while( iterFW->position <= srcPosition+250 && iterFW != Vehicles.end() )
	{
		AddPacket(iterFW->vehicleID, packetID, false);
		iterFW++;
	}

	// broadcast backward
	if( iterBW != Vehicles.begin()) iterBW--;
	while( iterBW->position >= srcPosition-250 && iterBW != Vehicles.begin() )
	{
		AddPacket(iterBW->vehicleID, packetID, false);
		iterBW--;
	}

}

void ReBroadcastPackets(void)
{
	// for each packet in each vehicle, rebroadcast that packet

	// sort Vehicles list only once here
	Vehicles.sort(vehiclePosCompare);

	// Vehicle iterator
    for( list<VanetVehicle>::iterator vehIter = Vehicles.begin();
    		vehIter != Vehicles.end();
    			++vehIter )
    {
    	// Packet iterator
        for(vector<int>::iterator packetIter = vehIter->packetList.begin();
        		packetIter != vehIter->packetList.end();
        			++packetIter )
        {
        	// per vehicle, per packet {vehIter, packetIter} code goes here
        	BroadcastPacket(vehIter->vehicleID, *packetIter, false);
        }
    }
}

void DoStatistics(unsigned int srcVehicleID, int packetID)
{
	// if this is the last vehicle, print logs and die
	if(srcVehicleID == g_PacketEndVID)
	{
		PrintStatistics();
		exit(1);
	}
}

void PrintStatistics(void)
{
	cout << "INFO simulation endtime " << g_simTime << " start " << g_packetStart << " delay " << (g_simTime-g_packetStart)  << '\n';
}

bool eventCompare (simEvent event1, simEvent event2)
{
	if(event1.time < event2.time)
		return true;
	else
		return false;
}

bool vehiclePosCompare (VanetVehicle veh1, VanetVehicle veh2)
{
	if(veh1.position < veh2.position)
		return true;
	else
		return false;
}
