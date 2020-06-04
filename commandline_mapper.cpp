#include <iostream>
#include <string>
#include "filereader.h"
#include "multimeter.h"
#include "pubSysCls.h"
#include <fstream>
#include <iomanip>

// Send message and wait for newline
void msgUser(const char* msg) {
	std::cout << msg;
	getchar();
}

void printVec3dList(const std::vector<Vec3d>& list)
{
	for (const Vec3d& p : list)
		std::cout << p << std::endl;
}

void measureAndPrint(MultiMeter& mm)
{
	std::cout << mm.measureBx() << " " << mm.measureBy() << " " << mm.measureBz() << std::endl;
}

int Map(const std::vector<Vec3d>& list, MultiMeter& mm, std::vector<Vec3d>& result)
{
	std::vector<Vec3d> emptyResult;
	result = emptyResult;
	Vec3d probePosition{ 0, 0, 0 };

	int xyCmToCounts = 750;
	int zCmToCounts = 3000;

	int ACC_LIM_RPM_PER_SEC = 30;
	int VEL_LIM_RPM = 50;
	int TIME_TILL_TIMEOUT = 10000;
	int MOVE_DISTANCE_CNTS;

	size_t portCount = 0;
	std::vector<std::string> comHubPorts;

	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	sFnd::SysManager* myMgr = sFnd::SysManager::Instance();                           //Create System Manager myMgr

	try
	{
		printf("SC-HUB SysManager is Initialized\n");   //Print to console
		sFnd::SysManager::FindComHubPorts(comHubPorts);
		printf("Found %d SC Hubs\n", comHubPorts.size());

		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {

			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());    //define the first SC Hub port (port 0) to be associated 
											// with COM portnum (as seen in device manager)
		}

		if (portCount > 0) {
			//printf("\n I will now open port \t%i \n \n", portnum);
			myMgr->PortsOpen(portCount);             //Open the port


			sFnd::IPort& myPort = myMgr->Ports(0);


			printf(" Port[%d]: state=%d, nodes=%d\n",
				myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());

			sFnd::INode& theNodeZ = myPort.Nodes(0);
			theNodeZ.EnableReq(false);
			myMgr->Delay(200);
			theNodeZ.Status.AlertsClear();
			theNodeZ.Motion.NodeStopClear();
			theNodeZ.EnableReq(true);
			printf("Node \t%zi enabled, Serial #: %d\n", 0, theNodeZ.Info.SerialNumber.Value());
			double timeoutZ = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;

			while (!theNodeZ.Motion.IsReady()) {
				if (myMgr->TimeStampMsec() > timeoutZ) {
					printf("Error: Timed out waiting for Node %d to enable\n", 0);
					msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}

			theNodeZ.Motion.MoveWentDone();
			theNodeZ.AccUnit(sFnd::INode::RPM_PER_SEC);			    //Set the units for Acceleration to RPM/SEC
			theNodeZ.VelUnit(sFnd::INode::RPM);					    //Set the units for Velocity to RPM
			theNodeZ.Motion.AccLimit = ACC_LIM_RPM_PER_SEC * 4;		//Set Acceleration Limit (RPM/Sec)
			theNodeZ.Motion.VelLimit = VEL_LIM_RPM * 4; 		    //Set Velocity Limit (RPM)

			sFnd::INode& theNodeX = myPort.Nodes(1);
			theNodeX.EnableReq(false);
			myMgr->Delay(200);
			theNodeX.Status.AlertsClear();
			theNodeX.Motion.NodeStopClear();
			theNodeX.EnableReq(true);
			printf("Node \t%zi enabled, Serial #: %d\n", 1, theNodeX.Info.SerialNumber.Value());
			double timeoutX = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;

			while (!theNodeX.Motion.IsReady()) {
				if (myMgr->TimeStampMsec() > timeoutX) {
					printf("Error: Timed out waiting for Node %d to enable\n", 1);
					msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}

			theNodeX.Motion.MoveWentDone();
			theNodeX.AccUnit(sFnd::INode::RPM_PER_SEC);			//Set the units for Acceleration to RPM/SEC
			theNodeX.VelUnit(sFnd::INode::RPM);					//Set the units for Velocity to RPM
			theNodeX.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
			theNodeX.Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)

			sFnd::INode& theNodeY = myPort.Nodes(2);
			theNodeY.EnableReq(false);
			myMgr->Delay(200);
			theNodeY.Status.AlertsClear();
			theNodeY.Motion.NodeStopClear();
			theNodeY.EnableReq(true);
			printf("Node \t%zi enabled, Serial #: %d\n", 2, theNodeY.Info.SerialNumber.Value());
			double timeoutY = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;

			while (!theNodeY.Motion.IsReady()) {
				if (myMgr->TimeStampMsec() > timeoutY) {
					printf("Error: Timed out waiting for Node %d to enable\n", 2);
					msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}

			theNodeY.Motion.MoveWentDone();
			theNodeY.AccUnit(sFnd::INode::RPM_PER_SEC);			//Set the units for Acceleration to RPM/SEC
			theNodeY.VelUnit(sFnd::INode::RPM);					//Set the units for Velocity to RPM
			theNodeY.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
			theNodeY.Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)

			size_t indexPoint = 1;
			for (const Vec3d& p : list)
			{
				Vec3d measurement;

				Vec3d displacement = p - probePosition;
				int MOVE_DISTANCE_CNTSZ = std::round(displacement.z * zCmToCounts);
				int MOVE_DISTANCE_CNTSX = std::round(displacement.x * xyCmToCounts);
				int MOVE_DISTANCE_CNTSY = - std::round(displacement.y * xyCmToCounts);

				int YOFFSET = 1.5 * xyCmToCounts;
				int YGETBACK = -3.0 * xyCmToCounts;

				//printf("Moving Node \t%zi \n", 0);
				theNodeZ.Motion.MovePosnStart(MOVE_DISTANCE_CNTSZ);			//Execute 10000 encoder count move
				//printf("%f estimated time.\n", theNodeZ.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSZ));
				timeoutZ = myMgr->TimeStampMsec() + theNodeZ.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSZ) + 100;			//define a timeout in case the node is unable to enable

				while (!theNodeZ.Motion.MoveIsDone()) {
					if (myMgr->TimeStampMsec() > timeoutZ) {
						printf("Error: Timed out waiting for move to complete\n");
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
				//printf("Node \t%zi Move Done\n", 0);


				//printf("Moving Node \t%zi \n", 1);
				theNodeX.Motion.MovePosnStart(MOVE_DISTANCE_CNTSX);			//Execute 10000 encoder count move
				//printf("%f estimated time.\n", theNodeX.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSX));
				timeoutX = myMgr->TimeStampMsec() + theNodeX.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSX) + 100;			//define a timeout in case the node is unable to enable

				while (!theNodeX.Motion.MoveIsDone()) {
					if (myMgr->TimeStampMsec() > timeoutX) {
						printf("Error: Timed out waiting for move to complete\n");
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
				//printf("Node \t%zi Move Done\n", 1);


				//printf("Moving Node \t%zi \n", 2);
				theNodeY.Motion.MovePosnStart(MOVE_DISTANCE_CNTSY);			//Execute 10000 encoder count move
				//printf("%f estimated time.\n", theNodeY.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSY));
				timeoutY = myMgr->TimeStampMsec() + theNodeY.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSY) + 100;			//define a timeout in case the node is unable to enable

				while (!theNodeY.Motion.MoveIsDone()) {
					if (myMgr->TimeStampMsec() > timeoutY) {
						printf("Error: Timed out waiting for move to complete\n");
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
				//printf("Node \t%zi Move Done\n", 2);

				myMgr->Delay(500);
				measurement.x = mm.measureBx();
				std::cout << "Point #: " << indexPoint << "/" << list.size() << std::endl;
				indexPoint++;
				std::cout << "Bx : " << measurement.x << std::endl;
				myMgr->Delay(500);
				//printf("Moving Node \t%zi \n", 2);
				theNodeY.Motion.MovePosnStart(YOFFSET);			//Execute 10000 encoder count move
				//printf("%f estimated time.\n", theNodeY.Motion.MovePosnDurationMsec(YOFFSET));
				timeoutY = myMgr->TimeStampMsec() + theNodeY.Motion.MovePosnDurationMsec(YOFFSET) + 100;			//define a timeout in case the node is unable to enable

				while (!theNodeY.Motion.MoveIsDone()) {
					if (myMgr->TimeStampMsec() > timeoutY) {
						printf("Error: Timed out waiting for move to complete\n");
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
				//printf("Node \t%zi Move Done\n", 2);
				myMgr->Delay(500);
				measurement.y = mm.measureBy();
				std::cout << "By : " << measurement.y << std::endl;
				myMgr->Delay(500);
				//printf("Moving Node \t%zi \n", 2);
				theNodeY.Motion.MovePosnStart(YOFFSET);			//Execute 10000 encoder count move
				//printf("%f estimated time.\n", theNodeY.Motion.MovePosnDurationMsec(YOFFSET));
				timeoutY = myMgr->TimeStampMsec() + theNodeY.Motion.MovePosnDurationMsec(YOFFSET) + 100;			//define a timeout in case the node is unable to enable

				while (!theNodeY.Motion.MoveIsDone()) {
					if (myMgr->TimeStampMsec() > timeoutY) {
						printf("Error: Timed out waiting for move to complete\n");
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
				//printf("Node \t%zi Move Done\n", 2);
				myMgr->Delay(500);
				measurement.z = mm.measureBz();
				std::cout << "Bz : " << measurement.z << std::endl;
				myMgr->Delay(500);
				//printf("Moving Node \t%zi \n", 2);
				theNodeY.Motion.MovePosnStart(YGETBACK);			//Execute 10000 encoder count move
				//printf("%f estimated time.\n", theNodeY.Motion.MovePosnDurationMsec(YGETBACK));
				timeoutY = myMgr->TimeStampMsec() + theNodeY.Motion.MovePosnDurationMsec(YGETBACK) + 100;			//define a timeout in case the node is unable to enable

				while (!theNodeY.Motion.MoveIsDone()) {
					if (myMgr->TimeStampMsec() > timeoutY) {
						printf("Error: Timed out waiting for move to complete\n");
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
				//printf("Node \t%zi Move Done\n", 2);

				result.push_back(measurement);

				probePosition = probePosition + displacement;
			}

			Vec3d origin{ 0,0,0 };
			Vec3d displacement = origin - probePosition;

			int MOVE_DISTANCE_CNTSZ = std::round(displacement.z * zCmToCounts);
			int MOVE_DISTANCE_CNTSX = std::round(displacement.x * xyCmToCounts);
			int MOVE_DISTANCE_CNTSY = -std::round(displacement.y * xyCmToCounts);

			//printf("Moving Node \t%zi \n", 0);
			theNodeZ.Motion.MovePosnStart(MOVE_DISTANCE_CNTSZ);			//Execute 10000 encoder count move
			//printf("%f estimated time.\n", theNodeZ.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSZ));
			timeoutZ = myMgr->TimeStampMsec() + theNodeZ.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSZ) + 100;			//define a timeout in case the node is unable to enable

			while (!theNodeZ.Motion.MoveIsDone()) {
				if (myMgr->TimeStampMsec() > timeoutZ) {
					printf("Error: Timed out waiting for move to complete\n");
					msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}
			//printf("Node \t%zi Move Done\n", 0);


			//printf("Moving Node \t%zi \n", 1);
			theNodeX.Motion.MovePosnStart(MOVE_DISTANCE_CNTSX);			//Execute 10000 encoder count move
			//printf("%f estimated time.\n", theNodeX.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSX));
			timeoutX = myMgr->TimeStampMsec() + theNodeX.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSX) + 100;			//define a timeout in case the node is unable to enable

			while (!theNodeX.Motion.MoveIsDone()) {
				if (myMgr->TimeStampMsec() > timeoutX) {
					printf("Error: Timed out waiting for move to complete\n");
					msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}
			//printf("Node \t%zi Move Done\n", 1);


			//printf("Moving Node \t%zi \n", 2);
			theNodeY.Motion.MovePosnStart(MOVE_DISTANCE_CNTSY);			//Execute 10000 encoder count move
			//printf("%f estimated time.\n", theNodeY.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSY));
			timeoutY = myMgr->TimeStampMsec() + theNodeY.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTSY) + 100;			//define a timeout in case the node is unable to enable

			while (!theNodeY.Motion.MoveIsDone()) {
				if (myMgr->TimeStampMsec() > timeoutY) {
					printf("Error: Timed out waiting for move to complete\n");
					msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}
			//printf("Node \t%zi Move Done\n", 2);

			// Disable all nodes
			myPort.Nodes(0).EnableReq(false);
			myPort.Nodes(1).EnableReq(false);
			myPort.Nodes(2).EnableReq(false);

		}
		else {
			printf("Unable to locate SC hub port\n");

			msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key

			return -1;  //This terminates the main program
		}

	}
	catch (sFnd::mnErr & theErr)    //This catch statement will intercept any error from the Class library
	{
		printf("Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key

		return -1;  //This terminates the main program
	}

	myMgr->PortsClose();
}

int main()
{
	std::cout << " -------------------------------------------" << std::endl;
	std::cout << "|  Command Line Auto Field Mapper v0.1      |" << std::endl;
	std::cout << "|  Umit H. Coskun. Last update : 2/20/2020  |" << std::endl;
	std::cout << " -------------------------------------------" << std::endl;

	std::ofstream outfile;

	// Init multimeter class
	MultiMeter dmm;
	// Declare variables
	std::vector<Vec3d> coordinateList;
	std::string outputPath;
	std::vector<Vec3d> resultList;
	char input;
	dmm.getVISA_Addr();
	while (1)
	{
		std::cout << " ---------------------------------------------------------------" << std::endl;
		std::cout << "|  Test measure[t], Load coordinates[l], Print coordinates[p]   |" << std::endl;
		std::cout << "|  Enter output file name[o], Start Mapping[m], Quit[q]         |" << std::endl;
		std::cout << " ---------------------------------------------------------------" << std::endl;
		std::cout << "Command[]: ";

		std::cout << std::setw(1);
		std::cin >> input;
			switch (input)
			{
			case 't':
			{
				measureAndPrint(dmm);
			}
			break;

			case 'q':
			{
				return 0;
			}
			break;

			case 'p':
			{
				printVec3dList(coordinateList);
			}
			break;

			case 'l':
			{
				std::string path;
				std::cout << "Enter the coordinate file name: ";
				std::cin >> path;
				FileReader reader(path);
				coordinateList = reader.getCoordinates();
			}
			break;

			case 'o':
			{
				std::cout << "Enter the output file name: ";
				std::cin >> outputPath;
				std::cout << "Output filename set to : " << outputPath << std::endl;
			}
			break;

			case 'm':
			{
				std::cout << "This will start automatic mapping. Please verify following: " << std::endl;
				std::cout << "Coordinates :" << std::endl;
				printVec3dList(coordinateList);
				std::cout << "Output file name: " << std::endl;
				std::cout << outputPath << std::endl;

				std::cout << "Do you really want to start mapping ? [y/n]: ";
				char answer;
				std::cin >> answer;
				switch (answer)
				{
				case 'y':
				{
					outfile.open(outputPath);
					Map(coordinateList, dmm, resultList);
					printVec3dList(resultList);
					for (size_t i = 0; i < coordinateList.size(); i++)
						outfile << coordinateList.at(i).x << " " << coordinateList.at(i).y << " " << coordinateList.at(i).z << " " << resultList.at(i).x << " " << resultList.at(i).y << " " << resultList.at(i).z << std::endl;

					outfile.close();
				}
				break;
				}
			}
			break;
			}
		
	}

}
