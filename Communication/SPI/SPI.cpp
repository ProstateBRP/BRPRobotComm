//============================================================================
// Name        : SPI.cpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where SPI Communication is handled.
//============================================================================

#include "SPI.hpp"

// SPI Communication Constructor
SPI::SPI(Packets *packets, FPGA_Utilities *fpga_util){
	_packets = packets;
	_fpga_util = fpga_util;
}

//-------------------------------------------------------------------
// Method for initializing the FPGA
// Input: None
// Output: void
//-------------------------------------------------------------------
void SPI::InitializeFPGA(){
	_fpga_util->InitializeFPGA();
}

//-------------------------------------------------------------------
// Method will call SPI communication command for each of the cards
// and then process the packets
// Input: card id, uint32_t* outgoingPacket, uint32_t* incomingPacket
// Output: void
//-------------------------------------------------------------------
incomingPacket SPI::SyncCard(int card_id, struct outgoingPacket outPacket){
	uint32_t* incomingPacket;
	uint32_t* outgoingPacket;

	struct incomingPacket inPacket;
	outPacket.startWord_packId = 0xfefefe01;
	outPacket.systemState = 0x00000000;

	incomingPacket = (uint32_t*)calloc(NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32Size_IncomingPacket0, sizeof(uint32_t) );
	outgoingPacket = (uint32_t*)calloc(NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32Size_OutgoingPacket0, sizeof(uint32_t));

	PackOutgoingPacket(outPacket, outgoingPacket);

	//PerformSPITransaction(card_id, outgoingPacket, incomingPacket);

	UnpackIncomingPacket(incomingPacket, &inPacket);

	PrintPacket(card_id,2, incomingPacket);

	free(incomingPacket);
	free(outgoingPacket);
	return inPacket;
}

//-------------------------------------------------------------------
// Method that prints packets coming into the controller from a specific card
// Input: card id, uint32_t* incomingPacket
// Output: void
//-------------------------------------------------------------------
void SPI::PrintPacket(int card_id, int desiredCard, uint32_t* incomingPacket){
	uint32_t checksum = 0xffffffff;
	if (card_id != desiredCard ){return;}
	// Print whole packet before Checksum
	// printf("*************Card %d**********Before Checksum*************\n", card_id);
	// for(int i=0;i<32;i++)
	// {
	// 	printf("%x \n", incomingPacket[i]);
	// }
	// Calculate the checksum
	for(int i=0;i<31;i++){
		checksum -=incomingPacket[i];
	}

	if(checksum == incomingPacket[31])
	{
		// printf("*************Card %d***********************\n", card_id);
		// for(int i=0;i<32;i++){
		// 		printf("%x \n", incomingPacket[i]);
		// }
		printf("1 \n");
	}

	else
	{
		// printf("Bad Checksum \n");
		printf("0 \n");
	}
	//=======================================================================================
	// if(checksum == incomingPacket[0])// && 0x89abcdef ==  incomingPacket[1])
	// {
	// 	// printf("*************Card %d***********************\n", card_id);
	// 	// for(int i=0;i<32;i++){
	// 	// 		printf("%x \n", incomingPacket[i]);
	// 	// }
	// 	printf("1 \n");
	// }
	// else
	// {
	// 	printf("0 \n");
	// }
}
void SPI::PrintGoodPacketCount(int card_id, int desiredCard, uint32_t* incomingPacket)
{
	uint32_t checksum = 0xffffffff;
	if (card_id != desiredCard ){return;}
	
	for(int i=0;i<31;i++){
		checksum -=incomingPacket[i];
	}

	if (counter < 100)
	{
		counter ++;
	}
	else
	{
		counter = 0;
		std::cout << "Good packets: " << good << std::endl;
		good = 0;
	}
	if(checksum == incomingPacket[31]){good++;}

}
//-------------------------------------------------------------------
// Method for writing SPI packets
// Input: the current card id, uint32_t* outgoingPacket
// Output: void
//-------------------------------------------------------------------
void SPI::WritePacket(int card, uint32_t* outgoingPacket){

	NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32 controlName;
	switch(card)
	{
		case 0:	controlName = NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32_OutgoingPacket0;
				break;
		case 1:	controlName = NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32_OutgoingPacket1;
				break;
		case 2:	controlName = NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32_OutgoingPacket2;
				break;
		case 3:	controlName = NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32_OutgoingPacket3;
				break;
		case 4:	controlName = NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32_OutgoingPacket4;
				break;
		case 5:	controlName = NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32_OutgoingPacket5;
				break;
		case 6:	controlName = NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32_OutgoingPacket6;
				break;
		case 7:	controlName = NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32_OutgoingPacket7;
				break;
		//default:controlName = NiFpga_NeuroRobotAllCardsSPI_ControlArrayU32_OutgoingPacket0;
	}

	if( controlName != 0x00000)
		_fpga_util->WriteArrayU32(controlName, outgoingPacket);
}

//-------------------------------------------------------------------
// Method for reading the SPI packets
// Input: receives the current card id, uint32_t* outgoingPacket
// Output: void
//-------------------------------------------------------------------
void SPI::ReadPacket(int card, uint32_t* incomingPacket){
	NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32 indicatorName;
	switch(card)
	{
		case 0:indicatorName = NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32_IncomingPacket0;
				break;
		case 1:indicatorName = NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32_IncomingPacket1;
				break;
		case 2:indicatorName = NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32_IncomingPacket2;
				break;
		case 3:indicatorName = NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32_IncomingPacket3;
				break;
		case 4:indicatorName = NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32_IncomingPacket4;
				break;
		case 5:indicatorName = NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32_IncomingPacket5;
				break;
		case 6:indicatorName = NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32_IncomingPacket6;
				break;
		case 7:indicatorName = NiFpga_NeuroRobotAllCardsSPI_IndicatorArrayU32_IncomingPacket7;
				break;
	}
	if(indicatorName !=  0x0000)
		_fpga_util->ReadArrayU32(indicatorName, incomingPacket);
}

//-------------------------------------------------------------------
// Method for Toggling the Transact bit, indicating SPI is ready to receive
// Input: the current card id, uint8_t value
// Output: void
//-------------------------------------------------------------------
void SPI::ToggleTransact(uint8_t value){

	_fpga_util->WriteBool(NiFpga_NeuroRobotAllCardsSPI_ControlBool_Transact0, value);
	_fpga_util->WriteBool(NiFpga_NeuroRobotAllCardsSPI_ControlBool_Transact1, value);
	_fpga_util->WriteBool(NiFpga_NeuroRobotAllCardsSPI_ControlBool_Transact2, value);
	_fpga_util->WriteBool(NiFpga_NeuroRobotAllCardsSPI_ControlBool_Transact3, value);
	_fpga_util->WriteBool(NiFpga_NeuroRobotAllCardsSPI_ControlBool_Transact4, value);
	_fpga_util->WriteBool(NiFpga_NeuroRobotAllCardsSPI_ControlBool_Transact5, value);
	_fpga_util->WriteBool(NiFpga_NeuroRobotAllCardsSPI_ControlBool_Transact6, value);
	_fpga_util->WriteBool(NiFpga_NeuroRobotAllCardsSPI_ControlBool_Transact7, value);

}

//-------------------------------------------------------------------
// Method for packing the outgoing data
// Input: takes struct outgoingPacket packet, uint32_t* packedData
// Output: void
//-------------------------------------------------------------------
void SPI::PackOutgoingPacket(struct outgoingPacket packet, uint32_t* packedData){
	uint32_t checksum = 0xffffffff;
	int i=0;

	packedData[0] = packet.startWord_packId;
	packedData[1] = packet.systemState;

	packedData[2] = ((packet.wave1_table && 0xFF) << 24) | ((packet.wave1_phase && 0xFFFF) << 8) | (packet.wave1_amp && 0xFF);
	packedData[3] = packet.wave1_freq;

	packedData[4] = ((packet.wave2_table && 0xFF) << 24) | ((packet.wave2_phase && 0xFFFF) << 8) | ((packet.wave2_amp) && 0xFF);
	packedData[5] = packet.wave2_freq;
	packedData[14] = packet.amp_enable;

	for(i=0;i<31;i++){
		checksum -=packedData[i];
	}
	packedData[31] = checksum;
}

//-------------------------------------------------------------------
// Method for unpacking the incoming data
// Input: takes uint32_t* unpackedData, struct incomingPacket* packet
// Output: void
//-------------------------------------------------------------------
void SPI::UnpackIncomingPacket(uint32_t* unpackedData, incomingPacket* packet){
	uint32_t checksum = 0xFFFFFFFF; // Starting value for the checksum

	// Compute checksum locally. We will compare this value with checksum sent by cards.
	for(int i=0;i<31;i++){
		checksum -=unpackedData[i];
	}

//	switch(packet->startWord_packId) {
	// Different messages should have different start words with different procedures
	switch(unpackedData[0]){
		case(0xfefefe01):
			// Check if the checksums matched else ignore packet
			// TODO: Warn high level code about stale data
			if (checksum != unpackedData[31]) {
//				printf("Bad Checksum! \n");

			} else {

				packet->startWord_packId = unpackedData[0];
				packet->systemState = unpackedData[1];
				packet->diffPairs_state = (unpackedData[2] & 0xFFFF0000) >> 16;
				packet->diffPairs_fault = unpackedData[2] & 0x0000FFFF;
				packet->encoderCount1 = unpackedData[3];
				packet->adc1 = (unpackedData[7] & 0xFFFF0000) >> 16;
				packet->adc2 = (unpackedData[7] & 0x0000FFFF);
				packet->adc3 = (unpackedData[8] & 0xFFFF0000) >> 16;
				packet->adc4 = (unpackedData[8] & 0x0000FFFF);
				if((packet->diffPairs_state & 0x0008) == 0){
					packet->home_limit = 1;
				}else{
					packet->home_limit = 0;
				}
			}

	}

}

//-------------------------------------------------------------------
// Method for reading in the current state of the FPGA
// Input: takes the id of the current card
// Output: returns a CurrentState type -- see .h file
//-------------------------------------------------------------------
CurrentState SPI::ReadCurrentState(int card){
	NiFpga_NeuroRobotAllCardsSPI_IndicatorU16 controlName;
	switch(card)
	{
		case 0:	controlName = NiFpga_NeuroRobotAllCardsSPI_IndicatorU16_CurrentState0;
				break;
		case 1:	controlName = NiFpga_NeuroRobotAllCardsSPI_IndicatorU16_CurrentState1;
				break;
		case 2:	controlName = NiFpga_NeuroRobotAllCardsSPI_IndicatorU16_CurrentState2;
				break;
		case 3:	controlName = NiFpga_NeuroRobotAllCardsSPI_IndicatorU16_CurrentState3;
				break;
		case 4:	controlName = NiFpga_NeuroRobotAllCardsSPI_IndicatorU16_CurrentState4;
				break;
		case 5:	controlName = NiFpga_NeuroRobotAllCardsSPI_IndicatorU16_CurrentState5;
				break;
		case 6:	controlName = NiFpga_NeuroRobotAllCardsSPI_IndicatorU16_CurrentState6;
				break;
		case 7:	controlName = NiFpga_NeuroRobotAllCardsSPI_IndicatorU16_CurrentState7;
				break;
		default:	controlName = NiFpga_NeuroRobotAllCardsSPI_IndicatorU16_CurrentState0;
				break;
	}
	return _fpga_util->ReadEnum(controlName);
}

void SPI::RunSPI(){
	int _numberOfCards = 8;

	// Arrays which are toggle transacted
	uint32_t outgoingPacketPointer[8][32] = {0};
	uint32_t incomingPacketPointer[8][32] = {0};

	// Pack and Write Outgoing Message
	for(int i = 0; i < _numberOfCards; i++)
	{
		_packets->outgoingPackets[i].startWord_packId = 0xfefefe01;
		_packets->outgoingPackets[i].systemState = 0x00000000;
		PackOutgoingPacket(_packets->outgoingPackets[i], outgoingPacketPointer[i]);
		WritePacket(i , outgoingPacketPointer[i]);
		// PrintPacket(i,1,outgoingPacketPointer[i]);
	}

	// Wait for FPGA to return to idle state
	while(ReadCurrentState(0)!=Idle && ReadCurrentState(1)!=Idle && ReadCurrentState(2)!=Idle &&
			ReadCurrentState(3)!=Idle && ReadCurrentState(4)!=Idle && ReadCurrentState(5)!=Idle &&
			ReadCurrentState(6)!=Idle && ReadCurrentState(7)!=Idle);

	// Toggle Transact for all cards all at once
	ToggleTransact(1);

	//Read and Unpack Incoming Message
	for(int i = 0; i < _numberOfCards; i++)
	{
		ReadPacket(i, incomingPacketPointer[i]);
		UnpackIncomingPacket(incomingPacketPointer[i], &_packets->incomingPackets[i]);
		// PrintPacket(i,2,incomingPacketPointer[i]);
	}

	return ;
}


//-------------------------------------------------------------------
// Static Method for the SPI thread
// Input: void *
// Output: void *
//-------------------------------------------------------------------
void* SPI::ThreadSPI(void* spi_struct){
//	// Obtain a pointer to an SPI_struct which contains pointers to
//	// the packets and the fpga utilities needed to communicate data
//	Logger& log = Logger::GetInstance();
//	SPI_struct* ss = (SPI_struct *)spi_struct;
//	Packets *allPackets = ss->packets;
//	FPGA_Utilities *fpga_util = ss->fpga_util;
//
//	// Value used in the SPI Thread for packing
//	int _numberOfCards = 8;
//
//	// Create the SPI object to get methods from
//	SPI _spiModule = SPI(fpga_util);
//
//	// Create Timer object for profiling code
//	Timer _timer = Timer();
//
//	// Arrays which are toggle transacted
//	uint32_t outgoingPacketPointer[8][32];
//	uint32_t incomingPacketPointer[8][32];
//
//	// While loop thread
//	while(1)
//	{
//		//Start profile code
//		_timer.tic();
//
//		// Pack and Write Outgoing Message
//		for(int i = 0; i < _numberOfCards; i++)
//		{
//			allPackets->outgoingPackets[i].startWord_packId = 0xfefefe01;
//			allPackets->outgoingPackets[i].systemState = 0x00000000;
//			_spiModule.PackOutgoingPacket(allPackets->outgoingPackets[i], outgoingPacketPointer[i]);
//			_spiModule.WritePacket(i , outgoingPacketPointer[i]);
//			//_spiModule.PrintPacket(i,2,outgoingPacketPointer[i]);
//		}
//
//		// Wait for FPGA to return to idle state
//		while(_spiModule.ReadCurrentState(0)!=Idle && _spiModule.ReadCurrentState(1)!=Idle && _spiModule.ReadCurrentState(2)!=Idle &&
//				_spiModule.ReadCurrentState(3)!=Idle && _spiModule.ReadCurrentState(4)!=Idle && _spiModule.ReadCurrentState(5)!=Idle &&
//				_spiModule.ReadCurrentState(6)!=Idle && _spiModule.ReadCurrentState(7)!=Idle);
//
//		// Toggle Transact for all cards all at once
//		_spiModule.ToggleTransact(1);
//
//		//Read and Unpack Incoming Message
//		for(int i = 0; i < _numberOfCards; i++)
//		{
//			_spiModule.ReadPacket(i, incomingPacketPointer[i]);
//			_spiModule.UnpackIncomingPacket(incomingPacketPointer[i], &allPackets->incomingPackets[i]);
//			//_spiModule.PrintPacket(i,1,incomingPacketPointer[i]);
//		}
//
//		do {
//			_timer.toc();
//		}while ((_timer.time()) < 700);
//
//		// End profile Code
//		//log.Log("Time SPI:" + to_string(_timer.time()), logger::INFO, true);
//
//	}

	return{ NULL };
}





