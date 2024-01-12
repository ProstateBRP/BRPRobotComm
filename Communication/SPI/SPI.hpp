//============================================================================
// Name        : SPI.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where SPI Communication is handled.
//============================================================================

#ifndef SPI_HPP_
#define SPI_HPP_

#include "FPGA_Utilities.hpp"
#include "Logger.hpp"
#include "Timer.hpp"

class SPI
{
public:
	//================ Constructor ================
	SPI();
	SPI(Packets *_packets, FPGA_Utilities *fpgaUtil);

	//================ Parameters =================
	// Parameters are declared in constructor
	Packets *_packets;
	FPGA_Utilities *_fpga_util;

	//================ Public Methods ==============
	int counter{0}; // Used only for counting good packets for printing
	int good{0}; // Used only for counting good packets for printing
	void InitializeFPGA();
	static void *ThreadSPI(void *);
	incomingPacket SyncCard(int card_id, outgoingPacket outPacket);

	void PrintPacket(int card_id, int desiredCard, uint32_t *incomingPacket);
	void PrintGoodPacketCount(int card_id, int desiredCard, uint32_t *incomingPacket);
	void WritePacket(int card, uint32_t *outgoingPacket);
	void ReadPacket(int card, uint32_t *incomingPacket);
	void ToggleTransact(uint8_t value);
	void SetControllerLEDState(int ledId, uint8_t r, uint8_t g, uint8_t b);
	void PackOutgoingPacket(outgoingPacket packet, uint32_t *packedData);
	void UnpackIncomingPacket(uint32_t *unpackedData, incomingPacket *packet);

	CurrentState ReadCurrentState(int card);

	void RunSPI();
};

struct SPI_struct
{
	Packets *packets;
	FPGA_Utilities *fpga_util;
};

#endif /* SPI_HPP_ */
