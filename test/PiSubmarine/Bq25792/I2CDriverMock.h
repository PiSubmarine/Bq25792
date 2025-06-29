#pragma once

#include <cstdint>
#include <functional>
#include <thread>

class I2CDriverMock
{
public:
	bool Read(uint8_t deviceAddress, uint8_t* rxData, size_t len)
	{

	}

	bool Write(uint8_t deviceAddress, const uint8_t* txData, size_t len)
	{

	}

	bool ReadAsync(uint8_t deviceAddress, uint8_t* rxData, size_t len, std::function<void(bool)> callback)
	{
		return false;
	}

	bool WriteAsync(uint8_t deviceAddress, const uint8_t* txData, size_t len, std::function<void(bool)> callback)
	{

	}

private:
	std::jthread m_WorkerThread;
};