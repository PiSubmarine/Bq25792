#pragma once

#include <cstdint>
#include <functional>
#include <thread>
#include <chrono>
#include <stop_token>
#include <vector>
#include <condition_variable>

namespace PiSubmarine::Bq25792
{

	struct I2CRequest
	{
		uint8_t DeviceAddress;
		uint8_t* RxData;
		size_t RxLen;
		std::vector<uint8_t> TxData;
		I2CCallback Callback;
		bool IsWrite = false;
	};

	class I2CDriverMock
	{
	public:
		I2CDriverMock(std::array<uint8_t, 0x49>& data) : m_Data(data)
		{
			m_WorkerThread = std::jthread([this](std::stop_token st) {
				this->WorkerMethod(st);
				});
		}

		bool GetSimulateError()
		{
			return m_SimalateError;
		}

		void SetSimulateError(bool value)
		{
			m_SimalateError = value;
		}

		bool Read(uint8_t deviceAddress, uint8_t* rxData, size_t len)
		{
			memcpy(rxData, m_Data.data() + m_DataOffset, len);
			return !m_SimalateError;
		}

		bool Write(uint8_t deviceAddress, uint8_t* txData, size_t len)
		{
			m_DataOffset = txData[0];
			memcpy(m_Data.data() + m_DataOffset, txData + 1, len - 1);
			return !m_SimalateError;
		}

		bool ReadAsync(uint8_t deviceAddress, uint8_t* rxData, size_t len, I2CCallback callback)
		{
			std::lock_guard lock(m_Mutex);
			if (m_HasRequest)
			{
				return false;
			}

			m_Request.DeviceAddress = deviceAddress;
			m_Request.RxData = rxData;
			m_Request.RxLen = len;
			m_Request.Callback = callback;
			m_Request.IsWrite = false;

			m_HasRequest = true;
			return true;
		}

		bool WriteAsync(uint8_t deviceAddress, uint8_t* txData, size_t len, I2CCallback callback)
		{
			std::lock_guard lock(m_Mutex);
			if (m_HasRequest)
			{
				return false;
			}

			m_Request.DeviceAddress = deviceAddress;
			m_Request.TxData.resize(len);
			memcpy(m_Request.TxData.data(), txData, len);
			m_Request.Callback = callback;
			m_Request.IsWrite = true;

			m_HasRequest = true;
			return true;
		}

	private:
		std::jthread m_WorkerThread;
		std::array<uint8_t, 0x49>& m_Data;
		std::chrono::milliseconds m_TransactionDelay = std::chrono::milliseconds(250);
		I2CRequest m_Request;
		bool m_SimalateError = false;
		bool m_HasRequest = false;
		uint8_t m_DataOffset = 0;

		std::mutex m_Mutex;

		void WorkerMethod(std::stop_token st)
		{
			while (!st.stop_requested())
			{
				std::this_thread::sleep_for(m_TransactionDelay);

				{
					std::lock_guard lock(m_Mutex);
					if (!m_HasRequest)
					{
						continue;
					}
				}

				bool ok = false;
				if (m_Request.IsWrite)
				{
					ok = Write(m_Request.DeviceAddress, m_Request.TxData.data(), m_Request.TxData.size());
				}
				else
				{
					ok = Read(m_Request.DeviceAddress, m_Request.RxData, m_Request.RxLen);
				}

				m_Request.Callback(m_Request.DeviceAddress, ok);

				{
					std::lock_guard lock(m_Mutex);
					m_HasRequest = false;
				}
			}
		}
	};

}