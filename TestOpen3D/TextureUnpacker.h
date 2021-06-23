#pragma once

#include <uvpCore.hpp>

#include <iostream>
#include <unordered_map>
#include <list>
#include <vector>
#include <string>
#include <cstring>
#include <array>

#include "open3d/Open3D.h"
#include "open3d/io/sensor/azure_kinect/MKVMetadata.h"
#include "open3d/geometry/RGBDImage.h"

using namespace uvpcore;
using namespace open3d;

//TODO: Make sense of all this stuff
inline void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg);

class TextureUnpacker
{
    friend void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg);

    typedef std::array<UvpMessageT*, static_cast<int>(UvpMessageT::MESSAGE_CODE::VALUE_COUNT)> UvpMessageArrayT;

    class UvpOpExecutorT
    {
    private:
        friend void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg);

        std::list<UvpMessageT*> m_ReceivedMessages;
        UvpMessageArrayT m_LastMessagePerCode;

        bool m_DebugMode;

        void destroyMessages()
        {
            // The application becomes the owner of UVP messages after receiving it,
            // so we have to make sure they are eventually deallocated by calling
            // the destory method on them (do not use the delete operator).
            for (UvpMessageT* pMsg : m_ReceivedMessages)
            {
                pMsg->destroy();
            }
            m_ReceivedMessages.clear();
        }

        void reset()
        {
            destroyMessages();
            m_LastMessagePerCode = { nullptr };
        }

        void handleMessage(UvpMessageT* pMsg)
        {
            // This method is called every time the packer sends a message to the application.
            // We need to handle the message properly.

            if (pMsg->m_Code == UvpMessageT::MESSAGE_CODE::PROGRESS_REPORT)
            {
                UvpProgressReportMessageT* pReportProgressMsg = static_cast<UvpProgressReportMessageT*>(pMsg);

                std::cout << "[UVP PROGRESS REPORT] Phase: " << static_cast<int>(pReportProgressMsg->m_PackingPhase);
                for (int i = 0; i < pReportProgressMsg->m_ProgressSize; i++)
                {
                    std::cout << ", Progress[" << i << "]: " << pReportProgressMsg->m_ProgressArray[i];
                }
                std::cout << "\n";
            }
            else if (pMsg->m_Code == UvpMessageT::MESSAGE_CODE::BENCHMARK)
            {
                UvpBenchmarkMessageT* pBenchmarkMsg = static_cast<UvpBenchmarkMessageT*>(pMsg);

                std::cout << "[UVP BENCHMARK] Device name: " << pBenchmarkMsg->m_DeviceName.c_str() << ", Total packing time (ms): " <<
                    pBenchmarkMsg->m_TotalPackTimeMs << ", Average packing time (ms): " << pBenchmarkMsg->m_AvgPackTimeMs << "\n";
            }

            m_LastMessagePerCode[static_cast<int>(pMsg->m_Code)] = pMsg;
            m_ReceivedMessages.push_back(pMsg);
        }


    public:
        UvpOpExecutorT(bool debugMode) :
            m_DebugMode(debugMode)
        {}

        ~UvpOpExecutorT()
        {
            destroyMessages();
        }

        UVP_ERRORCODE execute(UvpOperationInputT& uvpInput)
        {
            reset();

            uvpInput.m_pMessageHandler = opExecutorMessageHandler;
            uvpInput.m_pMessageHandlerData = this;

            if (m_DebugMode)
            {
                // Check whether the application configurated the operation input properly.
                // WARNING: this operation is time consuming (in particular it iterates over all UV data),
                // that is why it should only be executed when debugging the application. It should
                // never be used in production.
                const char* pValidationResult = uvpInput.validate();

                if (pValidationResult)
                {
                    throw std::runtime_error("Operation input validation failed: " + std::string(pValidationResult));
                }
            }

            UvpOperationT uvpOp(uvpInput);
            // Execute the actual operation - this call will block the current thread
            // until the operation is finished.
            UVP_ERRORCODE retCode = uvpOp.entry();

            return retCode;
        }

        UvpMessageT* getLastMessage(UvpMessageT::MESSAGE_CODE code)
        {
            return m_LastMessagePerCode[static_cast<int>(code)];
        }
    };
    
public:
    bool UnpackTexture(geometry::Image& im, geometry::TriangleMesh& mesh, bool debug);

    bool PerformTextureUnpack(geometry::Image& im, geometry::TriangleMesh& mesh, bool debug_info);
};

inline void opExecutorMessageHandler(void* m_pMessageHandlerData, UvpMessageT* pMsg)
{
    reinterpret_cast<TextureUnpacker::UvpOpExecutorT*>(m_pMessageHandlerData)->handleMessage(pMsg);
}